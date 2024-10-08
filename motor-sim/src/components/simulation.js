import CircularBuffer from "circular-buffer";
import _ from "lodash";

import { TimingStats } from "./utils.js";
import { float } from "three/webgpu";

export const π = Math.PI;

/* Normalize angle values to the range [-π, π]. */
export function normalized_angle(angle) {
  return (angle + 3 * π) % (2 * π) - π;
}

/* Check whether the angle is inside the interval; assuming normalized angles. */
export function interval_contains_angle(low, high, angle) {
  if (low < high) {
    return low <= angle && angle <= high;
  } else {
    return low <= angle || angle <= high;
  }
}

/* Get the bounds for the hall sensor toggling angles.

Assuming angle φ is in the range [0, 2π], if the U phase is at 0, then V is at 4π/6 and W is at 8π/6.
The hall sensors are placed between the phase poles at 2π/6, 6π/6 and 10π/6.

*/
export function hall_bounds(hall_toggle_angle) {
  return {
    hall_1_low: normalized_angle(2 * π / 6 - hall_toggle_angle),
    hall_1_high: normalized_angle(2 * π / 6 + hall_toggle_angle),
    hall_2_low: normalized_angle(6 * π / 6 - hall_toggle_angle),
    hall_2_high: normalized_angle(6 * π / 6 + hall_toggle_angle),
    hall_3_low: normalized_angle(10 * π / 6 - hall_toggle_angle),
    hall_3_high: normalized_angle(10 * π / 6 + hall_toggle_angle),
  };
}

/* Hall sensor toggle angle (with respect to rotor angle). */
export const hall_toggle_angle = 80 * π / 180;

/* Convert from RPM/Volts constant to the motor electric constant. 

The motor constant represents back emf / angular rotation (in radians/second). 
*/
export function RPM_Kv_to_Ke(RPM_Kv){
  return 60.0 / (RPM_Kv * 2.0 * π);
}

/* Initial parameters governing our motor physics. */
export const initial_parameters = {
  R_phase: 1.3, // phase resistance
  L_phase: 0.000_1, // phase inductance
  Ψ_m: RPM_Kv_to_Ke(1000), // motor Ke constant (also the magnetic flux linkage)
  τ_static: 0.0001, // static friction torque
  τ_dynamic: 0.00001, // dynamic friction torque
  V_bat: 12.0, // battery voltage
  R_bat: 1.0, // battery internal resistance
  L_bat: 0.000_001, // 1μH inductance of wire leads up to the battery
  R_disconnect: 10_000.0, // 10kΩ battery disconnected resistance (keep relatively low for numerical stability)
  L_disconnect: 0.001, // 1mH inductance to prevent the simulation from blowing up
  C_near: 0.000_050, // 50μF capacitance near the motor mosfets
  R_shunt: 0.010, // shunt resistance
  R_mosfet: 0.013, // mosfet resistance
  V_diode: 0.72, // mosfet reverse diode voltage drop
  J_rotor: 0.00000025, // (Kg*m^2) rotor moment of inertia
  M_rotor: 0.0005, // (Kg) rotor mass
  r_max: 0.001, // (m) rotor radial displacement for magnetic field dropoff
  ...hall_bounds(hall_toggle_angle), // hall sensor toggle angle bounds
}

/* Smallest current that we'll zero out in a single step. */
const I_ε = 0.000_001;

/* The initial state of the differentiable state equation for the motor. */
export const initial_state = {
  t: 0.0, // simulation time
  φ: 0.0, // motor angle
  ω: 0.0, // motor speed
  Iu: 0.0, // motor current phase U
  Iv: 0.0, // motor current phase V
  Iw: 0.0, // motor current phase W
  𝜈: 0.0, // rotor radial velocity
  I: 0.0, // current flowing into the battery (charging it when positive), or the terminal current when battery disconnected
  V: 0.0, // voltage presented to the battery by the capacitor bank near the motor connections (effectively the motor voltage).
}

/* Convention for marking the commanded state of each half-bridge. */
export const phase_switches = {
  0: "floating",
  1: "on_high",
  2: "on_low",
  floating: 0,
  on_high: 1,
  on_low: 2,
};


/* Inputs to the physical system for a freewheeling motor.

For this simulator, inputs means everything that affects the system that is independent
of the differential equations that describe the internal state of the system. This includes
the motor controller algorithm and the external load torque.
*/
const freewheeling_inputs = {
  U_switch: 0, // driver connection state for phase U
  V_switch: 0, // driver connection state for phase V
  W_switch: 0, // driver connection state for phase W
  τ_load: 0.0, // external load torque
  battery_connected: false, // whether the battery is plugged in (instantly)
};


/* Outputs of the physical system (for example the state of the driver sensors. */
export function measurable_outputs(state, parameters) {
  // Get salient parameters.
  const {hall_1_low, hall_1_high, hall_2_low, hall_2_high, hall_3_low, hall_3_high} = parameters;
  const {φ} = state;

  // The hall sensors mostly see the permanent magnet field, and they have a triggering point
  // based on the strength of the field. In practice, the rotor north pole must be within a
  // certain angle to the sensor to trigger it.
  const hall_1 = interval_contains_angle(hall_1_low, hall_1_high, φ);
  const hall_2 = interval_contains_angle(hall_2_low, hall_2_high, φ);
  const hall_3 = interval_contains_angle(hall_3_low, hall_3_high, φ);

  return {hall_1, hall_2, hall_3};
}

/* Convention for the currently conducting state of each half-bridge. */
const phase_states = {
  0: "neutral", // No current flowing.
  1: "high_conducting", // Current flowing through the high side; turned on.
  2: "high_diode", // Current flowing through the high side reverse diode.
  3: "low_diode", // Current flowing through the low side reverse diode.
  4: "low_conducting", // Current flowing through the low side; turned on.
  neutral: 0,
  high_conducting: 1,
  high_diode: 2,
  low_diode: 3,
  low_conducting: 4,
};


/* Compute the state of a half-bridge.

If there's current through the winding, the winding will raise its voltage to keep the current flowing.
However if there's no current, then we must overcome the reverse diode voltage drop and any other circuit
element to get any current started.
*/
function compute_half_bridge_state(A_switch, Ia, Vreverse, Va, Vmin, Vmax) {
  return (
    (A_switch == 1) ? 1 : // high conducting
    (A_switch == 2) ? 4 : // low conducting
    ( // A must be floating, we won't double check.
      (Ia > +I_ε) ? 2 : // high diode
      (Ia < -I_ε) ? 3 : // low diode
      (Va - Vmin > Vreverse) ? 2 : // high diode
      (Va - Vmax < -Vreverse) ? 3 : // low diode
      0 // neutral
    )
  );
}

/* Signed contribution of the mosfet resistance for the phase given the half-bridge state. */
const bridge_r_sign = [
  0.00, // neutral
  +1.0, // high_conducting
  0.00, // high_diode
  0.00, // low_diode
  -1.0, // low_conducting
];

/* Signed contribution of the mosfet reverse diode voltage per half-bridge state. */
const bridge_diode_sign = [
  0.00, // neutral
  0.00, // high_conducting
  +1.0, // high_diode
  -1.0, // low_diode
  0.00, // low_conducting
];

/* Sign of the upstream current coming out of the motor (positive when the motor is charging the battery). */
const bridge_current_sign = [
  0.00, // neutral
  +1.0, // high_conducting
  +1.0, // high_diode
  -1.0, // low_diode
  -1.0, // low_conducting
];

/* Sign of upstream voltage contribution given the state of the phase to phase connection. */
const vcc_matrix = [
  [0.00, 0.00, 0.00, 0.00, 0.00], // neutral A
  [0.00, 0.00, 0.00, +1.0, +1.0], // high_conducting A
  [0.00, 0.00, 0.00, +1.0, +1.0], // high_diode A
  [0.00, -1.0, -1.0, 0.00, 0.00], // low_diode A
  [0.00, -1.0, -1.0, 0.00, 0.00], // low_conducting A
];

/* Compute the differential equations at the current state. 

Store other info we compute along the way so we can display it in pretty plots.

We need the dt resolution in order to handle values close to 0 (motor stopping, current going to 0).
*/
function compute_state_diff(state, dt, parameters, inputs, outputs) {
  // Get salient parameters.
  const {
    R_phase, L_phase, Ψ_m, τ_static, τ_dynamic, 
    V_bat, R_bat, L_bat, C_near, R_disconnect, L_disconnect,
    R_shunt, R_mosfet, V_diode, J_rotor, 
    M_rotor, r_max,
  } = parameters;
  const {φ, ω, Iu, Iv, Iw, 𝜈, I, V} = state;
  const {U_switch, V_switch, W_switch, τ_load, battery_connected} = inputs;

  // Differential of the rotor angle is the rotor rotation.
  const dφ = ω;
  
  // Calculate the induced emf from the rotating magnetic field of the permanent magnet.
  const Vu_rotational_emf = Ψ_m * ω * Math.sin(φ);
  const Vv_rotational_emf = Ψ_m * ω * Math.sin(φ - 2 * Math.PI / 3);
  const Vw_rotational_emf = Ψ_m * ω * Math.sin(φ + 2 * Math.PI / 3);

  // Calculate the induced emf from the axial displacement of the magnet. Because magnetic
  // fields drop off quickly with distance, small displacements will have a large effect.
  // The distance in this case is the spacing between the rotor and the stator core; aka
  // the air gap. The air gap is quite tiny in high performance motors; we won't see the
  // displacements, but we will hear them as motor noise.
  const Vu_radial_emf = Ψ_m * 𝜈 / r_max * Math.cos(φ);
  const Vv_radial_emf = Ψ_m * 𝜈 / r_max * Math.cos(φ - 2 * Math.PI / 3);
  const Vw_radial_emf = Ψ_m * 𝜈 / r_max * Math.cos(φ + 2 * Math.PI / 3);
  
  // Total emf contributions from the moving permanent magnet field.
  const Vu_emf = Vu_rotational_emf; // + Vu_radial_emf;
  const Vv_emf = Vv_rotational_emf; // + Vv_radial_emf;
  const Vw_emf = Vw_rotational_emf; // + Vw_radial_emf;

  // Calculate the reverse voltage needed to start flowing current against the
  // battery and the reverse diodes of 2 mosfets.
  const Vreverse = V_bat + 2 * V_diode;
  // Get the minimum and maximum voltages of the 3 phases. We'll compare it
  // to the reverse voltage to determine the state of the half-bridges when no
  // current is flowing.
  const Vmin = Math.min(Vu_emf, Vv_emf, Vw_emf);
  const Vmax = Math.max(Vu_emf, Vv_emf, Vw_emf);

  // Compute the state of the half-bridges to determine which way the mosfet connections
  // are conducting for each motor phase.
  const U_state = compute_half_bridge_state(U_switch, Iu, Vreverse, Vu_emf, Vmin, Vmax);
  const V_state = compute_half_bridge_state(V_switch, Iv, Vreverse, Vv_emf, Vmin, Vmax);
  const W_state = compute_half_bridge_state(W_switch, Iw, Vreverse, Vw_emf, Vmin, Vmax);

  // Get the phase to phase conducting states.
  const UV_conducting = U_state && V_state;
  const VW_conducting = V_state && W_state;
  const WU_conducting = W_state && U_state;
  const UVW_conducting = U_state && V_state && W_state;

  // In the 3 phase space of the motor the voltages sum up to 0 and we consider the tie
  // point of our Y configured motor to also be at 0V. Thus we can compute the phase voltages
  // as the voltage difference between the phase terminal (after the shunt) and the neutral point.
  // However just 2 phases are connected sometimes and the extra unconnected phase shouldn't contribute
  // even if it has a voltage (because it's not enough to overcome the reverse diodes). Because of the
  // symmetry of all 3 phases, the neutral point is still 0V.
  function triple_duo_or_none(UV, VW, WU, U_zero, V_zero, W_zero){
    return {
      U: UVW_conducting ? (UV - WU) / 3.0 : UV_conducting ? UV / 2.0 : WU_conducting ? -WU / 2.0 : U_zero,
      V: UVW_conducting ? (VW - UV) / 3.0 : VW_conducting ? VW / 2.0 : UV_conducting ? -UV / 2.0 : V_zero,
      W: UVW_conducting ? (WU - VW) / 3.0 : WU_conducting ? WU / 2.0 : VW_conducting ? -VW / 2.0 : W_zero,
    }
  }

  // Calculate the voltage drop from the passive resistors on each phase.
  const V_Ru = (R_phase + R_shunt) * Iu;
  const V_Rv = (R_phase + R_shunt) * Iv;
  const V_Rw = (R_phase + R_shunt) * Iw;

  // Alias the driving voltage (we can either simulate a capacitor bank or straight to battery).
  const VCC = V;

  // Calculate the voltage drop from the 3 phase connections to the battery as 
  // pairs of conducting phases.
  const VCC_uv = vcc_matrix[U_state][V_state] * VCC;
  const VCC_vw = vcc_matrix[V_state][W_state] * VCC;
  const VCC_wu = vcc_matrix[W_state][U_state] * VCC;

  // Get the driving voltage seen by each phase.
  const {U: VCC_u, V: VCC_v, W: VCC_w} = triple_duo_or_none(VCC_uv, VCC_vw, VCC_wu, 0.0, 0.0, 0.0);

  // Get the mosfet voltage drop for each half-bridge, so we can compute power loss and heating.
  // The current convention is that its positive coming out of the motor terminals.
  const V_Mu = R_mosfet * bridge_r_sign[U_state] * Iu + V_diode * bridge_diode_sign[U_state];
  const V_Mv = R_mosfet * bridge_r_sign[V_state] * Iv + V_diode * bridge_diode_sign[V_state];
  const V_Mw = R_mosfet * bridge_r_sign[W_state] * Iw + V_diode * bridge_diode_sign[W_state];

  // Add up all voltage contributions for each phase.
  const V_u = Vu_emf - V_Ru - VCC_u - V_Mu;
  const V_v = Vv_emf - V_Rv - VCC_v - V_Mv;
  const V_w = Vw_emf - V_Rw - VCC_w - V_Mw;

  // Calculate the phase to phase voltages too.
  const V_uv = V_u - V_v;
  const V_vw = V_v - V_w;
  const V_wu = V_w - V_u;

  // Calculate the current change through each pair of phases.
  const dIuv = UV_conducting ? V_uv / (2 * L_phase) : 0.0;
  const dIvw = VW_conducting ? V_vw / (2 * L_phase) : 0.0;
  const dIwu = WU_conducting ? V_wu / (2 * L_phase) : 0.0;

  // Do the same trick for 
  const {U: dIu, V: dIv, W: dIw} = triple_duo_or_none(dIuv, dIvw, dIwu, -Iu/dt, -Iv/dt, -Iw/dt);

  // Calculate rotor torque due to motor magnetic field. The contributions
  // of the 3 phase windings add up linearly.
  const τ_emf = - (
    Ψ_m * Iu * Math.sin(φ) +
    Ψ_m * Iv * Math.sin(φ - 2 * Math.PI / 3) +
    Ψ_m * Iw * Math.sin(φ + 2 * Math.PI / 3)
  );

  // Get the total torque applied to the rotor before friction.
  const τ_applied = τ_emf + τ_load;
  // Calculate the sign of the static friction torque. It will oppose the applied
  // torque when the rotor is standing still.
  const static_sign = (ω == 0.0) ? -Math.sign(τ_applied) : -Math.sign(ω);
  // Calculate maximum friction contribution.
  const τ_friction = -ω * τ_dynamic + static_sign * τ_static;
  // Calculate the total torque applied to the rotor assuming that the applied
  // torque is greater than friction.
  const τ_total = τ_applied + τ_friction;
  // Check if applied torque is enough to start rotating the rotor.
  const frozen = (ω == 0.0) && Math.abs(τ_applied) <= τ_static;
  // Compute the delta assuming the rotor is not frozen.
  const dω_computed = τ_total / J_rotor;
  // Delta needed to stop the rotor in 1 step.
  const dω_stopping = -ω/dt;
  // Check if the rotor would stop and reverse direction in 1 step.
  const stopping = -Math.sign(ω) * dω_computed > -Math.sign(ω)*dω_stopping;
  // We need to prevent static friction from overshooting when the motor is stopped or stopping in 1 step.
  const dω = (frozen || stopping) ? dω_stopping : (τ_total / J_rotor);

  // TODO: enable radial velocity changes.
  const d𝜈 = 0.0;

  // Calculate the current flowing out of the motor terminals.
  const I_motor = (
    bridge_current_sign[U_state] * Iu + 
    bridge_current_sign[V_state] * Iv + 
    bridge_current_sign[W_state] * Iw);

  // The current flowing out of the motor goes either to the near capacitor or the battery.
  const I_near_cap = I_motor - I;
  // Compute the rate of charge of the capacitor bank.
  const dV = I_near_cap / C_near;
  // Get the battery connection parameters.
  const V_terminal = battery_connected ? (V_bat + I * R_bat) : (0.0 + I * R_disconnect);
  // To prevent the simulation from blowing up, we need to use a higher inductance to limit 
  // the magnitude of the current delta.
  const L_terminal = battery_connected ? L_bat : L_disconnect;
  // Compute the rate of change of the current on the power line. Whether connected or 
  // disconneted, current must flow. If it's suddenly disconnected, that means sparkies!
  const dI = (V - V_terminal) / L_terminal;

  const U_status = bridge_current_sign[U_state];
  const V_status = bridge_current_sign[V_state];
  const W_status = bridge_current_sign[W_state];

  return {
    diff: {t: 1.0, φ: dφ, ω: dω, Iu: dIu, Iv: dIv, Iw: dIw, 𝜈: d𝜈, V: dV, I: dI},
    info: {
      τ_emf, τ_applied, τ_friction, τ_total, frozen, stopping,
      V_Ru, V_Rv, V_Rw, V_Mu, V_Mv, V_Mw, V_u, V_v, V_w, VCC_u, VCC_v, VCC_w,
      V_terminal, I_near_cap, I_motor,
      Vu_rotational_emf, Vv_rotational_emf, Vw_rotational_emf,
      Vu_radial_emf, Vv_radial_emf, Vw_radial_emf,
      U_status, V_status, W_status,
      rpm: ω * 30 / π,
    },
  };
}

/* Post process the state after updating from the differential equations.

It's mostly to normalize angles back to the range [-π, π]. We might also want to zero out
very small values.
*/
function postprocess_state(state) {
  state.φ = normalized_angle(state.φ);
  return state;
}

/* Update the state using a simple Euler step `f(x+dx) = f(x) + df(x)/dt * dt`. */
function euler_step(state, diff, dt) {
  return _.mapValues(state, (value, key) => value + diff[key] * dt);
}

/* Update the state using the Runge Kuta algorithm, which samples the differential at multiple nearby points. */
function runge_kuta_step(state, diff, dt, parameters, inputs, outputs) {
  const k1 = diff;
  const k2 = compute_state_diff(euler_step(state, k1, dt / 2), dt / 2, parameters, inputs, outputs).diff;
  const k3 = compute_state_diff(euler_step(state, k2, dt / 2), dt / 2, parameters, inputs, outputs).diff;
  const k4 = compute_state_diff(euler_step(state, k3, dt), dt, parameters, inputs, outputs).diff;

  return _.mapValues(state, (value, key) => value + (k1[key] + 2 * k2[key] + 2 * k3[key] + k4[key]) * dt / 6);
}

/* Options to simulate a motor similar to the physical hardware. */
export const default_simulation_options = {
  start_state: initial_state,
  parameters: initial_parameters,
  update_inputs: freewheeling_inputs,
  update_memory: {},
  dt: 1.0 / 72_000_000,
  state_update: runge_kuta_step,
  max_stored_steps: 400,
  steps_number: 4_000,
  store_period: 2_000,
};


/* Simulation of a 3-phase permanent magnet brushless motor. */
export class Simulation {
  constructor(options={}) {
    this.options = {...default_simulation_options, ...options};
    const {start_state,  max_stored_steps} = this.options;

    // Initialize the state of the motor with a copy of the start state.
    this.state = {...start_state};
    // Store data in a circular buffer because copying this array does actually slow down the simulation.
    this.history_buffer = new CircularBuffer(max_stored_steps);
    // Start the simulation at step 0.
    this.step = 0;

    // Initialize the outputs and inputs and other derivatives, but do not advance state.
    this.compute_step();

    // Fill the history buffer with copies of the initial state; it makes graphs start uniformly.
    for (let i = 0; i < max_stored_steps; i++) this.history_buffer.push(this.flattened_state());

    // Compute runtime statistics.
    this.stats = new TimingStats();
    // Simulation slowdown compared to wall clock time.
    this.slowdown = 1.0;
    // Simulation running status.
    this.running = true;
  }

  get history() {
    return this.history_buffer.toarray();
  }
  
  /* Advance the simulation by a single step. */
  compute_step() {
    // Get salient options.
    const {parameters, update_inputs, update_memory, state_update, dt} = this.options;
    
    // Get the current state.
    const {state} = this;

    // Calculate sensor values.
    const outputs = this.outputs = measurable_outputs(state, parameters);
    // Calculate the motor driver inputs (driving algorithm) and external influences (load torque, battery connected).
    const inputs = this.inputs = update_inputs(state, parameters, outputs, update_memory);
    // Calculate the differential equations at the current state.
    const computation = compute_state_diff(state, dt, parameters, inputs, outputs);

    const diff = this.diff = computation.diff;
    this.info = computation.info;

    // Update the state using the differential equations.
    return postprocess_state(state_update(state, diff, dt, parameters, inputs, outputs));
  }

  /* Get a complete copy of the state at the current simulation step. */
  flattened_state() {
    return {step: this.step, ...this.state, ...this.outputs, ...this.inputs, ...this.info};
  }

  /* Advance the simulation for a batch of steps and compute statistics. */
  update() {
    const {steps_number, store_period} = this.options;

    for (let i = 0; i <steps_number; i++){

      this.state = this.compute_step();

      this.step += 1;

      const {step} = this;

      if (step % store_period == 0) {
        this.history_buffer.push(this.flattened_state());
      }
    }

    this.stats.update();
    this.slowdown = 1.0 / (Math.max(0.1, this.stats.fps) * this.options.dt * this.options.steps_number);
    this.running = true;
  }

  /* Display the current simulation state. */
  update_graphics(motor) {
    const state = this.flattened_state();
    const {φ, Iu, Iv, Iw, hall_1, hall_2, hall_3} = state;
    
    motor.rotor.rotation.y = normalized_angle(φ - π / 2);
    motor.red_led.material.emissiveIntensity = hall_1 ? 10.0 : 0.0;
    motor.green_led.material.emissiveIntensity = hall_2 ? 8.0 : 0.0;
    motor.blue_led.material.emissiveIntensity = hall_3 ? 12.0 : 0.0;
    motor.coil_U.material.emissiveIntensity = 0.20 * Math.abs(Iu / 6.0);
    motor.coil_V.material.emissiveIntensity = 0.20 * Math.abs(Iv / 6.0);
    motor.coil_W.material.emissiveIntensity = 0.15 * Math.abs(Iw / 6.0);
  }
}