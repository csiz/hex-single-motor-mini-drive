import CircularBuffer from "circular-buffer";
import _ from "lodash";

import { TimingStats } from "./utils.js";

export const π = Math.PI;
const sin = Math.sin;
const cos = Math.cos;
const atan2 = Math.atan2;
const hypot = Math.hypot;

const cos_0 = 1.0;
const cos_2π_3 = -0.5;
const cos_4π_3 = -0.5;

const sin_0 = 0.0;
const sin_2π_3 = 0.86602540378;
const sin_4π_3 = -0.86602540378;

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
export const hall_toggle_angle = 85 * π / 180;

/* Convert from RPM/Volts constant to the motor electric constant. 

The motor constant represents back emf / angular rotation (in radians/second). 
*/
export function RPM_Kv_to_Ke(RPM_Kv, N_pole_pairs=2){
  return 60.0 / (N_pole_pairs * RPM_Kv * 2.0 * π);
}

/* Initial parameters governing our motor physics. */
export const initial_parameters = {
  R_phase: 1.3, // phase resistance
  L_phase: 0.000_1, // phase inductance
  Ψ_m: RPM_Kv_to_Ke(1500), // ~0.01V*s motor Ke constant (also the magnetic flux linkage)
  τ_static: 0.0002, // static friction torque
  τ_dynamic: 0.000005, // dynamic friction torque
  V_bat: 8.0, // battery voltage
  R_bat: 1.0, // battery internal resistance
  /* 1μH inductance of wire leads up to the battery. */
  L_bat: 0.000_001,
  /* 100kΩ resistance of the battery disconnect switch. */
  R_disconnect: 10_000.0,
  L_disconnect: 0.001, // 1mH disconnection inductance to prevent the simulation from blowing up
  C_near: 0.000_050, // 50μF capacitance near the motor mosfets
  R_shunt: 0.010, // shunt resistance
  R_mosfet: 0.013, // mosfet resistance
  V_diode: 0.72, // mosfet reverse diode voltage drop
  J_rotor: 0.000_000_25, // (Kg*m^2) rotor moment of inertia
  M_rotor: 0.002, // (Kg) rotor mass
  r_gap: 0.000_300, // (m) Rotor magnet to stator core effective air gap.
  /* (N/m) Radial displacement spring constant. Tapping the rotor makes a ~10kHz tone (Maybe, I dunno`).
  from which the stiffness can be estimated from a simple mass-spring oscilator at
  frequency `f = 1/2π * sqrt(r_elasticity / M_rotor)`. */
  r_elasticity: 8_000_000.0,
  /* (N) Radial friction. We'll only use a bit of static friction for radial displacement
  and let the dynamic friction be entirely due to emf interactions. */
  r_friction: 0.02,
  ...hall_bounds(hall_toggle_angle), // hall sensor toggle angle bounds
}

/* Smallest current that we'll zero out in a single step. */
const I_ε = 0.000_001;
const ω_ε = 0.000_001;
const v_ε = 0.000_001;
const r_ε = 0.000_000_100;

/* The initial state of the differentiable state equation for the motor. */
export const initial_state = {
  t: 0.0, // simulation time
  φ: 0.0, // motor angle
  ω: 0.0, // motor speed
  I_u: 0.0, // motor current phase U
  I_v: 0.0, // motor current phase V
  I_w: 0.0, // motor current phase W
  rx: 0.0, // radial displacement of the rotor in x direction
  ry: 0.0, // radial displacement of the rotor in y direction
  rx_v: 0.0, // rotor radial velocity
  ry_v: 0.0, // rotor radial velocity
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

/* Values for plotting purposes. */
export const phase_switches_values = {
  0: 0,
  1: +1,
  2: -1,
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
export const phase_states = {
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
const bridge_direction = [
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


function avoid_zero_crossing(dt, x, dx){
  return x != 0.0 && Math.sign(x) != Math.sign(x + dx*dt) ? -x/dt : dx;
}

function add_without_zero_crossing_friction(dt, x, dx_applied, dx_friction){
  const friction_overpowers = Math.abs(dx_friction) >= Math.abs(dx_applied);
  const dx = dx_applied + dx_friction;
  const sign_change_in_a_step = Math.sign(x) != Math.sign(x + dx*dt);
  return (friction_overpowers && sign_change_in_a_step) ? -x/dt : dx;
}

function sign_first_nonzero(x, applied) {
  return Math.sign(x == 0 ? applied : x);
}

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
    M_rotor, r_gap, r_friction, r_elasticity,
  } = parameters;
  const {φ, ω, I_u, I_v, I_w, rx, ry, rx_v, ry_v, I, V} = state;
  const {U_switch, V_switch, W_switch, τ_load, battery_connected} = inputs;

  // Differential of the rotor angle is the rotor rotation.
  const dφ = ω;
  
  // Alias the driving voltage (we can either simulate a capacitor bank or straight to battery).
  const VCC = V;

  const φ_rv = atan2(ry_v, rx_v);
  const rv = hypot(rx_v, ry_v);

  const r = hypot(rx, ry);

  // Calculate the magnetic flux linkage of the motor. Radial displacement degrades the 
  // linkage because of the increased air gap.
  const Ψ = Ψ_m * 1.0 / (1.0 + r/r_gap);

  // TODO: use some fourrier shenanigans to get a sound snippet from the current graph picture.
  // Definitely need to do fourier analysis on these graphs.

  // Calculate the induced emf from the rotating magnetic field of the permanent magnet.
  const V_u_rotational_emf = Ψ * ω * sin(φ);
  const V_v_rotational_emf = Ψ * ω * sin(φ - 2 * π / 3);
  const V_w_rotational_emf = Ψ * ω * sin(φ + 2 * π / 3);

  // Calculate the induced emf from the axial displacement of the magnet. Because magnetic
  // fields drop off quickly with distance, small displacements will have a large effect.
  // The distance in this case is the spacing between the rotor and the stator core; aka
  // the air gap. The air gap is quite tiny in high performance motors; we won't see the
  // displacements, but we will hear them as motor noise.
  const V_u_radial_emf = rv * cos(φ_rv)             / (r + r_gap) * Ψ * cos(φ);
  const V_v_radial_emf = rv * cos(φ_rv - 2 * π / 3) / (r + r_gap) * Ψ * cos(φ - 2 * π / 3);
  const V_w_radial_emf = rv * cos(φ_rv + 2 * π / 3) / (r + r_gap) * Ψ * cos(φ + 2 * π / 3);
  
  // Total emf contributions from the moving permanent magnet field.
  const V_u_emf = V_u_rotational_emf + V_u_radial_emf;
  const V_v_emf = V_v_rotational_emf + V_v_radial_emf;
  const V_w_emf = V_w_rotational_emf + V_w_radial_emf;

  // Calculate the reverse voltage needed to start flowing current against the
  // battery and the reverse diodes of 2 mosfets.
  const Vreverse = VCC + 2 * V_diode;
  // Get the minimum and maximum voltages of the 3 phases. We'll compare it
  // to the reverse voltage to determine the state of the half-bridges when no
  // current is flowing.
  const Vmin = Math.min(V_u_emf, V_v_emf, V_w_emf);
  const Vmax = Math.max(V_u_emf, V_v_emf, V_w_emf);

  // Compute the state of the half-bridges to determine which way the mosfet connections
  // are conducting for each motor phase.
  const U_state = compute_half_bridge_state(U_switch, I_u, Vreverse, V_u_emf, Vmin, Vmax);
  const V_state = compute_half_bridge_state(V_switch, I_v, Vreverse, V_v_emf, Vmin, Vmax);
  const W_state = compute_half_bridge_state(W_switch, I_w, Vreverse, V_w_emf, Vmin, Vmax);

  const U_direction = bridge_direction[U_state];
  const V_direction = bridge_direction[V_state];
  const W_direction = bridge_direction[W_state];
  
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
  const V_Ru = (R_phase + R_shunt) * I_u;
  const V_Rv = (R_phase + R_shunt) * I_v;
  const V_Rw = (R_phase + R_shunt) * I_w;

  // Calculate the voltage drop from the 3 phase connections to the battery as 
  // pairs of conducting phases.
  const VCC_uv = vcc_matrix[U_state][V_state] * VCC;
  const VCC_vw = vcc_matrix[V_state][W_state] * VCC;
  const VCC_wu = vcc_matrix[W_state][U_state] * VCC;

  // Get the driving voltage seen by each phase.
  const {U: VCC_u, V: VCC_v, W: VCC_w} = triple_duo_or_none(VCC_uv, VCC_vw, VCC_wu, 0.0, 0.0, 0.0);

  // Get the mosfet voltage drop for each half-bridge, so we can compute power loss and heating.
  // The current convention is that its positive coming out of the motor terminals.
  const V_Mu = R_mosfet * bridge_r_sign[U_state] * I_u + V_diode * bridge_diode_sign[U_state];
  const V_Mv = R_mosfet * bridge_r_sign[V_state] * I_v + V_diode * bridge_diode_sign[V_state];
  const V_Mw = R_mosfet * bridge_r_sign[W_state] * I_w + V_diode * bridge_diode_sign[W_state];

  // Add up all voltage contributions for each phase.
  const V_Lu = V_u_emf - V_Ru - V_Mu - VCC_u;
  const V_Lv = V_v_emf - V_Rv - V_Mv - VCC_v;
  const V_Lw = V_w_emf - V_Rw - V_Mw - VCC_w;

  // Calculate the phase to phase voltages as simple differences.
  const V_Luv = V_Lu - V_Lv;
  const V_Lvw = V_Lv - V_Lw;
  const V_Lwu = V_Lw - V_Lu;

  // Calculate the current shared by each pair of phases; with currents
  // we must avoid double counting current flowing in and out of a terminal.
  const I_uv = (I_u - I_v) * 0.5;
  const I_vw = (I_v - I_w) * 0.5;
  const I_wu = (I_w - I_u) * 0.5;


  // Calculate the current change through each pair of phases.
  const dI_uv = avoid_zero_crossing(dt, I_uv, V_Luv / (2 * L_phase));
  const dI_vw = avoid_zero_crossing(dt, I_vw, V_Lvw / (2 * L_phase));
  const dI_wu = avoid_zero_crossing(dt, I_wu, V_Lwu / (2 * L_phase));

  // Do the same trick for currents; we avoided double counting, but now we must compensate when using our trick.
  const {U: dI_u, V: dI_v, W: dI_w} = triple_duo_or_none(2 * dI_uv, 2 * dI_vw, 2 * dI_wu, -I_u/dt, -I_v/dt, -I_w/dt);

  // Calculate rotor torque due to motor magnetic field. The contributions
  // of the 3 phase windings add up linearly.
  const τ_emf = - Ψ * (
    I_u * sin(φ) +
    I_v * sin(φ - 2 * π / 3) +
    I_w * sin(φ + 2 * π / 3)
  );

  // Get the total torque applied to the rotor before friction.
  const τ_applied = τ_emf + τ_load;

  // Calculate maximum friction contribution.
  const τ_friction = -ω * τ_dynamic - sign_first_nonzero(ω, τ_applied) * τ_static;

  // Calculate the total torque applied to the rotor, for bookkeeping.
  const τ_total = τ_applied + τ_friction;

  // We need to calculate the applied acceleration and the friction acceleration separately.
  const dω_applied = τ_applied / J_rotor;
  const dω_friction = τ_friction / J_rotor;

  // Add the acceleration contributions together without allowing friction to cause a zero crossing.
  const dω = add_without_zero_crossing_friction(dt, ω, dω_applied, dω_friction);
  
  const drx = rx_v;
  const dry = ry_v;

  const Fx_emf = - Ψ / (r + r_gap) * (
    I_u * cos_0 * cos(φ) +
    I_v * cos_2π_3 * cos(φ - 2 * π / 3) +
    I_w * cos_4π_3 * cos(φ + 2 * π / 3)
  );

  const Fy_emf = - Ψ / (r + r_gap) * (
    // I_u * sin_0 * cos(φ) +
    I_v * sin_2π_3 * cos(φ - 2 * π / 3) +
    I_w * sin_4π_3 * cos(φ + 2 * π / 3)
  );

  const Fx_elasticity = -r_elasticity * rx;
  const Fy_elasticity = -r_elasticity * ry;

  const drx_v_applied = (Fx_emf + Fx_elasticity) / M_rotor;
  const dry_v_applied = (Fy_emf + Fy_elasticity) / M_rotor;

  const φ_rv_applied = (rv != 0.0 ? φ_rv : atan2(dry_v_applied, drx_v_applied));

  const drx_v_friction = - r_friction * cos(φ_rv_applied) / M_rotor;
  const dry_v_friction = - r_friction * sin(φ_rv_applied) / M_rotor;

  const drx_v = add_without_zero_crossing_friction(dt, rx_v, drx_v_applied, drx_v_friction);
  const dry_v = add_without_zero_crossing_friction(dt, ry_v, dry_v_applied, dry_v_friction);
  
  // Calculate the current flowing out of the motor terminals. Beware that we're double counting 
  // current coming out and into a terminal.
  const I_motor = 0.5 * (U_direction * I_u + V_direction * I_v + W_direction * I_w); 

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
  const dI = avoid_zero_crossing(dt, I, (V - V_terminal) / L_terminal);



  return {
    diff: {t: 1.0, φ: dφ, ω: dω, I_u: dI_u, I_v: dI_v, I_w: dI_w, rx: drx, ry: dry, rx_v: drx_v, ry_v: dry_v, V: dV, I: dI},
    info: {
      τ_emf, τ_applied, τ_friction, τ_total,
      V_Ru, V_Rv, V_Rw, V_Mu, V_Mv, V_Mw, V_Lu, V_Lv, V_Lw, VCC_u, VCC_v, VCC_w,
      V_Luv, V_Lvw, V_Lwu,
      V_terminal, I_near_cap, I_motor,
      V_u_rotational_emf, V_v_rotational_emf, V_w_rotational_emf,
      V_u_radial_emf, V_v_radial_emf, V_w_radial_emf,
      V_u_emf, V_v_emf, V_w_emf,
      U_direction, V_direction, W_direction,
      rpm: ω * 30 / π, r, rv, φ_rv,
    },
  };
}

/* Post process the state after updating from the differential equations.

It's mostly to normalize angles back to the range [-π, π]. We might also want to zero out
very small values.
*/
function postprocess_state(state) {
  state.φ = normalized_angle(state.φ);
  state.I_u = Math.abs(state.I_u) < I_ε ? 0.0 : state.I_u;
  state.I_v = Math.abs(state.I_v) < I_ε ? 0.0 : state.I_v;
  state.I_w = Math.abs(state.I_w) < I_ε ? 0.0 : state.I_w;
  state.I = Math.abs(state.I) < I_ε ? 0.0 : state.I;
  state.ω = Math.abs(state.ω) < ω_ε ? 0.0 : state.ω;
  if (Math.abs(state.rx_v) < v_ε) {
    state.rx_v = 0.0;
    state.rx = Math.abs(state.rx) < r_ε ? 0.0 : state.rx;
  }
  if (Math.abs(state.ry_v) < v_ε) {
    state.ry_v = 0.0;
    state.ry = Math.abs(state.ry) < r_ε ? 0.0 : state.ry;
  }
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
  /* Clock speed of driving microcontroller, used to update the PWM counter. */
  processor_frequency: 72_000_000,
  /* Sound sample rate, used to record variables to play as noise. */
  sound_sample_rate: 48_000,
  /* How much sound recording to keep; this buffer is much larger than plot
  data history, but we'll keep fewer variables to save memory. */
  sound_buffer_size: 48_000 * 1.0,
};


/* Simulation of a 3-phase permanent magnet brushless motor. */
export class Simulation {
  constructor(options={}) {
    this.options = {...default_simulation_options, ...options};
    const {start_state,  max_stored_steps, sound_buffer_size} = this.options;

    // Initialize the state of the motor with a copy of the start state.
    this.state = {...start_state};
    // Store data in a circular buffer because copying this array does actually slow down the simulation.
    this.history_buffer = new CircularBuffer(max_stored_steps);
    // Store sound data in a circular buffer.
    this.sound_buffer = new CircularBuffer(sound_buffer_size);
    // Start the simulation at step 0.
    this.step = 0;

    // Initialize the outputs and inputs and other derivatives, but do not advance state.
    this.compute_next_state();

    // Compute runtime statistics.
    this.stats = new TimingStats();
    // Simulation slowdown compared to wall clock time.
    this.slowdown = 1.0;
  }

  get history() {
    return this.history_buffer.toarray();
  }

  get sounds() {
    return this.sound_buffer.toarray();
  }
  
  /* Advance the simulation by a single step. */
  compute_next_state() {
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
    const {steps_number, store_period, dt, sound_sample_rate} = this.options;

    for (let i = 0; i <steps_number; i++){

      this.state = this.compute_next_state();

      if (this.step % store_period == 0) {
        this.history_buffer.push(this.flattened_state());
      }

      const sound_period = Math.max(1, Math.round(1.0/(sound_sample_rate * dt)));

      if (this.step % sound_period == 0) {
        this.sound_buffer.push(this.flattened_state());
      }

      this.step += 1;
    }

    this.stats.update();
    this.slowdown = 1.0 / (Math.max(0.1, this.stats.fps) * this.options.dt * this.options.steps_number);
  }
}