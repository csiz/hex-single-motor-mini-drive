---
title: Understanding 3-phase motor control
---


```js
import {md, note, link, sparkline} from "./components/utils.js"
import {create_scene, setup_rendering, load_motor, load_texture} from "./components/visuals.js"
import * as THREE from "three";
import CircularBuffer from "circular-buffer";
```

```js
// Observable framework wants us to load files explicitly from the notebooks.
const model_file_url = await FileAttachment("./data/motor_model.3mf").href;
const wood_texture_url = await FileAttachment("./data/textures/laminate_floor_diff_1k.jpg").href;
const wood_displacement_url = await FileAttachment("./data/textures/laminate_floor_disp_1k.png").href;

const motor = load_motor(model_file_url);
const wood = {
  texture: await load_texture(wood_texture_url, 8, THREE.SRGBColorSpace), 
  displacement: await load_texture(wood_displacement_url, 8),
};
```


```js
const π = Math.PI;

function normalized_angle(angle) {
  return (angle + 3 * π) % (2 * π) - π;
}


function interval_contains_angle(low, high, angle) {
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
function hall_bounds(hall_toggle_angle) {
  return {
    hall_1_low: normalized_angle(2 * π / 6 - hall_toggle_angle),
    hall_1_high: normalized_angle(2 * π / 6 + hall_toggle_angle),
    hall_2_low: normalized_angle(6 * π / 6 - hall_toggle_angle),
    hall_2_high: normalized_angle(6 * π / 6 + hall_toggle_angle),
    hall_3_low: normalized_angle(10 * π / 6 - hall_toggle_angle),
    hall_3_high: normalized_angle(10 * π / 6 + hall_toggle_angle),
  };
}

// Hall sensor toggle angle (with respect to rotor angle).
const hall_toggle_angle = 80 * π / 180;

function RPM_Kv_to_Ke(RPM_Kv){
  return 60.0 / (RPM_Kv * 2.0 * π);
}

const initial_parameters = {
  R_phase: 1.3, // phase resistance
  L_phase: 0.0001, // phase inductance
  Ψ_m: RPM_Kv_to_Ke(1200), // motor Ke constant (also the magnetic flux linkage)
  τ_static: 0.001, // static friction torque
  τ_dynamic: 0.00001, // dynamic friction torque
  R_bat: 1.0, // battery internal resistance
  R_mosfet: 0.013, // mosfet resistance
  R_shunt: 0.010, // shunt resistance
  V_diode: 0.72, // mosfet reverse diode voltage drop
  J_rotor: 0.00000025, // (Kg*m^2) rotor moment of inertia
  M_rotor: 0.0005, // (Kg) rotor mass
  r_max: 0.001, // (m) rotor radial displacement for magnetic field dropoff
  ...hall_bounds(hall_toggle_angle), // hall sensor toggle angle bounds
}

const initial_state = {
  t: 0.0, // simulation time
  φ: 0.0, // motor angle
  ω: 0.0, // motor speed
  Iu: 0.0, // motor current phase U
  Iv: 0.0, // motor current phase V
  Iw: 0.0, // motor current phase W
  𝜈: 0.0, // rotor radial velocity
}


/* Inputs to the physical system for a freewheeling motor.

For this simulator, inputs means everything that affects the system that is independent
of the differential equations that describe the internal state of the system. This includes
the motor controller algorithm and the external load torque.
*/
function freewheeling_inputs(state, parameters, outputs) {
  return {
    U: "floating", // driver connection state for phase U
    V: "floating", // driver connection state for phase V
    W: "floating", // driver connection state for phase W
    τ_load: 0.0, // external load torque 
  }
}

/* Outputs of the physical system that can be read by the driver sensors. */
function measurable_outputs(state, parameters) {
  const {hall_1_low, hall_1_high, hall_2_low, hall_2_high, hall_3_low, hall_3_high} = parameters;
  const {φ} = state;

  const hall_1 = interval_contains_angle(hall_1_low, hall_1_high, φ);
  const hall_2 = interval_contains_angle(hall_2_low, hall_2_high, φ);
  const hall_3 = interval_contains_angle(hall_3_low, hall_3_high, φ);

  return {hall_1, hall_2, hall_3};
}
```

```js
function compute_state(state, parameters, inputs, dt) {
  const {R_phase, L_phase, Ψ_m, τ_static, τ_dynamic, R_bat, R_mosfet, R_shunt, V_diode, J_rotor, M_rotor, r_max} = parameters;
  const {t, φ, ω, Iu, Iv, Iw, 𝜈} = state;
  const {U, V, W, τ_load} = inputs;

  const dφ = ω;

  const V_Ru = (R_phase + R_shunt) * Iu;
  const V_Rv = (R_phase + R_shunt) * Iv;
  const V_Rw = (R_phase + R_shunt) * Iw;

  const Vu_rotational_emf = Ψ_m * ω * Math.sin(φ);
  const Vv_rotational_emf = Ψ_m * ω * Math.sin(φ - 2 * Math.PI / 3);
  const Vw_rotational_emf = Ψ_m * ω * Math.sin(φ + 2 * Math.PI / 3);

  const Vu_radial_emf = Ψ_m * 𝜈 / r_max * Math.cos(φ);
  const Vv_radial_emf = Ψ_m * 𝜈 / r_max * Math.cos(φ - 2 * Math.PI / 3)
  const Vw_radial_emf = Ψ_m * 𝜈 / r_max * Math.cos(φ + 2 * Math.PI / 3);

  const dIu = 1.0 / L_phase * (-V_Ru + Vu_rotational_emf);
  const dIv = 1.0 / L_phase * (-V_Rv + Vv_rotational_emf);
  const dIw = 1.0 / L_phase * (-V_Rw + Vw_rotational_emf);

  // Calculate rotor torque due to motor magnetic field.
  const τ = - (
    Ψ_m * Iu * Math.sin(φ) +
    Ψ_m * Iv * Math.sin(φ - 2 * Math.PI / 3) +
    Ψ_m * Iw * Math.sin(φ + 2 * Math.PI / 3)
  );


  const τ_applied = τ + τ_load;
  const static_sign = (ω == 0.0) ? -Math.sign(τ_applied) : -Math.sign(ω);
  const τ_friction = -ω * τ_dynamic + static_sign * τ_static;
  const τ_total = τ_applied + τ_friction;
  const τ_stopping = -ω/dt * J_rotor; // Torque needed to stop the rotor in 1 step.
  const frozen = (ω == 0.0) && Math.abs(τ_applied) <= τ_static;
  const stopping = -Math.sign(ω) * τ_total > -Math.sign(ω)*τ_stopping;
  const τ_final = (frozen || stopping) ? τ_stopping : τ_total;

  // We need to prevent static friction from overshooting when the motor is stopped or stopping in 1 step.
  const dω = τ_final / J_rotor;

  const d𝜈 = 0.0;

  return {
    diff: {t: 1.0, φ: dφ, ω: dω, Iu: dIu, Iv: dIv, Iw: dIw, 𝜈: d𝜈},
    info: {
      τ, τ_applied, τ_friction, τ_total, τ_stopping, frozen, stopping, τ_final,
      V_Ru, V_Rv, V_Rw, 
      Vu_rotational_emf, Vv_rotational_emf, Vw_rotational_emf,
      Vu_radial_emf, Vv_radial_emf, Vw_radial_emf,  
    },
  };
}

function postprocess_state(state) {
  const {φ} = state;
  return {...state, φ: normalized_angle(φ)};
}


function euler_step(state, diff, dt) {
  return _.mapValues(state, (value, key) => value + diff[key] * dt);
}

function runge_kuta_step(state, diff, dt, parameters, inputs) {
  const k1 = diff;
  const k2 = compute_state(euler_step(state, k1, dt / 2), parameters, inputs, dt / 2).diff;
  const k3 = compute_state(euler_step(state, k2, dt / 2), parameters, inputs, dt / 2).diff;
  const k4 = compute_state(euler_step(state, k3, dt), parameters, inputs, dt).diff;

  return _.mapValues(state, (value, key) => value + (k1[key] + 2 * k2[key] + 2 * k3[key] + k4[key]) * dt / 6);
}

```

```js
function * simulate({
  start_state=null, 
  parameters=null, 
  update_inputs=null, 
  update_memory=null, 
  dt=null,
  state_update=runge_kuta_step,
  max_stored_steps=1000,
  store_every_n_steps=1000,
  yield_every_n_steps=10000,
} = {}) {
  
  let state = {...start_state};
  
  let states = new CircularBuffer(max_stored_steps);

  for (let step = 0; true; step++) {
    let outputs = measurable_outputs(state, parameters);
    let inputs = update_inputs(state, parameters, outputs, update_memory);
    let {diff, info} = compute_state(state, parameters, inputs, dt);

    const full_state = {step, ...state, ...outputs, ...inputs, ...info};

    state = postprocess_state(state_update(state, diff, dt, parameters, inputs));

    if (step % store_every_n_steps == 0) states.push(full_state);

    if (step % yield_every_n_steps == 0) yield states.toarray();
  }
}
```

```js
const load_torque_slider = Inputs.range([-0.05, +0.05], {step: 0.001, value: 0.0, label: "Load Torque"});

function inputs_for_main_example(state, parameters, outputs, update_memory) {
  return {
    U: "floating", // driver connection state for phase U
    V: "floating", // driver connection state for phase V
    W: "floating", // driver connection state for phase W
    τ_load: load_torque_slider.value, // external load torque
  }
}

const reset_inputs = Inputs.button("Reset inputs", {
  reduce: () => {
    load_torque_slider.value = 0;
  },
  label: "Set Freewheeling"}
);
```


```js
const descriptions = new Map([
  ["<no selection>", html`
    We simulate a 3 phase motor with 1 winding per phase and 2 pole permanent magnet rotor
    (to simplify the illustration).`],
  [motor.stator, html`
    The stator is the stationary part of the motor. The coils wounded around the stator slots
    generate a magnetic field when current flows through them. The stator is made of magnetic
    material to guide the magnetic field lines between the stator poles. The hall sensors are
    also fixed to the stator, usually on a small circuit board, here they magically float.`],
  [motor.rotor, html`
    The rotor is the rotating part of the motor. The rotor contains permanent magnets that
    interact with the magnetic field generated by the stator. The rotor is mounted on a shaft
    that is supported by bearings.`],
  [motor.coil_U, html`
    The U phase coil generates a magnetic field when current flows through it. Driving the motor.`],
  [motor.coil_V, html`
    The V phase coil generates a magnetic field when current flows through it. Driving the motor.`],
  [motor.coil_W, html`
    The W phase coil generates a magnetic field when current flows through it. Driving the motor.`],
  [motor.hall_1, html`
    The hall sensor 1 is placed between the U and V phase poles. It senses the north magnetic field.`],
  [motor.hall_2, html`
    The hall sensor 2 is placed between the V and W phase poles. It senses the north magnetic field.`],
  [motor.hall_3, html`
    The hall sensor 3 is placed between the W and U phase poles. It senses the north magnetic field.`],
]);


const highlight_description = Mutable(descriptions.get("<no selection>"));

function get_object_with_description(intersected_objects) {
  if (intersected_objects.length == 0) return [];
  let object = intersected_objects[0];
  while (!descriptions.has(object)) {
    if (object.parent == null) return [];
    object = object.parent;
  }
  return [object];
}

function get_description(objects) {
  return objects.length > 0 ? descriptions.get(objects[0]) : descriptions.get("<no selection>");
}

function change_if_different(mutable, value) {
  if (mutable.value != value) mutable.value = value;
}

function show_description(objects) {
  change_if_different(highlight_description, get_description(objects));
}

const show_description_slowly = _.throttle(show_description, 500, {leading: false});
```


```js
const scene = create_scene(motor, wood);
const rendering = setup_rendering(640, 640, invalidation, scene, get_object_with_description, show_description_slowly);
const reset_camera = Inputs.button("Look straight at motor", {reduce: () => rendering.reset_camera(), label: "Reset Camera"});

const torque_slider_demo = function * () {
  const simulation_frequency = 72_000_000; // 72 MHz
  const sim_tick = 1.0 / simulation_frequency;

  const simulation_options = {
    start_state: initial_state,
    parameters: initial_parameters,
    update_inputs: inputs_for_main_example,
    update_memory: {},
    dt: sim_tick,
  };

  for (const simulation_history of simulate(simulation_options)) {

    // Update the scene with the current state of the simulation.
    const state = _.last(simulation_history);

    motor.rotor.rotation.y = normalized_angle(state.φ - π / 2);
    motor.red_led.material.emissiveIntensity = state.hall_1 ? 10.0 : 0.0;
    motor.green_led.material.emissiveIntensity = state.hall_2 ? 8.0 : 0.0;
    motor.blue_led.material.emissiveIntensity = state.hall_3 ? 12.0 : 0.0;
    motor.coil_U.material.emissiveIntensity = 0.20 * Math.abs(state.Iu / 6.0);
    motor.coil_V.material.emissiveIntensity = 0.20 * Math.abs(state.Iv / 6.0);
    motor.coil_W.material.emissiveIntensity = 0.15 * Math.abs(state.Iw / 6.0);

    rendering.render();

    yield simulation_history;
  }
}();
```

<div class="grid" style="grid-template-columns: 1fr 1fr;">
  <div class="card">
    <figure>${rendering.div}</figure>
    <p>${highlight_description}</p>
  </div>
  <div class="card">
    <div class="card tight">
      <div>${reset_camera}</div>
      <div>${reset_inputs}</div>
      <div>${load_torque_slider}</div>
    </div>
    <div class="card tight" style="display: flex; align-items: center;">
      <label style="min-width: 120px; margin-right: 6.5px;">Time</label>
      <div>${
        Plot.plot({
          marks: [
            Plot.tickX(_.filter(torque_slider_demo, ({step}) => (step % 500) == 0), {x: "t"})
          ]
        })
      }
      </div>
    </div>
    ${sparkline(torque_slider_demo, {label: "Motor Speed", y: "ω"})}
    ${sparkline(torque_slider_demo, {label: "Motor Angle", y: "φ"}, {domain: [-π, π]})}
    ${sparkline(torque_slider_demo, {label: "Load Torque", y: "τ_load"})}
    ${sparkline(
      torque_slider_demo, [
        {label: "Current Iu", y: "Iu", stroke: "#a0a"},
        {label: "Current Iv", y: "Iv", stroke: "#aa0"},
        {label: "Current Iw", y: "Iw", stroke: "#0aa"},
      ], 
      {domain: [-4.0, 4.0], height: 120},
    )}
    ${sparkline(
      torque_slider_demo, [
        {label: "EMF Vu", y: "Vu_rotational_emf", stroke: "#a0a"},
        {label: "EMF Vv", y: "Vv_rotational_emf", stroke: "#aa0"},
        {label: "EMF Vw", y: "Vw_rotational_emf", stroke: "#0aa"},
      ], 
      {domain: [-4.0, 4.0], height: 120},
    )}
  </div>
</div>


