---
title: Understanding 3-phase motor control
theme: dashboard
---


```js
import {md, note, link} from "./components/utils.js"
import {create_scene, setup_rendering, load_motor, load_texture} from "./components/visuals.js"
import * as THREE from "three";
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
  return (angle + 2 * π) % (2 * π);
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

const initial_parameters = {
  R: 1.3, // phase resistance
  L: 0.0001, // phase inductance
  Kv: 1.0, // motor Kv constant
  τ_static: 0.001, // static friction torque
  τ_dynamic: 0.00001, // dynamic friction torque
  R_bat: 1.0, // battery internal resistance
  R_mosfet: 0.013, // mosfet resistance
  R_shunt: 0.010, // shunt resistance
  V_diode: 0.72, // mosfet reverse diode voltage drop
  I_rotor: 0.00000025, // (Kg*m^2) rotor moment of inertia
  ...hall_bounds(hall_toggle_angle), // hall sensor toggle angle bounds
}

const initial_state = {
  t: 0.0, // simulation time
  φ: 0.0, // motor angle
  ω: 0.0, // motor speed
  Iu: 0.0, // motor current phase U
  Iv: 0.0, // motor current phase V
  Iw: 0.0, // motor current phase W
  Vu: 0.0, // motor voltage at phase U ouput (after mosfet and shunt)
  Vv: 0.0, // motor voltage at phase V
  Vw: 0.0, // motor voltage at phase W
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
  const {R, L, Kv, τ_static, τ_dynamic, R_bat, R_mosfet, R_shunt, V_diode, I_rotor, hall_1_low, hall_1_high, hall_2_low, hall_2_high, hall_3_low, hall_3_high} = parameters;
  const {t, φ, ω, Iu, Iv, Iw, Vu, Vv, Vw} = state;

  const hall_1 = interval_contains_angle(hall_1_low, hall_1_high, φ);
  const hall_2 = interval_contains_angle(hall_2_low, hall_2_high, φ);
  const hall_3 = interval_contains_angle(hall_3_low, hall_3_high, φ);

  return {hall_1, hall_2, hall_3};
}
```

```js
function state_differential(state, parameters, inputs, step_size) {
  const {R, L, Kv, τ_static, τ_dynamic, R_bat, R_mosfet, R_shunt, V_diode, I_rotor} = parameters;
  const {t, φ, ω, Iu, Iv, Iw, Vu, Vv, Vw} = state;
  const {U, V, W, τ_load} = inputs;

  const τ = 0.0; // rotor torque due to motor magnetic field
  const P = τ * ω; // motor power
  const α = 0.0; // motor angular acceleration
  

  const dφ = ω;

  const τ_applied = τ + τ_load;
  const static_sign = (ω == 0.0) ? -Math.sign(τ_applied) : -Math.sign(ω);
  const τ_friction = -ω * τ_dynamic + static_sign * τ_static;
  const τ_total = τ_applied + τ_friction;
  const τ_stopping = -ω/step_size * I_rotor; // Torque needed to stop the rotor in 1 step.
  const frozen = (ω == 0.0) && Math.abs(τ_applied) <= τ_static;
  const stopping = -Math.sign(ω) * τ_total > -Math.sign(ω)*τ_stopping;
  const τ_final = (frozen || stopping) ? τ_stopping : τ_total;

  // We need to prevent static friction from overshooting when the motor is stopped or stopping in 1 step.
  const dω = τ_final / I_rotor;
  const dIu = (Vu - Iu * R - L * α) / L;
  const dIv = (Vv - Iv * R - L * α) / L;
  const dIw = (Vw - Iw * R - L * α) / L;
  const dVu = -Kv * ω * Math.sin(φ) - Iu * R - L * α;
  const dVv = -Kv * ω * Math.sin(φ - 2 * Math.PI / 3) - Iv * R - L * α;
  const dVw = -Kv * ω * Math.sin(φ + 2 * Math.PI / 3) - Iw * R - L * α;
  const dt = 1.0;

  return {
    diff: {t: dt, φ: dφ, ω: dω, Iu: dIu, Iv: dIv, Iw: dIw, Vu: dVu, Vv: dVv, Vw: dVw},
    info: {τ, P, α, τ_applied, τ_friction, τ_total, τ_stopping, frozen, stopping, τ_final},
  };
}



function euler_step(state, parameters, inputs, dt) {
  const dstate = state_differential(state, parameters, inputs, dt).diff;
  return _.mapValues(state, (value, key) => value + dstate[key] * dt);
}

function runge_kuta_step(state, parameters, inputs, dt) {
  const k1 = state_differential(state, parameters, inputs, dt).diff;
  const k2 = state_differential(euler_step(state, parameters, inputs, dt / 2), parameters, inputs, dt / 2).diff;
  const k3 = state_differential(euler_step(state, parameters, inputs, dt / 2), parameters, inputs, dt / 2).diff;
  const k4 = state_differential(euler_step(state, parameters, inputs, dt), parameters, inputs, dt).diff;
  return _.mapValues(state, (value, key) => value + (k1[key] + 2 * k2[key] + 2 * k3[key] + k4[key]) * dt / 6);
}

```

```js
function * simulate(state, parameters, update_inputs, dt, max_stored_steps) {
  let outputs = measurable_outputs(state, parameters);
  let inputs = update_inputs(state, parameters, outputs);
  let states = [{...state, ...outputs, ...inputs}];
  for (let i = 0; true; i++) {
    state = runge_kuta_step(state, parameters, inputs, dt);
    outputs = measurable_outputs(state, parameters);
    inputs = update_inputs(state, parameters, outputs);

    // Normalize the motor angle to [0, 2π]
    state.φ = normalized_angle(state.φ);

    states.push({...state, ...outputs, ...inputs});
    if (states.length > max_stored_steps) {
      states.shift();
    }
    yield states;
  }
}
```

```js
const τ_slider = Inputs.range([-0.005, +0.005], {step: 0.0001, value: 0.0, label: "Load torque"});

display(τ_slider);

function inputs_from_load_torque_slider(state, parameters, outputs) {
  return {
    U: "floating", // driver connection state for phase U
    V: "floating", // driver connection state for phase V
    W: "floating", // driver connection state for phase W
    τ_load: τ_slider.value, // external load torque
  }
}
```


```js
const torque_slider_demo = function * () {
  const scene = create_scene(motor, wood);
  const rendering = setup_rendering(640, 640, invalidation, scene);

  display(rendering.canvas);

  for (const simulation of simulate(initial_state, initial_parameters, inputs_from_load_torque_slider, 0.0001, 500)) {

    // Update the scene with the current state of the simulation.
    const state = _.last(simulation);

    motor.rotor.rotation.y = normalized_angle(state.φ - π / 2);
    motor.red_led.material.emissiveIntensity = state.hall_1 ? 10.0 : 0.0;
    motor.green_led.material.emissiveIntensity = state.hall_2 ? 8.0 : 0.0;
    motor.blue_led.material.emissiveIntensity = state.hall_3 ? 12.0 : 0.0;
    motor.coil_U.material.emissiveIntensity = 0.20 * (Math.sin(state.φ)+1.0)*0.5;
    motor.coil_V.material.emissiveIntensity = 0.20 * (Math.sin(state.φ - 2 * Math.PI / 3)+1.0)*0.5;
    motor.coil_W.material.emissiveIntensity = 0.15 * (Math.sin(state.φ + 2 * Math.PI / 3)+1.0)*0.5;

    rendering.render();

    yield simulation;
  }
}();
```


```js
display(Plot.plot({
  // y: {domain: [-π, 3 * π]},
  marks: [
    Plot.lineY(torque_slider_demo, {x: "t", y: "ω", stroke: "red"}),
    Plot.lineY(torque_slider_demo, {x: "t", y: "φ", stroke: "blue"}),
    Plot.lineY(torque_slider_demo, {x: "t", y: "τ_load"}),
  ]
}));
```




<style>
.tooltip {
  position: relative;
  display: inline-block;
  border-bottom: 1px dotted black;
}

.tooltip .tooltiptext {
  visibility: hidden;
  width: 720px;
  background-color: black;
  color: #fff;
  text-align: center;
  border-radius: 6px;
  padding: 5px 0;
  
  /* Position the tooltip */
  position: absolute;
  z-index: 1;
  top: 100%;
  left: 50%;
  margin-left: -240px;
}

.tooltip:hover .tooltiptext {
  visibility: visible;
}
</style>