---
title: Understanding 3-phase motor control
theme: dashboard
---


```js
import {md, note, link} from "./components/utils.js"
import {create_scene, render_scene, load_motor, load_texture} from "./components/visuals.js"
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

/* Outputs of the physical system (effects).

For this simulator, outputs means all the descriptors of the system that are dependent only on the
current internal state of the system. Notably they don't depend directly on time and therefore don't
need to be included in the ODE solver. This includes the motor torque, power, angular acceleration,
and the hall sensor states.
*/
function dependent_outputs(state, parameters) {
  const {R, L, Kv, τ_static, τ_dynamic, R_bat, R_mosfet, R_shunt, V_diode, I_rotor, hall_1_low, hall_1_high, hall_2_low, hall_2_high, hall_3_low, hall_3_high} = parameters;
  const {t, φ, ω, Iu, Iv, Iw, Vu, Vv, Vw} = state;

  const hall_1 = interval_contains_angle(hall_1_low, hall_1_high, φ);
  const hall_2 = interval_contains_angle(hall_2_low, hall_2_high, φ);
  const hall_3 = interval_contains_angle(hall_3_low, hall_3_high, φ);

  const τ = 0.0; // rotor torque due to motor magnetic field
  const P = τ * ω; // motor power
  const α = 0.0; // motor angular acceleration


  return {hall_1, hall_2, hall_3, τ, P, α};

}
```

```js
function state_differential(state, parameters, inputs, step_size) {
  const {R, L, Kv, τ_static, τ_dynamic, R_bat, R_mosfet, R_shunt, V_diode, I_rotor} = parameters;
  const {t, φ, ω, Iu, Iv, Iw, Vu, Vv, Vw} = state;
  const {U, V, W, τ_load} = inputs;
  const {τ, α} = dependent_outputs(state, parameters);

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

  return {t: dt, φ: dφ, ω: dω, Iu: dIu, Iv: dIv, Iw: dIw, Vu: dVu, Vv: dVv, Vw: dVw};
}



function euler_step(state, parameters, inputs, dt) {
  const dstate = state_differential(state, parameters, inputs, dt);
  return _.mapValues(state, (value, key) => value + dstate[key] * dt);
}

function runge_kuta_step(state, parameters, inputs, dt) {
  const k1 = state_differential(state, parameters, inputs, dt);
  const k2 = state_differential(euler_step(state, parameters, inputs, dt / 2), parameters, inputs, dt / 2);
  const k3 = state_differential(euler_step(state, parameters, inputs, dt / 2), parameters, inputs, dt / 2);
  const k4 = state_differential(euler_step(state, parameters, inputs, dt), parameters, inputs, dt);
  return _.mapValues(state, (value, key) => value + (k1[key] + 2 * k2[key] + 2 * k3[key] + k4[key]) * dt / 6);
}

```

```js
function* simulate(state, parameters, update_inputs, dt, max_stored_steps) {
  let outputs = dependent_outputs(state, parameters);
  let inputs = update_inputs(state, parameters, outputs);
  let states = [{...state, ...outputs, ...inputs}];
  for (let i = 0; true; i++) {
    state = runge_kuta_step(state, parameters, inputs, dt);
    outputs = dependent_outputs(state, parameters);
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
const scene = create_scene(motor, wood);
const rendering_element = render_scene(640, 640, invalidation, scene, (outline_pass) => {
  // outline_pass.selectedObjects = [motor.coil_U, motor.coil_V, motor.coil_W];
});
```

```js
const τ_slider = Inputs.range([-0.005, +0.005], {step: 0.0001, value: 0.0, label: "Load torque"});

display(τ_slider);

function inputs_from_load_torque_slider(state, parameters, outputs) {
  motor.rotor.rotation.y = normalized_angle(state.φ - π / 2);
  motor.red_led.material.emissiveIntensity = outputs.hall_1 ? 10.0 : 0.0;
  motor.green_led.material.emissiveIntensity = outputs.hall_2 ? 8.0 : 0.0;
  motor.blue_led.material.emissiveIntensity = outputs.hall_3 ? 12.0 : 0.0;
  motor.coil_U.material.emissiveIntensity = 0.20 * (Math.sin(state.φ)+1.0)*0.5;
  motor.coil_V.material.emissiveIntensity = 0.20 * (Math.sin(state.φ - 2 * Math.PI / 3)+1.0)*0.5;
  motor.coil_W.material.emissiveIntensity = 0.15 * (Math.sin(state.φ + 2 * Math.PI / 3)+1.0)*0.5;

  return {
    U: "floating", // driver connection state for phase U
    V: "floating", // driver connection state for phase V
    W: "floating", // driver connection state for phase W
    τ_load: τ_slider.value, // external load torque
  }
}


let slider_simulation = simulate(initial_state, initial_parameters, inputs_from_load_torque_slider, 0.0001, 500);
```

```js
display(rendering_element);
```



```js
display(Plot.plot({
  // y: {domain: [-π, 3 * π]},
  marks: [
    Plot.lineY(slider_simulation, {x: "t", y: "ω", stroke: "red"}),
    Plot.lineY(slider_simulation, {x: "t", y: "φ", stroke: "blue"}),
    Plot.lineY(slider_simulation, {x: "t", y: "τ_load"}),
  ]
}));
```


```js
const phi = d3.ticks(0, 2 * Math.PI, 1000);
const phases = phi.map(t => {
  const u = Math.sin(t) * 0.5 + 0.5;
  const v = Math.sin(t + 2 * Math.PI / 3) * 0.5 + 0.5;
  const w = Math.sin(t + 4 * Math.PI / 3) * 0.5 + 0.5;

  const max = Math.max(u, v, w);
  const min = Math.min(u, v, w);
  const max_adj = max;
  const min_adj = min;
  const adj = - min_adj;

  const adj_u = u + adj;
  const adj_v = v + adj;
  const adj_w = w + adj;
  return {u, v, w, adj: -adj, max, min, max_adj, min_adj, adj_u, adj_v, adj_w};
});


display(Plot.plot({
  y: {domain: [0, 1.0]},
  marks: [
    Plot.lineY(phases, {x: phi, y: "u", stroke: 'red', label: 'U'}),
    Plot.lineY(phases, {x: phi, y: "v", stroke: 'green', label: 'V'}),
    Plot.lineY(phases, {x: phi, y: "w", stroke: 'blue', label: 'W'}),
    Plot.lineY(phases, {x: phi, y: "max_adj", stroke: 'black', label: 'Max', strokeDasharray: '5 10', strokeWidth: 3}),
    Plot.lineY(phases, {x: phi, y: "min_adj", stroke: 'black', label: 'Min', strokeDasharray: '5 10', strokeWidth: 3}),

    Plot.gridX({interval: Math.PI / 3, stroke: 'black', strokeWidth : 2}),
    Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 2}),
  ]
}))

display(Plot.plot({
  y: {domain: [0, 1.0]},
  marks: [
    Plot.lineY(phases, {x: phi, y: "adj", stroke: 'black', label: 'Adjustment'}),
    Plot.lineY(phases, {x: phi, y: "adj_u", stroke: 'red', label: 'U'}),
    Plot.lineY(phases, {x: phi, y: "adj_v", stroke: 'green', label: 'V'}),
    Plot.lineY(phases, {x: phi, y: "adj_w", stroke: 'blue', label: 'W'}),
    Plot.gridX({interval: Math.PI / 3, stroke: 'black', strokeWidth : 2}),
    Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 2}),
  ]
}))

```




Motor model parameters
----------------------

* [ ] rotor inertial mass
* [ ] phase inductance L
* [ ] phase resistance R
* [ ] motor Kv constant
* [ ] hysterisis angle alpha ${note(html`
	Apparently the phase delay between the motor flux and stator flux is constant and
	depends on the hysterisis loop characteristic of the rotor construction.
	${link("https://www.ijerd.com/paper/vol12-issue5/Version-1/K1257683.pdf")}
`)}
* [ ] hysterisis friction torque
	the hysterisis torque is also constant
* [ ] static friction torque (fixed torque at standstill)
* [ ] dynamic friction torque (fixed torque when rotating)
* [ ] resistance friction torque (this one is proportional to rotation speed)
* [ ] battery internal resistance
* [ ] mosfet resistance
* [ ] mosfet reverse diode voltage drop
* [ ] hall sensor toggle angle (the hall sensor senses positive magnetic field, so it
toggles at almost 90 degrees difference from the rotor angle; the field is null at 90 degrees).
* [ ] rotor mass (so we can measure axial deflection when motor is )
* [ ] rotor axial restoration force constant



Measured/datasheet characteristics:
* Rotor moment of inertia for 10g shell of 5mm radius: 0.00000025 Kg*m^2.
* Forward diode voltage Vds = 0.72V (up to 1V); body-diode can withstand 4A continuous current.
* Continuous drain current 8.5A (at high ambient temperature 70C).
* Drain source ON resistance 18mΩ.
* Phase winding resistance 1.3Ω (2.6Ω across 2 phases).
* Shunt resistance 10mΩ.
* Driver turn on propagation delay ~300ns.
* Driver turn off propagation delay ~100ns.
* Driver automatic deadtime ~200ns.
* Phase inductance 0.1mH (0.2mH across 2 phases).

References:
* https://www.controleng.com/articles/understanding-the-effect-of-pwm-when-controlling-a-brushless-dc-motor/
* https://www.youtube.com/watch?v=EHYEQM1sA3o&list=PLaBr_WzeIAixidGwqfcrQlwKZX4RZ2E7D&pp=iAQB



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