---
title: Understanding 3 Phase Motors
---

<div class="grid" style="grid-template-columns: 1fr 1fr;">
  <div class="card">
    <figure>${live_simulation}</figure>
    <div class="card tight">${simulation_info}</div>
    <div class="card tight">${simulation_interface}</div>
    <div class="card tight">${simulation_plots_interface}</div>
  </div>
  <div class="card">
    <div class="card tight" style="min-height: 5rem;">${highlight_description}</div>
    <div class="card tight">${simulation_plots}</div>
  </div>
</div>

<div class="card">

Test Motor Characteristics
--------------------------


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

References
----------

* Understanding the effect of PWM when controlling a brushless dc motor: https://www.controleng.com/articles/understanding-the-effect-of-pwm-when-controlling-a-brushless-dc-motor/
* Jantzen Lee - The Physics behind how motors work. Understanding motors (Episode 1): https://www.youtube.com/watch?v=EHYEQM1sA3o&list=PLaBr_WzeIAixidGwqfcrQlwKZX4RZ2E7D&pp=iAQB
* Texas Instruments - Field Oriented Control of Permanent Magnet Motors: https://www.youtube.com/watch?v=cdiZUszYLiA
* Clarke transformation: https://en.wikipedia.org/wiki/Alpha%E2%80%93beta_transformation
* Direct-quadrature-zero transformation: https://en.wikipedia.org/wiki/Direct-quadrature-zero_transformation
* Coordinate Transform in Motor Control (shows the math workings to get the unitary Clarke transform): https://www.infineon.com/dgdl/Infineon-AN205345_Coordinate_Transform_in_Motor_Control-ApplicationNotes-v03_00-EN.pdf?fileId=8ac78c8c7cdc391c017d0cff06bb58ed
* 3-Phase PM Synchronous Motor Torque Vector Control Using 56F805: https://www.nxp.com/docs/en/reference-manual/DRM018.pdf
* PMSM Permanent magnet synchronous motor with sinusoidal flux distribution: https://uk.mathworks.com/help/sps/ref/pmsm.html
* Electromotive forceL https://en.wikipedia.org/wiki/Electromotive_force
* Counter-electromotive force: https://en.wikipedia.org/wiki/Counter-electromotive_force
* Magnetic flux https://en.wikipedia.org/wiki/Magnetic_flux
* Magnetic flux density (magnetic field): https://en.wikipedia.org/wiki/Magnetic_field
* Magnetic moment https://en.wikipedia.org/wiki/Magnetic_moment
* Faraday induction law: https://en.wikipedia.org/wiki/Faraday%27s_law_of_induction
* Solenoid: https://en.wikipedia.org/wiki/Solenoid

</div>

```js
import * as THREE from "three";
import {formatSI} from "format-si-prefix";
import tex from "npm:@observablehq/tex";
import * as Plot from "@observablehq/plot";
import _ from "lodash";

import {select} from "d3-selection";

import {html} from "htl";
const htf = html.fragment;

import {note, link} from "./components/utils.js"
import {Simulation, initial_state, initial_parameters, π, normalized_angle, phase_switches_values, default_simulation_options} from "./components/simulation.js"
import {sixstep_commutation} from "./components/driver_algorithms.js"
import {create_scene, setup_rendering} from "./components/visuals.js"


const wave_sample_size = default_simulation_options.wave_buffer_size;
const speed_of_sound = 343.0; // m/s

// Configure 3D visuals
// --------------------

const {motor, scene, sound_wave, sound_wave_update} = await create_scene({wave_sample_size});


function get_emissive_color(coil) {
  const hsl_color = {};
  coil.material.emissive.getHSL(hsl_color);
  const {h, s, l} = hsl_color;

  return new THREE.Color().setHSL(h, 1.0, 0.3).getStyle();
}

const color_u = get_emissive_color(motor.coil_U);
const color_v = get_emissive_color(motor.coil_V);
const color_w = get_emissive_color(motor.coil_W);
const color_uv = get_emissive_color(motor.red_led);
const color_vw = get_emissive_color(motor.green_led);
const color_wu = get_emissive_color(motor.blue_led);
const color_x = "steelblue";
const color_y = "lightcoral";

function get_sound_normalizer(max_sound) {
  const max_log_sound = Math.log1p(max_sound);
  
  return function(x) {
    return Math.min(Math.log1p(Math.max(x, 0.0)) / max_log_sound, 1.0);
  };
}

function update_motor_visuals(simulation, {displacement_emphasis, max_sound, visuals_selection}) {
  const visuals_set = new Set(visuals_selection);

  const {φ, I_u, I_v, I_w, hall_1, hall_2, hall_3, rx, ry} = simulation.flattened_state();
  const scale = 1000.0; // Model coordinates is in mm so we have 1000mm per meter.
  
  // Set rotor angle to match the electrical angle in our 1 pole per phase simulation.
  motor.rotor.rotation.y = normalized_angle(φ - π / 2);
  
  // Emphasise radial displacement by scaling it up; 1um shows as 1mm in the scene.
  const radial_scaling = displacement_emphasis * scale;
  const scaled_rx = radial_scaling * rx;
  const scaled_ry = radial_scaling * ry;
  
  const show_displacement = visuals_set.has("Displacement");
  motor.rotor.position.z = show_displacement ? scaled_rx : 0.0;
  motor.rotor.position.x = show_displacement ? scaled_ry : 0.0;

  // Set the hall colours based on the hall sensor states. The different colours appear
  // at different intensities, adjusted them slightly to make them seem even (to myself).
  motor.red_led.material.emissiveIntensity = hall_1 ? 10.0 : 0.0;
  motor.green_led.material.emissiveIntensity = hall_2 ? 8.0 : 0.0;
  motor.blue_led.material.emissiveIntensity = hall_3 ? 12.0 : 0.0;

  motor.coil_U.material.emissiveIntensity = 0.20 * Math.abs(I_u / 6.0);
  motor.coil_V.material.emissiveIntensity = 0.20 * Math.abs(I_v / 6.0);
  motor.coil_W.material.emissiveIntensity = 0.15 * Math.abs(I_w / 6.0);

  const show_visual_sound = visuals_set.has("Visual Sound");
  if (show_visual_sound) {
    sound_wave.visible = true;
    
    const sound_distance = speed_of_sound * displacement_emphasis * simulation.options.wave_buffer_size / simulation.options.wave_sample_rate;
    const normalize_sound = get_sound_normalizer(max_sound);

    sound_wave_update(
      simulation.waves.map(({rv, φ_rv, rx, ry}) => ({x: scaled_rx, y: scaled_ry, v: normalize_sound(rv), φ: φ_rv})),
      sound_distance);
  } else {
    sound_wave.visible = false;
  }
}

/* Descriptions of the 3D model components for mouseover tooltips. */
const descriptions = new Map([
  ["<no selection>", html`
    We simulate a 3 phase motor with 1 winding per phase and 2 pole permanent magnet rotor
    (to simplify the illustration). However around the model to find out more.`],
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

/* Get the most specific object group with a description from those intersected by the mouse cursor. */
const show_object_with_description = _.throttle((intersected_objects) => {
  if (intersected_objects.length == 0) return [];
  let object = intersected_objects[0];
  // Walk the object hierarchy to find the first object with a description.
  while (!descriptions.has(object)) {
    // We reached the top without finding a description, don't highlight anything.
    if (object.parent == null) {
      highlight_description.value = descriptions.get("<no selection>");
      return [];
    }
    object = object.parent;
  }

  highlight_description.value = descriptions.get(object);
  return [object];
}, 100);


// Simulation information
// ----------------------

let simulation_info = Mutable();

const show_simulation_info = _.throttle((simulation, rendering) => {
  const {fps} = rendering.stats;
  const {slowdown, history_buffer, sound_buffer} = simulation;
  const {dt, store_period, steps_number, sound_sample_rate, max_stored_steps} = simulation.options;

  const plot_duration = dt * store_period * max_stored_steps;

  simulation_info.value = htf`<span style="display: grid; grid-template-columns: 1fr 1fr 1fr 1fr">
    <span>FPS: ${fps.toFixed(1)}</span>
    <span>Slowdown: ${formatSI(slowdown)}</span>
    <span>Speed: ${formatSI(1.0 / dt)}Hz</span>
    <span>${tex`\delta t`}: ${formatSI(dt)}s</span>

    <span>Plot window: ${formatSI(plot_duration)}s</span>
    <span>Window freq: ${formatSI(1.0 / plot_duration)}Hz</span>
    <span>Buffer: ${formatSI(history_buffer.size() * store_period * dt)}s</span>
    <span>🎵 buffer: ${formatSI(sound_buffer.size() / sound_sample_rate)}s</span>
  </span>`;
}, 100);



// Live simulation controls
// ------------------------

/* Use passive buttons for the controls to let the update loop pull their value. */
function consume_button(controls, task_map, if_consumed=()=>{}){
  if(controls.value) {
    controls.value = task_map[controls.value]();
    if_consumed();
  }
}


const simulation_controls = Inputs.button(
  [
    ["Play", () => "Play"], 
    ["Pause", () => "Pause"],
    ["Step", () => "Step"],
    ["Restart", () => "Restart"],
  ],
  {label: "Simulation controls", value: "Play"}
);

const audio_video_controls = Inputs.button(
  [
    ["Look at motor", () => "Look at motor"],
    ["Reset inputs", () => "Reset inputs"],
    ["Play 🎵", () => "Play 🎵"],
  ],
  {label: "Controls"}
);

const timing_sliders = Inputs.form({
  frequency: Inputs.range([1_125_000, 72_000_000], {
    transform: Math.log, 
    format: x => x.toFixed(0), 
    label: "Frequency (Hz)",
    value: 12_000_000,
  }),
  steps_number: Inputs.range([1, 10_000], {
    transform: Math.log, 
    format: x => x.toFixed(0), 
    label: "Steps per frame",
    value: 2500,
  }),
  store_period: Inputs.range([1, 10_000], {
    transform: Math.log, 
    format: x => x.toFixed(0), 
    label: "Store period",
    value: 250,
  }),
});

const timing_sliders_defaults = timing_sliders.value;


const load_mode_map = {
  "↻ Move +": (load_torque) => +load_torque,
  "↺ Move -": (load_torque) => -load_torque,
  "Towards Angle": (load_torque, target_angle, φ) => load_torque * normalized_angle(target_angle / 180 * π - φ) / (π / 2),
  "Unloaded": () => 0.0,
};

const driver_mode_map = {
  "Disconnect Battery": ()=>[0, 0, 0],
  "Connect and Idle": ()=>[0, 0, 0],
  "Break Lo": ()=>[2, 2, 2],
  "Break Hi": ()=>[1, 1, 1],
  "↺ Drive -": (state, parameters, outputs, update_memory)=>sixstep_commutation(outputs, false),
  "↻ Drive +": (state, parameters, outputs, update_memory)=>sixstep_commutation(outputs, true),
  "⇓ Drive U +": ()=>[1, 2, 2],
  "⇖ Drive V +": ()=>[2, 1, 2],
  "⇗ Drive W +": ()=>[2, 2, 1],
  "⇑ Drive U -": ()=>[2, 1, 1],
  "⇘ Drive V -": ()=>[1, 2, 1],
  "⇙ Drive W -": ()=>[1, 1, 2],
  "↘ Drive UV +": ()=>[1, 2, 0],
  "← Drive VW +": ()=>[0, 1, 2],
  "↗ Drive WU +": ()=>[2, 0, 1],
  "↖ Drive UV -": ()=>[2, 1, 0],
  "→ Drive VW -": ()=>[0, 2, 1],
  "↙ Drive WU -": ()=>[1, 0, 2],
};

const visuals_selection_list = [
  "Displacement",
  "Visual Sound",
];

const default_visuals_selection = visuals_selection_list;

const interactive_sliders = Inputs.form({
  displacement_emphasis: Inputs.range([1.0, 10_000.0], {
    format: x => x.toFixed(0),
    label: "Displacement Exaggeration",
    transform: Math.log,
    value: 100.0,
  }),
  max_sound: Inputs.range([0.010, 10.0], {
    format: x => x.toFixed(2),
    label: "Visual Sound Base",
    transform: Math.log,
    value: 1.0, // 1m/s velocity of the rotor sound sourc will blow up the visuals
  }),
  visuals_selection: Inputs.checkbox(visuals_selection_list, {
    label: "Show visuals",
    value: default_visuals_selection,
  }),
  load_torque: Inputs.range([0.000_1, 0.02], {
    transform: Math.log,
    format: x => x.toFixed(4),
    label: "Load Torque",
    value: 0.01,
  }),
  target_angle: Inputs.range([0, +360], {
    format: x => x.toFixed(0),
    label: "Target Angle",
    value: 0.0,
  }),
  load_mode: Inputs.radio(Object.keys(load_mode_map), {
    label: "Load Torque Mode", 
    value: "Unloaded"
  }),
  driver_mode: Inputs.radio(Object.keys(driver_mode_map), {
    label: "Driver control", 
    value: "↻ Drive +",
  }),
});

const interactive_sliders_defaults = interactive_sliders.value;


const simulation_interface = Inputs.form([
  simulation_controls,
  audio_video_controls,
  timing_sliders,
  interactive_sliders,
]);


// Plotting options
// ----------------

const plot_selection_map = {
  "Rotational Frequency": [
    {label: `freq`, y: (item) => item.ω / (2 * π)}, 
    {least_domain: [-1_000, 1_000], units: "Hz"}
  ],
  "Motor Speed": [
    {label: `RPM`, y: "rpm"}, 
    {least_domain: [-6_000, 6_000]}
  ],
  "Motor Angle": [
    {label: "φ", y: "φ"}, 
    {least_domain: [-π, π], units: "rad"}
  ],
  "Torque": [
    [
      {label: `τ_{total}`, y: "τ_total"},
      {label: `τ_{emf}`, y: "τ_emf", stroke: color_x},
      {label: `τ_{friction}`, y: "τ_friction", stroke: color_y},
    ],
    {least_domain: [-0.010, +0.010], units: "Nm"},
  ],
  "Battery Current": [
    {label: "I_{bat}", y: "I"}, 
    {least_domain: [-2.0, +2.0], units: "A"}
  ],
  "Capacitor Voltage": [
    {label: "V_{near}", y: "V"},
    {least_domain: [0.0, +12.0], units: "V", symmetric_domain: false}
  ],
  "Radial Displacement Angle": [
    {label: "φ_{r}", y: "φ_r"},
    {least_domain: [-π, π], units: "rad"}
  ],
  "Radial Displacement": [
    {label: "r", y: "r"}, 
    {least_domain: [0.0, +0.000_005], units: "m", symmetric_domain: false},
  ],
  "Radial Velocity Angle": [
    {label: "φ_{rv}", y: "φ_rv"},
    {least_domain: [-π, π], units: "rad"}
  ],
  "Radial Velocity": [
    {label: `ṙ`, y: "rv"},
    {least_domain: [-0.010, +0.010], units: "m/s"},
  ],
  "Radial Acceleration": [
    {label: `r̈`, y: "drv"},
    {least_domain: [-1_000, +1_000], units: "m/s²"},
  ],
  "Currents": [
    [
      {label: "I_U", y: "I_u", stroke: color_u},
      {label: "I_V", y: "I_v", stroke: color_v},
      {label: "I_W", y: "I_w", stroke: color_w},
    ], 
    {least_domain: [-2.0, 2.0], height: 120, units: "A"},
  ],
  "Total EMF": [
    [
      {label: "V_U", y: "V_u_emf", stroke: color_u},
      {label: "V_V", y: "V_v_emf", stroke: color_v},
      {label: "V_W", y: "V_w_emf", stroke: color_w},
    ], 
    {least_domain: [-12.0, 12.0], height: 120, units: "V"},
  ],
  "Rotational EMF": [
    [
      {label: "V_U", y: "V_u_rotational_emf", stroke: color_u},
      {label: "V_V", y: "V_v_rotational_emf", stroke: color_v},
      {label: "V_W", y: "V_w_rotational_emf", stroke: color_w},
    ], 
    {least_domain: [-12.0, 12.0], height: 120, units: "V"},
  ],
  "Radial EMF": [
    [
      {label: "V_U", y: "V_u_radial_emf", stroke: color_u},
      {label: "V_V", y: "V_v_radial_emf", stroke: color_v},
      {label: "V_W", y: "V_w_radial_emf", stroke: color_w},
    ], 
    {least_domain: [-12.0, 12.0], height: 120, units: "V"},
  ],
  "VCC per phase": [
    [
      {label: "V_U", y: "VCC_u", stroke: color_u},
      {label: "V_V", y: "VCC_v", stroke: color_v},
      {label: "V_W", y: "VCC_w", stroke: color_w},
    ], 
    {least_domain: [-12.0, 12.0], height: 120, units: "V"},
  ],
  "Hall sensors": [
    [
      {label: "H1", y: "hall_1", stroke: color_uv},
      {label: "H2", y: "hall_2", stroke: color_vw},
      {label: "H3", y: "hall_3", stroke: color_wu},
    ], 
    {least_domain: [0, 1.0], height: 120, symmetric_domain: false},
  ],
  "Command direction": [
    [
      {label: "U", y: (state) => phase_switches_values[state.U_switch], stroke: color_u},
      {label: "V", y: (state) => phase_switches_values[state.V_switch] , stroke: color_v},
      {label: "W", y: (state) => phase_switches_values[state.W_switch], stroke: color_w},
    ], 
    {least_domain: [-1.0, 1.0], height: 120},
  ],
  "Bridge direction": [
    [
      {label: "U", y: "U_direction", stroke: color_u},
      {label: "V", y: "V_direction", stroke: color_v},
      {label: "W", y: "W_direction", stroke: color_w},
    ], 
    {least_domain: [-1.0, 1.0], height: 120},
  ],
  "MOSFET voltage drop": [
    [
      {label: "V_U", y: "V_Mu", stroke: color_u},
      {label: "V_V", y: "V_Mv", stroke: color_v},
      {label: "V_W", y: "V_Mw", stroke: color_w},
    ], 
    {least_domain: [-1.0, 1.0], height: 120, units: "V"},
  ],
  "Phase to phase inductor voltage": [
    [
      {label: "V_{UV}", y: "V_Luv", stroke: color_u},
      {label: "V_{VW}", y: "V_Lvw", stroke: color_v},
      {label: "V_{WU}", y: "V_Lwu", stroke: color_w},
    ], 
    {least_domain: [-12.0, 12.0], height: 120, units: "V"},
  ],
};

const plot_selection_defaults = [
  "Currents", 
  "Motor Speed", 
  "Motor Angle", 
  "Torque", 
  "Total EMF", 
  "Rotational EMF", 
  "Radial EMF", 
  "Radial Displacement Angle", 
  "Radial Displacement", 
  "Radial Velocity Angle",
  "Radial Velocity",
];


const plot_shown_value_map = {
  "Latest Value": (row) => _.last(row),
  "Root Mean Square": (row) => Math.hypot(...row) / Math.sqrt(row.length),
  "Hide": (row) => undefined,
};

const plot_shown_value_choice = Inputs.radio(Object.keys(plot_shown_value_map), {
  label: "Show numbers", 
  value: "Root Mean Square",
});

const plot_selection = Inputs.checkbox(Object.keys(plot_selection_map), {
  label: "Plots", 
  value: plot_selection_defaults,
});

const plot_preselections = Inputs.button(
  [
    ["Show Default", () => "Show Default"],
    ["Show All", () => "Show All"],
    ["Hide Plots", () => "Hide Plots"],
    ["User Selection", () => "User Selection"],
    ["Reset Selection", () => "Reset Selection"],
  ],
  {label: "Plot selections"},
);


const simulation_plots_interface = Inputs.form([
  plot_preselections,
  plot_selection,
  plot_shown_value_choice,
]);

const simulation_plots_interface_defaults = simulation_plots_interface.value;

let plot_selection_by_user = plot_selection_defaults;

function plot_save_and_switch(new_selection){
  return () => {
    plot_selection_by_user = plot_selection.value;
    plot_selection.value = new_selection;
  };
}

function plot_restore_selection(){
  const old_selection = plot_selection.value;
  plot_selection.value = plot_selection_by_user;
  plot_selection_by_user = old_selection;
}

function plot_reset_selection(){
  plot_selection_by_user = plot_selection.value;
  simulation_plots_interface.value = simulation_plots_interface_defaults;
}

// Simulation plots
// ----------------

const default_sparkline_plot = {label: "<sparkline>", y: [], stroke: "black"};
const default_sparkline_options = {x: "t", least_domain: null, symmetric_domain: true, height: 60, units: "", title: "", shown_value: (row) => undefined};

function sparkline(data, lines={}, options={}){
  const {x, least_domain, symmetric_domain, height, units, shown_value, title} = Object.assign({}, default_sparkline_options, options);
  
  if (!Array.isArray(lines)) lines = [lines];

  lines.map((line) => Object.assign({}, default_sparkline_plot, line));

  const values = lines.map(({y}) => {
    if (typeof y === "string") return data.map((item) => item[y]);
    if (typeof y === "function") return data.map((item) => y(item));
    return y;
  });

  const min_value = Math.min(...values.map(row => Math.min(...row)));
  const max_value = Math.max(...values.map(row => Math.max(...row)));

  const min_domain = least_domain ? Math.min(least_domain[0], min_value) : min_value;
  const max_domain = least_domain ? Math.max(least_domain[1], max_value) : max_value;

  const domain = symmetric_domain ? [
    Math.min(min_domain, -max_domain),
    Math.max(max_domain, -min_domain),
  ] : [
    min_domain,
    max_domain,
  ];

  
  const selected_values = values.map(shown_value);

  const sub_labels = lines.map(({label, stroke}, i) => {
    const v = selected_values[i];
    const formatted_v = v === undefined ?  "" : `${v > 0.0 ? "+" : v < 0.0 ? "-" : ""}${formatSI(Math.abs(v))}${units}`;
    return htf`<div style="color: ${stroke};">${tex`${label}`} <span style="font-family: monospace; float: right;">${formatted_v}</span></div>`
  });
  
  const labels = htf`<div style="display: flex: flex-direction: column; min-width: 150px; margin-right: 6.5px;">
    <div style="font-weight: bold;">${title}</div>
    ${sub_labels}
  </div>`;

  const plot = Plot.plot({
    height,
    axis: null,
    figure: true,
    y: {domain},
    marks:[
      lines.map(({stroke}, i) => Plot.lineY(data, {x, y: values[i], stroke})),
      Plot.ruleY([0], {stroke: "gray", strokeDasharray: "8,2", curve: "linear"}),
    ],
  });

  return htf`<div style="margin: 10px 0px; padding: 5px 0px; box-shadow: 3px 3px 5px rgb(0 0 0 / .2); display: flex; flex-direction: row;">
    ${labels}
    ${plot}
  </div>`;
}

const simulation_plots = Mutable();

const show_simulation_plots = _.throttle((history, selected_plots, shown_value) => {

  simulation_plots.value = selected_plots.map((title) => {
    const [lines, options] = plot_selection_map[title];
    return sparkline(history, lines, {...options, title, shown_value});
  });
  
}, 1000.0/60);


// Simulations sounds
// ------------------

const sound_scaling = {
  r: 1.0 / 0.002,
  rv: 1.0 / 1.0,
}

// TODO: ground the sound generation to physics! and rename the sound normalizer to also something more common
function play_simulation_sound(simulation, max_sound){
  const audio_context = new AudioContext({sampleRate: simulation.options.sound_sample_rate});
  // Create an empty three-second stereo buffer at the sample rate of the AudioContext.
  const noise_buffer = audio_context.createBuffer(
    2,
    simulation.options.sound_buffer_size,
    simulation.options.sound_sample_rate,
  );

  const left_channel = noise_buffer.getChannelData(0);
  const right_channel = noise_buffer.getChannelData(1);

  const normalize_sound = get_sound_normalizer(max_sound);
  const angular_spread = π / 3.0;
  const diff_scale = π / angular_spread;

  function normed_diff(a, b) {
    return (a - b + 3.0 * π) % (2.0 * π) - π;
  }

  function capped_cos(angle) {
    return Math.cos(Math.min(Math.max(angle, -π*0.5), π*0.5));
  }

  function directional_scaled_cos(angle_a, angle_b) {
    const diff_pos = normed_diff(angle_a, angle_b);
    const diff_cos_is_negative = (diff_pos > π*0.5 || diff_pos < -π*0.5)
    return diff_cos_is_negative ? -capped_cos(normed_diff(angle_a, angle_b + π) * diff_scale) : capped_cos(diff_pos * diff_scale);
  }

  simulation.sounds.forEach(({rx_v, ry_v, rx, ry, r, rv, φ_rv, φ_r, drv}, i) => {
    const intensity = normalize_sound(drv);

    left_channel[i] = directional_scaled_cos(φ_rv, 0.0) * intensity;
    right_channel[i] = directional_scaled_cos(φ_rv, π * 0.5) * intensity;
  });

  // Create a buffer source.
  const noise = audio_context.createBufferSource();
  noise.buffer = noise_buffer;

  // Connect to output.
  noise.connect(audio_context.destination);

  // Start the noise.
  noise.start();
}


// Start the simulation
// --------------------

function interactive_inputs(state, parameters, outputs, update_memory) {
  const {driver_mode, load_mode, load_torque, target_angle} = interactive_sliders.value;

  const {φ} = state;
  const driver_function = driver_mode_map[driver_mode];
  const [U_switch, V_switch, W_switch] = driver_function(state, parameters, outputs, update_memory);

  const τ_load = load_mode_map[load_mode](load_torque, target_angle, φ);

  return {
    U_switch, // driver connection state for phase U
    V_switch, // driver connection state for phase V
    W_switch, // driver connection state for phase W
    τ_load, // external load torque
    battery_connected: driver_mode != "Disconnect Battery", // whether the battery is connected
  }
}

function setup_simulation(){
  return new Simulation({
    start_state: initial_state,
    parameters: initial_parameters,
    update_inputs: interactive_inputs,
    update_memory: {},
    wave_buffer_size: wave_sample_size,
  });
}

let simulation = setup_simulation();

function show_visuals(){
  const selection = plot_selection.value;
  const shown_value = plot_shown_value_map[plot_shown_value_choice.value];

  show_simulation_plots(simulation.history, selection, shown_value);

  update_motor_visuals(simulation, interactive_sliders.value);
}

function update_and_show(){
  const {frequency, steps_number, store_period} = timing_sliders.value;

  simulation.options.steps_number = Math.ceil(steps_number);
  simulation.options.store_period = Math.ceil(store_period);
  simulation.options.dt = 1.0 / frequency;

  simulation.update();

  show_visuals();
}

const rendering = setup_rendering(scene, invalidation);

function * simulate_live () {
  while(true){
    consume_button(audio_video_controls, {
      "Look at motor": () => { rendering.reset_camera(); },
      "Reset inputs": () => { 
        timing_sliders.value = timing_sliders_defaults; 
        interactive_sliders.value = interactive_sliders_defaults;
      },
      "Play 🎵": () => { play_simulation_sound(simulation, interactive_sliders.value.max_sound); },
    });

    consume_button(plot_preselections, {
      "Show Default": plot_save_and_switch(plot_selection_defaults),
      "Show All": plot_save_and_switch(Object.keys(plot_selection_map)),
      "Hide Plots": plot_save_and_switch([]),
      "User Selection": plot_restore_selection,
      "Reset Selection": plot_reset_selection,
    }, show_visuals);

    consume_button(simulation_controls, {
      "Play": () => { update_and_show(); return "Play"; },
      "Pause": () => "Pause",
      "Step": () => { update_and_show(); return "Pause"; },
      "Restart": () => {
        simulation = setup_simulation();
        update_and_show();
        return "Play";
      },
    });
    
    rendering.render({highlight_filter: show_object_with_description});

    show_simulation_info(simulation, rendering);

    yield rendering.div;
  }
}

```

```js
const live_simulation = simulate_live();
```

