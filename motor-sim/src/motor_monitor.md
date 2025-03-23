---
title: Motor monitor
---


Motor command dashboard
-----------------------

<div>${connect_buttons}</div>
<div>${motor_controller_status}</div>
<div>${data_stream_buttons}</div>
<div>${command_buttons}</div>
<div>
  ${command_value_slider}
  ${command_timeout_slider}
</div>


```js

const time_conversion = 1/23400 * 1000;

const adc_voltage_reference = 3.3;
const motor_shunt_resistance = 0.010;
const amplifier_gain = 20.0;
const adc_max_value = 0xFFF;
const current_conversion = adc_voltage_reference / (adc_max_value * motor_shunt_resistance * amplifier_gain);

const max_measurable_current = adc_max_value / 2 * current_conversion;

const drive_resistance = 2.0; // 2.0 Ohm measured with voltmeter between 1 phase and the other 2 in parallel.
const drive_voltage = 10.0; // 10.0 V // TODO: get it from the chip

const max_calibration_current = 0.9 * drive_voltage / drive_resistance;

const phase_inductance = 0.000_145; // 290 uH measured with LCR meter across phase pairs.
const phase_resistance = 2.0; // 2.0 Ohm

```

```js

let motor_controller = Mutable(null);

let motor_controller_status = Mutable(html`<span>Not connected.</span>`);

function update_motor_controller_status(status){
  motor_controller_status.value = status;
}


async function open_port_and_read(){
  if (motor_controller.value) {
    console.info("Forgetting previous port");
    try {
      await motor_controller.value.com_port.forget();
    } catch (error) {
      // Ignore network errors when forgetting, likely due to previous disconnect.
      if (error.name != "NetworkError") throw error;
    }
  }
  try {
    motor_controller.value = await motor.connect_usb_motor_controller();

    update_motor_controller_status(html`<span>Connected, waiting for data.</span>`);

    for await (const data_received of motor_controller.value.reading_loop()) {
      update_motor_controller_status(html`<span>Connected; received: ${data_received} bytes.</span>`);
    }

  } catch (error) {
    motor_controller.value = null;

    if (error.name === "SecurityError") {
      update_motor_controller_status(html`<span style="color: red">Permission for port dialog denied.</span>`);
      return;
    }

    if (error.message === "EOF") {
      update_motor_controller_status(html`<span style="color: red">End of connection.</span>`);
      return;
    }
    // Check for NetworkError due to disconnect.
    if (error.name === "NetworkError") {
      update_motor_controller_status(html`<span style="color: red">Connection lost.</span>`);
      return;
    }

    if (error.name === "NotFoundError") {
      update_motor_controller_status(html`<span style="color: red">No device found or nothing selected.</span>`);
      return;
    }

    update_motor_controller_status(html`<span style="color: red">Connection lost; unknown error: ${error}</span>`);
    throw error;
  }
}

invalidation.then(async function(){
  if (motor_controller.value) {
    console.info("Invalidating motor controller; forgetting port.");
    await motor_controller.value.com_port.forget();
    motor_controller.value = null;
    motor_controller_status.value = html`<span style="color: red">Invalidated!</span>`;
  }
});



const connect_buttons = Inputs.button(
  [
    ["Connect", open_port_and_read],
  ],
  {label: "Connect to COM"},
);

open_port_and_read();
```

```js
const command_value_slider = Inputs.range([0, 1], {value: 0.2, step: 0.05, label: "Command value:"});

const command_value_fraction = Generators.input(command_value_slider);

const command_timeout_slider = Inputs.range([0, motor.MAX_TIMEOUT*time_conversion], {value: 2000, step: 100, label: "Command timeout (ms):"});

const command_timeout_millis = Generators.input(command_timeout_slider);
```

```js

// Control functions

let raw_readout_data = Mutable();

const max_data_points = motor.HISTORY_SIZE;

const command_timeout = Math.floor(command_timeout_millis / time_conversion);
const command_value = Math.floor(command_value_fraction * motor.PWM_BASE);


async function command_and_stream(command, options = {}){
  if (!motor_controller) return;
    
  try {
    await motor_controller.send_command({command, command_timeout, command_value, ...options});

    // Start reading the data stream.
    for await (const data_snapshot of motor_controller.stream_readouts(options)) {
      raw_readout_data.value = data_snapshot.length > max_data_points ? data_snapshot.slice(-max_data_points) : data_snapshot;
    }
  } catch (error) {
    update_motor_controller_status(html`<span style="color: red">Error streaming data: ${error}</span>`);
    throw error;
  }
}

async function command(command){
  if (!motor_controller) return;
  
  await motor_controller.send_command({command, command_timeout, command_value});
}


const data_stream_buttons = Inputs.button(
  [
    ["ADC snapshot", async function(){
      await command_and_stream(motor.GET_READOUTS_SNAPSHOT, {expected_messages: motor.HISTORY_SIZE, max_missed_messages: 0});
    }],
    ["Freewheel Snapshot", async function(){
      await command(motor.SET_STATE_FREEWHEEL);
      await command_and_stream(motor.GET_READOUTS_SNAPSHOT, {expected_messages: motor.HISTORY_SIZE, max_missed_messages: 0});
    }],
    ["ADC stream", async function(){
      await command_and_stream(motor.GET_READOUTS, {max_missed_messages: 128});
    }],
    ["Test all permutations", async function(){
      await command_and_stream(motor.SET_STATE_TEST_ALL_PERMUTATIONS);
    }],
    ["Test ground short", async function(){
      await command_and_stream(motor.SET_STATE_TEST_GROUND_SHORT);
    }],
    ["Test positive short", async function(){
      await command_and_stream(motor.SET_STATE_TEST_POSITIVE_SHORT);
    }],
    ["Test U directions", async function(){
      await command_and_stream(motor.SET_STATE_TEST_U_DIRECTIONS);
    }],
    ["Test U increasing", async function(){
      await command_and_stream(motor.SET_STATE_TEST_U_INCREASING);
    }],
    ["Test U decreasing", async function(){
      await command_and_stream(motor.SET_STATE_TEST_U_DECREASING);
    }],
    ["Test V increasing", async function(){
      await command_and_stream(motor.SET_STATE_TEST_V_INCREASING);
    }],
    ["Test V decreasing", async function(){
      await command_and_stream(motor.SET_STATE_TEST_V_DECREASING);
    }],
    ["Test W increasing", async function(){
      await command_and_stream(motor.SET_STATE_TEST_W_INCREASING);
    }],
    ["Test W decreasing", async function(){
      await command_and_stream(motor.SET_STATE_TEST_W_DECREASING);
    }],
  ],
  {label: "Read data"},
);

const command_buttons = Inputs.button(
  [
    ["Stop", async function(){
      await command(motor.SET_STATE_OFF);
    }],
    ["Drive+", async function(){
      await command(motor.SET_STATE_DRIVE_POS);
    }],
    ["Drive-", async function(){
      await command(motor.SET_STATE_DRIVE_NEG);
    }],
    ["Freewheel", async function(){
      await command(motor.SET_STATE_FREEWHEEL);
    }],
    ["Hold U positive", async function(){
      await command(motor.SET_STATE_HOLD_U_POSITIVE);
    }],
    ["Hold V positive", async function(){
      await command(motor.SET_STATE_HOLD_V_POSITIVE);
    }],
    ["Hold W positive", async function(){
      await command(motor.SET_STATE_HOLD_W_POSITIVE);
    }],
    ["Hold U negative", async function(){
      await command(motor.SET_STATE_HOLD_U_NEGATIVE);
    }],
    ["Hold V negative", async function(){
      await command(motor.SET_STATE_HOLD_V_NEGATIVE);
    }],
    ["Hold W negative", async function(){
      await command(motor.SET_STATE_HOLD_W_NEGATIVE);
    }],
  ],
  {label: "Commands"},
);

```



Motor driver phase currents
----------------------------
<div class="card tight">${data_plots["Electric Position"]}</div>
<div class="card tight">${data_plots["Speed"]}</div>
<div class="card tight">${data_plots["Measured Current"]}</div>
<div class="card tight">${data_plots["DQ0 Currents"]}</div>
<div class="card tight">${data_plots["Inferred Voltage"]}</div>
<div class="card tight">${data_plots["PWM Settings"]}</div>


```js
const identity_calibration_factors = {
  u_positive: 1.0,
  u_negative: 1.0,
  v_positive: 1.0,
  v_negative: 1.0,
  w_positive: 1.0,
  w_negative: 1.0,
};

function load_calibration_factors(){
  return get_stored_or_default("current_calibration", identity_calibration_factors);
}

// Load or make default calibration
const saved_calibration_factors = load_calibration_factors();

let calibration_factors = Mutable(saved_calibration_factors);

function update_calibration_factors(new_calibration_factors){
  calibration_factors.value = new_calibration_factors;
}

function reload_calibration_factors(){
  update_calibration_factors(load_calibration_factors());
}

function save_calibration_factors(){
  localStorage.setItem("current_calibration", JSON.stringify(calibration_factors.value));
}

function reset_calibration_factors(){
  localStorage.removeItem("current_calibration");
  update_calibration_factors(identity_calibration_factors);
}
```


```js

function matrix_multiply(m, v){
  // Return the matrix-vector product of a 3x3 matrix and a 3-vector.
  return m.map((row) => d3.sum(row.map((x, i) => x * v[i])));
}

const power_invariant_clarke_matrix = [
  [Math.sqrt(2/3), -0.5 * Math.sqrt(2/3), -0.5 * Math.sqrt(2/3)],
  [0, Math.sqrt(2)/2, -Math.sqrt(2)/2],
  [1/Math.sqrt(3), 1/Math.sqrt(3), 1/Math.sqrt(3)],
];

const power_invariant_simplified_clarke_matrix = [
  [Math.sqrt(2/3), -0.5 * Math.sqrt(2/3), -0.5 * Math.sqrt(2/3)],
  [0, Math.sqrt(2)/2, -Math.sqrt(2)/2],
];

function calculate_data_stats(raw_readout_data){

  const ref_readout_mean = d3.mean(raw_readout_data, (d) => d.ref_readout);
  const start_readout_number = raw_readout_data[0].readout_number;


  const data_with_current = raw_readout_data.map((d) => {
    const readout_number = d.readout_number - start_readout_number;
    const time = readout_number * time_conversion;
  
    const u_readout = -current_conversion * (d.u_readout - d.ref_readout);
    const v_readout = +current_conversion * (d.v_readout - d.ref_readout);
    const w_readout = -current_conversion * (d.w_readout - d.ref_readout);
    const ref_diff = current_conversion * (d.ref_readout - ref_readout_mean);
  
    const calibrated_u = u_readout * (u_readout >= 0 ? 
      calibration_factors.u_positive :
      calibration_factors.u_negative);

    const calibrated_v = v_readout * (v_readout >= 0 ?
      calibration_factors.v_positive :
      calibration_factors.v_negative);

    const calibrated_w = w_readout * (w_readout >= 0 ?
      calibration_factors.w_positive :
      calibration_factors.w_negative);

    const sum = calibrated_u + calibrated_v + calibrated_w;

    const u = calibrated_u - sum / 3.0;
    const v = calibrated_v - sum / 3.0;
    const w = calibrated_w - sum / 3.0;

    const [alpha, beta] = matrix_multiply(power_invariant_simplified_clarke_matrix, [u, v, w]);

    const current_angle = Math.atan2(beta, alpha);
    const radial_magnitude = Math.sqrt(alpha * alpha + beta * beta);

    return {...d, u_readout, v_readout, w_readout, u, v, w, ref_diff, time, sum, alpha, beta, current_angle, radial_magnitude};
  });

  const derivative_start_index = 8;
  const derivative_end_index = raw_readout_data.length - 2;

  function derivative_3_points(index, df, data, circular = false){
    if (index < derivative_start_index || index > derivative_end_index) return null;
    const left = data[index - 1];
    const mid = data[index];
    const right = data[index + 1];
    const left_derivative = df(left, mid) / (mid.time - left.time) * 1000;
    const right_derivative = df(mid, right) / (right.time - mid.time) * 1000;
    return (left_derivative + right_derivative) / 2.0;
  }

  function voltage_3_points(index, df, data){
    const derivative = derivative_3_points(index, df, data);
    return derivative == null ? null : derivative * phase_inductance;
  }

  function diff_phi(a, b){
    const diff = b.current_angle - a.current_angle;
    return diff > Math.PI ? diff - 2 * Math.PI : diff < -Math.PI ? diff + 2 * Math.PI : diff;
  }

  function speed_3_points(index, df, data){
    const derivative = derivative_3_points(index, df, data);
    return derivative == null ? null : derivative / (1000 * 2 * Math.PI); // rotations per millisecond
  }

  const data_with_derivatives = data_with_current.map((d, i, data) => {
    return {
      ...d,
      radial_speed: speed_3_points(i, diff_phi, data),
      u_voltage: voltage_3_points(i, (a, b) => b.u - a.u, data),
      v_voltage: voltage_3_points(i, (a, b) => b.v - a.v, data),
      w_voltage: voltage_3_points(i, (a, b) => b.w - a.w, data),
    };
  });

  const data = data_with_derivatives;

  return {data, ref_readout_mean, start_readout_number};
}

const {data, ref_readout_mean} = calculate_data_stats(raw_readout_data);

```



```js

const colors = {
  u: "cyan",
  v: "orangered",
  w: "purple",
  ref_diff: "gray",
  sum: "black",
  radial_magnitude: "gold",
  current_angle: "green",
  radial_speed: "steelblue",
  alpha: "red",
  beta: "blue",
};


const π = Math.PI;
const ε = 0.1;

const hall_3_as_angle = (d) => d.hall_3 ? d.hall_1 ? +π/3 -ε : d.hall_2 ? -π/3 +ε : 0.0 : null;
const hall_1_as_angle = (d) => d.hall_1 ? d.hall_3 ? +π/3 +ε : d.hall_2 ? +π -ε : +2*π/3 : null;
const hall_2_as_angle = (d) => d.hall_2 ? d.hall_1 ? -π +ε : d.hall_3 ? -π/3 -ε : -2*π/3 : null;


const data_plot_layout = {
  "Electric Position": {
    description: html`<p>The position of the rotor with respect to the electric phases, 0 when magnetic N is aligned with phase U.</p>`,
    y: {label: "Electric position (rad)", domain: [-Math.PI, Math.PI]},
    width: 1200, height: 150,
    mark_functions: {
      "Current (Park) Angle": (data) => Plot.line(data, {x: "time", y: "current_angle", stroke: colors.current_angle, label: "current angle", curve: "step"}),
      "Hall Angle": (data) => [
        Plot.line(data, {x: "time", y: hall_3_as_angle, stroke: colors.u, label: 'hall 3', curve: 'step', strokeWidth: 3}),
        Plot.line(data, {x: "time", y: hall_1_as_angle, stroke: colors.v, label: 'hall 1', curve: 'step', strokeWidth: 3}),
        Plot.line(data, {x: "time", y: hall_2_as_angle, stroke: colors.w, label: 'hall 2', curve: 'step', strokeWidth: 3}),
      ],
      "Grid": (data) => [
        Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 2}),
        Plot.gridX({interval: 0.2, stroke: 'black', strokeWidth : 1}),
        Plot.gridY({interval: Math.PI / 2, stroke: 'black', strokeWidth : 2}),
      ],
    }
  },
  "Speed": {
    description: html`<p>The speed of the rotor in rotations per millisecond.</p>`,
    y: {label: "Speed (rotations/ms)"},
    width: 1200, height: 150,
    mark_functions: {
      "Speed": (data) => Plot.line(data, {x: "time", y: "radial_speed", stroke: colors.radial_speed, label: "radial speed", curve: 'step'}),
      "Grid": (data) => [
        Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 2}),
        Plot.gridX({interval: 0.2, stroke: 'black', strokeWidth : 1}),
        Plot.gridY({stroke: 'black', strokeWidth : 2}),
      ],
    }
  },
  "Measured Current": {
    description: html`<p>The measured current values for each phase.</p>`,
    y: {label: "Current (A)"},
    width: 1200, height: 400,
    mark_functions: {
      "Current U": (data) => Plot.line(data, {x: "time", y: "u", stroke: colors.u, label: 'Current U', curve: 'step'}),
      "Current V": (data) => Plot.line(data, {x: "time", y: "v", stroke: colors.v, label: 'Current V', curve: 'step'}),
      "Current W": (data) => Plot.line(data, {x: "time", y: "w", stroke: colors.w, label: 'Current W', curve: 'step'}),
      "Sum": (data) => Plot.line(data, {x: "time", y: "sum", stroke: colors.sum, label: 'Sum', curve: 'step'}),
      "Reference Change": (data) => Plot.line(data, {x: "time", y: "ref_diff", stroke: colors.ref_diff, label: 'Ref Diff', curve: 'step'}),
      "Grid": (data) => [
        Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 2}),
        Plot.gridX({interval: 0.2, stroke: 'black', strokeWidth : 1}),
        Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 2}),
      ],
    }
  },
  "DQ0 Currents": {
    description: html`<p>The DQ currents calculated from the measured currents.</p>`,
    y: {label: "Current (A)"},
    width: 1200, height: 400,
    mark_functions: {
      "Alpha": (data) => Plot.line(data, {x: "time", y: "alpha", stroke: colors.alpha, label: 'Alpha', curve: 'step'}),
      "Beta": (data) => Plot.line(data, {x: "time", y: "beta", stroke: colors.beta, label: 'Beta', curve: 'step'}),
      "Radial Magnitude": (data) => Plot.line(data, {x: "time", y: "radial_magnitude", stroke: colors.radial_magnitude, label: 'Radial Magnitude', curve: 'step'}),
      "Grid": (data) => [
        Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 2}),
        Plot.gridX({interval: 0.2, stroke: 'black', strokeWidth : 1}),
        Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 2}),
      ],
    }
  },
  "Inferred Voltage": {
    description: html`<p>The inferred voltage values for each phase (V = L*dI/dt).</p>`,
    y: {label: "Voltage (V)"},
    width: 1200, height: 300,
    mark_functions: {
      "Voltage U": (data) => Plot.line(data, {x: "time", y: "u_voltage", stroke: colors.u, label: 'Voltage U', curve: 'step'}),
      "Voltage V": (data) => Plot.line(data, {x: "time", y: "v_voltage", stroke: colors.v, label: 'Voltage V', curve: 'step'}),
      "Voltage W": (data) => Plot.line(data, {x: "time", y: "w_voltage", stroke: colors.w, label: 'Voltage W', curve: 'step'}),
      "Grid": (data) => [
        Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 2}),
        Plot.gridX({interval: 0.2, stroke: 'black', strokeWidth : 1}),
        Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 2}),
      ],
    }
  },
  "PWM Settings": {
    description: html`<p>The PWM value currently set for each phase.</p>`,
    y: {label: "PWM", domain: [0, motor.PWM_BASE]},
    width: 1200, height: 150,
    mark_functions: {
      "PWM U": (data) => Plot.line(data, {x: "time", y: "u_pwm", stroke: colors.u, label: 'PWM U', curve: 'step', strokeDasharray: "1 4", strokeWidth: 2}),
      "PWM V": (data) => Plot.line(data, {x: "time", y: "v_pwm", stroke: colors.v, label: 'PWM V', curve: 'step', strokeDasharray: "1 3", strokeWidth: 2}),
      "PWM W": (data) => Plot.line(data, {x: "time", y: "w_pwm", stroke: colors.w, label: 'PWM W', curve: 'step', strokeDasharray: "2 4", strokeWidth: 2}),
      "Grid": (data) => [
        Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 2}),
        Plot.gridX({interval: 0.2, stroke: 'black', strokeWidth : 1}),
        Plot.gridY({interval: 128, stroke: 'black', strokeWidth : 1}),
      ],
    }
  },
}; // End of data_plot_layout

const data_plot_default_selection = Object.fromEntries(
  Object.entries(data_plot_layout).map(([subtitle, plot_group]) => {
    return [subtitle, {
      show: true,
      shown_marks: Object.keys(plot_group.mark_functions),
    }];
  })
);

let data_plot_selection = Mutable(get_stored_or_default("data_plot_selection", data_plot_default_selection));
// let data_plot_selection = Mutable(data_plot_default_selection);

function update_data_plot_selection(new_selection){
  data_plot_selection.value = new_selection;
  localStorage.setItem("data_plot_selection", JSON.stringify(new_selection));
}

function pick_selected(selected_keys, object){
  return Object.keys(object).filter((key) => selected_keys.includes(key)).map((key) => object[key]);
}

function selectable_plots(data, selection, update_selection){
  const domain = [0, data[data.length - 1].time];
  
  // We want to render a list of plots, each with a title, a description, and configurable selection of data.
  // Because they share the x axis, we can select a subset of the data. We can also show the same vertical
  // cursor when the user hovers over the plots.
  const built_plots = Object.fromEntries(Object.entries(data_plot_layout).map(([subtitle, plot_group]) => {
    // Create a plot for each group.
    const {description, y, width, height, mark_functions} = plot_group;


    // First, make the title into a checkbox to toggle the plot on and off.
    const subtitle_checkbox = Inputs.checkbox([subtitle], {
      value: selection[subtitle].show ? [subtitle] : [],
      format: (subtitle) => html`<h4 style="min-width: 20em; font-size: 1.5em; font-weight: normal;">${subtitle}</h4>`,
    });

    subtitle_checkbox.addEventListener("input", function(){
      const show = subtitle_checkbox.value.length > 0;
      selection[subtitle].show = show;
      update_data_plot_selection(selection);
    });


    if (!selection[subtitle].show) {
      return [subtitle, html`<div class="card tight">${subtitle_checkbox}</div>`];
    }

    // Then, make the marks into checkboxes to toggle them on and off.
    const marks_checkboxes = Inputs.checkbox(Object.keys(mark_functions), {value: selection[subtitle].shown_marks, label: "Display:"});

    marks_checkboxes.addEventListener("input", function(){
      const shown_marks = marks_checkboxes.value;
      selection[subtitle].shown_marks = shown_marks;
      update_data_plot_selection(selection);
    });

    // Finally, create the plot with the selected marks.
    const selected_marks = pick_selected(selection[subtitle].shown_marks, mark_functions).map((mark_function) => mark_function(data));

    const plot = Plot.plot({
      x: {label: "Time (ms)", domain},
      y,
      width, height,
      color: {legend: true},
      marks: selected_marks,
    });

    return [subtitle, html`<div>${subtitle_checkbox}${description}${marks_checkboxes}${plot}</div>`];
  }));


  return built_plots;
}


```

```js
const data_plots = selectable_plots(data, data_plot_selection, update_data_plot_selection);


```

Calibration procedures
----------------------

<div class="card tight">
  <div>${calibration_buttons}</div>
  <div>${calibration_plots}</div>
</div>

```js
let calibration_results = Mutable([]);

function store_calibration_result(calibration_data){
  calibration_results.value = [calibration_data, ...calibration_results.value];
}
```


```js
const short_duration = motor.HISTORY_SIZE / 12 * time_conversion;

const calibration_zones = [
  {pwm: 0.1, start: short_duration * 1.4, end: short_duration * 1.95},
  {pwm: 0.2, start: short_duration * 2.4, end: short_duration * 2.95},
  {pwm: 0.3, start: short_duration * 3.4, end: short_duration * 3.95},
  {pwm: 0.4, start: short_duration * 4.4, end: short_duration * 4.95},
  {pwm: 0.5, start: short_duration * 5.4, end: short_duration * 5.95},
  {pwm: 0.6, start: short_duration * 6.4, end: short_duration * 6.95},
  {pwm: 0.7, start: short_duration * 7.4, end: short_duration * 7.95},
  {pwm: 0.8, start: short_duration * 8.4, end: short_duration * 8.95},
  {pwm: 0.9, start: short_duration * 9.4, end: short_duration * 9.95},
];


const calibration_points = 32;




async function run_calibration(){
  if (!motor_controller) return;
  
  const settle_time = 300;
  const settle_timeout = settle_time * 3;
  const settle_strength = Math.floor(motor.PWM_BASE * 2 / 10);

  console.info("Calibration starting");

  // Note: hold pwm is clamped by the motor driver

  await motor_controller.send_command({command: motor.SET_STATE_HOLD_U_POSITIVE, command_timeout: settle_timeout, command_value: settle_strength});
  await wait(settle_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_U_INCREASING, command_timeout: 0, command_value: 0});

  const u_positive_data = calculate_data_stats(await motor_controller.get_readouts({expected_messages: motor.HISTORY_SIZE, max_missed_messages: 0})).data;

  console.info("U positive done");

  await motor_controller.send_command({command: motor.SET_STATE_HOLD_W_NEGATIVE, command_timeout: settle_timeout, command_value: settle_strength});
  await wait(settle_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_W_DECREASING, command_timeout: 0, command_value: 0});

  const w_negative_data = calculate_data_stats(await motor_controller.get_readouts({expected_messages: motor.HISTORY_SIZE, max_missed_messages: 0})).data;

  console.info("W negative done");

  await motor_controller.send_command({command: motor.SET_STATE_HOLD_V_POSITIVE, command_timeout: settle_timeout, command_value: settle_strength});
  await wait(settle_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_V_INCREASING, command_timeout: 0, command_value: 0});

  const v_positive_data = calculate_data_stats(await motor_controller.get_readouts({expected_messages: motor.HISTORY_SIZE, max_missed_messages: 0})).data;

  console.info("V positive done");

  await motor_controller.send_command({command: motor.SET_STATE_HOLD_U_NEGATIVE, command_timeout: settle_timeout, command_value: settle_strength});
  await wait(settle_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_U_DECREASING, command_timeout: 0, command_value: 0});

  const u_negative_data = calculate_data_stats(await motor_controller.get_readouts({expected_messages: motor.HISTORY_SIZE, max_missed_messages: 0})).data;

  console.info("U negative done");

  await motor_controller.send_command({command: motor.SET_STATE_HOLD_W_POSITIVE, command_timeout: settle_timeout, command_value: settle_strength});
  await wait(settle_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_W_INCREASING, command_timeout: 0, command_value: 0});

  const w_positive_data = calculate_data_stats(await motor_controller.get_readouts({expected_messages: motor.HISTORY_SIZE, max_missed_messages: 0})).data;

  console.info("W positive done");

  await motor_controller.send_command({command: motor.SET_STATE_HOLD_V_NEGATIVE, command_timeout: settle_timeout, command_value: settle_strength});
  await wait(settle_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_V_DECREASING, command_timeout: 0, command_value: 0});

  const v_negative_data = calculate_data_stats(await motor_controller.get_readouts({expected_messages: motor.HISTORY_SIZE, max_missed_messages: 0})).data;

  console.info("V negative done");

  const calibration_data = {
    u_positive_data,
    u_negative_data,
    v_positive_data,
    v_negative_data,
    w_positive_data,
    w_negative_data,
  };

  console.info("Calibration done");
  
  store_calibration_result(calibration_data);
}



function compute_zone_calibration({measurements, targets}){
  // Make sure the lengths are equal.
  if (measurements.length !== targets.length) throw new Error("Data length mismatch");

  const factor = d3.mean(targets, (target, i) => target / measurements[i]); 

  const slow_calibration = piecewise_linear({
    X: [0.0, ...measurements.map((x) => x)], 
    Y: [0.0, ...targets.map((y) => y / factor)],
  });

  const n = calibration_points / 2 + 1;

  // Recalibrate to evenly spaced points for fast processing.

  const X = even_spacing(max_calibration_current, n);
  const Y = X.map((x) => slow_calibration(x));


  const func = even_piecewise_linear({x_min: 0, x_max: max_calibration_current, Y});

  const sample = X.map((x) => ({reading: x, target: func(x)}));
  
  return {
    measurements,
    targets,
    factor,
    func,
    sample,
  };
}


function compute_calibration_stats(data_selector, readout_selector){
  if (calibration_results.length < 1) return null;

  const phase_calibration_data = calibration_results.map(data_selector);
  // Check the length of the data is the same for all collections.
  const latest_data = phase_calibration_data[0];
  if (phase_calibration_data.some((d) => d.length !== latest_data.length)) {
    console.error(phase_calibration_data.map((d) => d.length));
    throw new Error("Data length mismatch");
  }
  
  return latest_data.map((d, i) => {
    const calibration_data = calibration_results.map(data_selector).map((d) => d[i]);
    const readouts = calibration_data.map(readout_selector);
    return {
      time: d.time,
      readout_latest: readout_selector(d),
      readout_mean: d3.mean(readouts),
      readout_std: d3.deviation(readouts),
    };
  });
}

const calibration_stats = {
  u_positive_stats: compute_calibration_stats((d) => d.u_positive_data, (d) => d.u_readout),
  u_negative_stats: compute_calibration_stats((d) => d.u_negative_data, (d) => -d.u_readout),
  v_positive_stats: compute_calibration_stats((d) => d.v_positive_data, (d) => d.v_readout),
  v_negative_stats: compute_calibration_stats((d) => d.v_negative_data, (d) => -d.v_readout),
  w_positive_stats: compute_calibration_stats((d) => d.w_positive_data, (d) => d.w_readout),
  w_negative_stats: compute_calibration_stats((d) => d.w_negative_data, (d) => -d.w_readout),
};

const predicted_targets = calibration_zones.map((zone) => zone.pwm * drive_voltage / drive_resistance);

function select_by_zone(data){
  return calibration_zones.map((zone) => {
    return data.filter((d) => d.time > zone.start && d.time < zone.end);
  });
}

function compute_phase_calibration(data){
  if (!data) return null;

  return compute_zone_calibration({
    measurements: select_by_zone(data).map((zone_data) => d3.mean(zone_data, (d) => d.readout_mean)),
    targets: predicted_targets,
  });
}

const calibration = {
  u_positive: compute_phase_calibration(calibration_stats.u_positive_stats),
  u_negative: compute_phase_calibration(calibration_stats.u_negative_stats),
  v_positive: compute_phase_calibration(calibration_stats.v_positive_stats),
  v_negative: compute_phase_calibration(calibration_stats.v_negative_stats),
  w_positive: compute_phase_calibration(calibration_stats.w_positive_stats),
  w_negative: compute_phase_calibration(calibration_stats.w_negative_stats),
};


const calibration_plots = [
  html`<div>Number of calibration data sets: ${calibration_results.length}</div>`,
  ...(calibration_results.length < 1 ? [html`<div>No calibration data</div>`] : [
    html`<h3>Calibration results</h3>`,
    Plot.plot({
      marks: [
        Plot.line(calibration_stats.u_positive_stats, {x: "time", y: "readout_latest", stroke: colors.u, label: "u positive"}),
        Plot.line(calibration_stats.v_positive_stats, {x: "time", y: "readout_latest", stroke: colors.v, label: "v positive"}),
        Plot.line(calibration_stats.w_positive_stats, {x: "time", y: "readout_latest", stroke: colors.w, label: "w positive"}),
        Plot.line(calibration_stats.u_negative_stats, {x: "time", y: "readout_latest", stroke: colors.u, strokeDasharray: "2 5", label: "u negative"}),
        Plot.line(calibration_stats.v_negative_stats, {x: "time", y: "readout_latest", stroke: colors.v, strokeDasharray: "2 5", label: "v negative"}),
        Plot.line(calibration_stats.w_negative_stats, {x: "time", y: "readout_latest", stroke: colors.w, strokeDasharray: "2 5", label: "w negative"}),
        Plot.rect(calibration_zones, {x1: "start", x2: "end", y1: 0, y2: max_calibration_current, fill: "rgba(0, 0, 0, 0.05)"}),
        Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 1}),
        Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 1}),
      ],
      x: {label: "Time (ms)"},
      y: {label: "Current (A)"},
      width: 1200, height: 400,
    }),
    html`<div>Phase correction factors:</div><table>
      <tr><td>U:</td><td>${calibration.u_positive.factor.toFixed(3)}</td><td>${calibration.u_negative.factor.toFixed(3)}</td></tr>
      <tr><td>V:</td><td>${calibration.v_positive.factor.toFixed(3)}</td><td>${calibration.v_negative.factor.toFixed(3)}</td></tr>
      <tr><td>W:</td><td>${calibration.w_positive.factor.toFixed(3)}</td><td>${calibration.w_negative.factor.toFixed(3)}</td></tr>
    </table>`,
    Plot.plot({
      marks: [
        Plot.line([[0, 0], [max_calibration_current, max_calibration_current]], {stroke: "gray", label: "reference"}),
        Plot.line(calibration.u_positive.sample, {x: "reading", y: "target", stroke: colors.u, label: "u", marker: "circle"}),
        Plot.line(calibration.v_positive.sample, {x: "reading", y: "target", stroke: colors.v, label: "v", marker: "circle"}),
        Plot.line(calibration.w_positive.sample, {x: "reading", y: "target", stroke: colors.w, label: "w", marker: "circle"}),
        Plot.line(calibration.u_negative.sample, {x: "reading", y: "target", stroke: colors.u, strokeDasharray: "2 5", label: "u linear", marker: "circle"}),
        Plot.line(calibration.v_negative.sample, {x: "reading", y: "target", stroke: colors.v, strokeDasharray: "2 5", label: "v linear", marker: "circle"}),
        Plot.line(calibration.w_negative.sample, {x: "reading", y: "target", stroke: colors.w, strokeDasharray: "2 5", label: "w linear", marker: "circle"}),
        Plot.gridX({interval: 0.5, stroke: 'black', strokeWidth : 1}),
        Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 1}),
      ],
      x: {label: "Current Reading (A)"},
      y: {label: "Current Estimate (A)"},
      width: 1200, height: 400,
    }),
  ]),
  ...(calibration_results.length < 2 ? [html`<div>No calibration statistics</div>`] : [
    html`<h3>Calibration statistics</h3>`,
    Plot.plot({
      marks: [
        Plot.line(calibration_stats.u_positive_stats, {x: "time", y: "readout_mean", stroke: colors.u, label: "u positive"}),
        Plot.line(calibration_stats.v_positive_stats, {x: "time", y: "readout_mean", stroke: colors.v, label: "v positive"}),
        Plot.line(calibration_stats.w_positive_stats, {x: "time", y: "readout_mean", stroke: colors.w, label: "w positive"}),
        Plot.line(calibration_stats.u_negative_stats, {x: "time", y: "readout_mean", stroke: colors.u, strokeDasharray: "2 5", label: "u negative"}),
        Plot.line(calibration_stats.v_negative_stats, {x: "time", y: "readout_mean", stroke: colors.v, strokeDasharray: "2 5", label: "v negative"}),
        Plot.line(calibration_stats.w_negative_stats, {x: "time", y: "readout_mean", stroke: colors.w, strokeDasharray: "2 5", label: "w negative"}),
        Plot.rect(calibration_zones, {x1: "start", x2: "end", y1: 0, y2: max_calibration_current, fill: "rgba(0, 0, 0, 0.05)"}),
        Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 1}),
        Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 1}),
      ],
      x: {label: "Time (ms)"},
      y: {label: "Current (A)"},
      width: 1200, height: 400,
    }),
    Plot.plot({
      marks: [
        Plot.areaY(calibration_stats.u_positive_stats, {x: "time", y1: (d) => d.readout_mean - d.readout_std, y2: (d) => d.readout_mean + d.readout_std, fill: colors.u}),
        Plot.areaY(calibration_stats.v_positive_stats, {x: "time", y1: (d) => d.readout_mean - d.readout_std, y2: (d) => d.readout_mean + d.readout_std, fill: colors.v}),
        Plot.areaY(calibration_stats.w_positive_stats, {x: "time", y1: (d) => d.readout_mean - d.readout_std, y2: (d) => d.readout_mean + d.readout_std, fill: colors.w}),
        Plot.areaY(calibration_stats.u_negative_stats, {x: "time", y1: (d) => -d.readout_mean - d.readout_std, y2: (d) => -d.readout_mean + d.readout_std, fill: colors.u}),
        Plot.areaY(calibration_stats.v_negative_stats, {x: "time", y1: (d) => -d.readout_mean - d.readout_std, y2: (d) => -d.readout_mean + d.readout_std, fill: colors.v}),
        Plot.areaY(calibration_stats.w_negative_stats, {x: "time", y1: (d) => -d.readout_mean - d.readout_std, y2: (d) => -d.readout_mean + d.readout_std, fill: colors.w}),
        Plot.rect(calibration_zones, {x1: "start", x2: "end", y1: -max_calibration_current, y2: max_calibration_current, fill: "rgba(0, 0, 0, 0.05)"}),
        Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 1}),
        Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 1}),
      ],
      x: {label: "Time (ms)"},
      y: {label: "Current (A)"},
      width: 1200, height: 600,
    }),
  ]),
];



const calibration_buttons = Inputs.button(
  [
    ["Start Calibration", async function(){
      await run_calibration();
    }],
    ["Save Calibration", function(){
      update_calibration_factors({
        u_positive: calibration.u_positive.factor,
        u_negative: calibration.u_negative.factor,
        v_positive: calibration.v_positive.factor,
        v_negative: calibration.v_negative.factor,
        w_positive: calibration.w_positive.factor,
        w_negative: calibration.w_negative.factor,
      });
      save_calibration_factors();
    }],
    ["Reload Calibration", function(){
      reload_calibration_factors();
    }],
    ["Reset Calibration", function(){
      reset_calibration_factors();
    }],
  ],
  {label: "Collect calibration data"},
);

```



```js
// Imports
// -------

import {html} from "htl";
import {localStorage, get_stored_or_default} from "./components/local_storage.js";
import {round, uint32_to_bytes, bytes_to_uint32, timeout_promise, wait, clean_id}  from "./components/utils.js";
import {even_spacing, piecewise_linear, even_piecewise_linear} from "./components/math_utils.js";
import * as motor from "./components/usb_motor_controller.js";

```