---
title: Motor monitor
---
<main class="hero">

Motor Commands
--------------

<div>${connect_buttons}</div>
<div>${connection_status}</div>
<div>${data_request_buttons}</div>
<div>
  <span>${command_options_input}</span>
  <span>${command_snapshot_delay_slider}</span>
</div>
<div>${stop_buttons}</div>
<div>${test_buttons}</div>
<div>
  <span>${simple_drive_buttons}</span>
  <span>${command_timeout_slider}</span>
  <span>${command_pwm_slider}</span>
</div>
<div>
  <span>${advanced_drive_buttons}</span>
  <span>${command_angle_slider}</span>
  <span>${command_angular_speed_slider}</span>
  <span>${command_torque_current_slider}</span>
  <span>${command_power_slider}</span>
</div>


Motor Driving Data
------------------
<div class="card tight">
<p>Controls for the plotting time window:</p>
  <span>${time_period_input}</span>
  <span>${time_offset_input}</span>
  <span>${plot_curve_input}</span>
  <span>${timeline_position_input}</span>
</div>
<div class="card tight">${plot_power}</div>
<div class="card tight">${plot_runtime_stats}</div>
<div class="card tight">${plot_cycle_loop_stats}</div>
<div class="card tight">${plot_electric_position}</div>
<div class="card tight">${plot_electric_offsets}</div>
<div class="card tight">${plot_speed}</div>
<div class="card tight">${plot_measured_voltage}</div>
<div class="card tight">${plot_measured_temperature}</div>
<div class="card tight">${plot_measured_current}</div>
<div class="card tight">${plot_voltages}</div>
<div class="card tight">${plot_inferred_voltages}</div>
<div class="card tight">${plot_dq0_currents}</div>
<div class="card tight">${plot_dq0_voltages}</div>
<div class="card tight">${plot_pwm_settings}</div>
<div class="card tight">${plot_readout_flags}</div>
<div class="card tight">${plot_motor_values}</div>


Motor Control Parameters
------------------------

<div class="card tight">
  <div>${control_parameters_buttons}</div>
  <h3>Active Control Parameters</h3>
  <p>These are the currently active parameters for the motor controller.</p>
  <pre>${active_control_parameters_table}</pre>
</div>

<div class="card tight">
  <h3>Control Parameters</h3>
  <div>${Object.values(control_parameters_input)}</div>
</div>


Current Calibration Procedures
------------------------------

<div class="card tight">
  <div>${current_calibration_buttons}</div>
  <div>${current_calibration_pwm_slider}</div>
  <div>Number of calibration data sets: ${current_calibration.results.length}</div>
</div>
<div class="card tight">
  <h3>Current Calibration Results</h3>
  <div>
    <p>Phase current correction factors:</p>
    <pre>${current_calibration_table}</pre>
    <p>Active phase current correction factors:</p>
    <pre>${active_current_calibration_table}</pre>
  </div>
  <div>${current_calibration_result_to_display_input}</div>
  <div>${current_calibration_plot}</div>
  <div>${current_calibration_optimization_iteration_input}</div>
  <div>${current_calibration_optimization_phase_input}</div>
  <div>${current_calibration_optimizing_plot}</div>
  <div>${current_calibration_optimizing_gradients_plot}</div>
  <div>
    <p>Current calibration factors at displayed iteration:</p>
    <pre>${current_calibration_iteration_table}</pre>
  </div>
</div>
<div class="card tight">  
  <h3>Current Calibration Statistics</h3>
  <div>${current_calibration_positive_mean_plot}</div>
  <div>${current_calibration_negative_mean_plot}</div>
</div>


Flash Data Storage
------------------

<div class="card tight">
  <p>Commit the uploaded calibration data to flash memory.</p>
  <div>${flash_buttons}</div>
</div>


Unit Tests
----------

<div class="card tight">
  <p>Run the unit tests for the motor controller. These tests focus on internal math functions.</p>
  <div>${unit_test_buttons}</div>
  <div>${unit_test_results}</div>
</div>


</main>

```js

const stdev_95_z_score = 1.959964; // 95% confidence interval for normal distribution


const colors = {
  u: "rgb(117, 112, 179)",
  v: "rgb(217, 95, 2)",
  w: "rgb(231, 41, 138)",
  web_angle: "rgb(178, 228, 0)",
  angle: "rgb(39, 163, 185)",
  web_current_magnitude: "rgb(197, 152, 67)",
  inductor_angle: "rgb(102, 166, 30)",
  voltage_angle: "rgb(0, 185, 124)",
  angle_driven: "rgb(166, 30, 132)",
  angular_speed: "rgb(41, 194, 173)",
  web_angular_speed: "rgb(156, 196, 47)",
  alpha_current: "rgb(199, 0, 57)",
  beta_current: "rgb(26, 82, 118)",
  other: "rgb(27, 158, 119)",
  ref_readout: "rgb(102, 102, 102)",
  sum: "rgb(0, 0, 0)",
};

const colors_categories = Object.values(colors);



```

```js

let motor_controller = Mutable(null);

let connection_status = Mutable(html`<pre>Not connected.</pre>`);


function display_connection_error(error){
  if (error.message === "EOF") {
    connection_status.value = html`<pre>End of connection.</pre>`;
  } else if (error.name === "NotFoundError") {
    connection_status.value = html`<pre style="color: red">No device found or nothing selected.</pre>`;
  } else if (error.name === "SecurityError") {
    connection_status.value = html`<pre style="color: red">Permission for port dialog denied.</pre>`;
  } else if (error.name === "NetworkError") {
    connection_status.value = html`<pre style="color: red">Connection lost.</pre>`;
  } else {
    connection_status.value = html`<pre style="color: red">Connection lost; unknown error: ${error}</pre>`;
    throw error;
  }
}

function format_bytes(bytes){
  if (bytes < 1024) return `${bytes.toFixed(2)} bytes`;
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(2)} KiB`;
  if (bytes < 1024 * 1024 * 1024) return `${(bytes / (1024 * 1024)).toFixed(2)} MiB`;
  return `${(bytes / (1024 * 1024 * 1024)).toFixed(2)} GiB`;
}

async function disconnect_motor_controller(show_status = true){
  if (motor_controller.value) {
    console.info("Disconnecting motor controller; forgetting port.");
    await motor_controller.value.forget();
    motor_controller.value = null;
    if (show_status) {
      connection_status.value = html`<pre style="color: orange">Disconnected!</pre>`;
    }
  }
}

function display_connection_stats({bytes_received, bytes_discarded, receive_rate}){
  bytes_received = `received: ${format_bytes(bytes_received).padStart(12)}`;
  bytes_discarded = `discarded: ${format_bytes(bytes_discarded).padStart(12)}`;
  receive_rate = `download rate: ${format_bytes(receive_rate).padStart(12)}/s`;

  connection_status.value = html`<pre>Connected; ${bytes_received}; ${receive_rate}; ${bytes_discarded}.</pre>`;
}

async function connect_motor_controller(){
  try {
    await disconnect_motor_controller(false);

    const new_controller = await connect_usb_motor_controller();

    connection_status.value = html`<pre>Connected, waiting for data.</pre>`;


    // Wait for the reading loop and calibrations in parallel.
    await Promise.all([
      new_controller.reading_loop(display_connection_stats),
      (async function(){
        await new_controller.load_current_calibration();
        await new_controller.load_control_parameters();
        motor_controller.value = new_controller;
      })(),
    ]);

  } catch (error) {
    motor_controller.value = null;

    display_connection_error(error);
  }
}

invalidation.then(disconnect_motor_controller);



const connect_buttons = Inputs.button(
  [
    ["Connect", connect_motor_controller],
    ["Disconnect", disconnect_motor_controller],
    // ["Reset data & inputs", () => (clear_stored_data(), location.reload())],
  ],
  {label: "Connect to COM"},
);

d3.select(connect_buttons).selectAll("button").style("height", "3em");


connect_motor_controller();
```

```js
const command_options_input = Inputs.checkbox(
  ["Take snapshot after command"],
  {
    value: [],
    label: "Command options:",
  },
);
d3.select(command_options_input).select("div").style("width", "100%");
d3.select(command_options_input).select("div label").style("width", "100em");

const command_snapshot_delay_slider = inputs_wide_range([0, 1000], {value: 500, step: 1, label: "Snapshot delay (ms):"});

const command_snapshot_delay = Generators.input(command_snapshot_delay_slider);

const command_options = Generators.input(command_options_input);

const command_pwm_slider = inputs_wide_range([0, 1.0], {value: 0.05, step: 0.001, label: "Command value:"});

const command_pwm_fraction = Generators.input(command_pwm_slider);

const command_timeout_slider = inputs_wide_range([0, max_timeout*millis_per_cycle], {value: 510, step: 5, label: "Command timeout (ms):"});

const command_timeout_millis = Generators.input(command_timeout_slider);

const command_angular_speed_slider = inputs_wide_range([0, max_angular_speed], {value: 1, step: 0.1, label: "Angular speed value (degrees/ms)"});

const command_angular_speed_degrees_div_ms = Generators.input(command_angular_speed_slider);

const command_angle_slider = inputs_wide_range([-180, 180], {value: 0, step: 1, label: "Command angle (degrees):"});

const command_angle_degrees = Generators.input(command_angle_slider);

const command_torque_current_slider = inputs_wide_range([0, max_drive_current], {value: 0.200, step: 0.010, label: "Command torque (Amps):"});

const command_torque_current_amps = Generators.input(command_torque_current_slider);

const command_power_slider = inputs_wide_range([0, max_drive_power], {value: 0.200, step: 0.010, label: "Command power (Watts):"});

const command_power_watts = Generators.input(command_power_slider);
```

```js
// Data stream output
// ------------------


const target_data_size = 4000 / millis_per_cycle;
const max_data_size = 2 * target_data_size;


let data = Mutable([]);

function reset_data(){
  data.value = [];
};

function push_data(readout){
  data.value.push(readout);
  if (data.value.length > max_data_size) data.value = data.value.slice(-target_data_size);
  data.value = data.value;
};

```


```js
// Control functions
// -----------------

const command_snapshot = command_options.includes("Take snapshot after command");

const command_timeout = Math.floor(command_timeout_millis * cycles_per_millisecond);

const command_pwm = Math.round(command_pwm_fraction * pwm_base);

const command_angle = degrees_to_angle_units(command_angle_degrees);

const command_angular_speed = degrees_per_millisecond_to_speed_units(command_angular_speed_degrees_div_ms);

const command_torque_current = Math.floor(command_torque_current_amps / current_conversion);

const command_power = convert_watts_to_power_units(command_power_watts);

async function send_command(command, options = {}){
  if (!motor_controller) return;
  
  await motor_controller.send_command({command, command_timeout, ...options});
}

let latest_stream_timeout = null;

function command_and_stream(command, options = {}, delay_ms = 0){
  if (latest_stream_timeout) clearTimeout(latest_stream_timeout);
  latest_stream_timeout = null;

  if (!motor_controller) return;

  latest_stream_timeout = setTimeout(async function(){
    try {
      // Start reading the data stream.
      await motor_controller.command_and_stream(
        {command, ...options},
        {readout_callback: push_data, ...options});

    } catch (error) {
      console.error("Error streaming data:", error);
    }
  }, delay_ms);
}

const data_request_buttons = Inputs.button(
  [
    ["Uninterrupted snapshot", function(){
      reset_data();
      command_and_stream(command_codes.GET_READOUTS_SNAPSHOT, {expected_code: command_codes.READOUT, expected_messages: history_size});
    }],
    ["Stream 3 phase data", function(){
      reset_data();
      command_and_stream(command_codes.STREAM_FULL_READOUTS, {expected_code: command_codes.FULL_READOUT, command_timeout: 1});
    }],
    ["STOP stream", async function(){
      await send_command(command_codes.STREAM_FULL_READOUTS, {command_timeout: 0});
    }],
  ],
  {label: "Read data"},
);

d3.select(data_request_buttons).selectAll("button").style("height", "4em");


const test_command_options = {
  command_value: command_pwm, 
  command_timeout: command_snapshot, 
  expected_messages: command_snapshot ? history_size : 0,
  expected_code: command_snapshot ? command_codes.READOUT : command_codes.FULL_READOUT,
};

async function test_command(command){
  if (command_snapshot){
    reset_data();
    command_and_stream(command, test_command_options)
  } else {
    await send_command(command, test_command_options);
  }
}

const test_buttons = Inputs.button(
  [
    ["Test all permutations", function(){
      test_command(command_codes.SET_STATE_TEST_ALL_PERMUTATIONS, test_command_options);
    }],
    ["Test ground short", function(){
      test_command(command_codes.SET_STATE_TEST_GROUND_SHORT, test_command_options);
    }],
    ["Test positive short", function(){
      test_command(command_codes.SET_STATE_TEST_POSITIVE_SHORT, test_command_options);
    }],
    ["Test U directions", function(){
      test_command(command_codes.SET_STATE_TEST_U_DIRECTIONS, test_command_options);
    }],
    ["Test U increasing", function(){
      test_command(command_codes.SET_STATE_TEST_U_INCREASING, test_command_options);
    }],
    ["Test U decreasing", function(){
      test_command(command_codes.SET_STATE_TEST_U_DECREASING, test_command_options);
    }],
    ["Test V increasing", function(){
      test_command(command_codes.SET_STATE_TEST_V_INCREASING, test_command_options);
    }],
    ["Test V decreasing", function(){
      test_command(command_codes.SET_STATE_TEST_V_DECREASING, test_command_options);
    }],
    ["Test W increasing", function(){
      test_command(command_codes.SET_STATE_TEST_W_INCREASING, test_command_options);
    }],
    ["Test W decreasing", function(){
      test_command(command_codes.SET_STATE_TEST_W_DECREASING, test_command_options);
    }],
  ],
  {label: "Test sequence"},
);

d3.select(test_buttons).selectAll("button").style("height", "4em");


async function snapshot_if_checked(command, options = {}){
  await send_command(command, {
    expected_code: command_codes.FULL_READOUT,
    ...options,
  });

  if (command_snapshot){
    command_and_stream(
      command_codes.GET_READOUTS_SNAPSHOT, 
      {
        expected_messages: history_size, 
        expected_code: command_codes.READOUT,
      },
      command_snapshot_delay,
    );
  }
}

const stop_buttons = Inputs.button(
  [
    ["Stop / Brake", async function(){
      await snapshot_if_checked(command_codes.SET_STATE_OFF);
    }],
    ["Freewheel", async function(){
      await snapshot_if_checked(command_codes.SET_STATE_FREEWHEEL);
    }],
  ],
  {label: "Stop Commands"},
);

d3.select(stop_buttons)
  .style("margin-top", "1em")
  .style("margin-bottom", "1em")
  .selectAll("button")
    .style("height", "5em")
    .style("font-weight", "bold")
    .style("color", "darkred");

const simple_drive_buttons = Inputs.button(
  [
    ["Hold U positive", async function(){
      await snapshot_if_checked(command_codes.SET_STATE_HOLD_U_POSITIVE, {command_value: command_pwm});
    }],
    ["Hold V positive", async function(){
      await snapshot_if_checked(command_codes.SET_STATE_HOLD_V_POSITIVE, {command_value: command_pwm});
    }],
    ["Hold W positive", async function(){
      await snapshot_if_checked(command_codes.SET_STATE_HOLD_W_POSITIVE, {command_value: command_pwm});
    }],
    ["Hold U negative", async function(){
      await snapshot_if_checked(command_codes.SET_STATE_HOLD_U_NEGATIVE, {command_value: command_pwm});
    }],
    ["Hold V negative", async function(){
      await snapshot_if_checked(command_codes.SET_STATE_HOLD_V_NEGATIVE, {command_value: command_pwm});
    }],
    ["Hold W negative", async function(){
      await snapshot_if_checked(command_codes.SET_STATE_HOLD_W_NEGATIVE, {command_value: command_pwm});
    }],
    ["Drive 6S +", async function(){
      await snapshot_if_checked(command_codes.SET_STATE_DRIVE_6_SECTOR, {command_value: +command_pwm});
    }],
    ["Drive 6S -", async function(){
      await snapshot_if_checked(command_codes.SET_STATE_DRIVE_6_SECTOR, {command_value: -command_pwm});
    }],
  ],
  {label: "Simple drive commands"},
);

d3.select(simple_drive_buttons).selectAll("button").style("height", "4em");


const advanced_drive_buttons = Inputs.button(
  [
    ["Set Angle", async function(){
      await snapshot_if_checked(command_codes.SET_ANGLE, {command_value: command_angle});
    }],
    ["Drive periodic +", async function(){
      await snapshot_if_checked(command_codes.SET_STATE_DRIVE_PERIODIC, {
        command_value: command_pwm, 
        command_second: +command_angular_speed, 
        command_third: command_angle,
      });
    }],
    ["Drive periodic -", async function(){
      await snapshot_if_checked(command_codes.SET_STATE_DRIVE_PERIODIC, {
        command_value: command_pwm, 
        command_second: -command_angular_speed, 
        command_third: command_angle,
      });
    }],
    ["Drive smooth +", async function(){
      await snapshot_if_checked(command_codes.SET_STATE_DRIVE_SMOOTH, {command_value: +command_pwm});
    }],
    ["Drive smooth -", async function(){
      await snapshot_if_checked(command_codes.SET_STATE_DRIVE_SMOOTH, {command_value: -command_pwm});
    }],
    ["Drive torque +", async function(){
      await snapshot_if_checked(command_codes.SET_STATE_DRIVE_TORQUE, {command_value: +command_torque_current});
    }],
    ["Drive torque -", async function(){
      await snapshot_if_checked(command_codes.SET_STATE_DRIVE_TORQUE, {command_value: -command_torque_current});
    }],
    ["Drive power +", async function(){
      await snapshot_if_checked(command_codes.SET_STATE_DRIVE_BATTERY_POWER, {command_value: +command_power});
    }],
    ["Drive power -", async function(){
      await snapshot_if_checked(command_codes.SET_STATE_DRIVE_BATTERY_POWER, {command_value: -command_power});
    }],
  ],
  {label: "Advanced drive commands"},
);
d3.select(advanced_drive_buttons).selectAll("button").style("height", "4em");

```




```js

// Plotting
// --------

const max_timeline_period = 2000; // ms

const history_duration = Math.ceil(history_size * millis_per_cycle);

const time_period_input = Inputs.range([1, max_timeline_period], {
  value: max_timeline_period,
  transform: Math.log,
  step: 0.5,
  label: "Time window Duration (ms):",
});

const time_offset_input = inputs_wide_range([0, 1.0], {
  value: 1.0, 
  step: 0.01,
  label: "Time window Offset (ms):",
});

const plot_curve_input = Inputs.radio(new Map([
  ["Connected steps", d3.curveStep],
  ["Interrupted steps", horizontal_step],
  ["Linear", d3.curveLinear],
  ["Smooth", d3.curveCatmullRom],
  ["Step after", d3.curveStepAfter],
  ["Step before", d3.curveStepBefore],
  ]),
  {
    value: d3.curveStep, 
    label: "Drawing options:",
  },
);

const plot_curve = Generators.input(plot_curve_input);


const timeline_position_input = plot_line({
  subtitle: "Time Window Selection",
  description: "Select the time window to plot; drag or resize. Click to see all.",
  width: 1200, height: 150,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Electric position (degrees)",
  y_domain: [-180, 180],
  y: "angle", 
  color: colors.angle,
  include_brush: true,
});

const timeline_position = Generators.input(timeline_position_input);

timeline_position_input.addEventListener("input", function(){
  const {selection} = timeline_position_input.value;  
  if (selection){
    time_period_input.value = (selection[1] - selection[0]) * max_timeline_period;
    const max_choice = (max_timeline_period - time_period_input.value) / max_timeline_period;
    time_offset_input.value = selection[0] / max_choice;
  } else {
    time_period_input.value = max_timeline_period;
    time_offset_input.value = 0;
  }
});

function merge_timeline_inputs(){
  const max_choice = (max_timeline_period - time_period_input.value) / max_timeline_period;

  merge_input_value(timeline_position_input, {
    selection: time_period_input.value === max_timeline_period ? null : [
      time_offset_input.value * max_choice,
      time_offset_input.value * max_choice + (time_period_input.value / max_timeline_period),
    ],
  });
}

time_period_input.addEventListener("input", merge_timeline_inputs);
time_offset_input.addEventListener("input", merge_timeline_inputs);
merge_timeline_inputs();

```

```js
function filter_window(data, time_domain){
  return data.filter((d) => d.time >= time_domain[0] && d.time <= time_domain[1]);
}

// Pick evenly spaced data points to pass to the plot; we can't draw more pixels than we have.
function sparsify(data, target_points = 1080){
  const max_plot_points = target_points;
  const plot_points_skip = Math.ceil(data.length / max_plot_points);
  return data.filter(({readout_index}) => readout_index % plot_points_skip === 0);
}

const timeline_end = data.length == 0 ? 0 : data[data.length - 1].time;
const timeline_start = data.length == 0 ? -16 : Math.max(data[0].time, timeline_end - max_timeline_period);
const timeline_period = timeline_end - timeline_start;
const timeline_domain = [timeline_start, timeline_end];

timeline_position_input.update({
  data: sparsify(filter_window(data, timeline_domain)), 
  x_domain: timeline_domain,
});


const selected_time_domain = !timeline_position.selection ? [timeline_start, timeline_end] : [
  timeline_position.selection[0] * timeline_period + timeline_start,
  timeline_position.selection[1] * timeline_period + timeline_start,
];


const data_in_time_window = sparsify(filter_window(data, selected_time_domain));



Object.values(monitoring_plots).forEach((plot) => plot.update({
  data: data_in_time_window, 
  x_domain: selected_time_domain,
}));
```


```js

const curve = plot_curve;

const plot_power = plot_lines({
  subtitle: "Power",
  description: "Power consumed by command_codes.",
  width: 1200, height: 300,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Power (W)",
  channels: [
    {y: "total_power", label: "Total Power", color: colors.sum},
    {
      y: "total_power_avg", label: "Total Power (2ms average)", color: d3.color(colors.sum).darker(1),
      draw_extra: setup_stdev_95({stdev: (d) => d.total_power_stdev}),
    },
    {y: "resistive_power", label: "Resistive Power", color: colors.web_current_magnitude},
    {y: "emf_power", label: "EMF Power", color: colors.angle},
    {
      y: "emf_power_avg", label: "EMF Power (2ms average)", color: d3.color(colors.angle).darker(1),
      draw_extra: setup_stdev_95({stdev: (d) => d.emf_power_stdev}),
    },
    {y: "inductive_power", label: "Inductive Power", color: colors.inductor_angle},
    
    {y: "web_total_power", label: "Total Power (computed online)", color: colors.web_angle},
    {y: "web_emf_power", label: "EMF Power (computed online)", color: colors.u},
    {y: "web_resistive_power", label: "Resistive Power (computed online)", color: colors.v},
    {y: "web_inductive_power", label: "Inductive Power (computed online)", color: colors.w},

  ],
  curve,
});


const plot_runtime_stats = plot_lines({
  subtitle: "Motor driver runtime stats",
  description: "Timing data for the motor driver interrupt routines and the main loop.",
  width: 1200, height: 150,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Update frequency (Hz)",
  channels: [
    {y: "tick_rate", label: "Tick rate", color: colors.sum},
    {y: "adc_update_rate", label: "ADC & PWM rate", color: colors.u},
  ],
  curve,
});

const plot_cycle_loop_stats = plot_lines({
  subtitle: "Motor driver cycle loop stats",
  description: "Timing data for the motor driver cycle loop routines.",
  width: 1200, height: 150,
  x: "time",
  x_label: "Time (ms)",
  y_label: "PWM counter value",
  y_domain: [0, pwm_period],
  channels: [
    {y: "cycle_start_tick", label: "Tick at start", color: colors.u},
    {y: "cycle_end_tick", label: "Tick at end", color: colors.v},
    {y: (d) => (pwm_period + d.cycle_end_tick - d.cycle_start_tick) % pwm_period , label: "Cycle duration", color: colors.w},
    {y: (d) => d.cycle_start_tick - pwm_base, label: "Ticks at start since mid cycle", color: d3.color(colors.u).brighter(1)},
  ],
  curve,
});



const plot_electric_position = plot_lines({
  subtitle: "Electric position",
  description: "Angular position of the rotor with respect to the electric phases, 0 when magnetic N is aligned with phase U.",
  width: 1200, height: 200,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Electric position (degrees)",
  y_domain: [-180, 180],
  channels: [
    {y: "angle", label: "Magnet Angle", color: colors.angle},
    {y: (d) => d.current_detected ? d.inductor_angle : null, label: "Inductor Angle", color: colors.web_angle},
    {y: (d) => d.web_current_magnitude > 0.010 ? d.web_inductor_angle : null, label: "Inductor Angle (computed online)", color: colors.inductor_angle},
    {
      y: (d) => d.emf_detected ? d.web_emf_voltage_angle : null, label: "EMF Voltage Angle (computed online)", color: colors.voltage_angle,
      draw_extra: setup_stdev_95({stdev: (d) => d.emf_angle_error_stdev}),
    },
  ],
  curve,
});

const plot_electric_offsets = plot_lines({
  subtitle: "Electric Offsets",
  description: "Offsets for the electric angles.",
  width: 1200, height: 200,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Angle (degrees)",
  channels: [
    {y: (d) => d.current_detected ? d.inductor_angle_offset : null, label: "Inductor Angle Offset", color: colors.inductor_angle},
    {
      y: (d) => d.emf_detected ? d.emf_voltage_angle_offset : null, label: "EMF Voltage Angle Offset (online - chip)", color: colors.voltage_angle,
      draw_extra: setup_stdev_95({stdev: (d) => d.emf_angle_error_stdev}),
    },
    {y: "angle_adjustment", label: "Magnet Angle Correction", color: d3.color(colors.angle).darker(1)},
    {y: "emf_angle_error_stdev", label: "EMF Angle Error (stdev)", color: d3.color(colors.voltage_angle).darker(1)},
  ],
  curve,
});


const plot_speed = plot_lines({
  subtitle: "Rotor Speed",
  description: "Angular speed of the rotor in degrees per millisecond.",
  width: 1200, height: 300,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Angular Speed (degrees/ms)",
  channels: [
    {y: "angular_speed", label: "Magnet Angular Speed", color: colors.angular_speed},
    {y: (d) => d.rotor_acceleration * 10, label: "Rotor Acceleration (10ms speed diff)", color: colors.angle_driven},
  ],
  curve,
});


const plot_measured_voltage = plot_lines({
  subtitle: "Measured Voltage",
  description: "Measured voltage values for VCC.",
  width: 1200, height: 150,
  x: "time",
  y_domain: [0, 24],
  x_label: "Time (ms)",
  y_label: "Voltage (V)",
  channels: [
    {y: "vcc_voltage", label: "VCC Voltage", color: colors.v},
  ],
  curve,
});

const plot_measured_temperature = plot_lines({
  subtitle: "Measured Temperature",
  description: "Measured temperature values for the MCU.",
  width: 1200, height: 150,
  x: "time",
  y_domain: [0, 100],
  x_label: "Time (ms)",
  y_label: "Temperature (°C)",
  channels: [
    {y: "temperature", label: "MCU Temp (inaccurate)", color: colors.w},
  ],
  curve,
});

const plot_measured_current = plot_lines({
  subtitle: "Measured Current",
  description: "Measured current values for each phase.",
  width: 1200, height: 400,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Current (A)",
  channels: [
    {y: "u_current", label: "Current U", color: colors.u},
    {y: "v_current", label: "Current V", color: colors.v},
    {y: "w_current", label: "Current W", color: colors.w},
    {y: "u_readout", label: "Current U (uncalibrated)", color: d3.color(colors.u).darker(1)},
    {y: "v_readout", label: "Current V (uncalibrated)", color: d3.color(colors.v).darker(1)},
    {y: "w_readout", label: "Current W (uncalibrated)", color: d3.color(colors.w).darker(1)},
    {y: "battery_current", label: "Battery Current", color: colors.other},
    {y: (d) => d.avg_current * 3, label: "Sum", color: colors.sum},
    {y: "ref_readout", label: "Reference Diff", color: colors.ref_readout},
  ],
  curve,
});


const plot_voltages = plot_lines({
  subtitle: "Voltages",
  description: "Measured voltage values for each phase.",
  width: 1200, height: 300,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Voltage (V)",
  channels: [
    {y: "u_drive_voltage", label: "Drive Voltage U", color: colors.u},
    {y: "v_drive_voltage", label: "Drive Voltage V", color: colors.v},
    {y: "w_drive_voltage", label: "Drive Voltage W", color: colors.w},
    {y: "u_R_voltage", label: "Resistive Voltage U", color: d3.color(colors.u).darker(1)},
    {y: "v_R_voltage", label: "Resistive Voltage V", color: d3.color(colors.v).darker(1)},
    {y: "w_R_voltage", label: "Resistive Voltage W", color: d3.color(colors.w).darker(1)},
  ],
  curve,
});

const plot_inferred_voltages = plot_lines({
  subtitle: "Inferred Voltage",
  description: html`Inferred EMF voltage values for each phase: ${tex`V_{emf} = IR + L(dI/dt) - V_{drive}`}.`,
  width: 1200, height: 300,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Voltage (V)",
  channels: [
    {y: "u_emf_voltage", label: "EMF Voltage U", color: colors.u},
    {y: "v_emf_voltage", label: "EMF Voltage V", color: colors.v},
    {y: "w_emf_voltage", label: "EMF Voltage W", color: colors.w},
    {y: "u_L_voltage", label: "Inductor Voltage U", color: d3.color(colors.u).brighter(1)},
    {y: "v_L_voltage", label: "Inductor Voltage V", color: d3.color(colors.v).brighter(1)},
    {y: "w_L_voltage", label: "Inductor Voltage W", color: d3.color(colors.w).brighter(1)},
  ],
  curve,
});


const plot_dq0_currents = plot_lines({
  subtitle: "DQ0 Currents",
  description: "DQ0 currents after Clarke and Park (direct-quadrature-zero) transforming the measured currents.",
  width: 1200, height: 400,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Current (A)",
  channels: [
    {y: "alpha_current", label: "Current Alpha", color: colors.alpha_current},
    {y: "beta_current", label: "Current Beta", color: colors.beta_current},
    {y: "current_magnitude", label: "Current Magnitude", color: colors_categories[2]},
    {y: "web_alpha_current", label: "Current Alpha (computed online)", color: d3.color(colors.alpha_current).brighter(1)},
    {y: "web_beta_current", label: "Current Beta (computed online)", color: d3.color(colors.beta_current).brighter(1)},
    {y: "steady_state_beta_current", label: "Steady State Beta Current", color: d3.color(colors.alpha_current).darker(1)},
    {y: "web_current_magnitude", label: "Current Magnitude (computed online)", color: colors.web_current_magnitude},
    {
      y: "web_current_magnitude_avg", label: "Current Magnitude (2ms average)", color: d3.color(colors.web_current_magnitude).brighter(1),
      draw_extra: setup_stdev_95({stdev: (d) => d.web_current_magnitude_stdev}),
    },
  ],
  curve,
});

const plot_dq0_voltages = plot_lines({
  subtitle: "DQ0 Voltages",
  description: "DQ0 voltages after Clarke and Park (direct-quadrature-zero) transforming the inferred voltages.",
  width: 1200, height: 400,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Voltage (V)",
  channels: [
    {y: "beta_emf_voltage", label: "EMF Voltage Beta", color: colors.beta_current},
    {y: "alpha_emf_voltage", label: "EMF Voltage Alpha", color: colors.alpha_current},
    {y: "emf_voltage_magnitude", label: "EMF Voltage Magnitude", color: colors.web_current_magnitude},
    {y: "emf_angle_error_stdev", label: "EMF Voltage Stdev", color: d3.color(colors.web_current_magnitude).darker(1)},
    {y: "web_alpha_emf_voltage", label: "Voltage Alpha (computed online)", color: d3.color(colors.alpha_current).brighter(1)},
    {y: "web_beta_emf_voltage", label: "Voltage Beta (computed online)", color: d3.color(colors.beta_current).brighter(1)},
    {y: "web_emf_voltage_magnitude", label: "Voltage Magnitude (computed online)", color: colors.web_current_magnitude},
    {
      y: "web_emf_voltage_magnitude_avg", label: "Voltage Magnitude (2ms average)", color: d3.color(colors.web_current_magnitude).brighter(1),
      draw_extra: setup_stdev_95({stdev: (d) => d.web_emf_voltage_magnitude_stdev}),
    },
    {y: "drive_voltage_magnitude", label: "Drive Voltage Magnitude", color: colors_categories[2]},
  ],
  curve,
});

const plot_pwm_settings = plot_lines({
  subtitle: "PWM Settings",
  description: "The PWM value currently set for each phase.",
  width: 1200, height: 300,
  x: "time",
  x_label: "Time (ms)",
  y_label: "PWM",
  y_domain: [0, pwm_base],
  channels: [
    {y: "u_pwm", label: "PWM U", color: colors.u},
    {y: "v_pwm", label: "PWM V", color: colors.v},
    {y: "w_pwm", label: "PWM W", color: colors.w},
    {y: "live_max_pwm", label: "Live Max PWM", color: colors_categories[0]},
  ],
  curve,
});

const plot_readout_flags = plot_lines({
  subtitle: "Readout Flags",
  description: "Flags indicating the state of the readouts.",
  width: 1200, height: 150,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Flag setting",
  channels: [
    {y: "emf_detected", label: "EMF detected", color: colors_categories[3]},
    {y: "emf_fix", label: "EMF position fix", color: colors_categories[4]},
    {y: "current_detected", label: "Current detected", color: colors_categories[6]},
    {y: "angle_fix", label: "Rotor position fix", color: colors_categories[7]},
    {y: "incorrect_rotor_angle", label: "Incorrect Rotor Angle", color: colors_categories[8]},
    {y: "rotor_direction_flip_imminent", label: "Rotor Direction Flip Imminent", color: colors_categories[12]},
    {y: "hall_u", label: "Hall U", color: colors.u},
    {y: "hall_v", label: "Hall V", color: colors.v},
    {y: "hall_w", label: "Hall W", color: colors.w},
  ],
  curve,
});

const plot_motor_values = plot_lines({
  subtitle: "Motor Values",
  description: "Various values related to the motor's operation.",
  width: 1200, height: 400,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Value",
  channels: [
    {y: "motor_constant", label: "Motor Constant (EMF and torque)", color: colors.angle},
    {y: "rotations", label: "Rotations", color: colors_categories[1]},
    {y: "debug_1", label: "Debug 1", color: colors.v},
    {y: "debug_2", label: "Debug 2", color: colors.w},
  ],
  curve,
});

const monitoring_plots = {
  plot_power,
  plot_runtime_stats,
  plot_cycle_loop_stats,
  plot_electric_position,
  plot_electric_offsets,
  plot_speed,
  plot_measured_voltage,
  plot_measured_temperature,
  plot_measured_current,
  plot_voltages,
  plot_inferred_voltages,
  plot_dq0_currents,
  plot_dq0_voltages,
  plot_pwm_settings,
  plot_readout_flags,
  plot_motor_values,
};

autosave_inputs(monitoring_plots);
```


```js
// Current calibration
// -------------------

const current_calibration_pwm_slider = inputs_wide_range([0, pwm_base], {value: pwm_base, step: 1.0, label: "Calibration max PWM:"});

const current_calibration_pwm = Generators.input(current_calibration_pwm_slider);
```

```js


function stringify_active_current_calibration() {
  return `motor_controller.current_calibration = ${JSON.stringify(motor_controller?.current_calibration, null, 2)}`;
}

const active_current_calibration_table =  Mutable(stringify_active_current_calibration());

const default_current_calibration_result = {results: [], ...compute_current_calibration([])};

const current_calibration_buttons = !motor_controller ? html`<p>Not connected to motor!</p>` : Inputs.button(
  [
    ["Start Current Calibration", wait_previous(async function(value){
      const calibration_data = await run_current_calibration(motor_controller, current_calibration_pwm);
      const results = [...value.results, calibration_data];
      return {results, ...compute_current_calibration(results)};
    })],
    ["Reset Results", wait_previous(async function(value){
      return default_current_calibration_result;
    })],
    ["Use Locally", wait_previous(async function(value){
      if (!value.current_calibration) return value;
      motor_controller.current_calibration = value.current_calibration;
      active_current_calibration_table.value = stringify_active_current_calibration();
      return value;
    })],
    ["Use Default Locally", wait_previous(async function(value){
      motor_controller.current_calibration = default_current_calibration;
      active_current_calibration_table.value = stringify_active_current_calibration();
      return value;
    })],
    ["Upload to Driver", wait_previous(async function(value){
      const current_calibration = value.current_calibration ?? motor_controller.current_calibration;
      await motor_controller.upload_current_calibration(current_calibration);
      active_current_calibration_table.value = stringify_active_current_calibration();
      return value;
    })],
    ["Reload from Driver", wait_previous(async function(value){
      await motor_controller.load_current_calibration();
      active_current_calibration_table.value = stringify_active_current_calibration();
      return value;
    })],
  ],
  {
    label: "Current Calibration",
    value: default_current_calibration_result
  },
);

d3.select(current_calibration_buttons).style("width", "100%");

const current_calibration = !motor_controller ? default_current_calibration_result : Generators.input(current_calibration_buttons);
```



```js

// Write out the current calibration results in copyable format.
const current_calibration_table = `current_calibration = ${JSON.stringify(current_calibration?.current_calibration, null, 2)}`;

// Select which of the calibration runs to display.

const current_calibration_result_to_display_input = Inputs.select(d3.range(current_calibration.results.length), {
  value: current_calibration.results.length - 1,
  label: "Select calibration result to display:",
});
const current_calibration_result_to_display = Generators.input(current_calibration_result_to_display_input);
```

```js
const current_calibration_result = current_calibration.results[current_calibration_result_to_display];

const current_calibration_iterations = current_calibration_result?.iterations ?? [];

// Select the optimization iteration to display.
const current_calibration_optimization_iteration_input = !current_calibration_result ? 
  html`<p>No optimization iterations available.</p>` : 
  inputs_wide_range(
    [0, current_calibration_iterations.length - 1], {
    step: 1,
    label: "Select optimization iteration to display:",
  });


const current_calibration_optimization_phase_input = Inputs.radio(
  new Map([
    ["U positive", "u_positive_gradients"],
    ["U negative", "u_negative_gradients"],
    ["V positive", "v_positive_gradients"],
    ["V negative", "v_negative_gradients"],
    ["W positive", "w_positive_gradients"],
    ["W negative", "w_negative_gradients"],
  ]),
  {
    label: "Select calibration phase to display:",
  },
);

const current_calibration_optimization_iteration = Generators.input(current_calibration_optimization_iteration_input);
const current_calibration_optimization_phase = Generators.input(current_calibration_optimization_phase_input);

set_input_value(current_calibration_optimization_iteration_input, current_calibration_iterations?.length - 1);
set_input_value(current_calibration_optimization_phase_input, "u_positive_gradients");

```

```js


const current_calibration_plot = plot_lines({
  data: current_calibration_result?.sample,
  subtitle: "Current Calibration",
  description: "Current calibration results for each phase.",
  width: 1200, height: 400,
  x_domain: [0, history_size * millis_per_cycle],
  x: "time",
  x_label: "Time (ms)",
  y_label: "Current (A)",
  channels: [
    {y: "u_positive_uncalibrated_current", label: "U positive", color: colors.u},
    {y: "u_negative_uncalibrated_current", label: "U negative", color: d3.color(colors.u).darker(1)},
    {y: "v_positive_uncalibrated_current", label: "V positive", color: colors.v},
    {y: "v_negative_uncalibrated_current", label: "V negative", color: d3.color(colors.v).darker(1)},
    {y: "w_positive_uncalibrated_current", label: "W positive", color: colors.w},
    {y: "w_negative_uncalibrated_current", label: "W negative", color: d3.color(colors.w).darker(1)},
    {
      y: "expected",
      draw_extra: setup_faint_area({y0: 0.0, y1: "expected"}),
      label: "Expected", color: "gray",
    }
  ],
});

const current_calibration_iteration = current_calibration_iterations[current_calibration_optimization_iteration];
const current_calibration_gradients = current_calibration_iteration?.[current_calibration_optimization_phase];

// Write out the current calibration results in copyable format.
const current_calibration_iteration_table = `iteration_current_calibration = ${JSON.stringify(current_calibration_iterations[current_calibration_optimization_iteration]?.current_calibration, null, 2)}`;

const current_calibration_optimizing_plot = plot_lines({
  data: current_calibration_gradients,
  subtitle: "Current Calibration - Optimizing resistance & inductance",
  description: "Current calibration optimization results for each phase.",
  width: 1200, height: 300,
  x_domain: [0, history_size * millis_per_cycle],
  x: "time",
  x_label: "Time (ms)",
  y_label: "Voltage (V)",
  channels: [
    {y: "resistance_drop", label: "Resistance Drop", color: colors.u},
    {y: "inductance_drop", label: "Inductance Drop", color: colors.v},
    {y: "drive_voltage", label: "Drive Voltage", color: colors.w},
    {
      y: "residual",
      draw_extra: setup_faint_area({y0: 0.0, y1: "residual"}),
      label: "Residual", color: "grey",
    },
  ],
});

const current_calibration_optimizing_gradients_plot = plot_lines({
  data: current_calibration_gradients,
  subtitle: "Current Calibration - resistance & inductance gradients",
  description: "Current calibration optimization results for each phase.",
  width: 1200, height: 300,
  x_domain: [0, history_size * millis_per_cycle],
  x: "time",
  x_label: "Time (ms)",
  y_label: "Factor Change",
  channels: [
    {
      y: "current_factor_gradient", label: "Current Factor Gradient", color: colors.u,
      draw_extra: setup_stdev_95({stdev: (d) => Math.sqrt(d.current_factor_variance)}),
    },
    {
      y: "inductance_factor_gradient", label: "Inductance Factor Gradient", color: colors.v,
      draw_extra: setup_stdev_95({stdev: (d) => Math.sqrt(d.inductance_factor_variance)}),
    },
  ],
});

const current_calibration_positive_mean_plot = plot_lines({
  data: current_calibration.stats,
  subtitle: "Mean Response - Positive",
  description: "Current calibration mean results for each phase driven positive.",
  width: 1200, height: 300,
  x_domain: [0, history_size * millis_per_cycle],
  x: "time",
  x_label: "Time (ms)",
  y_label: "Current (A)",
  channels: [
    {
      y: "u_positive", 
      draw_extra: setup_stdev_95({stdev: "u_positive_stdev"}),
      label: "U positive mean", color: colors.u,
    },
    {
      y: "v_positive",
      draw_extra: setup_stdev_95({stdev: "v_positive_stdev"}),
      label: "V positive mean", color: colors.v,
    },
    {
      y: "w_positive",
      draw_extra: setup_stdev_95({stdev: "w_positive_stdev"}),
      label: "W positive mean", color: colors.w,
    },
    {
      y: "expected",
      draw_extra: setup_stdev_95({stdev: "expected_stdev"}),
      label: "Expected", color: "gray",
    },
  ],
});

const current_calibration_negative_mean_plot = plot_lines({
  data: current_calibration.stats,
  subtitle: "Mean Response - Negative (inverted)",
  description: "Current calibration mean results for each phase driven negative.",
  width: 1200, height: 300,
  x_domain: [0, history_size * millis_per_cycle],
  x: "time",
  x_label: "Time (ms)",
  y_label: "Current (A)",
  channels: [
    {
      y: "u_negative", 
      draw_extra: setup_stdev_95({stdev: "u_negative_stdev"}),
      label: "U negative mean", color: colors.u,
    },
    {
      y: "v_negative",
      draw_extra: setup_stdev_95({stdev: "v_negative_stdev"}),
      label: "V negative mean", color: colors.v,
    },
    {
      y: "w_negative",
      draw_extra: setup_stdev_95({stdev: "w_negative_stdev"}),
      label: "W negative mean", color: colors.w,
    },
    {
      y: "expected",
      draw_extra: setup_stdev_95({stdev: "expected_stdev"}),
      label: "Expected", color: "gray",
    },
  ],
});

autosave_inputs({
  current_calibration_plot,
  current_calibration_optimizing_plot,
  current_calibration_optimizing_gradients_plot,
  current_calibration_positive_mean_plot,
  current_calibration_negative_mean_plot,
});
```


```js
// Control Parameters
// -------------------

function stringify_active_control_parameters() {
  return `motor_controller.control_parameters = ${JSON.stringify(motor_controller?.control_parameters, null, 2)}`;
}

let active_control_parameters_table =  Mutable(stringify_active_control_parameters());

const control_parameters_input = Object.fromEntries(
  [
    ["rotor_angle_ki", "Rotor Angle KI"],
    ["rotor_angular_speed_ki", "Rotor Angular Speed KI"],
    ["rotor_acceleration_ki", "Rotor Acceleration KI"],
    ["motor_constant_ki", "Motor Constant KI"],
    ["resistance_ki", "Resistance KI"],
    ["inductance_ki", "Inductance KI"],
    ["max_pwm_change", "Maximum PWM Change per cycle"],
    ["max_angle_change", "Maximum Angle Change per cycle"],
    ["min_emf_voltage", "Minimum EMF Voltage for detection"],
    ["min_emf_speed", "Minimum EMF Speed for detection"],
    ["lead_angle_control_ki", "Lead Angle Control KI"],
    ["torque_control_ki", "Torque Control KI"],
    ["battery_power_control_ki", "Battery Power Control KI"],
    ["speed_control_ki", "Speed Control KI"],
    ["probing_angular_speed", "Probing Angular Speed"],
    ["probing_max_pwm", "Probing Max PWM"],
    ["emf_angle_error_variance_threshold", "EMF Angle Correction Variance Threshold"],
    ["min_emf_for_motor_constant", "Minimum EMF to compute the motor constant"],
  ].map(([key, label]) => [
    key, 
    Inputs.number([], {
      label,
      value: motor_controller?.control_parameters?.[key],
    })
  ])
);

function show_active_control_parameters() {
  active_control_parameters_table.value = stringify_active_control_parameters();
  Object.entries(control_parameters_input).forEach(([key, input]) => {
    input.value = motor_controller?.control_parameters?.[key];
  });
}

let control_parameters_buttons = !motor_controller ? html`<p>Motor controller not connected.</p>` : Inputs.button(
  [
    ["Upload to Driver", wait_previous(async function(value){
      const control_parameters = Object.fromEntries(Object.entries(control_parameters_input).map(([key, input]) => [key, input.value]));
      await motor_controller.upload_control_parameters(control_parameters);
      show_active_control_parameters();
      return value;
    })],
    ["Reload from Driver", wait_previous(async function(value){
      await motor_controller.load_control_parameters();
      show_active_control_parameters();
      return value;
    })],
    ["Reset to Defaults", wait_previous(async function(value){
      await motor_controller.reset_control_parameters();
      show_active_control_parameters();
      return value;
    })],
  ],
  {
    label: "Control Parameters",
    value: motor_controller?.control_parameters ?? {},
  },
);

```


```js
const flash_buttons = !motor_controller ? html`<p>Not connected to motor!</p>` : Inputs.button(
  [
    ["Commit to Flash", async function(){
      await motor_controller.send_command({command: command_codes.SAVE_SETTINGS_TO_FLASH});
    }],
  ],
  {
    label: "Flash Memory",
  },
);

```

```js
let unit_test_results = Mutable([]);

async function command_unit_test(test_code, subtitle){
  const expected = unit_test_expected[test_code];

  const output = (await motor_controller.command_and_read(
    {command: test_code},
    {expected_messages: 1, expected_code: command_codes.UNIT_TEST_OUTPUT},
  ))[0];

  const passed = output == expected;

  // Display the output and expected result side by side underneath a title with the 
  // test name and whether it passed or failed. Use pre tags for the output and expected values.
  const displayed_result = html`<div>
    <h3>${subtitle} : ${passed ? 
      html`<span style='color: steelblue;'>PASS</span>` : 
      html`<span style='color: red;'>FAIL</span>`
    }</h3>
    ${passed ? "" : html`
      <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 1em;">
        <div>
          <strong>Output:</strong> <pre>${output}</pre>
        </div>
        <div>
          <strong>Expected:</strong> <pre>${expected}</pre>
        </div>
      </div>`
    }
    </div>`;

  unit_test_results.value = [...unit_test_results.value, displayed_result];

  return passed;
}

const unit_test_buttons = !motor_controller ? html`<p>Not connected to motor!</p>` : Inputs.button(
  [
    ["Run All Tests", async function(){
      unit_test_results.value = [];

      const all_passed = [
        await command_unit_test(command_codes.RUN_UNIT_TEST_FUNKY_ATAN, "Funky atan2"),
        await command_unit_test(command_codes.RUN_UNIT_TEST_FUNKY_ATAN_PART_2, "Funky atan2 part 2"),
        await command_unit_test(command_codes.RUN_UNIT_TEST_FUNKY_ATAN_PART_3, "Funky atan2 part 3"),
      ].reduce((acc, passed) => acc && passed, true);

      unit_test_results.value = [
        html`<h3>${all_passed ? 
          html`<span style='color: steelblue;'>All tests passed!</span>` : 
          html`<span style='color: red;'>Some tests failed!</span>`
        }</h3>`,
        ...unit_test_results.value,
      ];
    }],
  ],
  {
    label: "Unit Test Functions",
  },
);

```

```js
// Imports
// -------

import {plot_lines, plot_line, setup_faint_area, horizontal_step, setup_stdev_95, draw_line} from "./components/plotting_utils.js";

import {localStorage, get_stored_or_default, clear_stored_data} from "./components/local_storage.js";

import {round, timeout_promise, wait}  from "./components/utils.js";

import {
  enabled_checkbox, autosave_inputs, any_checked_input, 
  set_input_value, merge_input_value, wait_previous,
  inputs_wide_range,  
} from "./components/input_utils.js";

import {interpolate_degrees, normalize_degrees} from "./components/angular_math.js";

import {command_codes, connect_usb_motor_controller, MotorController} from "./components/motor_controller.js";

import {run_current_calibration, compute_current_calibration} from "./components/motor_current_calibration.js";


import {
  cycles_per_millisecond, millis_per_cycle, max_timeout, angle_base, pwm_base, pwm_period, 
  history_size, default_current_calibration, max_calibration_current,
  degrees_to_angle_units, degrees_per_millisecond_to_speed_units,
  current_conversion, max_drive_current, max_drive_power, max_angular_speed,
  convert_power_units_to_watts, convert_watts_to_power_units,
} from "./components/motor_constants.js";

import {unit_test_expected} from "./components/motor_unit_tests.js";

```