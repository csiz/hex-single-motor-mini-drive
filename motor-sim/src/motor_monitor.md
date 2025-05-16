---
title: Motor monitor
---
<main class="hero">

Motor Commands
--------------


<div>${connect_buttons}</div>
<div>${connection_status}</div>
<div>${data_request_buttons}</div>
<div>${command_options_input}</div>
<div>${test_buttons}</div>
<div>${command_buttons}</div>
<div>
  ${command_pwm_slider}
  ${command_timeout_slider}
  ${command_leading_angle_slider}
</div>

Motor Driving Data
------------------
<div class="card tight">
<p>Controls for the plotting time window:</p>
  <span>${time_period_input}</span>
  <span>${time_offset_input}</span>
  <span>${plot_options_input}</span>
  <span>${timeline_position_input}</span>
</div>
<div class="card tight">${plot_power}</div>
<div class="card tight">${plot_runtime_stats}</div>
<div class="card tight">${plot_cycle_loop_stats}</div>
<div class="card tight">${plot_electric_position}</div>
<div class="card tight">${plot_speed}</div>
<div class="card tight">${plot_measured_voltage}</div>
<div class="card tight">${plot_measured_temperature}</div>
<div class="card tight">${plot_measured_current}</div>
<div class="card tight">${plot_dq0_currents}</div>
<div class="card tight">${plot_dq0_voltages}</div>
<div class="card tight">${plot_inferred_voltages}</div>
<div class="card tight">${plot_pwm_settings}</div>


Position Calibration Procedures
-------------------------------

<div class="card tight">
  <div>${position_calibration_buttons}</div>
  <div>Number of calibration data sets: ${position_calibration.results.length}</div>
</div>
<div class="card tight">
  <h3>Position Calibration Results</h3>
  <div>
    <p>Position calibration angles:</p>
    <pre>${position_calibration_table}</pre>
  </div>
  <div>${position_calibration_result_to_display_input}</div>
  <div>${position_calibration_pos_plot}</div>
  <div>${position_calibration_pos_speed_plot}</div>
  <div>${position_calibration_neg_plot}</div>
  <div>${position_calibration_neg_speed_plot}</div>
  <p>Angles where the hall sensors toggle states (hall sector transitions):</p>
  <div>${position_calibration_transitions_table}</div>
  <p>Mid points of the hall sensor sectors:</p>
  <div>${position_calibration_centers_table}</div>
  <h3>Position Calibration Statistics</h3>
  <p>Hall sensor transitions when driving in the positive direction vs the negative direction:</p>
  <div>${position_calibration_sensor_spans_table}</div>
  <p>Hall sensor hysterisis (difference between the positive and negative transition) & actual locations (ideally they are 0, 120 -120 degrees):</p>
  <div>${position_calibration_sensor_locations_table}</div>
</div>


Current Calibration Procedures
------------------------------

<div class="card tight">
  <div>${current_calibration_buttons}</div>
  <div>Number of calibration data sets: ${current_calibration.results.length}</div>
</div>
<div class="card tight">
  <h3>Current Calibration Results</h3>
  <div>
    <p>Phase correction factors:</p>
    <pre>${current_calibration_table}</pre>
  </div>
  <div>${current_calibration_interpolate_plot}</div>
  <div>${current_calibration_result_to_display_input}</div>
  <div>${current_calibration_plot}</div>
</div>
<div class="card tight">  
  <h3>Current Calibration Statistics</h3>
  <div>${current_calibration_positive_mean_plot}</div>
  <div>${current_calibration_negative_mean_plot}</div>
</div>


</main>

```js

const stdev_95_z_score = 1.959964; // 95% confidence interval for normal distribution


const colors = {
  u: "rgb(117, 112, 179)",
  v: "rgb(217, 95, 2)",
  w: "rgb(231, 41, 138)",
  ref_diff: "rgb(102, 102, 102)",
  sum: "black",
  angle: "black",
  motor_angle: "rgb(39, 163, 185)",
  current_magnitude: "rgb(197, 152, 67)",
  current_angle: "rgb(102, 166, 30)",
  voltage_angle: d3.color("rgb(102, 166, 30)").darker(1),
  angle_if_breaking: d3.color("rgb(102, 166, 30)").darker(2),
  angular_speed: "black",
  current_alpha: "rgb(199, 0, 57)",
  current_beta: "rgb(26, 82, 118)",
  other: "rgb(27, 158, 119)",
};


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

    motor_controller.value = await connect_usb_motor_controller();

    motor_controller.value.current_calibration = get_stored_or_default("current_calibration", current_calibration_default);

    connection_status.value = html`<pre>Connected, waiting for data.</pre>`;

    await motor_controller.value.reading_loop(display_connection_stats);
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
    ["Reset data & inputs", () => (clear_stored_data(), location.reload())],
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

const command_options = Generators.input(command_options_input);

const command_pwm_slider = Inputs.range([0, 1], {value: 0.2, step: 0.05, label: "Command value:"});

const command_pwm_fraction = Generators.input(command_pwm_slider);

const command_timeout_slider = Inputs.range([0, max_timeout*millis_per_cycle], {value: 2000, step: 5, label: "Command timeout (ms):"});

const command_timeout_millis = Generators.input(command_timeout_slider);

const command_leading_angle_slider = Inputs.range([-180, 180], {value: 95, step: 1, label: "Leading angle (degrees):"});

const command_leading_angle_degrees = Generators.input(command_leading_angle_slider);
```



```js
// Data stream output
// ------------------


const target_data_size = 2500 / millis_per_cycle;
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


const command_timeout = Math.floor(command_timeout_millis * cycles_per_millisecond);
const command_pwm = Math.floor(command_pwm_fraction * pwm_base);
const command_leading_angle = Math.floor(angle_base + angle_base * command_leading_angle_degrees / 360) % angle_base;

async function command(command, options = {}){
  if (!motor_controller) return;
  
  await motor_controller.send_command({command, command_timeout, command_pwm, command_leading_angle, ...options});
}

let latest_stream_timeout = null;

function command_and_stream(delay_ms, command, options = {}){
  if (latest_stream_timeout) clearTimeout(latest_stream_timeout);
  latest_stream_timeout = null;

  if (!motor_controller) return;

  latest_stream_timeout = setTimeout(async function(){
    try {
      reset_data();
      // Start reading the data stream.
      await motor_controller.command_and_stream(
        {command, command_timeout: 0, command_pwm: 0, command_leading_angle: 0, ...options},
        {readout_callback: push_data, ...options});

    } catch (error) {
      console.error("Error streaming data:", error);
    }
  }, delay_ms);
}

const data_request_buttons = Inputs.button(
  [
    ["ADC snapshot", function(){
      command_and_stream(0, command_codes.GET_READOUTS_SNAPSHOT, {expected_messages: history_size});
    }],
    ["ADC stream", function(){
      command_and_stream(0, command_codes.STREAM_FULL_READOUTS, {expected_code: command_codes.FULL_READOUT, command_timeout: 1});
    }],
    ["STOP stream", async function(){
      await command(command_codes.STREAM_FULL_READOUTS, {command_timeout: 0});
    }],
  ],
  {label: "Read data"},
);

d3.select(data_request_buttons).selectAll("button").style("height", "4em");


function snapshot_if_checked(delay_ms, code = null){
  if (command_options.includes("Take snapshot after command")){
    command_and_stream(delay_ms, code ?? command_codes.GET_READOUTS_SNAPSHOT, {expected_messages: history_size});
  } else {
    if (code) command(code);
  }
}

const test_buttons = Inputs.button(
  [
    ["Test all permutations", function(){
      snapshot_if_checked(0, command_codes.SET_STATE_TEST_ALL_PERMUTATIONS);
    }],
    ["Test ground short", function(){
      snapshot_if_checked(0, command_codes.SET_STATE_TEST_GROUND_SHORT);
    }],
    ["Test positive short", function(){
      snapshot_if_checked(0, command_codes.SET_STATE_TEST_POSITIVE_SHORT);
    }],
    ["Test U directions", function(){
      snapshot_if_checked(0, command_codes.SET_STATE_TEST_U_DIRECTIONS);
    }],
    ["Test U increasing", function(){
      snapshot_if_checked(0, command_codes.SET_STATE_TEST_U_INCREASING);
    }],
    ["Test U decreasing", function(){
      snapshot_if_checked(0, command_codes.SET_STATE_TEST_U_DECREASING);
    }],
    ["Test V increasing", function(){
      snapshot_if_checked(0, command_codes.SET_STATE_TEST_V_INCREASING);
    }],
    ["Test V decreasing", function(){
      snapshot_if_checked(0, command_codes.SET_STATE_TEST_V_DECREASING);
    }],
    ["Test W increasing", function(){
      snapshot_if_checked(0, command_codes.SET_STATE_TEST_W_INCREASING);
    }],
    ["Test W decreasing", function(){
      snapshot_if_checked(0, command_codes.SET_STATE_TEST_W_DECREASING);
    }],
    ["Test V increasing", function(){
      snapshot_if_checked(0, command_codes.SET_STATE_TEST_V_INCREASING);
    }],
    ["Test V decreasing", function(){
      snapshot_if_checked(0, command_codes.SET_STATE_TEST_V_DECREASING);
    }],
    ["Test W increasing", function(){
      snapshot_if_checked(0, command_codes.SET_STATE_TEST_W_INCREASING);
    }],
    ["Test W decreasing", function(){
      snapshot_if_checked(0, command_codes.SET_STATE_TEST_W_DECREASING);
    }],
  ],
  {label: "Test sequence"},
);



d3.select(test_buttons).selectAll("button").style("height", "4em");

const command_buttons = Inputs.button(
  [
    ["Stop", async function(){
      await command(command_codes.SET_STATE_OFF);
      snapshot_if_checked(0);
    }],
    ["Drive +", async function(){
      await command(command_codes.SET_STATE_DRIVE_POS);
      snapshot_if_checked(500);
    }],
    ["Drive -", async function(){
      await command(command_codes.SET_STATE_DRIVE_NEG);
      snapshot_if_checked(500);
    }],
    ["Drive smooth +", async function(){
      await command(command_codes.SET_STATE_DRIVE_SMOOTH_POS);
      snapshot_if_checked(500);
    }],
    ["Drive smooth -", async function(){
      await command(command_codes.SET_STATE_DRIVE_SMOOTH_NEG);
      snapshot_if_checked(500);
    }],
    ["Freewheel", async function(){
      await command(command_codes.SET_STATE_FREEWHEEL);
      snapshot_if_checked(0);
    }],
    ["Hold U positive", async function(){
      await command(command_codes.SET_STATE_HOLD_U_POSITIVE);
      snapshot_if_checked(500);
    }],
    ["Hold V positive", async function(){
      await command(command_codes.SET_STATE_HOLD_V_POSITIVE);
      snapshot_if_checked(500);
    }],
    ["Hold W positive", async function(){
      await command(command_codes.SET_STATE_HOLD_W_POSITIVE);
      snapshot_if_checked(500);
    }],
    ["Hold U negative", async function(){
      await command(command_codes.SET_STATE_HOLD_U_NEGATIVE);
      snapshot_if_checked(500);
    }],
    ["Hold V negative", async function(){
      await command(command_codes.SET_STATE_HOLD_V_NEGATIVE);
      snapshot_if_checked(500);
    }],
    ["Hold W negative", async function(){
      await command(command_codes.SET_STATE_HOLD_W_NEGATIVE);
      snapshot_if_checked(500);
    }],
  ],
  {label: "Commands"},
);

d3.select(command_buttons).selectAll("button").style("height", "4em");

```




```js

// Plotting
// --------

const max_time_period = 2000; // ms

const history_duration = Math.ceil(history_size * millis_per_cycle);

const time_period_input = Inputs.range([1, max_time_period], {
  value: history_duration,
  transform: Math.log,
  step: 0.5,
  label: "Time window Duration (ms):",
});

const time_offset_input = Inputs.range([0, 1.0], {
  value: 1.0, 
  step: 0.01,
  label: "Time window Offset (ms):",
});
d3.select(time_offset_input).select("div").style("width", "640px");
d3.select(time_offset_input).select("input[type=range]").style("width", "100em");


const plot_options_input = Inputs.checkbox(
  ["Connected lines"],
  {
    value: ["Connected lines"],
    label: "Plot options:",
  },
);

const plot_options = Generators.input(plot_options_input);


const timeline_position_input = plot_line({
  subtitle: "Time Window Selection",
  description: "Select the time window to plot; drag or resize. Click to see all.",
  width: 1200, height: 150,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Electric position (degrees)",
  y_domain: [-180, 180],
  y: "motor_angle", 
  color: colors.motor_angle,
  include_brush: true,
});

const timeline_position = Generators.input(timeline_position_input);

timeline_position_input.addEventListener("input", function(){
  const {selection} = timeline_position_input.value;  
  if (selection){
    time_period_input.value = (selection[1] - selection[0]) * max_time_period;
    const max_choice = (max_time_period - time_period_input.value) / max_time_period;
    time_offset_input.value = selection[0] / max_choice;
  } else {
    time_period_input.value = max_time_period;
    time_offset_input.value = 0;
  }
});

function merge_timeline_inputs(){
  const max_choice = (max_time_period - time_period_input.value) / max_time_period;

  merge_input_value(timeline_position_input, {
    selection: time_period_input.value === max_time_period ? null : [
      time_offset_input.value * max_choice,
      time_offset_input.value * max_choice + (time_period_input.value / max_time_period),
    ],
  });
}

time_period_input.addEventListener("input", merge_timeline_inputs);
time_offset_input.addEventListener("input", merge_timeline_inputs);
merge_timeline_inputs();

```

```js
const data_time_end = (data.length == 0 ? 0 : data[data.length - 1].time);
const data_time_start = data_time_end - max_time_period;

timeline_position_input.update({
  data: sparsify(data.filter((d) => d.time >= data_time_start)), 
  x_domain: [data_time_start, data_time_end],
});
```


```js

const curve = plot_options.includes("Connected lines") ? d3.curveStep : horizontal_step;

const plot_power = plot_lines({
  subtitle: "Power",
  description: "Power consumed by command_codes.",
  width: 1200, height: 150,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Power (W)",
  channels: [
    {y: "total_power", label: "Total Power", color: colors.sum},
    {y: "torque", label: "Torque", color: colors.motor_angle},
    {y: "hold", label: "Hold", color: colors.current_angle},
    {y: "resistive_power", label: "Resistive Power", color: colors.current_magnitude},
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
    {y: "hall_unobserved_rate", label: "Hall overflow rate", color: colors.v},
    {y: "hall_observed_rate", label: "Hall trigger rate", color: colors.w},
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
  width: 1200, height: 150,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Electric position (degrees)",
  y_domain: [-180, 180],
  channels: [
    {
      y: "angle", label: "Angle", color: colors.angle,
      draw_extra: setup_faint_area({
        y0: (d) => d.angle - stdev_95_z_score * d.angle_stdev, 
        y1: (d) => d.angle + stdev_95_z_score * d.angle_stdev,
      }),
    },
    {y: "motor_angle", label: "Motor Angle", color: colors.motor_angle},
    {y: "motor_current_angle", label: "Motor Current Angle", color: colors.current_angle},
    {y: "current_angle", label: "Current (Park) Angle", color: colors.current_angle},
    {y: "voltage_angle", label: "Voltage (Park) Angle", color: colors.voltage_angle},
    {y: "hall_u_as_angle", label: "Hall U", color: colors.u},
    {y: "hall_v_as_angle", label: "Hall V", color: colors.v},
    {y: "hall_w_as_angle", label: "Hall W", color: colors.w},
  ],
  curve,
});

const plot_speed = plot_lines({
  subtitle: "Rotor Speed",
  description: "Angular speed of the rotor in degrees per millisecond.",
  width: 1200, height: 150,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Angular Speed (degrees/ms)",
  channels: [
    {
      y: "angular_speed", label: "Angular Speed", color: colors.angular_speed,
      draw_extra: setup_faint_area({
        y0: d => d.angular_speed - stdev_95_z_score * d.angular_speed_stdev, 
        y1: d => d.angular_speed + stdev_95_z_score * d.angular_speed_stdev,
      }),
    },
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
    {y: "vcc_voltage", label: "VCC Voltage (V)", color: colors.v},
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
    {y: "u", label: "Current U", color: colors.u},
    {y: "v", label: "Current V", color: colors.v},
    {y: "w", label: "Current W", color: colors.w},
    {y: "sum", label: "Sum", color: colors.sum},
    {y: "ref_diff", label: "Ref Diff", color: colors.ref_diff},
  ],
  curve,
});

const plot_dq0_currents = plot_lines({
  subtitle: "DQ0 Currents",
  description: "DQ0 currents after Clarke and Park transforming the measured currents.",
  width: 1200, height: 400,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Current (A)",
  channels: [
    {y: "current_alpha", label: "Current Alpha", color: colors.current_alpha},
    {y: "current_beta", label: "Current Beta", color: colors.current_beta},
    {y: "current_magnitude", label: "Current (Park) Magnitude", color: colors.current_magnitude},
  ],
  curve,
});

const plot_dq0_voltages = plot_lines({
  subtitle: "DQ0 Voltages",
  description: "DQ0 voltages after Clarke and Park transforming the inferred voltages.",
  width: 1200, height: 400,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Voltage (V)",
  channels: [
    {y: "voltage_alpha", label: "Voltage Alpha", color: colors.current_alpha},
    {y: "voltage_beta", label: "Voltage Beta", color: colors.current_beta},
    {y: "voltage_magnitude", label: "Voltage (Park) Magnitude", color: colors.current_magnitude},
  ],
  curve,
});


const plot_inferred_voltages = plot_lines({
  subtitle: "Inferred Voltage",
  description: html`Inferred voltage values for each phase: ${tex`V = IR + L(dI/dt)`}.`,
  width: 1200, height: 300,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Voltage (V)",
  channels: [
    {y: "u_voltage", label: "Voltage U", color: colors.u},
    {y: "v_voltage", label: "Voltage V", color: colors.v},
    {y: "w_voltage", label: "Voltage W", color: colors.w},
    {y: "u_L_voltage", label: "Inductor Voltage U", color: d3.color(colors.u).brighter(1)},
    {y: "v_L_voltage", label: "Inductor Voltage V", color: d3.color(colors.v).brighter(1)},
    {y: "w_L_voltage", label: "Inductor Voltage W", color: d3.color(colors.w).brighter(1)},
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
  ],
  curve,
});


autosave_inputs({
  plot_power,
  plot_runtime_stats,
  plot_cycle_loop_stats,
  plot_electric_position,
  plot_speed,
  plot_measured_voltage,
  plot_measured_temperature,
  plot_measured_current,
  plot_dq0_currents,
  plot_dq0_voltages,
  plot_inferred_voltages,
  plot_pwm_settings,
});

```



```js

const time_domain = !timeline_position.selection ? [data_time_start, data_time_end] : [
  timeline_position.selection[0] * max_time_period + data_time_start,
  timeline_position.selection[1] * max_time_period + data_time_start,
];

const data_in_time_window = data.filter((d) => d.time >= time_domain[0] && d.time <= time_domain[1]);

[
  plot_power,
  plot_runtime_stats,
  plot_cycle_loop_stats,
  plot_electric_position,
  plot_speed,
  plot_measured_voltage,
  plot_measured_temperature,
  plot_measured_current,
  plot_dq0_currents,
  plot_dq0_voltages,
  plot_inferred_voltages,
  plot_pwm_settings,
].forEach((plot) => plot.update({data: sparsify(data_in_time_window), x_domain: time_domain}));

```



```js
// Position Calibration
// --------------------

let position_calibration = Mutable({results: [], ...compute_position_calibration([])});


const position_calibration_buttons = Inputs.button(
  [
    ["Start Position Calibration", async function(){
      if (!motor_controller) return;

      for (let i = 0; i < 10; i++){
        const results = [...position_calibration.value.results, await run_position_calibration(motor_controller)];
        const position_results = compute_position_calibration(results);
        position_calibration.value = {results, ...position_results};
      }
    }],
  ],
  {label: "Collect position calibration data"},
);

d3.select(position_calibration_buttons).style("width", "100%");


```


```js
const position_calibration_result_to_display_input = Inputs.select(
  d3.range(0, position_calibration.results.length),
  {
    value: position_calibration.results.length - 1,
    label: "Select position calibration data set:",
  },
);
const position_calibration_result_to_display = Generators.input(position_calibration_result_to_display_input);


const position_calibration_transitions_table = Inputs.table(Object.values(position_calibration.transition_stats), {rows: 12+1});

const position_calibration_centers_table = Inputs.table(Object.values(position_calibration.center_angles), {rows: 6+1});

const position_calibration_sensor_spans_table = Inputs.table(Object.values(position_calibration.sensor_spans), {rows: 6+1});

const position_calibration_sensor_locations_table = Inputs.table(Object.values(position_calibration.sensor_locations), {rows: 3+1});


// Write out the position calibration results in copyable format.
const position_calibration_table = position_calibration.position_calibration === null ? 
  `position_calibration = null` :
  `position_calibration = {\n  ${Object.entries(position_calibration.position_calibration).map(([key, value]) => `"${key}": ${JSON.stringify(value)}`).join(",\n  ")},\n}`;

```


```js

const position_calibration_selected_result = position_calibration.results[position_calibration_result_to_display];

const position_calibration_pos_plot = plot_lines({
  data: position_calibration_selected_result?.drive_positive,
  subtitle: "Electric position | drive positive then break",
  description: "Angular position of the rotor with respect to the electric phases, 0 when magnetic N is aligned with phase U.",
  width: 1200, height: 150,
  x_domain: [0, history_size * millis_per_cycle],
  y_domain: [-180, 180],
  x: "time",
  x_label: "Time (ms)",
  y_label: "Electric position (degrees)",
  channels: [
    {
      y: "angle", label: "Angle", color: colors.angle,
      draw_extra: setup_faint_area({
        y1: (d) => d.angle - stdev_95_z_score * d.angle_stdev,
        y2: (d) => d.angle + stdev_95_z_score * d.angle_stdev,
      }),
    },
    {y: "angle_if_breaking", label: "Angle If Breaking", color: colors.angle_if_breaking},
    {y: "hall_u_as_angle", label: "Hall U", color: colors.u},
    {y: "hall_v_as_angle", label: "Hall V", color: colors.v},
    {y: "hall_w_as_angle", label: "Hall W", color: colors.w},
  ],
  curve: d3.curveStep,
});


const position_calibration_pos_speed_plot = plot_lines({
  data: position_calibration_selected_result?.drive_positive,
  subtitle: "Rotor Speed | drive positive then break",
  description: "Angular speed of the rotor in degrees per millisecond.",
  width: 1200, height: 150,
  x_domain: [0, history_size * millis_per_cycle],
  x: "time",
  x_label: "Time (ms)",
  y_label: "Angular Speed (degrees/ms)",
  channels: [
    {
      y: "angular_speed", label: "Angular Speed", color: colors.angular_speed, 
      draw_extra: setup_faint_area({
        y0: d => d.angular_speed - stdev_95_z_score * d.angular_speed_stdev, 
        y1: d => d.angular_speed + stdev_95_z_score * d.angular_speed_stdev,
      }),
    },
  ],
  curve: d3.curveStep,
});

const position_calibration_neg_plot = plot_lines({
  data: position_calibration_selected_result?.drive_negative,
  subtitle: "Electric position | drive negative then break",
  description: "Angular position of the rotor with respect to the electric phases, 0 when magnetic N is aligned with phase U.",
  width: 1200, height: 150,
  x_domain: [0, history_size * millis_per_cycle],
  y_domain: [-180, 180],
  x: "time",
  x_label: "Time (ms)",
  y_label: "Electric position (degrees)",
  channels: [
    {
      y: "angle", label: "Angle", color: colors.angle, 
      draw_extra: setup_faint_area({
        y1: (d) => d.angle - stdev_95_z_score * d.angle_stdev,
        y2: (d) => d.angle + stdev_95_z_score * d.angle_stdev,
      }),
    },
    {y: "angle_if_breaking", label: "Angle If Breaking", color: d3.color(colors.current_angle).darker(2)},
    {y: "hall_u_as_angle", label: "Hall U", color: colors.u},
    {y: "hall_v_as_angle", label: "Hall V", color: colors.v},
    {y: "hall_w_as_angle", label: "Hall W", color: colors.w},
  ],
  curve: d3.curveStep,
});

const position_calibration_neg_speed_plot = plot_lines({
  data: position_calibration_selected_result?.drive_negative,
  subtitle: "Rotor Speed | drive negative then break",
  description: "Angular speed of the rotor in degrees per millisecond.",
  width: 1200, height: 150,
  x_domain: [0, history_size * millis_per_cycle],
  x: "time",
  x_label: "Time (ms)",
  y_label: "Angular Speed (degrees/ms)",
  channels: [
    {
      y: "angular_speed", label: "Angular Speed", color: colors.angular_speed,
      draw_extra: setup_faint_area({
        y0: d => d.angular_speed - stdev_95_z_score * d.angular_speed_stdev,
        y1: d => d.angular_speed + stdev_95_z_score * d.angular_speed_stdev,
      }),
    },
  ],
  curve: d3.curveStep,
});

autosave_inputs({
  position_calibration_pos_plot,
  position_calibration_pos_speed_plot,
  position_calibration_neg_plot,
  position_calibration_neg_speed_plot,
});


```


```js
// Current calibration
// -------------------


const current_calibration_buttons = Inputs.button(
  [
    ["Start Current Calibration", async function(value){
      value = await value;
      if (!motor_controller) return value;
      const calibration_data = await run_current_calibration(motor_controller);
      const results = [...value.results, calibration_data];
      const {stats, samples, current_calibration, funcs} = compute_current_calibration(results);

      return {results, stats, samples, current_calibration, funcs};
    }],
    ["Save Current Calibration", async function(value){
      value = await value;
      if (!value.current_calibration) return value;
      if (!motor_controller) return value;
      motor_controller.current_calibration = value.current_calibration;
      localStorage.setItem("current_calibration", JSON.stringify(motor_controller.current_calibration));
      return value;
    }],
    ["Reset Current Calibration", async function(value){
      value = await value;
      localStorage.removeItem("current_calibration");
      if (!motor_controller) {
        motor_controller.current_calibration = current_calibration_default;
      }
      return {results: [], ...compute_current_calibration([])};
    }],
  ],
  {
    label: "Collect current calibration data",
    value: {results: [], ...compute_current_calibration([])}
  },
);

d3.select(current_calibration_buttons).style("width", "100%");

const current_calibration = Generators.input(current_calibration_buttons);
```



```js

// Write out the current calibration results in copyable format.
const current_calibration_table = `current_calibration = ${JSON.stringify(current_calibration.current_calibration, null, 2)}`;


// Select which of the calibration runs to display.

const current_calibration_result_to_display_input = Inputs.select(d3.range(current_calibration.results.length), {
  value: current_calibration.results.length - 1,
  label: "Select calibration result to display:",
});
const current_calibration_result_to_display = Generators.input(current_calibration_result_to_display_input);

```

```js

const current_calibration_plot = plot_lines({
  data: current_calibration.results[current_calibration_result_to_display],
  subtitle: "Current Calibration",
  description: "Current calibration results for each phase.",
  width: 1200, height: 400,
  x_domain: [0, history_size * millis_per_cycle],
  x: "time",
  x_label: "Time (ms)",
  y_label: "Current (A)",
  channels: [
    {y: "u_positive", label: "U positive", color: colors.u},
    {y: "u_negative", label: "U negative", color: d3.color(colors.u).darker(1)},
    {y: "v_positive", label: "V positive", color: colors.v},
    {y: "v_negative", label: "V negative", color: d3.color(colors.v).darker(1)},
    {y: "w_positive", label: "W positive", color: colors.w},
    {y: "w_negative", label: "W negative", color: d3.color(colors.w).darker(1)},
    {
      y: "target",
      draw_extra: setup_faint_area({y0: 0.0, y1: "target"}),
      label: "Target", color: "gray",
    }
  ],
});


const current_calibration_interpolate_plot = plot_lines({
  data: current_calibration.samples,
  subtitle: "Current Calibration Interpolation",
  description: "Current calibration interpolation results for each phase.",
  width: 1200, height: 400,
  x_domain: [0, max_calibration_current],
  x: "reading",
  x_label: "Current Reading (A)",
  y_label: "Current Estimate (A)",
  channels: [
    {y: "u_positive", label: "U positive", color: colors.u},
    {y: "u_negative", label: "U negative", color: d3.color(colors.u).darker(1)},
    {y: "v_positive", label: "V positive", color: colors.v},
    {y: "v_negative", label: "V negative", color: d3.color(colors.v).darker(1)},
    {y: "w_positive", label: "W positive", color: colors.w},
    {y: "w_negative", label: "W negative", color: d3.color(colors.w).darker(1)},
    {y: "target", label: "Target", color: "gray"},
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
      draw_extra: setup_faint_area({
        y0: d => d.u_positive - d.u_positive_stdev * stdev_95_z_score,
        y1: d => d.u_positive + d.u_positive_stdev * stdev_95_z_score,
      }),
      label: "U positive mean", color: colors.u,
    },
    {
      y: "v_positive",
      draw_extra: setup_faint_area({
        y0: d => d.v_positive - d.v_positive_stdev * stdev_95_z_score,
        y1: d => d.v_positive + d.v_positive_stdev * stdev_95_z_score,
      }),
      label: "V positive mean", color: colors.v,
    },
    {
      y: "w_positive",
      draw_extra: setup_faint_area({
        y0: d => d.w_positive - d.w_positive_stdev * stdev_95_z_score,
        y1: d => d.w_positive + d.w_positive_stdev * stdev_95_z_score,
      }),
      label: "W positive mean", color: colors.w,
    },
    {
      y: "target",
      draw_extra: setup_faint_area({y0: 0.0, y1: "target"}),
      label: "Target", color: "gray",
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
      draw_extra: setup_faint_area({
        y0: d => d.u_negative - d.u_negative_stdev * stdev_95_z_score,
        y1: d => d.u_negative + d.u_negative_stdev * stdev_95_z_score,
      }),
      label: "U negative mean", color: colors.u,
    },
    {
      y: "v_negative",
      draw_extra: setup_faint_area({
        y0: d => d.v_negative - d.v_negative_stdev * stdev_95_z_score,
        y1: d => d.v_negative + d.v_negative_stdev * stdev_95_z_score,
      }),
      label: "V negative mean", color: colors.v,
    },
    {
      y: "w_negative",
      draw_extra: setup_faint_area({
        y0: d => d.w_negative - d.w_negative_stdev * stdev_95_z_score,
        y1: d => d.w_negative + d.w_negative_stdev * stdev_95_z_score,
      }),
      label: "W negative mean", color: colors.w,
    },
    {
      y: "target",
      draw_extra: setup_faint_area({y0: 0.0, y1: "target"}),
      label: "Target", color: "gray",
    },
  ],
});

autosave_inputs({
  current_calibration_plot,
  current_calibration_interpolate_plot,
  current_calibration_positive_mean_plot,
  current_calibration_negative_mean_plot,
});

```



```js
// Imports
// -------

import {plot_lines, plot_line, setup_faint_area, horizontal_step} from "./components/plotting_utils.js";

import {localStorage, get_stored_or_default, clear_stored_data} from "./components/local_storage.js";

import {round, uint32_to_bytes, bytes_to_uint32, timeout_promise, wait, clean_id}  from "./components/utils.js";


import {enabled_checkbox, autosave_inputs, any_checked_input, set_input_value, merge_input_value} from "./components/input_utils.js";

import {interpolate_degrees, shortest_distance_degrees, normalize_degrees, circular_stats_degrees} from "./components/angular_math.js";

import {command_codes, connect_usb_motor_controller, MotorController} from "./components/motor_controller.js";

import {run_current_calibration, compute_current_calibration, max_calibration_current} from "./components/motor_current_calibration.js";

import {run_position_calibration, compute_position_calibration} from "./components/motor_position_calibration.js";

import {
  cycles_per_millisecond, millis_per_cycle, max_timeout, angle_base, pwm_base, pwm_period, 
  history_size, current_calibration_default,
} from "./components/motor_constants.js";


// Pick evenly spaced data points to pass to the plot; we can't draw more pixels than we have.
function sparsify(data, target_points = 1080){
  const max_plot_points = target_points;
  const plot_points_skip = Math.ceil(data.length / max_plot_points);
  return data.filter((d, i) => i % plot_points_skip === 0);
}

```