---
title: Motor monitor
---
<main class="hero">

Motor Commands
--------------

<div>${connect_buttons}</div>
<div>${motor_controller_status}</div>
<div>${data_request_buttons}</div>
<div>${test_buttons}</div>
<div>${command_options_input}</div>
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
</div>
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
  <div>Number of calibration data sets: ${position_calibration_results.length}</div>
</div>
<div class="card tight">
  <h3>Position Calibration Results</h3>
  <div>${position_calibration_result_to_display_input}</div>
  <div>${position_calibration_pos_plot}</div>
  <div>${position_calibration_pos_speed_plot}</div>
  <div>${position_calibration_neg_plot}</div>
  <div>${position_calibration_neg_speed_plot}</div>
  <div>${position_calibration_table}</div>
  <div>${position_calibration_hall_details_table}</div>
  <div>${position_calibration_hysterisis_table}</div>
</div>


Current Calibration Procedures
------------------------------

<div class="card tight">
  <div>${current_calibration_buttons}</div>
  <div>Number of calibration data sets: ${current_calibration_results.length}</div>
</div>
<div class="card tight">
  <h3>Current Calibration Results</h3>
  <div>${current_calibration_table}</div>
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

const std_95_z_score = 1.959964; // 95% confidence interval for normal distribution


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
  current_angular_speed: "rgb(27, 158, 119)",
  current_alpha: "rgb(199, 0, 57)",
  current_beta: "rgb(26, 82, 118)",
};


```

```js

let motor_controller = Mutable(null);

let motor_controller_status = Mutable(html`<pre>Not connected.</pre>`);

function update_motor_controller_status(status){
  motor_controller_status.value = status;
}

function display_port_error(error){
  if (error.message === "EOF") {
    update_motor_controller_status(html`<pre>End of connection.</pre>`);
  } else if (error.name === "NotFoundError") {
    update_motor_controller_status(html`<pre style="color: red">No device found or nothing selected.</pre>`);
  } else if (error.name === "SecurityError") {
    update_motor_controller_status(html`<pre style="color: red">Permission for port dialog denied.</pre>`);
  } else if (error.name === "NetworkError") {
    update_motor_controller_status(html`<pre style="color: red">Connection lost.</pre>`);
  } else {
    update_motor_controller_status(html`<pre style="color: red">Connection lost; unknown error: ${error}</pre>`);
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
      update_motor_controller_status(html`<pre style="color: orange">Disconnected!</pre>`);
    }
  }
}

function display_datastream_status({bytes_received, bytes_discarded, receive_rate}){
  bytes_received = format_bytes(bytes_received).padStart(12);
  bytes_discarded = format_bytes(bytes_discarded).padStart(12);
  receive_rate = `${format_bytes(receive_rate).padStart(12)}/s`;
  update_motor_controller_status(html`<pre>Connected; received: ${bytes_received}; download rate: ${receive_rate}; discarded: ${bytes_discarded}.</pre>`);
}

async function connect_motor_controller(){
  try {
    await disconnect_motor_controller(false);

    motor_controller.value = await motor.connect_usb_motor_controller();

    update_motor_controller_status(html`<pre>Connected, waiting for data.</pre>`);

    await motor_controller.value.reading_loop(display_datastream_status);
  } catch (error) {
    motor_controller.value = null;

    display_port_error(error);
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

const command_timeout_slider = Inputs.range([0, MAX_TIMEOUT*millis_per_cycle], {value: 2000, step: 5, label: "Command timeout (ms):"});

const command_timeout_millis = Generators.input(command_timeout_slider);

const command_leading_angle_slider = Inputs.range([-180, 180], {value: 95, step: 1, label: "Leading angle (degrees):"});

const command_leading_angle_degrees = Generators.input(command_leading_angle_slider);
```



```js

// Load Calibration
// ----------------


function load_current_calibration_factors(){
  return get_stored_or_default("current_calibration", current_calibration_default);
}

// Load or make default calibration
const saved_current_calibration_factors = load_current_calibration_factors();

let current_calibration_factors = Mutable(saved_current_calibration_factors);

function update_current_calibration_factors(new_calibration_factors){
  current_calibration_factors.value = new_calibration_factors;
}

function reload_current_calibration_factors(){
  update_current_calibration_factors(load_current_calibration_factors());
}

function save_current_calibration_factors(){
  localStorage.setItem("current_calibration", JSON.stringify(current_calibration_factors.value));
}

function reset_current_calibration_factors(){
  localStorage.removeItem("current_calibration");
  update_current_calibration_factors(current_calibration_default);
}
```


```js

const process_readout = process_readout_with_calibration({current_calibration_factors});

function calculate_data_stats(raw_readout_data){
  return online_map(raw_readout_data, process_readout);
}
```


```js
// Data stream output
// ------------------


const target_data_size = 2500 / millis_per_cycle;
const max_data_size = 2 * target_data_size;


let data = Mutable([]);

function update_data (new_data){
  data.value = new_data;
};

```


```js
// Control functions
// -----------------


const command_timeout = Math.floor(command_timeout_millis * cycles_per_millisecond);
const command_pwm = Math.floor(command_pwm_fraction * PWM_BASE);
const command_leading_angle = Math.floor(256 + 256 * command_leading_angle_degrees / 360) % 256;


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
      await motor_controller.send_command({command, command_timeout: 0, command_pwm: 0, command_leading_angle: 0, ...options});
  
      let data = [];
      let processed_readout = undefined;
      function readout_callback(raw_readout){
        processed_readout = process_readout(processed_readout, raw_readout, data.length, data);
        data.push(processed_readout);
        if (data.length > max_data_size) data = data.slice(-target_data_size);
        update_data(data);
      }
      // Start reading the data stream.
      await motor_controller.stream_readouts({
        readout_callback,
        ...options,
      });
    } catch (error) {
      console.error("Error streaming data:", error);
      update_motor_controller_status(html`<pre style="color: red">Error streaming data: ${error.message}</pre>`);
    }
  }, delay_ms);
}

const data_request_buttons = Inputs.button(
  [
    ["ADC snapshot", async function(){
      command_and_stream(0, motor.GET_READOUTS_SNAPSHOT, {expected_messages: HISTORY_SIZE});
    }],
    ["ADC stream", async function(){
      await command_and_stream(0, motor.STREAM_FULL_READOUTS, {expected_code: motor.FULL_READOUT, command_timeout: 1});
    }],
    ["STOP stream", async function(){
      await command(motor.STREAM_FULL_READOUTS, {command_timeout: 0});
    }],
  ],
  {label: "Read data"},
);

d3.select(data_request_buttons).selectAll("button").style("height", "4em");

const test_buttons = Inputs.button(
  [
    ["Test all permutations", async function(){
      await command_and_stream(0, motor.SET_STATE_TEST_ALL_PERMUTATIONS);
    }],
    ["Test ground short", async function(){
      await command_and_stream(0, motor.SET_STATE_TEST_GROUND_SHORT);
    }],
    ["Test positive short", async function(){
      await command_and_stream(0, motor.SET_STATE_TEST_POSITIVE_SHORT);
    }],
    ["Test U directions", async function(){
      await command_and_stream(0, motor.SET_STATE_TEST_U_DIRECTIONS);
    }],
    ["Test U increasing", async function(){
      await command_and_stream(0, motor.SET_STATE_TEST_U_INCREASING);
    }],
    ["Test U decreasing", async function(){
      await command_and_stream(0, motor.SET_STATE_TEST_U_DECREASING);
    }],
    ["Test V increasing", async function(){
      await command_and_stream(0, motor.SET_STATE_TEST_V_INCREASING);
    }],
    ["Test V decreasing", async function(){
      await command_and_stream(0, motor.SET_STATE_TEST_V_DECREASING);
    }],
    ["Test W increasing", async function(){
      await command_and_stream(0, motor.SET_STATE_TEST_W_INCREASING);
    }],
    ["Test W decreasing", async function(){
      await command_and_stream(0, motor.SET_STATE_TEST_W_DECREASING);
    }],
    ["Test V increasing", async function(){
      await command_and_stream(0, motor.SET_STATE_TEST_V_INCREASING);
    }],
    ["Test V decreasing", async function(){
      await command_and_stream(0, motor.SET_STATE_TEST_V_DECREASING);
    }],
    ["Test W increasing", async function(){
      await command_and_stream(0, motor.SET_STATE_TEST_W_INCREASING);
    }],
    ["Test W decreasing", async function(){
      await command_and_stream(0, motor.SET_STATE_TEST_W_DECREASING);
    }],
  ],
  {label: "Test sequence"},
);

function maybe_stream(delay_ms, command, options = {}){
  if (command_options.includes("Take snapshot after command")){
    command_and_stream(delay_ms, command, options);
  }
}


d3.select(test_buttons).selectAll("button").style("height", "4em");

const command_buttons = Inputs.button(
  [
    ["Stop", async function(){
      await command(motor.SET_STATE_OFF);
      maybe_stream(0, motor.GET_READOUTS_SNAPSHOT, {expected_messages: HISTORY_SIZE});
    }],
    ["Drive +", async function(){
      await command(motor.SET_STATE_DRIVE_POS);
      maybe_stream(500, motor.GET_READOUTS_SNAPSHOT, {expected_messages: HISTORY_SIZE});
    }],
    ["Drive -", async function(){
      await command(motor.SET_STATE_DRIVE_NEG);
      maybe_stream(500, motor.GET_READOUTS_SNAPSHOT, {expected_messages: HISTORY_SIZE});
    }],
    ["Drive smooth +", async function(){
      await command(motor.SET_STATE_DRIVE_SMOOTH_POS);
      maybe_stream(500, motor.GET_READOUTS_SNAPSHOT, {expected_messages: HISTORY_SIZE});
    }],
    ["Drive smooth -", async function(){
      await command(motor.SET_STATE_DRIVE_SMOOTH_NEG);
      maybe_stream(500, motor.GET_READOUTS_SNAPSHOT, {expected_messages: HISTORY_SIZE});
    }],
    ["Freewheel", async function(){
      await command(motor.SET_STATE_FREEWHEEL);
      maybe_stream(0, motor.GET_READOUTS_SNAPSHOT, {expected_messages: HISTORY_SIZE});
    }],
    ["Hold U positive", async function(){
      await command(motor.SET_STATE_HOLD_U_POSITIVE);
      maybe_stream(500, motor.GET_READOUTS_SNAPSHOT, {expected_messages: HISTORY_SIZE});
    }],
    ["Hold V positive", async function(){
      await command(motor.SET_STATE_HOLD_V_POSITIVE);
      maybe_stream(500, motor.GET_READOUTS_SNAPSHOT, {expected_messages: HISTORY_SIZE});
    }],
    ["Hold W positive", async function(){
      await command(motor.SET_STATE_HOLD_W_POSITIVE);
      maybe_stream(500, motor.GET_READOUTS_SNAPSHOT, {expected_messages: HISTORY_SIZE});
    }],
    ["Hold U negative", async function(){
      await command(motor.SET_STATE_HOLD_U_NEGATIVE);
      maybe_stream(500, motor.GET_READOUTS_SNAPSHOT, {expected_messages: HISTORY_SIZE});
    }],
    ["Hold V negative", async function(){
      await command(motor.SET_STATE_HOLD_V_NEGATIVE);
      maybe_stream(500, motor.GET_READOUTS_SNAPSHOT, {expected_messages: HISTORY_SIZE});
    }],
    ["Hold W negative", async function(){
      await command(motor.SET_STATE_HOLD_W_NEGATIVE);
      maybe_stream(500, motor.GET_READOUTS_SNAPSHOT, {expected_messages: HISTORY_SIZE});
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

const history_duration = Math.ceil(HISTORY_SIZE * millis_per_cycle);

const time_period_input = Inputs.range([1, max_time_period], {
  value: history_duration,
  transform: Math.log,
  step: 0.5,
  label: "Time window Duration (ms):",
});
const time_period = Generators.input(time_period_input);

const time_offset_input = Inputs.range([0, 1.0], {
  value: 0.0, 
  step: 0.01,
  label: "Time window Rewind (ms):",
});
d3.select(time_offset_input).select("div").style("width", "640px");
d3.select(time_offset_input).select("input[type=range]").style("width", "100em");

const time_offset = Generators.input(time_offset_input);

const plot_options_input = Inputs.checkbox(
  ["Connected lines"],
  {
    value: ["Connected lines"],
    label: "Plot options:",
  },
);

const plot_options = Generators.input(plot_options_input);


```


```js

const curve = plot_options.includes("Connected lines") ? d3.curveStep : horizontal_step;

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
  y_domain: [0, PWM_PERIOD],
  channels: [
    {y: "cycle_start_tick", label: "Tick at start", color: colors.u},
    {y: "cycle_end_tick", label: "Tick at end", color: colors.v},
    {y: (d) => (PWM_PERIOD + d.cycle_end_tick - d.cycle_start_tick) % PWM_PERIOD , label: "Cycle duration", color: colors.w},
    {y: (d) => d.cycle_start_tick - PWM_BASE, label: "Ticks at start since mid cycle", color: d3.color(colors.u).brighter(1)},
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
        y0: (d) => d.angle - std_95_z_score * d.angle_std, 
        y1: (d) => d.angle + std_95_z_score * d.angle_std,
      }),
    },
    {y: "motor_angle", label: "Motor Angle", color: colors.motor_angle},
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
    {y: "current_angular_speed", label: "Current Speed", color: colors.current_angular_speed},
    {
      y: "angular_speed", label: "Angular Speed", color: colors.angular_speed,
      draw_extra: setup_faint_area({
        y0: d => d.angular_speed - std_95_z_score * d.angular_speed_std, 
        y1: d => d.angular_speed + std_95_z_score * d.angular_speed_std,
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
  y_domain: [0, PWM_BASE],
  channels: [
    {y: "u_pwm", label: "PWM U", color: colors.u},
    {y: "v_pwm", label: "PWM V", color: colors.v},
    {y: "w_pwm", label: "PWM W", color: colors.w},
  ],
  curve,
});


autosave_inputs({
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
const time_end = (data.length == 0 ? 0 : data[data.length - 1].time) - max_time_period * time_offset;

const time_domain = [time_end - time_period, time_end];

const data_in_time_window = data.filter((d) => d.time >= time_domain[0] && d.time <= time_domain[1]);

// TODO: make brush input for the time window


const max_plot_points = 720;

const plot_points_skip = Math.ceil(data_in_time_window.length / max_plot_points);

const selected_data = data_in_time_window.filter((d, i) => i % plot_points_skip === 0);

[
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
].forEach((plot) => plot.update({data: selected_data, x_domain: time_domain}));


```



```js
// Position Calibration
// --------------------

let position_calibration_results = Mutable();

function store_position_calibration_results(calibration_data){
  position_calibration_results.value = [...position_calibration_results.value ?? [], calibration_data];
}
```

```js
const position_calibration_buttons = Inputs.button(
  [
    ["Start Position Calibration", async function(){
      for (let i = 0; i < 10; i++){
        await run_position_calibration();
      }
    }],
  ],
  {label: "Collect position calibration data"},
);

d3.select(position_calibration_buttons).style("width", "100%");


async function run_position_calibration(){
  if (!motor_controller) return;

  console.info("Position calibration starting");
  
  const drive_time = 200;
  const drive_timeout = Math.floor((drive_time + 300) * cycles_per_millisecond);

  const drive_strength = Math.floor(PWM_BASE * 2 / 10);
  const drive_options = {command_timeout: drive_timeout, command_pwm: drive_strength};

  const test_options = {command_timeout: 0, command_pwm: 0};

  await motor_controller.send_command({command: motor.SET_STATE_DRIVE_POS, ...drive_options});  
  await wait(drive_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_GROUND_SHORT, ...test_options});

  const drive_positive = calculate_data_stats(await motor_controller.get_readouts({expected_messages: HISTORY_SIZE}));

  console.info("Drive positive done");

  await motor_controller.send_command({command: motor.SET_STATE_DRIVE_NEG, ...drive_options});
  await wait(drive_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_GROUND_SHORT, ...test_options});
  
  const drive_negative = calculate_data_stats(await motor_controller.get_readouts({expected_messages: HISTORY_SIZE}));
  
  
  console.info("Drive negative done");

  console.info("Position calibration done");

  if (drive_positive.length != HISTORY_SIZE) {
    console.error("Drive positive data is not valid", drive_positive);
    return;
  }
  if (drive_negative.length != HISTORY_SIZE) {
    console.error("Drive negative data is not valid", drive_negative);
    return;
  }

  const position_calibration_data = {
    drive_positive,
    drive_negative,
    concatenated: [
      ...drive_positive.map(d => ({...d, direction: "positive"})),
      ...drive_negative.map(d => ({...d, direction: "negative"})),
    ],
  };
  
  store_position_calibration_results(position_calibration_data);
}
```


```js
const position_calibration_result_to_display_input = Inputs.select(
  d3.range(0, position_calibration_results.length),
  {
    value: position_calibration_results.length - 1,
    label: "Select position calibration data set:",
  },
);
const position_calibration_result_to_display = Generators.input(position_calibration_result_to_display_input);
```


```js

const position_calibration_selected_result = position_calibration_results[position_calibration_result_to_display];
const position_calibration_detailed_result = {
  drive_positive: position_calibration_selected_result.drive_positive,
  drive_negative: position_calibration_selected_result.drive_negative,
};

const position_calibration_pos_plot = plot_lines({
  data: position_calibration_detailed_result.drive_positive,
  subtitle: "Electric position | drive positive then break",
  description: "Angular position of the rotor with respect to the electric phases, 0 when magnetic N is aligned with phase U.",
  width: 1200, height: 150,
  x_domain: [0, HISTORY_SIZE * millis_per_cycle],
  y_domain: [-180, 180],
  x: "time",
  x_label: "Time (ms)",
  y_label: "Electric position (degrees)",
  channels: [
    {
      y: "angle", label: "Angle", color: colors.angle,
      draw_extra: setup_faint_area({
        y1: (d) => d.angle - std_95_z_score * d.angle_std,
        y2: (d) => d.angle + std_95_z_score * d.angle_std,
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
  data: position_calibration_detailed_result.drive_positive,
  subtitle: "Rotor Speed | drive positive then break",
  description: "Angular speed of the rotor in degrees per millisecond.",
  width: 1200, height: 150,
  x_domain: [0, HISTORY_SIZE * millis_per_cycle],
  x: "time",
  x_label: "Time (ms)",
  y_label: "Angular Speed (degrees/ms)",
  channels: [
    {y: "current_angular_speed", label: "Current Speed", color: colors.current_angular_speed},
    {
      y: "angular_speed", label: "Angular Speed", color: colors.angular_speed, 
      draw_extra: setup_faint_area({
        y0: d => d.angular_speed - std_95_z_score * d.angular_speed_std, 
        y1: d => d.angular_speed + std_95_z_score * d.angular_speed_std,
      }),
    },
  ],
  curve: d3.curveStep,
});

const position_calibration_neg_plot = plot_lines({
  data: position_calibration_detailed_result.drive_negative,
  subtitle: "Electric position | drive negative then break",
  description: "Angular position of the rotor with respect to the electric phases, 0 when magnetic N is aligned with phase U.",
  width: 1200, height: 150,
  x_domain: [0, HISTORY_SIZE * millis_per_cycle],
  y_domain: [-180, 180],
  x: "time",
  x_label: "Time (ms)",
  y_label: "Electric position (degrees)",
  channels: [
    {
      y: "angle", label: "Angle", color: colors.angle, 
      draw_extra: setup_faint_area({
        y1: (d) => d.angle - std_95_z_score * d.angle_std,
        y2: (d) => d.angle + std_95_z_score * d.angle_std,
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
  data: position_calibration_detailed_result.drive_negative,
  subtitle: "Rotor Speed | drive negative then break",
  description: "Angular speed of the rotor in degrees per millisecond.",
  width: 1200, height: 150,
  x_domain: [0, HISTORY_SIZE * millis_per_cycle],
  x: "time",
  x_label: "Time (ms)",
  y_label: "Angular Speed (degrees/ms)",
  channels: [
    {y: "current_angular_speed", label: "Current Speed", color: colors.current_angular_speed},
    {
      y: "angular_speed", label: "Angular Speed", color: colors.angular_speed,
      draw_extra: setup_faint_area({
        y0: d => d.angular_speed - std_95_z_score * d.angular_speed_std,
        y1: d => d.angular_speed + std_95_z_score * d.angular_speed_std,
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

// Store the inferred angle for every hall sensor switching event.
function extract_hall_switching_events(data){
  return data.flatMap((d, i) => {
    if (d.angle_if_breaking == null) return [];
    if (i == 0) return [];
    const prev = data[i - 1];
    const angle_if_breaking = interpolate_degrees(prev.angle_if_breaking, d.angle_if_breaking, 0.5);

    const prev_sector = prev.sector;
    const next_sector = d.sector;

    if (prev_sector == null || next_sector == null) return [];

    if (prev_sector == next_sector) return [];

    return [{time: d.time, angle_if_breaking, switching: `hall_sector_${prev_sector}_to_${next_sector}`}];

  });
}

const position_calibration_switching_events = position_calibration_results.flatMap((calibration_data) => {
  return extract_hall_switching_events(calibration_data.concatenated);
});

const position_calibration_switching_angles = position_calibration_switching_events.reduce((acc, d) => {
  acc[d.switching]?.push(d.angle_if_breaking);
  return acc;
}, {
  hall_sector_5_to_0: [],
  hall_sector_0_to_1: [],
  hall_sector_1_to_2: [],
  hall_sector_2_to_3: [],
  hall_sector_3_to_4: [],
  hall_sector_4_to_5: [],

  hall_sector_1_to_0: [], 
  hall_sector_2_to_1: [],
  hall_sector_3_to_2: [],
  hall_sector_4_to_3: [],
  hall_sector_5_to_4: [],
  hall_sector_0_to_5: [],
});

const position_calibration_statistics = Object.entries(position_calibration_switching_angles).map(([key, values]) => {
  const { circular_mean: mean, circular_std: std } = circular_stats_degrees(values);
  return {key, n: values.length, mean, std};
});

const position_calibration_table = Inputs.table(position_calibration_statistics, {rows: 12+1});

```

```js

const position_calibration_hall_details = [
  ["drive_positive_hall_u", "hall_sector_4_to_5", "hall_sector_1_to_2"],
  ["drive_negative_hall_u", "hall_sector_5_to_4", "hall_sector_2_to_1"],
  ["drive_positive_hall_v", "hall_sector_0_to_1", "hall_sector_3_to_4"],
  ["drive_negative_hall_v", "hall_sector_1_to_0", "hall_sector_4_to_3"],
  ["drive_positive_hall_w", "hall_sector_2_to_3", "hall_sector_5_to_0"],
  ["drive_negative_hall_w", "hall_sector_3_to_2", "hall_sector_0_to_5"],
].map(([key, a_key, b_key]) => {
  const a_stats = position_calibration_statistics.find(d => d.key === a_key);
  const b_stats = position_calibration_statistics.find(d => d.key === b_key);

  const a = a_stats.mean;
  const b = b_stats.mean;

  const diff = (b - a + 360) % 360;
  const location = interpolate_degrees(a, b, 0.5);
  const err = diff - 180;
  return {key, a, b, diff, location, err};
});

const position_calibration_hall_details_table = Inputs.table(position_calibration_hall_details, {rows: 6+1});

const position_calibration_hysterisis = [
  ["hysterisis_hall_u", "drive_negative_hall_u", "drive_positive_hall_u"],
  ["hysterisis_hall_v", "drive_negative_hall_v", "drive_positive_hall_v"],
  ["hysterisis_hall_w", "drive_negative_hall_w", "drive_positive_hall_w"],
].map(([key, a_key, b_key]) => {
  const a_stats = position_calibration_hall_details.find(d => d.key === a_key);
  const b_stats = position_calibration_hall_details.find(d => d.key === b_key);
  const hysterisis = shortest_distance_degrees(a_stats.location, b_stats.location);
  const mid_location = interpolate_degrees(a_stats.location, b_stats.location, 0.5);

  return {key, hysterisis, mid_location};
});

const position_calibration_hysterisis_table = Inputs.table(position_calibration_hysterisis, {rows: 3+1});
```


```js
// Current calibration
// -------------------

let calculated_current_calibration_factors = Mutable(null);

let current_calibration_results = Mutable();
let current_calibration = Mutable();
let current_calibration_stats = Mutable();

const short_duration = HISTORY_SIZE / 12 * millis_per_cycle;

const current_calibration_zones = [
  {pwm: 0.1, settle_start: short_duration * 1.4, settle_end: short_duration * 1.95},
  {pwm: 0.2, settle_start: short_duration * 2.4, settle_end: short_duration * 2.95},
  {pwm: 0.3, settle_start: short_duration * 3.4, settle_end: short_duration * 3.95},
  {pwm: 0.4, settle_start: short_duration * 4.4, settle_end: short_duration * 4.95},
  {pwm: 0.5, settle_start: short_duration * 5.4, settle_end: short_duration * 5.95},
  {pwm: 0.6, settle_start: short_duration * 6.4, settle_end: short_duration * 6.95},
  {pwm: 0.7, settle_start: short_duration * 7.4, settle_end: short_duration * 7.95},
  {pwm: 0.8, settle_start: short_duration * 8.4, settle_end: short_duration * 8.95},
  {pwm: 0.9, settle_start: short_duration * 9.4, settle_end: short_duration * 9.95},
];


const drive_resistance = 2.0; // 2.0 Ohm measured with voltmeter between 1 phase and the other 2 in parallel.
const drive_voltage = 10.0; // 10.0 V // TODO: get it from the chip


const calibration_reference = drive_voltage / drive_resistance;

const current_calibration_points = 32;
const current_calibration_reading_points = even_spacing(calibration_reference, current_calibration_points / 2 + 1);
const current_calibration_targets = current_calibration_zones.map((zone) => zone.pwm * calibration_reference);


function compute_current_calibration(calibration_data){
  
  const calibration_data_by_zone = current_calibration_zones.map((zone) => {
    return calibration_data.filter((d) => d.time > zone.settle_start && d.time < zone.settle_end);
  });

  function compute_zone_calibration({measurements, targets}){
    // Make sure the lengths are equal.
    if (measurements.length !== targets.length) throw new Error("Data length mismatch");

    const factor = d3.mean(targets, (target, i) => target / measurements[i]); 
    
    const slow_calibration = piecewise_linear({
      X: [0.0, ...measurements.map((x) => x)], 
      Y: [0.0, ...targets.map((y) => y / factor)],
    });

    // Recalibrate to evenly spaced points for fast processing.

    const Y = current_calibration_reading_points.map((x) => slow_calibration(x));

    const func = even_piecewise_linear({x_min: 0, x_max: calibration_reference, Y});

    const sample = current_calibration_reading_points.map((x) => ({reading: x, target: func(x)}));
    
    return {
      measurements,
      targets,
      factor,
      func,
      sample,
    };
  }

  function compute_phase_calibration(phase_selector){    
    return compute_zone_calibration({
      measurements: calibration_data_by_zone.map((zone_data) => d3.mean(zone_data, (d) => d[phase_selector])),
      targets: current_calibration_targets,
    });
  }

  return {
    u_positive: compute_phase_calibration("u_positive"),
    u_negative: compute_phase_calibration("u_negative"),
    v_positive: compute_phase_calibration("v_positive"),
    v_negative: compute_phase_calibration("v_negative"),
    w_positive: compute_phase_calibration("w_positive"),
    w_negative: compute_phase_calibration("w_negative"),
  };
}

function update_current_calibration_results(calibration_data){

  current_calibration_results.value = [...current_calibration_results.value ?? [], calibration_data];

  const valid_calibration_results = current_calibration_results.value.filter((calibration_data, i) => {
    try {
      compute_current_calibration(calibration_data);
      return true;
    } catch (e) {
      console.error(`Invalid calibration data (index ${i}); error: ${e}`);
      return false;
    }
  });

  if (valid_calibration_results.length == 0) return;

  const calibration_stats = d3.range(HISTORY_SIZE).map((i) => {
    return {
      time: valid_calibration_results[0][i].time,
      target: d3.mean(valid_calibration_results, (data) => data[i].target),
      u_positive: d3.mean(valid_calibration_results, (data) => data[i].u_positive),
      u_positive_std: d3.deviation(valid_calibration_results, (data) => data[i].u_positive),
      u_negative: d3.mean(valid_calibration_results, (data) => data[i].u_negative),
      u_negative_std: d3.deviation(valid_calibration_results, (data) => data[i].u_negative),
      v_positive: d3.mean(valid_calibration_results, (data) => data[i].v_positive),
      v_positive_std: d3.deviation(valid_calibration_results, (data) => data[i].v_positive),
      v_negative: d3.mean(valid_calibration_results, (data) => data[i].v_negative),
      v_negative_std: d3.deviation(valid_calibration_results, (data) => data[i].v_negative),
      w_positive: d3.mean(valid_calibration_results, (data) => data[i].w_positive),
      w_positive_std: d3.deviation(valid_calibration_results, (data) => data[i].w_positive),
      w_negative: d3.mean(valid_calibration_results, (data) => data[i].w_negative),
      w_negative_std: d3.deviation(valid_calibration_results, (data) => data[i].w_negative),
    };
  });

  const calibration = compute_current_calibration(calibration_stats);

  current_calibration.value = calibration;
  
  current_calibration_stats.value = calibration_stats;

  calculated_current_calibration_factors.value = {
    u_positive: calibration.u_positive.factor,
    u_negative: calibration.u_negative.factor,
    v_positive: calibration.v_positive.factor,
    v_negative: calibration.v_negative.factor,
    w_positive: calibration.w_positive.factor,
    w_negative: calibration.w_negative.factor,
  };
}

```

```js

const current_calibration_buttons = Inputs.button(
  [
    ["Start Current Calibration", async function(){
      await run_current_calibration();
    }],
    ["Save Current Factors", function(){
      if (!calculated_current_calibration_factors) return;
      update_current_calibration_factors(calculated_current_calibration_factors);
      save_current_calibration_factors();
    }],
    ["Reload Current Factors", function(){
      reload_current_calibration_factors();
    }],
    ["Reset Current Factors", function(){
      reset_current_calibration_factors();
    }],
  ],
  {label: "Collect current calibration data"},
);

d3.select(current_calibration_buttons).style("width", "100%");
```


```js


async function run_current_calibration(){
  if (!motor_controller) return;
  
  const settle_time = 100;
  const settle_timeout = Math.floor((settle_time + 300) * cycles_per_millisecond);
  const settle_strength = Math.floor(PWM_BASE * 2 / 10);

  const drive_options = {command_timeout: settle_timeout, command_pwm: settle_strength};
  const test_options = {command_timeout: 0, command_pwm: 0};

  console.info("Current calibration starting");

  // Note: hold pwm is clamped by the motor driver

  await motor_controller.send_command({command: motor.SET_STATE_HOLD_U_POSITIVE, ...drive_options});
  await wait(settle_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_U_INCREASING, ...test_options});

  const u_positive = calculate_data_stats(await motor_controller.get_readouts({expected_messages: HISTORY_SIZE}));

  console.info("U positive done");

  await motor_controller.send_command({command: motor.SET_STATE_HOLD_W_NEGATIVE, ...drive_options});
  await wait(settle_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_W_DECREASING, ...test_options});

  const w_negative = calculate_data_stats(await motor_controller.get_readouts({expected_messages: HISTORY_SIZE}));

  console.info("W negative done");

  await motor_controller.send_command({command: motor.SET_STATE_HOLD_V_POSITIVE, ...drive_options});
  await wait(settle_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_V_INCREASING, ...test_options});

  const v_positive = calculate_data_stats(await motor_controller.get_readouts({expected_messages: HISTORY_SIZE}));

  console.info("V positive done");

  await motor_controller.send_command({command: motor.SET_STATE_HOLD_U_NEGATIVE, ...drive_options});
  await wait(settle_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_U_DECREASING, ...test_options});

  const u_negative = calculate_data_stats(await motor_controller.get_readouts({expected_messages: HISTORY_SIZE}));

  console.info("U negative done");

  await motor_controller.send_command({command: motor.SET_STATE_HOLD_W_POSITIVE, ...drive_options});
  await wait(settle_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_W_INCREASING, ...test_options});

  const w_positive = calculate_data_stats(await motor_controller.get_readouts({expected_messages: HISTORY_SIZE}));

  console.info("W positive done");

  await motor_controller.send_command({command: motor.SET_STATE_HOLD_V_NEGATIVE, ...drive_options});
  await wait(settle_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_V_DECREASING, ...test_options});

  const v_negative = calculate_data_stats(await motor_controller.get_readouts({expected_messages: HISTORY_SIZE}));

  console.info("V negative done");

  // Check all calibration data is complete.
  if (u_positive.length !== HISTORY_SIZE) {
    console.error("U positive calibration data incomplete", u_positive);
    return;
  }
  if (u_negative.length !== HISTORY_SIZE) {
    console.error("U negative calibration data incomplete", u_negative);
    return;
  }
  if (v_positive.length !== HISTORY_SIZE) {
    console.error("V positive calibration data incomplete", v_positive);
    return;
  }
  if (v_negative.length !== HISTORY_SIZE) {
    console.error("V negative calibration data incomplete", v_negative);
    return;
  }
  if (w_positive.length !== HISTORY_SIZE) {
    console.error("W positive calibration data incomplete", w_positive);
    return;
  }
  if (w_negative.length !== HISTORY_SIZE) {
    console.error("W negative calibration data incomplete", w_negative);
    return;
  }

  const targets = d3.range(HISTORY_SIZE).map((i) => {
    const t = i * millis_per_cycle;
    const zone = current_calibration_zones.filter((zone) => zone.settle_start <= t && t <= zone.settle_end);

    if (zone.length == 0) return null;
    if (zone.length > 1) {
      console.error("Multiple zones found", zone);
      return null;
    }
    return zone[0].pwm * calibration_reference;
  });

  // Make a new table with each calibration phase as a column.
  const calibration_data = d3.range(HISTORY_SIZE).map((i) => {
    return {
      time: u_positive[i].time,
      target: targets[i],
      u_positive: u_positive[i].u_readout,
      u_negative: -u_negative[i].u_readout,
      v_positive: v_positive[i].v_readout,
      v_negative: -v_negative[i].v_readout,
      w_positive: w_positive[i].w_readout,
      w_negative: -w_negative[i].w_readout,
    };
  });

  console.info("Current calibration done");
  
  update_current_calibration_results(calibration_data);
}

```


```js
const current_calibration_table = html`<div>Phase correction factors:</div>
  <pre>${JSON.stringify(calculated_current_calibration_factors, null, 2)}</pre>`;

const current_calibration_result_to_display_input = Inputs.select(d3.range(current_calibration_results.length), {
  value: current_calibration_results.length - 1,
  label: "Select calibration result to display:",
});
const current_calibration_result_to_display = Generators.input(current_calibration_result_to_display_input);

```

```js

const current_calibration_plot = plot_lines({
  data: current_calibration_results[current_calibration_result_to_display],
  subtitle: "Current Calibration",
  description: "Current calibration results for each phase.",
  width: 1200, height: 400,
  x_domain: [0, HISTORY_SIZE * millis_per_cycle],
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

const calibration_samples = current_calibration_reading_points.map((x, i) => {
  return {
    reading: x,
    target: x,
    u_positive: current_calibration.u_positive.sample[i].target,
    u_negative: current_calibration.u_negative.sample[i].target,
    v_positive: current_calibration.v_positive.sample[i].target,
    v_negative: current_calibration.v_negative.sample[i].target,
    w_positive: current_calibration.w_positive.sample[i].target,
    w_negative: current_calibration.w_negative.sample[i].target,
  };
});

const current_calibration_interpolate_plot = plot_lines({
  data: calibration_samples,
  subtitle: "Current Calibration Interpolation",
  description: "Current calibration interpolation results for each phase.",
  width: 1200, height: 400,
  x_domain: [0, calibration_reference],
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
  data: current_calibration_stats,
  subtitle: "Mean Response - Positive",
  description: "Current calibration mean results for each phase driven positive.",
  width: 1200, height: 300,
  x_domain: [0, HISTORY_SIZE * millis_per_cycle],
  x: "time",
  x_label: "Time (ms)",
  y_label: "Current (A)",
  channels: [
    {
      y: "u_positive", 
      draw_extra: setup_faint_area({
        y0: d => d.u_positive - d.u_positive_std * std_95_z_score,
        y1: d => d.u_positive + d.u_positive_std * std_95_z_score,
      }),
      label: "U positive mean", color: colors.u,
    },
    {
      y: "v_positive",
      draw_extra: setup_faint_area({
        y0: d => d.v_positive - d.v_positive_std * std_95_z_score,
        y1: d => d.v_positive + d.v_positive_std * std_95_z_score,
      }),
      label: "V positive mean", color: colors.v,
    },
    {
      y: "w_positive",
      draw_extra: setup_faint_area({
        y0: d => d.w_positive - d.w_positive_std * std_95_z_score,
        y1: d => d.w_positive + d.w_positive_std * std_95_z_score,
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
  data: current_calibration_stats,
  subtitle: "Mean Response - Negative (inverted)",
  description: "Current calibration mean results for each phase driven negative.",
  width: 1200, height: 300,
  x_domain: [0, HISTORY_SIZE * millis_per_cycle],
  x: "time",
  x_label: "Time (ms)",
  y_label: "Current (A)",
  channels: [
    {
      y: "u_negative", 
      draw_extra: setup_faint_area({
        y0: d => d.u_negative - d.u_negative_std * std_95_z_score,
        y1: d => d.u_negative + d.u_negative_std * std_95_z_score,
      }),
      label: "U negative mean", color: colors.u,
    },
    {
      y: "v_negative",
      draw_extra: setup_faint_area({
        y0: d => d.v_negative - d.v_negative_std * std_95_z_score,
        y1: d => d.v_negative + d.v_negative_std * std_95_z_score,
      }),
      label: "V negative mean", color: colors.v,
    },
    {
      y: "w_negative",
      draw_extra: setup_faint_area({
        y0: d => d.w_negative - d.w_negative_std * std_95_z_score,
        y1: d => d.w_negative + d.w_negative_std * std_95_z_score,
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

import {plot_lines, setup_faint_area, horizontal_step} from "./components/plotting_utils.js";
import {localStorage, get_stored_or_default, clear_stored_data} from "./components/local_storage.js";
import {round, uint32_to_bytes, bytes_to_uint32, timeout_promise, wait, clean_id}  from "./components/utils.js";
import {even_spacing, piecewise_linear, even_piecewise_linear} from "./components/math_utils.js";
import * as motor from "./components/usb_motor_controller.js";

import {enabled_checkbox, autosave_inputs, any_checked_input} from "./components/input_utils.js";
import {process_readout_with_calibration, cycles_per_millisecond, millis_per_cycle, online_map, online_function_chain, current_calibration_default} from "./components/readout_processing.js";

import {interpolate_degrees, shortest_distance_degrees, normalize_degrees, circular_stats_degrees} from "./components/angular_math.js";

import {MAX_TIMEOUT, PWM_BASE, PWM_PERIOD, HISTORY_SIZE} from "./components/motor_driver_constants.js";

```