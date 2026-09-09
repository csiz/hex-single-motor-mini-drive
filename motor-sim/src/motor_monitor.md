---
title: Motor monitor
---

<!-- This is the monitor page for the motor controller, the code here is used to showcase
the typical usage of the javascript motor interface in order to control the motor.

Note that we are using the Observable Framework (https://observablehq.com/framework/) for
reactivity. The code blocks encoded by ```js ... ``` are reactive and will automatically
update when the underlying data changes. Most top level variables are effectively async 
iterators and the framework allows us to work with them as if they are simple variables,
simplifying the code. It also allows for imports and misc functions to be written last.

The code is organized as follow:

1. The HTML layout is specified at the top.

2. The first code block contains the connecting logic for the motor controller and handles
the resulting data stream and status updates. We can connect to multiple motors at a time.

3. The second block defines the interactive buttons and inputs for controlling the motor.

4. The following blocks setup the plotting and motor feedback. The UI configuration is
long and verbose, but intentionally kept on this page for completeness. You may use ctrl+F
to search for any term visible on the application page. Every bit of text from the motor
monitor page can be found written somewhere in this file.

5. Finally we have on device unit tests, imports, and other utility code.

-->

<main class="hero">


Motor Commands
--------------
<div class="card tight">
<div>${connect_buttons}</div>
<div>${connection_status}</div>
<div>${opened_ports_radio}</div>
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
<div>
  <span>${seek_drive_buttons}</span>
  <span>${command_seek_rotation_slider}</span>
  <span>${shared_seek_buttons}</span>
</div>
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


Current Calibration Procedures
------------------------------

<div class="card tight">
  <div>${current_calibration_buttons}</div>
  <p>Active electrical phase properties:</p>
  <pre>${active_current_calibration_table}</pre>
  <div>${Object.values(current_calibration_input)}</div>
</div>
<div class="card tight">
  <div>${current_calibration_run_buttons}</div>
  <div>${current_calibration_pwm_slider}</div>
  <div>${current_calibration_test_speed_slider}</div>
  <div>${current_calibration_test_duration_slider}</div>
  <div>${current_calibration_angle_slider}</div>
  <div>${current_calibration_checkboxes}</div>
  <h3>Current Calibration Results</h3>
  <div>${current_calibration_optimization_iteration_input}</div>
  <div>
    <p>Electrical phase properties at displayed iteration:</p>
    <pre>${current_calibration_iteration_table}</pre>
    <div>${current_calibration_optimizing_plot}</div>
    <div>${current_calibration_angles_plot}</div>
    <div>${current_calibration_angles_scatter}</div>
    <div>${current_calibration_optimizing_gradients_plot}</div>
  </div>

</div>


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


Flash Data Storage
------------------

<div class="card tight">
  <p>Commit the uploaded calibration data to flash memory.</p>
  <div>${flash_buttons}</div>
  <pre>${flash_status}</pre>
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
// USB Connection and Data
// -----------------------

// Active data to be displayed on the main plots.
let data = Mutable([]);

// Active motor controller (can be null or selected from multiple motor connections).
let motor_controller = Mutable(null);

// Currently active USB port.
let active_port = Mutable(null);

// Opened USB ports, and respectively the motor controllers and data associated with each.
let opened_ports = Mutable(new Map());

// Prominently displayed status for the motor monitor page.
let connection_status = Mutable(html`<pre>Not connected.</pre>`);


// Data management
// ---------------

function set_readout_series(new_data, motor_uri){
  if (motor_uri === active_port.value) data.value = new_data;

  opened_ports.value.get(motor_uri).data = new_data;
}

function update_readout_series(readout, motor_uri){
  // Reset the data when the controller detects a new readout series and resets the index to 0.
  const first_readout = (readout.readout_index === 0);

  const previous_data = opened_ports.value.get(motor_uri)?.data ?? [];

  const new_data = first_readout ? [readout] : capped_push(previous_data, readout);

  if (motor_uri === active_port.value) data.value = new_data;

  opened_ports.value.get(motor_uri).data = new_data;
};


function set_status(status, motor_uri){
  if (!motor_controller.value || motor_uri === active_port.value) connection_status.value = status;

  opened_ports.value.get(motor_uri).status = status;
}

function set_error(error, motor_uri){
  if (!motor_controller.value || motor_uri === active_port.value) {
    connection_status.value = displayable_connection_error(error);
  }

  opened_ports.value.get(motor_uri).error = error;
}

function set_motor_controller(controller, motor_uri){
  opened_ports.value.get(motor_uri).motor_controller = controller;

  // Self assign to update the `Mutable`.
  opened_ports.value = opened_ports.value;
}

function switch_to_port(motor_uri){
  const port_state = opened_ports.value.get(motor_uri);

  if (!port_state) return false;

  active_port.value = motor_uri;

  motor_controller.value = port_state.motor_controller;
  connection_status.value = port_state.error ? displayable_connection_error(port_state.error) : port_state.status;
  data.value = port_state.data;

  return true;
}

// Initialize Motor Driver via USB or WiFi
// ---------------------------------------

const default_connect_options = {force_ws: false};

async function connect_motor_controller(options = {}) {
  const {force_ws} = {...default_connect_options, ...options};
  
  try {
    const port_index = force_ws ? null : await prompt_com_port_get_index();

    const motor_uri = (port_index === null || force_ws) ? default_ws_uri : `usb:${port_index}`;

    // Switch to the existing motor controller if this port is already opened and initialized.
    if (switch_to_port(motor_uri)) return;

    // Append this port to the opened ports list so we can capture it's errors too.
    opened_ports.value = opened_ports.value.set(motor_uri, {
      motor_uri,
      motor_controller: null,
      data: [],
      status: html`<pre>Opening USB com port.</pre>`,
      error: undefined,
    });

    const motor_controller = new MotorController({
      motor_uri,
      onstatus: (status) => {
        set_status(html`<pre>Connected; ${format_data_rate(status)}.</pre>`, motor_uri);
      },
      onmessage: (readout) => {
        update_readout_series(readout, motor_uri);
      },
      onready: (motor_controller) => {
        set_status(html`<pre>Connected, waiting for your commands.</pre>`, motor_uri);
        set_motor_controller(motor_controller, motor_uri);
        switch_to_port(motor_uri);
      },
      onerror: (error) => {
        set_error(error, motor_uri);
      },
      onclose: () => {
        opened_ports.value.delete(motor_uri);
        // Self assign to update the `Mutable`.
        opened_ports.value = opened_ports.value;
      },
    });


    // Switch the display to the newly opening port.
    switch_to_port(motor_uri);

  } catch (error) {
    connection_status.value = displayable_connection_error(error);
  }
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

```

```js

// Setup Buttons and Inputs
// ------------------------

// Automatically connect the motor driver if we have permissions from previous session.
connect_motor_controller({force_ws: false});

// Disconnect when the notebook is reloaded.
invalidation.then(disconnect_motor_controller);

// Buttons to manually connect, disconnect and manage multiple motor connections.
const connect_buttons = Inputs.button(
  [
    ["Connect USB", () => connect_motor_controller({force_ws: false})],
    ["Connect WiFi", () => connect_motor_controller({force_ws: true})],
    ["Disconnect", disconnect_motor_controller],
    // ["Reset data & inputs", () => (clear_stored_data(), location.reload())],
  ],
  {label: "Connect to COM"},
);
d3.select(connect_buttons).selectAll("button").style("height", "3em");

// The following variables are pairs of input html elements and their corresponding input value observers.

const command_options_input = Inputs.checkbox(
  ["Take snapshot after command"],
  {
    value: [],
    label: "Command options:",
  },
);
d3.select(command_options_input).select("div").style("width", "100%");
d3.select(command_options_input).select("div label").style("width", "100em");
const command_snapshot = transformed_input_value(command_options_input, (options) => options.includes("Take snapshot after command"));

// Delay the command snapshot by a set millisecond duration from the start of the main command.
const command_snapshot_delay_slider = inputs_wide_range([0, 1000], {value: 500, step: 1, label: "Snapshot delay (ms):"});
const command_snapshot_delay = Generators.input(command_snapshot_delay_slider);

// Slider to choose the command specific PWM setting.
const command_pwm_slider = inputs_wide_range([0, 1.0], {value: 0.05, step: 0.001, label: "Command value:"});
const command_pwm = transformed_input_value(command_pwm_slider, (value) => Math.round(value * PWM_BASE));

// Timeout duration for each command. For safety, the motor runs each drive command for a short period until commanded again.
const command_timeout_slider = inputs_wide_range([0, max_timeout*millis_per_cycle], {value: 510, step: 5, label: "Command timeout (ms):"});
const command_timeout = transformed_input_value(command_timeout_slider, (millis) => Math.floor(millis * cycles_per_millisecond));

// Choose the angular speed target for certain commands.
const command_angular_speed_slider = inputs_wide_range(
  [
    0.0,
    speed_units_to_rotations_per_millisecond(max_angular_speed)
  ], 
  {value: 0.20, step: 0.01, label: "Angular speed value (rotations/ms)"});
const command_angular_speed = transformed_input_value(command_angular_speed_slider, rotations_per_millisecond_to_speed_units);

// Choose the target angle for certain commands.
const command_angle_slider = inputs_wide_range([-Math.PI, Math.PI], {value: 0, step: 0.01, label: "Command angle (radians):"});
const command_angle = transformed_input_value(command_angle_slider, radians_to_angle_units);

// Choose the torque target for torque driving modes.
const command_torque_current_slider = inputs_wide_range([0, max_drive_current], {value: 0.750, step: 0.010, label: "Command torque (Amps):"});
const command_torque_current = transformed_input_value(command_torque_current_slider, (amps) => amps * CURRENT_UNITS_PER_AMP);

// Choose the power target for power driving modes.
const command_power_slider = inputs_wide_range([0, max_drive_power], {value: 1.500, step: 0.010, label: "Command power (Watts):"});
const command_power = Generators.input(command_power_slider);

// Choose the position for target seeking.
const command_seek_rotation_slider = inputs_wide_range([-1024, +1024], {value: 0, step: 1, label: "Seek angle (rotations):"});
const command_seek_rotation = Generators.input(command_seek_rotation_slider);


// We must start a new code cell for the variables above to become observables.
```
```js

const opened_ports_radio = Inputs.radio(
  opened_ports.keys(),
  {
    label: "Opened ports", 
    value: active_port,
    format: (port, i) => {
      return html`<pre>Motor ${i}</pre>`;
    },
  }
);
opened_ports_radio.addEventListener("input", function(){
  const port = opened_ports_radio.value;
  if (port) switch_to_port(port);
});

// Control functions
// -----------------

// Send a command, automatically adding the shared timeout parameter.
async function send_command(message){
  if (!motor_controller) return;

  if (typeof message === "object") {
    message = {
      ...message,
      timeout: command_timeout,
    };
  }

  await motor_controller.send_command(message);
}

// Take a snapshot of continuously recorded readouts from the motor controller.
async function take_readout_snapshot(command_options = {}){
  if (!motor_controller) return;

  // Remember the current controller and port before we await.
  const controller = motor_controller;
  const port = active_port;

  const reply_data = await controller.send_command_and_await_reply({
    message: MessageCode.GET_READOUTS_SNAPSHOT,
    expected_code: MessageCode.READOUT,
    expected_messages: HISTORY_SIZE,
    ...command_options,
  });

  set_readout_series(reply_data, port);
}

// Run the test driving commands. The test commands can send a snapshot taken
// during the test procedure if requested (using the timeout parameter as a flag.)
async function test_command(message_code){
  const message = {
    message_code, 
    pwm_value: command_pwm, 
    test_speed: command_angular_speed,
    test_duration: command_timeout,
    take_snapshot: command_snapshot ? 1 : 0,
  };

  if (command_snapshot){
    await take_readout_snapshot({
      message,
      expected_code: MessageCode.READOUT,
      expected_messages: HISTORY_SIZE,
    });
  } else {
    await send_command(message);
  }
}

// Take a snapshot after a user specified delay using a slider in the UI.
const delayed_readout_snapshot = _.debounce(take_readout_snapshot, command_snapshot_delay);

// Take a snapshot if the snapshot option is checked in the UI.
async function snapshot_if_checked(message){
  await send_command(message);

  if (command_snapshot) delayed_readout_snapshot();
}


async function stream_all_motors(){
  for (const {motor_controller} of opened_ports.values()) {
    if (!motor_controller) continue;
    motor_controller.reset_history();
    await motor_controller.send_command({message_code: MessageCode.STREAM_FULL_READOUTS, stream_state: 1});
  }
}

let command_interval = null;
const command_loop_period = 1;

function stop_command_loop(){
  if(command_interval) {
    clearInterval(command_interval);
    command_interval = null;
  }
}

function start_command_loop(loop_function){
  stop_command_loop();
  stream_all_motors();
  command_interval = setInterval(loop_function, command_loop_period);
}

// All the buttons
// ---------------



const data_request_buttons = Inputs.button(
  [
    ["Uninterrupted snapshot", async function(){ 
      await take_readout_snapshot(); 
    }],
    ["Stream motor data", async function(){
      motor_controller.reset_history();
      await send_command({message_code: MessageCode.STREAM_FULL_READOUTS, stream_state: 1});
    }],
    ["STOP stream", async function(){
      stop_command_loop();

      await send_command({message_code: MessageCode.STREAM_FULL_READOUTS, stream_state: 0});
    }],
    ["Stream all motors", stream_all_motors],
    ["Stop all motors", async function(){
      stop_command_loop();

      for (const {motor_controller} of opened_ports.values()) {
        if (!motor_controller) continue;
        await motor_controller.send_command({message_code: MessageCode.STREAM_FULL_READOUTS, stream_state: 0});
      }
    }],
  ],
  {label: "Read data"},
);
d3.select(data_request_buttons).selectAll("button").style("height", "4em");

const test_buttons_to_code = [
  ["Test all permutation", MessageCode.SET_STATE_TEST_ALL_PERMUTATIONS],
  ["Test ground short", MessageCode.SET_STATE_TEST_GROUND_SHORT],
  ["Test positive short", MessageCode.SET_STATE_TEST_POSITIVE_SHORT],
  ["Test U directions", MessageCode.SET_STATE_TEST_U_DIRECTIONS],
  ["Test U increasing", MessageCode.SET_STATE_TEST_U_INCREASING],
  ["Test U decreasing", MessageCode.SET_STATE_TEST_U_DECREASING],
  ["Test V increasing", MessageCode.SET_STATE_TEST_V_INCREASING],
  ["Test V decreasing", MessageCode.SET_STATE_TEST_V_DECREASING],
  ["Test W increasing", MessageCode.SET_STATE_TEST_W_INCREASING],
  ["Test W decreasing", MessageCode.SET_STATE_TEST_W_DECREASING],
  ["Resistance calibration", MessageCode.SET_STATE_RESISTANCE_CALIBRATION],
  ["Inductance calibration", MessageCode.SET_STATE_INDUCTANCE_CALIBRATION],
  ["Rotating chirp calibration", MessageCode.SET_STATE_ROTATING_CALIBRATION_CHIRP],
  ["Fixed chirp calibration", MessageCode.SET_STATE_FIXED_CALIBRATION_CHIRP],
];

const test_buttons = Inputs.button(
  test_buttons_to_code.map(([label, code]) => [label, () => test_command(code)]),
  {label: "Test sequence"},
);
d3.select(test_buttons).selectAll("button").style("height", "4em");


const stop_buttons = Inputs.button(
  [
    ["Stop / Brake", async function(){
      stop_command_loop();

      await snapshot_if_checked({message_code: MessageCode.SET_STATE_OFF});
    }],
    ["Freewheel", async function(){
      stop_command_loop();

      await snapshot_if_checked({message_code: MessageCode.SET_STATE_FREEWHEEL});
    }],
    ["Stop all motors", async function(){
      stop_command_loop();

      for (const {motor_controller} of opened_ports.values()) {
        if (!motor_controller) continue;
        await motor_controller.send_command({message_code: MessageCode.SET_STATE_OFF});
      }
    }]
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
    ["Drive 6S +", async function(){
      await snapshot_if_checked({message_code: MessageCode.SET_STATE_DRIVE_6_SECTOR, pwm_value: +command_pwm});
    }],
    ["Drive 6S -", async function(){
      await snapshot_if_checked({message_code: MessageCode.SET_STATE_DRIVE_6_SECTOR, pwm_value: -command_pwm});
    }],
    ["Hold U positive", async function(){
      await snapshot_if_checked({message_code: MessageCode.SET_STATE_HOLD_U_POSITIVE, pwm_value: command_pwm});
    }],
    ["Hold V positive", async function(){
      await snapshot_if_checked({message_code: MessageCode.SET_STATE_HOLD_V_POSITIVE, pwm_value: command_pwm});
    }],
    ["Hold W positive", async function(){
      await snapshot_if_checked({message_code: MessageCode.SET_STATE_HOLD_W_POSITIVE, pwm_value: command_pwm});
    }],
    ["Hold U negative", async function(){
      await snapshot_if_checked({message_code: MessageCode.SET_STATE_HOLD_U_NEGATIVE, pwm_value: command_pwm});
    }],
    ["Hold V negative", async function(){
      await snapshot_if_checked({message_code: MessageCode.SET_STATE_HOLD_V_NEGATIVE, pwm_value: command_pwm});
    }],
    ["Hold W negative", async function(){
      await snapshot_if_checked({message_code: MessageCode.SET_STATE_HOLD_W_NEGATIVE, pwm_value: command_pwm});
    }],
  ],
  {label: "Simple drive commands"},
);
d3.select(simple_drive_buttons).selectAll("button").style("height", "4em");


const advanced_drive_buttons = Inputs.button(
  [
    ["Set Angle", async function(){
      await snapshot_if_checked({
        message_code: MessageCode.SET_ANGLE, 
        angle: command_angle,
      });
    }],
    ["Drive periodic +", async function(){
      await snapshot_if_checked({
        message_code: MessageCode.SET_STATE_DRIVE_PERIODIC,
        pwm_value: command_pwm, 
        angular_speed: +command_angular_speed, 
        angle: command_angle,
      });
    }],
    ["Drive periodic -", async function(){
      await snapshot_if_checked({
        message_code: MessageCode.SET_STATE_DRIVE_PERIODIC,
        pwm_value: command_pwm, 
        angular_speed: -command_angular_speed, 
        angle: command_angle,
      });
    }],
    ["Drive smooth +", async function(){
      await snapshot_if_checked({
        message_code: MessageCode.SET_STATE_DRIVE_SMOOTH,
        pwm_value: +command_pwm
      });
    }],
    ["Drive smooth -", async function(){
      await snapshot_if_checked({
        message_code: MessageCode.SET_STATE_DRIVE_SMOOTH,
        pwm_value: -command_pwm
      });
    }],
    ["Drive torque +", async function(){
      await snapshot_if_checked({
        message_code: MessageCode.SET_STATE_DRIVE_TORQUE,
        target_current: +command_torque_current
      });
    }],
    ["Drive torque -", async function(){
      await snapshot_if_checked({
        message_code: MessageCode.SET_STATE_DRIVE_TORQUE,
        target_current: -command_torque_current
      });
    }],
    ["Drive power +", async function(){
      await snapshot_if_checked({
        message_code: MessageCode.SET_STATE_DRIVE_BATTERY_POWER,
        target_power: +command_power,
      });
    }],
    ["Drive power -", async function(){
      await snapshot_if_checked({
        message_code: MessageCode.SET_STATE_DRIVE_BATTERY_POWER,
        target_power: -command_power,
      });
    }],
    ["Drive speed +", async function(){
      await snapshot_if_checked({
        message_code: MessageCode.SET_STATE_DRIVE_SPEED,
        target_speed: +command_angular_speed
      });
    }],
    ["Drive speed -", async function(){
      await snapshot_if_checked({
        message_code: MessageCode.SET_STATE_DRIVE_SPEED,
        target_speed: -command_angular_speed
      });
    }],
    ["Drive torque speed +", async function(){
      await snapshot_if_checked({
        message_code: MessageCode.SET_STATE_DRIVE_TORQUE_SPEED,
        target_speed: +command_angular_speed,
        target_current: +command_torque_current,
      });
    }],
    ["Drive torque speed -", async function(){
      await snapshot_if_checked({
        message_code: MessageCode.SET_STATE_DRIVE_TORQUE_SPEED,
        target_speed: -command_angular_speed,
        target_current: -command_torque_current,
      });
    }],
  ],
  {label: "Advanced drive commands"},
);
d3.select(advanced_drive_buttons).selectAll("button").style("height", "4em");


const seek_drive_buttons = Inputs.button(
  [
    ["Set Rotations", async function(){
      await snapshot_if_checked({
        message_code: MessageCode.SET_ROTATIONS, 
        rotations: command_seek_rotation,
      });
    }],
    ["Seek angle", async function(){
      await snapshot_if_checked({
        message_code: MessageCode.SET_STATE_SEEK_ANGLE,
        target_rotation: command_seek_rotation, 
        target_angle: command_angle,
        target_speed: command_angular_speed,
        target_current: command_torque_current,
      });
    }],
    ["Go to zero", async function(){
      await snapshot_if_checked({
        message_code: MessageCode.SET_STATE_SEEK_ANGLE,
        target_rotation: 0, 
        target_angle: 0,
        target_speed: command_angular_speed,
        target_current: command_torque_current,
      });
    }],
  ],
  {label: "Seek to target position"},
);
d3.select(seek_drive_buttons).selectAll("button").style("height", "4em");



const shared_prediction_millis = 100;

function get_average_position(){
  const valid_ports = Array.from(opened_ports.values()).filter(({motor_controller, data}) => {
    return motor_controller && data?.length >= 1;
  });

  if (valid_ports.length < 2) return;

  return Math.round(d3.mean(valid_ports, ({data}) => {
    const last_readout = _.last(data);
    return last_readout.rotations + last_readout.angular_speed / 360 * shared_prediction_millis;
  }));
}


const shared_seek_buttons = Inputs.button(
  [
    ["Virtual spring", function(){

      start_command_loop(() => {
        const target_position = get_average_position();
        if (!_.isFinite(target_position)) return;

        command_seek_rotation_slider.value = target_position;

        for (const {motor_controller: controller} of opened_ports.values()) {
          if (!controller) continue;

          controller.send_command({
            message_code: MessageCode.SET_STATE_SEEK_ANGLE_WITH_TORQUE,
            timeout: command_timeout,
            target_angle: command_angle,
            max_drive_current: command_torque_current,
            target_rotation: target_position,
          });
        }
      });

    }],

    ["Follow selected motor", function(){

      start_command_loop(() => {
        // Can't use the `data` generator because it updates too fast and stops our buttons from working.
        const target_position = _.last(Array.from(opened_ports.values()).filter(({motor_controller: controller}) => {
          return motor_controller === controller;
        })[0]?.data)?.rotations;

        if (!_.isFinite(target_position)) return;

        command_seek_rotation_slider.value = target_position;

        for (const {motor_controller: controller} of opened_ports.values()) {
          if (!controller) continue;

          controller.send_command({
            message_code: MessageCode.SET_STATE_SEEK_ANGLE_WITH_POWER,
            timeout: command_timeout,
            target_angle: command_angle,
            max_drive_power: controller === motor_controller ? 0 : command_power,
            target_rotation: target_position,
          });
        }
      });

    }],

    ["Match all positions", function(){

      start_command_loop(() => {
        const target_position = get_average_position();
        if (!_.isFinite(target_position)) return;

        command_seek_rotation_slider.value = target_position;

        for (const {motor_controller: controller} of opened_ports.values()) {
          if (!controller) continue;

          controller.send_command({
            message_code: MessageCode.SET_STATE_SEEK_ANGLE_WITH_POWER,
            timeout: command_timeout,
            target_angle: command_angle,
            max_drive_power: command_power,
            target_rotation: target_position,
          });
        }
      });

    }],
  ],
  {label: "Multi-motor control"},
);
d3.select(shared_seek_buttons).selectAll("button").style("height", "4em");


invalidation.then(stop_command_loop);
```




```js

// Readout Plotting
// ----------------

const max_timeline_period = 2000; // ms

const history_duration = Math.ceil(HISTORY_SIZE * millis_per_cycle);

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
  y_label: "Electric position (radians)",
  y_domain: [-Math.PI, Math.PI],
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
  description: "Power consumed by the motor.",
  width: 1200, height: 300,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Power (W)",
  channels: [
    {y: "total_power", label: "Total Power", color: colors.sum},
    {y: "total_power_average", label: "Total Power (on chip average)", color: d3.color(colors.sum).darker(1)},
    {
      y: "total_power_avg", label: "Total Power (2ms average)", color: d3.color(colors.sum).darker(1),
      draw_extra: setup_stdev_95({stdev: (d) => d.total_power_stdev}),
    },
    {y: "resistive_power", label: "Resistive Power", color: colors.web_current_magnitude},
    {y: "resistive_power_average", label: "Resistive Power (on chip average)", color: d3.color(colors.web_current_magnitude).darker(1)},
    {y: "emf_power", label: "EMF Power", color: colors.angle},
    {
      y: "emf_power_avg", label: "EMF Power (2ms average)", color: d3.color(colors.angle).darker(1),
      draw_extra: setup_stdev_95({stdev: (d) => d.emf_power_stdev}),
    },
    {y: "inductive_power", label: "Inductive Power", color: colors.current_angle},
    
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
    {y: "main_loop_rate", label: "Main loop rate", color: colors.sum},
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
    {y: (d) => d.cycle_start_tick - PWM_BASE, label: "Ticks at start since mid cycle", color: d3.color(colors.u).brighter(1)},
  ],
  curve,
});



const plot_electric_position = plot_lines({
  subtitle: "Electric position",
  description: "Angular position of the rotor with respect to the electric phases, 0 when magnetic N is aligned with phase U.",
  width: 1200, height: 200,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Electric position (radians)",
  y_domain: [-Math.PI, Math.PI],
  channels: [
    {y: "angle", label: "Magnet Angle", color: colors.angle},
    {y: (d) => d.current_detected ? d.current_angle : null, label: "Current Angle", color: colors.web_angle},
    {y: (d) => d.web_current_magnitude > 0.010 ? d.web_current_angle : null, label: "Current Angle (computed online)", color: colors.current_angle},
    {y: "emf_voltage_angle", label: "EMF Voltage Angle", color: colors.voltage_angle},
    {
      y: (d) => d.web_emf_voltage_angle, label: "EMF Voltage Angle (computed online)", color: colors.voltage_angle,
      draw_extra: setup_stdev_95({stdev: (d) => d.emf_angle_error_stdev}),
    },
    {y: (d) => d.drive_voltage_magnitude > 0 ? d.drive_voltage_angle : null, label: "Drive Voltage Angle", color: colors_categories[2]},
  ],
  curve,
});

const plot_electric_offsets = plot_lines({
  subtitle: "Electric Offsets",
  description: "Offsets for the electric angles.",
  width: 1200, height: 200,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Angle (radians)",
  channels: [
    {y: (d) => d.current_detected ? d.current_angle_offset : null, label: "Current Angle Offset", color: colors.current_angle},
    {
      y: (d) => d.emf_detected ? d.emf_voltage_angle_offset : null, label: "EMF Voltage Angle Offset", color: colors.voltage_angle,
      draw_extra: setup_stdev_95({stdev: (d) => d.emf_angle_error_stdev}),
    },
    {y: "angle_adjustment", label: "Magnet Angle Correction", color: d3.color(colors.angle).darker(1)},
    {y: "emf_angle_error_stdev", label: "EMF Angle Error (stdev)", color: d3.color(colors.voltage_angle).darker(1)},
    {y: "lead_angle", label: "Lead Angle", color: colors.v},
    {y: (d) => d.drive_voltage_magnitude > 0 ? d.drive_voltage_angle_offset : null, label: "Drive Voltage Angle Offset", color: colors_categories[2]},
  ],
  curve,
});


const plot_speed = plot_lines({
  subtitle: "Rotor Speed",
  description: "Angular speed of the rotor in rotations per millisecond.",
  width: 1200, height: 300,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Angular Speed (rotations/ms)",
  channels: [
    {y: "angular_speed", label: "Magnet Angular Speed", color: colors.angular_speed},
    {y: (d) => d.rotor_acceleration * 10, label: "Rotor Acceleration (10ms speed diff)", color: colors.angle_driven},
    {y: "emf_voltage_angular_speed", label: "EMF Voltage Angular Speed", color: colors.voltage_angle},
  ],
  curve,
});


const plot_measured_voltage = plot_lines({
  subtitle: "VCC Voltage",
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
  subtitle: "Temperature",
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
  subtitle: "Currents",
  description: "Measured current values for each phase.",
  width: 1200, height: 400,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Current (A)",
  channels: [
    {y: "u_current", label: "Current U", color: colors.u},
    {y: "v_current", label: "Current V", color: colors.v},
    {y: "w_current", label: "Current W", color: colors.w},
    {y: "u_current_diff", label: "Current U diff", color: d3.color(colors.u).darker(1)},
    {y: "v_current_diff", label: "Current V diff", color: d3.color(colors.v).darker(1)},
    {y: "w_current_diff", label: "Current W diff", color: d3.color(colors.w).darker(1)},
    {y: "battery_current", label: "Battery Current", color: colors.other},
    {y: "zero_current", label: "Current Sum/3", color: colors.sum},
    {y: "ref_readout", label: "Reference value", color: colors.ref_readout},
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
    {y: "u_partial_voltage", label: "Partial Voltage U", color: d3.color(colors.u).darker(1)},
    {y: "v_partial_voltage", label: "Partial Voltage V", color: d3.color(colors.v).darker(1)},
    {y: "w_partial_voltage", label: "Partial Voltage W", color: d3.color(colors.w).darker(1)},
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
    {y: "direct_current", label: "Current on Direct axis", color: colors.direct_current},
    {y: "quadrature_current", label: "Current on Quadrature", color: colors.quadrature_current},
    {y: "current_magnitude", label: "Current Magnitude", color: colors_categories[2]},
    {y: "web_direct_current", label: "Current on Direct axis (computed online)", color: d3.color(colors.direct_current).brighter(1)},
    {y: "web_quadrature_current", label: "Current on Quadrature (computed online)", color: d3.color(colors.quadrature_current).brighter(1)},
    {y: "steady_state_drive_current", label: "Drive Current if stalling", color: d3.color(colors.direct_current).darker(1)},
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
    {y: "direct_emf_voltage", label: "EMF Voltage on Direct axis", color: colors.direct_current},
    {y: "quadrature_emf_voltage", label: "EMF Voltage on Quadrature", color: colors.quadrature_current},
    {y: "emf_voltage_magnitude", label: "EMF Voltage Magnitude", color: colors.web_current_magnitude},
    {y: "emf_angle_error_stdev", label: "EMF Voltage Stdev", color: d3.color(colors.web_current_magnitude).darker(1)},
    {y: "web_direct_emf_voltage", label: "Voltage on Direct axis (computed online)", color: d3.color(colors.direct_current).brighter(1)},
    {y: "web_quadrature_emf_voltage", label: "Voltage on Quadrature (computed online)", color: d3.color(colors.quadrature_current).brighter(1)},
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
  channels: [
    {y: "u_pwm", label: "PWM U", color: colors.u},
    {y: "v_pwm", label: "PWM V", color: colors.v},
    {y: "w_pwm", label: "PWM W", color: colors.w},
    {y: "live_max_pwm", label: "Live Max PWM", color: colors_categories[0]},
    {y: "active_pwm", label: "Active PWM", color: colors_categories[1]},
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
    {y: "nominal_vcc_voltage", label: "Nominal VCC Voltage", color: colors_categories[8]},
    {y: "overpowered", label: "Overpowered", color: colors_categories[9]},
    {y: "overheating", label: "Overheating", color: colors_categories[10]},
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
    {y: "target", label: "Target", color: colors_categories[2]},
    {y: "seek_integral", label: "Seek Integral", color: colors_categories[3]},
    {y: "u_resistance", label: "Phase U Resistance", color: colors.u},
    {y: "v_resistance", label: "Phase V Resistance", color: colors.v},
    {y: "w_resistance", label: "Phase W Resistance", color: colors.w},
    {y: "phase_inductance_base", label: "Phase Inductance Baseline", color: colors_categories[4]},
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

const current_calibration_pwm_slider = inputs_wide_range([0, PWM_BASE], {value: PWM_BASE * 0.04, step: 1.0, label: "Calibration max PWM:"});

const current_calibration_pwm = Generators.input(current_calibration_pwm_slider);


const current_calibration_test_speed_slider = inputs_wide_range(
  [
    -speed_units_to_rotations_per_millisecond(max_angular_speed), 
    +speed_units_to_rotations_per_millisecond(max_angular_speed)
  ], 
  {value: 1.0, label: "Calibration test speed (rotations/ms):"});

const current_calibration_test_speed = transformed_input_value(current_calibration_test_speed_slider, rotations_per_millisecond_to_speed_units);

const current_calibration_test_duration_slider = inputs_wide_range([0, 16*HISTORY_SIZE], {value: HISTORY_SIZE, step: 1, label: "Calibration test duration:"});

const current_calibration_duration = Generators.input(current_calibration_test_duration_slider);

// Choose the target angle for certain commands.
const current_calibration_angle_slider = inputs_wide_range([-Math.PI, Math.PI], {value: 0, step: 0.01, label: "Command angle (radians):"});
const current_calibration_angle = transformed_input_value(current_calibration_angle_slider, radians_to_angle_units);


function stringify_active_current_calibration() {
  return `motor_controller.current_calibration = ${JSON.stringify(motor_controller?.current_calibration, null, 2)}`;
}

const active_current_calibration_table =  Mutable(stringify_active_current_calibration());


const calibration_parameters = {
  u_current_zero: {
    label: "U Current Zero", 
    description: `The zero-current offset for the U phase.`
  },
  v_current_zero: {
    label: "V Current Zero", 
    description: `The zero-current offset for the V phase.`
  },
  w_current_zero: {
    label: "W Current Zero", 
    description: `The zero-current offset for the W phase.`
  },
  resistance: {
    label: "Resistance", 
    description: `The resistance of average phase coil.`
  },
  inductance: {
    label: "Phase Inductance", 
    description: `The inductance of the motor's phases.`
  },
  inductance_bias: {
    label: "Inductance Bias", 
    description: `The bias of the motor's phase inductance.`
  },
  inductance_bias_angle: {
    label: "Inductance Bias Angle", 
    description: `The angle of the inductance bias.`
  },
  saturation_angle: {
    label: "Saturation Angle", 
    description: `The angle of the motor's saturation.`
  },
  saturation_factor: {
    label: "Saturation Factor", 
    description: `The factor of the motor's saturation.`
  },
  motor_constant: {
    label: "Motor Constant", 
    description: `The motor constant, which relates torque and current, or speed and emf.`
  },
  friction_torque: {
    label: "Friction Torque", 
    description: `The torque required to overcome the motor's friction.`
  },
  rotor_mass: {
    label: "Rotor Mass", 
    description: `The mass of the rotor.`
  },
}

const current_calibration_input = Object.fromEntries(
  Object.entries(calibration_parameters).map(([key, {label, description}]) => {
    
    let parameter_input = Inputs.number([], {
      label,
      value: motor_controller?.current_calibration?.[key],
    });

    d3.select(parameter_input).style("width", "100%").style("margin", "1em 0em 1em 0em")
      .select("div").style("width", "50em")
        .append("span").text(description).style("margin", "0em 1em 0em 1em").style("width", "100em");

    return [key, parameter_input];
  })
);

const current_calibration_checkboxes = enabled_checkbox(Object.entries(calibration_parameters).map(([key, {label, description}]) => label));
  
function get_checked_parameters(){
  return Object.fromEntries(
    Object.entries(calibration_parameters).map(([key, {label}]) => {
      return [key, current_calibration_checkboxes.value.includes(label)];
    })
  );
}

function show_active_current_calibration() {
  active_current_calibration_table.value = stringify_active_current_calibration();
  Object.entries(current_calibration_input).forEach(([key, input]) => {
    input.value = motor_controller?.current_calibration?.[key];
  });
}

const current_calibration_data = Mutable(null);

const update_current_calibration_data = (new_value) => {
  current_calibration_data.value = new_value;
  if (new_value.is_stable) {
    Object.entries(current_calibration_input).forEach(([key, input]) => {
      input.value = new_value.current_calibration[key];
    });
  }
};
```

```js

async function run_current_calibration(motor_controller, message_options) {
  if (!motor_controller.current_calibration) {
    console.error("We didn't load the active calibration data from the driver, we can't proceed without it.");
    throw new Error("Missing current calibration data");
  }

  const current_calibration = {...motor_controller.current_calibration};
  const control_parameters = motor_controller.control_parameters;

  console.info("Starting motor spin...");
  await motor_controller.send_command({
    message_code: MessageCode.SET_STATE_DRIVE_SMOOTH,
    timeout: Math.floor(2000 * cycles_per_millisecond),
    pwm_value: +command_pwm,
  });

  await wait(1000);

  console.info("Current calibration starting");

  // Run a calibration instance.
  const sample = await motor_controller.send_command_and_await_reply({
    message: {
      ...message_options,
      take_snapshot: 1,
    },
    expected_messages: HISTORY_SIZE,
    expected_code: MessageCode.READOUT,
  });
  
  // Check all calibration data is complete.
  if (sample.length !== HISTORY_SIZE) {
    console.error("U positive calibration data incomplete", sample);
    return;
  }

  // Test if we've had nominal VCC voltage throughout the calibration run.
  const all_nominal_vcc_voltage = sample.every(({nominal_vcc_voltage}) => nominal_vcc_voltage);

  if (!all_nominal_vcc_voltage) {
    console.warn("Nominal VCC voltage was not maintained during the calibration run, we can't trust the drive voltage.");
    return {
      sample,
      is_stable: false,
    }
  }

  const enabled_parameters = get_checked_parameters();


  // Test if the voltage was always 0.
  const all_zero_drive_voltage = sample.every(({u_drive_voltage, v_drive_voltage, w_drive_voltage}) => 
    u_drive_voltage === 0 && v_drive_voltage === 0 && w_drive_voltage === 0
  );

  // For all 0 voltages we calibrate the baseline offset of the currents.
  if (all_zero_drive_voltage) {
    const u_current_zero = sample.reduce((sum, {u_current}) => sum + u_current, 0) / sample.length + current_calibration?.u_current_zero;
    const v_current_zero = sample.reduce((sum, {v_current}) => sum + v_current, 0) / sample.length + current_calibration?.v_current_zero;
    const w_current_zero = sample.reduce((sum, {w_current}) => sum + w_current, 0) / sample.length + current_calibration?.w_current_zero; 
  
    return {
      sample,
      is_stable: true,
      current_calibration: {
        u_current_zero,
        v_current_zero,
        w_current_zero,
      }
    }
  }

  const max_iterations = 1000;
  const relative_stability = 0.001;
  const relative_min = relative_stability / 10;
  const relative_max = 10;
  const relative_increase = 1.2;
  const relative_decrease = 0.5;

  class Parameter {
    constructor({value, initial_learning_rate}) {
      this.value = value;
      this.initial_learning_rate = initial_learning_rate;
      this.learning_rate = initial_learning_rate;
      this.sign = 0;
    }

    reset(value) {
      this.value = value;
      this.learning_rate = this.initial_learning_rate;
      this.sign = 0;
    }
  }

  let parameters = {
    resistance: new Parameter({
      value: current_calibration?.resistance ?? 0.01,
      initial_learning_rate: 0.01,
    }),
    inductance: new Parameter({
      value: current_calibration?.inductance ?? 0.00001, 
      initial_learning_rate: 0.000_001,
    }),
    inductance_bias: new Parameter({
      value: current_calibration?.inductance_bias ?? 0.0,
      initial_learning_rate: 0.000_001,
    }),
    inductance_bias_angle: new Parameter({
      value: current_calibration?.inductance_bias_angle ?? 0.0,
      initial_learning_rate: Math.PI / 4 / relative_max,
    }),
    saturation_factor: new Parameter({
      value: current_calibration?.saturation_factor ?? 0.0,
      initial_learning_rate: 0.000_001,
    }),
    saturation_angle: new Parameter({
      value: current_calibration?.saturation_angle ?? 0.0,
      initial_learning_rate: Math.PI / 4 / relative_max,
    }),
    motor_constant: new Parameter({
      value: current_calibration?.motor_constant ?? 0.0,
      initial_learning_rate: 0.000_001,
    }),
  };


  let is_stable = false;

  let iterations = [];

  for (let i = 0; !is_stable && (i < max_iterations); i++) {

    const sample_with_gradients = sample.map((readout) => {
      const {
        direct_current, quadrature_current,
        direct_current_diff, quadrature_current_diff,
        direct_drive_voltage, quadrature_drive_voltage,

        angular_speed, predicted_angle,
      } = readout;

      const d_di_dt = direct_current_diff * pwm_cycles_per_second;
      const q_di_dt = quadrature_current_diff * pwm_cycles_per_second;

      const R = parameters.resistance.value;

      const direct_resistive_voltage = direct_current * R;
      const quadrature_resistive_voltage = quadrature_current * R;

      const L_0 = parameters.inductance.value;
      const L_bias = parameters.inductance_bias.value;
      const L_bias_angle = normalize_radians(predicted_angle + parameters.inductance_bias_angle.value);
      const L_saturation_angle = normalize_radians(predicted_angle + parameters.saturation_angle.value);

      const A_saturation = direct_current * d_di_dt - quadrature_current * q_di_dt;
      const B_saturation = quadrature_current * d_di_dt + direct_current * q_di_dt;

      const K_saturation = parameters.saturation_factor.value;

      const K_motor = parameters.motor_constant.value;

      // Convert rotations per millisecond to radians per second.
      const omega = angular_speed * 1000.0 * 2 * Math.PI;
      
      const direct_inductance_bias_voltage = (
        d_di_dt * L_bias * Math.cos(2 * L_bias_angle) +
        q_di_dt * L_bias * Math.sin(2 * L_bias_angle) +
        omega * L_bias * Math.sin(2 * L_bias_angle) * direct_current +
        omega * L_bias * Math.cos(2 * L_bias_angle) * quadrature_current
      );

      const quadrature_inductance_bias_voltage = (
        d_di_dt * L_bias * Math.sin(2 * L_bias_angle) +
        q_di_dt * -L_bias * Math.cos(2 * L_bias_angle) +
        + omega * L_bias * Math.cos(2 * L_bias_angle) * direct_current +
        + omega * L_bias * Math.sin(2 * L_bias_angle) * quadrature_current
      );

      const direct_inductance_saturation_voltage = (
        K_saturation * A_saturation * Math.cos(L_saturation_angle) +
        -K_saturation * B_saturation * Math.sin(L_saturation_angle)
      );

      const quadrature_inductance_saturation_voltage = (
        K_saturation * A_saturation * Math.sin(L_saturation_angle) +
        K_saturation * B_saturation * Math.cos(L_saturation_angle)
      );

      const direct_inductance_voltage = (
        d_di_dt * L_0 + 
        direct_inductance_bias_voltage +
        direct_inductance_saturation_voltage +
        -omega * L_0 * quadrature_current
      );

      const quadrature_inductance_voltage = (
        q_di_dt * L_0 +
        quadrature_inductance_bias_voltage +
        quadrature_inductance_saturation_voltage +
        + omega * L_0 * direct_current
      );

      const quadrature_emf_voltage = - K_motor * omega;

      const direct_residual = direct_resistive_voltage + direct_inductance_voltage - direct_drive_voltage;
      const quadrature_residual = quadrature_resistive_voltage + quadrature_inductance_voltage - quadrature_drive_voltage - quadrature_emf_voltage;

      const residual_angle = Math.atan2(quadrature_residual, direct_residual);

      const two_current_angle = normalize_radians(2 * (predicted_angle + Math.atan2(quadrature_current, direct_current)));


      const loss = square(direct_residual) + square(quadrature_residual);

      const sqrt_loss = Math.sqrt(loss);
      
      const resistance_gradient = (
        direct_residual * direct_current +
        quadrature_residual * quadrature_current
      );

      const inductance_gradient = (
        direct_residual * (d_di_dt - omega * quadrature_current) +
        quadrature_residual * (q_di_dt + omega * direct_current)
      );

      const inductance_bias_gradient = (
        direct_residual * (
          d_di_dt * Math.cos(2 * L_bias_angle) +
          q_di_dt * Math.sin(2 * L_bias_angle) +
          omega * direct_current * Math.sin(2 * L_bias_angle) +
          omega * quadrature_current * Math.cos(2 * L_bias_angle)
        ) +
        quadrature_residual * (
          d_di_dt * Math.sin(2 * L_bias_angle) +
          -q_di_dt * Math.cos(2 * L_bias_angle) +
          omega * direct_current * Math.cos(2 * L_bias_angle) +
          omega * quadrature_current * Math.sin(2 * L_bias_angle)
        )
      );

      const inductance_bias_angle_gradient = (
        2 * L_bias * direct_residual * (
          -d_di_dt * Math.sin(2 * L_bias_angle) +
          +q_di_dt * Math.cos(2 * L_bias_angle) +
          omega * direct_current * Math.cos(2 * L_bias_angle) +
          -omega * quadrature_current * Math.sin(2 * L_bias_angle)
        ) +
        2 * L_bias * quadrature_residual * (
          d_di_dt * Math.cos(2 * L_bias_angle) +
          q_di_dt * Math.sin(2 * L_bias_angle) +
          -omega * direct_current * Math.sin(2 * L_bias_angle) +
          omega * quadrature_current * Math.cos(2 * L_bias_angle)
        )
      );

      const saturation_factor_gradient = (
        2 * direct_residual * (
          A_saturation * Math.cos(L_saturation_angle) +
          -B_saturation * Math.sin(L_saturation_angle)
        ) + 
        2 * quadrature_residual * (
          A_saturation * Math.sin(L_saturation_angle) +
          B_saturation * Math.cos(L_saturation_angle)
        )
      );

      const saturation_angle_gradient = (
        2 * K_saturation * direct_residual * (
          -A_saturation * Math.sin(L_saturation_angle) +
          -B_saturation * Math.cos(L_saturation_angle)
        ) +
        2 * K_saturation * quadrature_residual * (
          A_saturation * Math.cos(L_saturation_angle) +
          -B_saturation * Math.sin(L_saturation_angle)
        )
      );

      // Is this the correct sign? add an explicit + or -:
      const motor_constant_gradient = (
        quadrature_residual * omega
      );


      return {
        ...readout,
        
        direct_resistive_voltage,
        quadrature_resistive_voltage,

        direct_inductance_bias_voltage,
        quadrature_inductance_bias_voltage,
        direct_inductance_saturation_voltage,
        quadrature_inductance_saturation_voltage,

        direct_inductance_voltage,
        quadrature_inductance_voltage,

        loss,
        sqrt_loss,
        
        direct_residual,
        quadrature_residual,
        residual_angle,
        two_current_angle,

        resistance_gradient,
        inductance_gradient,
        inductance_bias_gradient,
        inductance_bias_angle_gradient,
        saturation_factor_gradient,
        saturation_angle_gradient,
        motor_constant_gradient,
      };
    });

    iterations.push({
      iteration: i,
      current_calibration: {
        sqrt_loss: Math.sqrt(d3.mean(sample_with_gradients, (d) => d.loss)),
        resistance: parameters.resistance.value,
        inductance: parameters.inductance.value,
        inductance_bias: parameters.inductance_bias.value,
        inductance_bias_angle: parameters.inductance_bias_angle.value,
        inductance_bias_angle_p_pi: normalize_radians(parameters.inductance_bias_angle.value + Math.PI),
        saturation_factor: parameters.saturation_factor.value,
        saturation_angle: parameters.saturation_angle.value,
        saturation_angle_p_half_pi: normalize_radians(parameters.saturation_angle.value + Math.PI / 2),
        motor_constant: parameters.motor_constant.value,

        resistance_learning_rate: parameters.resistance.learning_rate,
        inductance_learning_rate: parameters.inductance.learning_rate,
        inductance_bias_learning_rate: parameters.inductance_bias.learning_rate,
        inductance_bias_angle_learning_rate: parameters.inductance_bias_angle.learning_rate,
        saturation_factor_learning_rate: parameters.saturation_factor.learning_rate,
        saturation_angle_learning_rate: parameters.saturation_angle.learning_rate,
        motor_constant_learning_rate: parameters.motor_constant.learning_rate,
      },
      sample_with_gradients,
    });

    // Resilient Backpropagation
    // -------------------------
    // 
    // Update steps and learning rates using the sign of the gradient to the unexplained residual loss.



    is_stable = true;

    const gradients = {
      resistance: d3.mean(sample_with_gradients, (d) => d.resistance_gradient),
      inductance: d3.mean(sample_with_gradients, (d) => d.inductance_gradient),
      inductance_bias: d3.mean(sample_with_gradients, (d) => d.inductance_bias_gradient),
      inductance_bias_angle: d3.mean(sample_with_gradients, (d) => d.inductance_bias_angle_gradient),
      saturation_factor: d3.mean(sample_with_gradients, (d) => d.saturation_factor_gradient),
      saturation_angle: d3.mean(sample_with_gradients, (d) => d.saturation_angle_gradient),
      motor_constant: d3.mean(sample_with_gradients, (d) => d.motor_constant_gradient),
    }

    for (const [key, gradient] of Object.entries(gradients)) {
      if (!enabled_parameters[key]) continue;

      const new_sign = Math.sign(gradient);
      if ((new_sign == 0.0) || (new_sign != parameters[key].sign)) {
        parameters[key].learning_rate = Math.max(relative_min * parameters[key].initial_learning_rate, parameters[key].learning_rate * relative_decrease);
      } else {
        parameters[key].learning_rate = Math.min(relative_max * parameters[key].initial_learning_rate, parameters[key].learning_rate * relative_increase);
      }

      parameters[key].value -= parameters[key].learning_rate * new_sign;
      parameters[key].sign = new_sign;

      if (parameters[key].learning_rate > (parameters[key].initial_learning_rate * relative_stability)) {
        is_stable = false;
      }
    }
    
    // Update calibration values after pushing the iteration data! The iteration should then
    // contain the calibration values that were used to calculate the gradients and other values.

    if (parameters.resistance.value < 0.0) parameters.resistance.reset(0.0);
    if (parameters.inductance.value < 0.0) parameters.inductance.reset(0.0);

    if (parameters.inductance_bias.value < 0.0) {
      parameters.inductance_bias.reset(0.0);
      parameters.inductance_bias_angle.value = parameters.inductance_bias_angle.value + Math.PI/12;
    }

    if (parameters.saturation_factor.value < 0.0) {
      parameters.saturation_factor.reset(- 0.5 * parameters.saturation_factor.value);
      parameters.saturation_angle.value = parameters.saturation_angle.value + Math.PI/12;
    }

    parameters.inductance_bias_angle.value = normalize_radians(parameters.inductance_bias_angle.value);
    parameters.saturation_angle.value = normalize_radians(parameters.saturation_angle.value);
    
    if (parameters.motor_constant.value < 0.0) parameters.motor_constant.reset(0.0);
  }

  const current_calibration_data = {
    sample,
    is_stable,
    iterations,
    current_calibration: {
      resistance: parameters.resistance.value,
      inductance: parameters.inductance.value,
      inductance_bias: parameters.inductance_bias.value,
      inductance_bias_angle: parameters.inductance_bias_angle.value,
      saturation_factor: parameters.saturation_factor.value,
      saturation_angle: parameters.saturation_angle.value,
      motor_constant: parameters.motor_constant.value,
    }
  };

  return current_calibration_data;
}

async function run_current_calibration_by_code(message_code) {
  if (!motor_controller) return;
  const calibration_data = await run_current_calibration(
    motor_controller, 
    {
      message_code,
      pwm_value: current_calibration_pwm,
      test_speed: current_calibration_test_speed,
      test_duration: current_calibration_duration,
      test_angle: current_calibration_angle,
    }
  );
  update_current_calibration_data(calibration_data);
  set_readout_series(calibration_data.sample, active_port);
}

const current_calibration_run_buttons = Inputs.button(
  test_buttons_to_code.map(([label, code]) => [label, () => run_current_calibration_by_code(code)]),
  {label: "Run calibration sequence"},
);
d3.select(current_calibration_run_buttons).selectAll("button").style("height", "4em");


const current_calibration_buttons = !motor_controller ? html`<p>Not connected to motor!</p>` : Inputs.button(
  [
    ["Upload to Driver", async function(){
      if (!motor_controller) return;
      const current_calibration = {
        ...motor_controller.current_calibration,
        ...Object.fromEntries(Object.entries(current_calibration_input).map(([key, input]) => [key, input.value])),
      };
      await motor_controller.upload_current_calibration(current_calibration);
      show_active_current_calibration();
      unsaved_changes();
    }],
    ["Reload from Driver", async function(){
      await motor_controller.load_current_calibration();
      show_active_current_calibration();
    }],
    ["Reset to Defaults", async function(){
      await motor_controller.reset_current_calibration();
      show_active_current_calibration();
      unsaved_changes();
    }],
  ],
  {
    label: "Current Calibration",
  },
);

d3.select(current_calibration_buttons).style("width", "100%");
```



```js
const current_calibration_iterations = current_calibration_data?.iterations ?? [];

// Select the optimization iteration to display.
const current_calibration_optimization_iteration_input = !current_calibration_data ? 
  html`<p>No optimization iterations available.</p>` : 
  inputs_wide_range(
    [0, current_calibration_iterations.length - 1], {
    step: 1,
    label: "Select optimization iteration to display:",
  });

const current_calibration_optimization_iteration = Generators.input(current_calibration_optimization_iteration_input);

set_input_value(current_calibration_optimization_iteration_input, current_calibration_iterations?.length - 1);


```

```js

const current_calibration_iteration = current_calibration_iterations[current_calibration_optimization_iteration];
const current_calibration_gradients = current_calibration_iteration?.sample_with_gradients ?? [];

// Write out the current calibration results in copyable format.
const current_calibration_iteration_table = `current_calibration = ${JSON.stringify(current_calibration_iterations[current_calibration_optimization_iteration]?.current_calibration, null, 2)}`;

const current_calibration_optimizing_plot = plot_lines({
  data: current_calibration_gradients,
  subtitle: "Current Calibration - Optimizing resistance & inductance",
  description: "Current calibration optimization results for each phase.",
  width: 1200, height: 300,
  x_domain: [0, HISTORY_SIZE * millis_per_cycle],
  x: "time",
  x_label: "Time (ms)",
  y_label: "Voltage (V)",
  channels: [
    {y: "direct_drive_voltage", label: "Direct Drive Voltage", color: d3.color(colors.u).brighter(1)},
    {y: "quadrature_drive_voltage", label: "Quadrature Drive Voltage", color: d3.color(colors.v).brighter(1)},

    {y: "direct_resistive_voltage", label: "Direct Resistance Drop", color: colors.u},
    {y: "quadrature_resistive_voltage", label: "Quadrature Resistance Drop", color: colors.v},

    {y: "direct_inductance_voltage", label: "Direct Inductance Drop", color: d3.color(colors.u).darker(1)},
    {y: "quadrature_inductance_voltage", label: "Quadrature Inductance Drop", color: d3.color(colors.v).darker(1)},

    {y: "direct_inductance_bias_voltage", label: "Direct Inductance Bias", color: d3.color(colors.u).darker(1)},
    {y: "quadrature_inductance_bias_voltage", label: "Quadrature Inductance Bias", color: d3.color(colors.v).darker(1)},
    {y: "direct_inductance_saturation_voltage", label: "Direct Inductance Saturation", color: d3.color(colors.u).darker(2)},
    {y: "quadrature_inductance_saturation_voltage", label: "Quadrature Inductance Saturation", color: d3.color(colors.v).darker(2)},

    {y: "direct_residual", label: "Direct Residual", color: d3.color(colors.u).darker(2)},
    {y: "quadrature_residual", label: "Quadrature Residual", color: d3.color(colors.v).darker(2)},

    {y: "sqrt_loss", label: "Sqrt Loss", color: d3.color(colors_categories[2]).darker(1)},

  ],
  curve,
});


const current_calibration_angles_plot = plot_lines({
  data: current_calibration_gradients,
  subtitle: "Current Calibration - Optimizing angles",
  description: "Current calibration optimization results for each phase.",
  width: 1200, height: 300,
  x_domain: [0, HISTORY_SIZE * millis_per_cycle],
  x: "time",
  x_label: "Time (ms)",
  y_label: "Angle (radians)",
  channels: [
    {y: "residual_angle", label: "Residual Angle", color: colors_categories[2]},
    {y: "web_current_angle", label: "Current Angle", color: colors_categories[0]},
    {y: "two_current_angle", label: "2 x Current Angle", color: colors_categories[5]},
    {y: "web_current_angular_speed", label: "Current Angular Speed", color: colors_categories[1]},
    {y: "drive_voltage_angle", label: "Drive Voltage Angle", color: colors_categories[3]},
    {y: (d)=>normalize_radians(d.drive_voltage_angle - d.web_current_angle), label: "Drive-Current Angle Diff", color: colors_categories[5]},
  ],
  curve,
});

const current_calibration_angles_scatter = plot_lines({
  data: current_calibration_gradients.slice(24).sort((a, b) => a.drive_voltage_angle - b.drive_voltage_angle),
  subtitle: "Current Calibration - Angle Scatter",
  description: "Scatter plot of current calibration angles.",
  width: 1200, height: 300,
  x_domain: [-Math.PI, Math.PI],
  x: "drive_voltage_angle",
  x_label: "Drive Voltage Angle (radians)",
  y_label: "Drive-Current Angle Diff (radians)",
  channels: [
    {y: "web_current_magnitude", label: "Current Magnitude", color: colors_categories[4]},
    {y: (d)=>normalize_radians(d.drive_voltage_angle - d.web_current_angle), label: "Drive-Current Angle Diff", color: colors_categories[5]},
    {y: (d)=>normalize_radians(d.drive_voltage_angle - d.web_current_angle) / d.web_current_angular_speed, label: "Drive-Current Angle Diff / Angular Speed", color: colors_categories[6]}
  ],
  curve: horizontal_step,
});

const current_calibration_optimizing_gradients_plot = plot_lines({
  data: current_calibration_gradients,
  subtitle: "Current Calibration - resistance & inductance gradients",
  description: "Current calibration optimization results for each phase.",
  width: 1200, height: 300,
  x_domain: [0, HISTORY_SIZE * millis_per_cycle],
  x: "time",
  x_label: "Time (ms)",
  y_label: "Gradients (unitless)",
  channels: [
    {y: "resistance_gradient", label: "Resistance Gradient", color: colors_categories[0]},
    {y: "inductance_gradient", label: "Inductance Gradient", color: colors_categories[1]},
    {y: "inductance_bias_gradient", label: "Inductance Bias Gradient", color: colors_categories[2]},
    {y: "inductance_bias_angle_gradient", label: "Inductance Bias Angle Gradient", color: colors_categories[3]},
    {y: "saturation_angle_gradient", label: "Saturation Angle Gradient", color: colors_categories[4]},
    {y: "saturation_factor_gradient", label: "Saturation Factor Gradient", color: colors_categories[5]},
  ],
  curve,
});

autosave_inputs({
  current_calibration_optimizing_plot,
  current_calibration_optimizing_gradients_plot,
  current_calibration_angles_scatter,
  current_calibration_angles_plot,
  current_calibration_checkboxes,
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
    ["min_emf_speed", {
      label: "Minimum EMF Speed for detection", 
      description: `Minimum EMF speed required to declare EMF detected. There is noise in the EMF measurement, so
      we want a threshold just slightly above the noise floor; some spurious EMF readings are tolerated.`
    }],
    ["emf_probing_interval", {
      label: "EMF Probing Interval",
      description: "Interval for probing the EMF angle when it is too noisy to use. Determines how frequently we check the EMF angle."
    }],
    ["probing_angular_speed", {
      label: "Probing Angular Speed",
      description: `Probing angular speed. We use this default speed to drive a current around the coils in order to
      move the rotor to measure its position from the back EMF. Used when we don't have hall sensors and no angle fix.`
    }],
    ["max_hold_pwm", {
      label: "Max Hold PWM",
      description: `Maximum PWM for holding commands.`
    }],
    ["min_emf_for_motor_constant", {
      label: "Threshold for motor constant",
      description: "Minimum EMF voltage magnitude required to compute the motor constant. Below this threshold it is too noisy because we divide by voltage."
    }],
    ["motor_direction", {
      label: "Motor direction", 
      description: "Direction of the motor rotation (+1 for default, -1 to reverse rotation direction)."
    }],
    ["angle_fix_max_certainty", {
      label: "Angle Fix Max Certainty", 
      description: "Number of incorrect direction detections before losing the angle fix."
    }],
    ["vcc_undervoltage", {
      label: "VCC Undervoltage Threshold",
      description: "Threshold for VCC undervoltage detection. Below this voltage we consider the supply to be too low."
    }],
    ["max_resistive_power", {
      label: "Maximum Resistive Power",
      description: "Maximum resistive power allowed. This is a proxy for the maximum temperature of the motor coils."
    }],
    ["max_power_draw", {
      label: "Max Power Draw",
      description: "Maximum power draw allowed. This is a proxy for the maximum current draw at constant supply voltage."
    }],
    ["resistive_power_ki", {
      label: "Resistive Power KI",
      description: "Resistive power integral gain. How fast we average the resistive power to avoid spikes."
    }],
    ["power_draw_ki", {
      label: "Power Draw KI",
      description: "Power draw integral gain. How fast we average the power draw to avoid spikes."
    }],
    ["rotor_angle_ki", {
      label: "Rotor Angle KI", 
      description: "Integral gain for the rotor angle observer from the measured EMF angle.",
    }],
    ["rotor_angular_speed_ki", {
      label: "Rotor Angular Speed KI", 
      description: `Integral gain for speed of the angle based on the same error used for the EMF angle (an 
      incorrect angle prediction implies a speed error as well).`
    }],
    ["rotor_acceleration_ki", {
      label: "Rotor Acceleration KI", 
      description: "Integral gain for the rotor acceleration observer; same as above, more resolution."
    }],
    ["emf_angle_ki", {
      label: "EMF Angle KI", 
      description: "Integral gain for the EMF angle observer from the measured EMF angle."
    }],
    ["emf_angular_speed_ki", {
      label: "EMF Angular Speed KI", 
      description: "Integral gain for the EMF angular speed observer from the measured EMF angle."
    }],
    ["hall_angle_ki", {
      label: "Position adjustment from hall angle KI", 
      description: "Integral gain for the angle adjustment based on the hall sensors."
    }],
    ["lead_angle_control_ki", {
      label: "Lead Angle Control KI",
      description: "Lead angle control gain. The lead angle controls how far ahead we drive the voltage for maximum torque/efficiency."
    }],
    ["torque_control_ki", {
      label: "Torque Control KI",
      description: "Torque control gain. Controls how fast we adjust the PWM to achieve the desired torque."
    }],
    ["torque_control_kff", {
      label: "Torque Control KFF",
      description: "Torque control feedforward gain. Controls how much we drive the PWM based on the desired torque."
    }],
    ["battery_power_control_ki", {
      label: "Battery Power Control KI",
      description: "Battery power control gain. Controls how fast we adjust the PWM to achieve the desired battery power."
    }],
    ["battery_power_control_kff", {
      label: "Battery Power Control KFF",
      description: "Battery power control feedforward gain. Controls how much we drive the PWM based on the desired battery power."
    }],
    ["speed_control_ki", {
      label: "Speed Control KI",
      description: "Speed control gain. Controls how fast we adjust the PWM to achieve the desired speed."
    }],
    ["speed_control_kff", {
      label: "Speed Control KFF",
      description: "Speed control feedforward gain. Controls how much we drive the PWM based on the desired speed."
    }],
    ["seek_kff", {
      label: "Seek prediction factor",
      description: "We compute the integral error using the predicted position a few milliseconds ahead. This parameter controls how far ahead we predict."
    }],
    ["seek_ki", {
      label: "Seek KI",
      description: "Seek via torque integral gain. How fast we adjust the driving power to adjust for small errors."
    }],
    ["seek_kp", {
      label: "Seek KP",
      description: "Seek via torque proportional gain. The spring constant for the torque control; the torque we apply per distance from the target."
    }],
    ["seek_kd", {
      label: "Seek KD",
      description: "Seek via torque derivative gain. Dampening factor to lower torque when the error is decreasing quickly."
    }],
    ["phase_resistance_ki", {
      label: "Phase Resistance KI",
      description: "Integral gain for phase resistance estimation. Helps in accurately estimating the resistance of each motor phase in star configuration."
    }],
    ["phase_inductance_ki", {
      label: "Phase Inductance KI",
      description: "Integral gain for phase inductance estimation. Helps in accurately estimating the inductance of each motor phase in star configuration."
    }],
    ["inductance_bias_angle_ki", {
      label: "Inductance Bias Angle KI",
      description: "Integral gain for inductance bias angle estimation. To be explained later..."
    }],
    ["saturation_factor_ki", {
      label: "Saturation Factor KI",
      description: "Integral gain for saturation factor. To be explained later..."
    }],
    ["motor_constant_ki", {
      label: "Motor Constant KI", 
      description: "Integral gain for the motor constant observer; the relation between speed and EMF magnitude."
    }],
    ["friction_torque_ki", {
      label: "Friction Torque KI", 
      description: "Integral gain for the friction torque observer; the relation between speed and torque required to overcome friction."
    }],
    ["rotor_mass_ki", {
      label: "Rotor Mass KI", 
      description: "Integral gain for the rotor mass observer; the relation between speed and torque required to accelerate the rotor."
    }],
  ].map(([key, {label, description}]) => {

    let parameter_input = Inputs.number([], {
      label,
      value: motor_controller?.control_parameters?.[key],
    });

    d3.select(parameter_input).style("width", "100%").style("margin", "1em 0em 1em 0em")
      .select("div").style("width", "50em")
        .append("span").text(description).style("margin", "0em 1em 0em 1em").style("width", "100em");

    return [key, parameter_input];
  })
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
      unsaved_changes();
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
      unsaved_changes();
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
const flash_status = Mutable(`...`);

function unsaved_changes(){
  if (!motor_controller) return false;
  flash_status.value = `Unsaved changes.`;
}

const flash_buttons = !motor_controller ? html`<p>Not connected to motor!</p>` : Inputs.button(
  [
    ["Commit to Flash", async function(){
      await motor_controller.send_command_and_await_reply({
        message: {message_code: MessageCode.SAVE_SETTINGS_TO_FLASH},
        expected_messages: 1,
        expected_code: MessageCode.SETTINGS_SAVED_TO_FLASH,
      });

      flash_status.value = `Settings saved to flash memory.`;
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

  const output = await motor_controller.send_command_and_await_reply({
    message: test_code,
    expected_messages: 1,
    expected_code: MessageCode.UNIT_TEST_OUTPUT,
  });

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
// Imports, Constants & Helpers
// ----------------------------

import * as d3 from "d3";

import {plot_lines, plot_line, setup_faint_area, horizontal_step, setup_stdev_95, draw_line} from "./components/plotting_utils.js";

import {localStorage, get_stored_or_default, clear_stored_data} from "./components/local_storage.js";

import {round, timeout_promise, wait}  from "./components/utils.js";

import {
  enabled_checkbox, autosave_inputs, any_checked_input, 
  set_input_value, merge_input_value, wait_previous,
  inputs_wide_range, transformed_input_value,
} from "./components/input_utils.js";

import {square, invalid_to_zero, dq0_transform} from "./components/motor_controller/math_utils.js";

import {normalize_radians} from "./components/motor_controller/angular_math.js";

import {MessageCode, prompt_com_port_get_index, default_ws_uri, MotorController} from "./components/motor_controller/motor_controller.js";

import {
  PWM_BASE, HISTORY_SIZE, CURRENT_UNITS_PER_AMP, VOLTAGE_UNITS_PER_VOLT,
  pwm_cycles_per_second, cycles_per_millisecond, millis_per_cycle, max_timeout, pwm_period, 
  radians_to_angle_units, 
  rotations_per_millisecond_to_speed_units,
  speed_units_to_rotations_per_millisecond,
  max_drive_current, max_drive_power, max_angular_speed,
} from "./components/motor_controller/constants.js";

import {unit_test_expected} from "./components/motor_controller/driver_unit_tests.js";

import _ from "lodash";



const colors = {
  u: "rgb(117, 112, 179)",
  v: "rgb(217, 95, 2)",
  w: "rgb(231, 41, 138)",
  web_angle: "rgb(178, 228, 0)",
  angle: "rgb(39, 163, 185)",
  web_current_magnitude: "rgb(197, 152, 67)",
  current_angle: "rgb(102, 166, 30)",
  voltage_angle: "rgb(0, 185, 124)",
  angle_driven: "rgb(166, 30, 132)",
  angular_speed: "rgb(41, 194, 173)",
  web_angular_speed: "rgb(156, 196, 47)",
  direct_current: "rgb(199, 0, 57)",
  quadrature_current: "rgb(26, 82, 118)",
  other: "rgb(27, 158, 119)",
  ref_readout: "rgb(102, 102, 102)",
  sum: "rgb(0, 0, 0)",
};

const colors_categories = Object.values(colors);

// Connection info formatting utils
// --------------------------------

// Format a floating number of bytes with the common endings KiB, MiB, GiB (products of 1024).
function format_bytes(bytes){
  if (bytes < 1024) return `${bytes.toFixed(2)} bytes`;
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(2)} KiB`;
  if (bytes < 1024 * 1024 * 1024) return `${(bytes / (1024 * 1024)).toFixed(2)} MiB`;
  return `${(bytes / (1024 * 1024 * 1024)).toFixed(2)} GiB`;
}

// Format the receiving data rate from the connection status.
function format_data_rate({bytes_received, bytes_discarded, receive_rate}){
  bytes_received = `received: ${format_bytes(bytes_received).padStart(12)}`;
  bytes_discarded = `discarded: ${format_bytes(bytes_discarded).padStart(12)}`;
  receive_rate = `download rate: ${format_bytes(receive_rate).padStart(12)}/s`;

  return `${bytes_received} | ${bytes_discarded} | ${receive_rate}`;
}

// Format the connection error message as an html element.
function displayable_connection_error(error){
  if (error.message === "EOF") {
    return html`<pre>End of connection.</pre>`;
  } else if (error.name === "NotFoundError") {
    return html`<pre style="color: purple">No device found or nothing selected.</pre>`;
  } else if (error.name === "SecurityError") {
    return html`<pre style="color: purple">Permission for port dialog denied.</pre>`;
  } else if (error.name === "NetworkError") {
    return html`<pre style="color: red">Transmission error to usb device.</pre>`;
  } else {
    console.log("Motor connection error:", error);
    return html`<pre style="color: red">Connection lost; unknown error: ${error}</pre>`;
  }
}


// Memory consumption capping
// --------------------------

// Desired amount of data readouts per motor driver to keep in memory.
const target_data_size = 4000 / (millis_per_cycle * 4);

// Maximum amount of data readouts per motor driver to keep in memory, is
// used so we don't resize the array with every new readout. When the max
// cap is reached we resize to the target size, amortizing resize ops.
const max_data_size = 2 * target_data_size;

// Push to array, but keep array size capped to a limit.
function capped_push(data, value){
  let new_data = (data.length > max_data_size) ? data.slice(-target_data_size) : data;

  new_data.push(value);

  return new_data;
}

```