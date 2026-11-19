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
<div>${stop_buttons}</div>
<div>
  <span>${simple_drive_buttons}</span>
  <span>${test_buttons}</span>
  <span>${command_duration_slider}</span>
  <span>${command_pwm_slider}</span>
  <span>${quickselect_angle_buttons}</span>
  <span>${command_angle_slider}</span>
  <span>${command_angular_speed_slider}</span>
  <span>${command_torque_current_slider}</span>
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
<div class="card tight">${Object.values(monitoring_plots)}</div>


Calibration Procedures
----------------------

<div class="card tight">
  <div>${calibration_buttons}</div>
  <p>Active electrical phase properties:</p>
  <pre>${active_calibration_table}</pre>
  <div>${Object.values(calibration_input)}</div>
</div>
<div class="card tight">
  <h3>Calibration Results</h3>
  <div>${Object.values(calibration_series_plots)}</div>

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


let calibration_series = Mutable([]);


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
  
  data.value = port_state.motor_controller.data;

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


    const motor_controller = new MotorController({
      motor_uri,
      onstatus: (status) => {
        set_status(html`<pre>Connected; ${format_data_rate(status)}.</pre>`, motor_uri);
      },
      onmessage: (message) => {
        if (motor_uri != active_port.value) return;
        
        if (message.message_code == MessageCode.READOUT) {
          data.value = motor_controller.data;
          // TODO: circular definition
        // } else if (message.message_code == MessageCode.CALIBRATION) {
        //   show_active_calibration();
        // } else if (message.message_code == MessageCode.CONTROL_PARAMETERS) {
        //   show_active_control_parameters();
        } else if (message.message_code == MessageCode.CALIBRATION_INSTANCE) {
          calibration_series.value = [...motor_controller.calibration_series.values()];
        } else {
          // ... nothing to do really
        }

      },
      onready: (motor_controller) => {
        set_status(html`<pre>Connected, waiting for your commands.</pre>`, motor_uri);
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

    // Append this port to the opened ports list so we can capture it's errors too.
    opened_ports.value = opened_ports.value.set(motor_uri, {
      motor_uri,
      motor_controller: motor_controller,
      status: html`<pre>Opening USB com port.</pre>`,
      error: undefined,
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

// Slider to choose the command specific PWM setting.
const command_pwm_slider = inputs_wide_range([0, PWM_BASE], {value: PWM_BASE * 0.05, step: 1, label: "Command value:"});
const command_pwm = transformed_input_value(command_pwm_slider, (value) => Math.floor(value));

// Timeout duration for each command. For safety, the motor runs each drive command for a short period until commanded again.
const command_duration_slider = inputs_wide_range([0, max_timeout*millis_per_cycle], {value: 510, step: 1, label: "Command timeout (ms):"});
const command_duration = transformed_input_value(command_duration_slider, (millis) => Math.floor(millis * cycles_per_millisecond));

// Choose the angular speed target for certain commands.
const command_angular_speed_slider = inputs_wide_range(
  [
    0.0,
    speed_units_to_rotations_per_millisecond(max_angular_speed)
  ], 
  {value: 0.10, step: 0.01, label: "Angular speed value (rotations/ms)"});
const command_angular_speed = transformed_input_value(command_angular_speed_slider, rotations_per_millisecond_to_speed_units);



// Choose the target angle for certain commands.
const command_angle_slider = inputs_wide_range([-Math.PI, Math.PI], {value: 0, step: 0.01, label: "Command angle (radians):"});
const command_angle = transformed_input_value(command_angle_slider, radians_to_angle_units);

// 6 quickselect bottons for the angles with U+ being 0, U- being 180, V+ being 60, V- being 240, W+ being 120, W- being 300
const quickselect_angle_buttons = Inputs.button(
  [
    ["U+", function() { set_input_value(command_angle_slider, 0); }],
    ["W-", function() { set_input_value(command_angle_slider, Math.PI / 3); }],
    ["V+", function() { set_input_value(command_angle_slider, 2 * Math.PI / 3); }],
    ["U-", function() { set_input_value(command_angle_slider, -Math.PI); }],
    ["W+", function() { set_input_value(command_angle_slider, -2 * Math.PI / 3); }],
    ["V-", function() { set_input_value(command_angle_slider, -1 * Math.PI / 3); }]
  ],
  {label: "Quickselect angles"}
);

// Choose the torque target for torque driving modes.
const command_torque_current_slider = inputs_wide_range([0, max_drive_current], {value: 0.750, step: 0.010, label: "Command torque (Amps):"});
const command_torque_current = transformed_input_value(command_torque_current_slider, (amps) => amps * CURRENT_UNITS_PER_AMP);

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

const command_drive_state = {
  motor_flags: ENABLE_ALL_OUTPUTS,
  closed_loop: false,
  duration: command_duration,
  target_value: command_pwm, 
  target_speed: command_angular_speed, 
  angle_value: command_angle,
};

async function stream_all_motors(){
  for (const {motor_controller} of opened_ports.values()) {
    if (!motor_controller) continue;
    motor_controller.reset_history();
    await motor_controller.send_command({message_code: MessageCode.STREAM_READOUTS, stream_state: 1});
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
    ["Get readout snapshot", async function(){
      if (!motor_controller) return;
      motor_controller.reset_history();
      await motor_controller.send_command({message_code: MessageCode.GET_READOUTS_SNAPSHOT, stream_state: HISTORY_SIZE});
    }],
    ["Stream motor data", async function(){
      if (!motor_controller) return;
      motor_controller.reset_history();
      await motor_controller.send_command({message_code: MessageCode.STREAM_READOUTS, stream_state: 1});
    }],
    ["STOP stream", async function(){
      stop_command_loop();
      if (!motor_controller) return;
      await motor_controller.send_command({message_code: MessageCode.STREAM_READOUTS, stream_state: 0});
    }],
    ["Stream all motors", stream_all_motors],
    ["Stop all motors", async function(){
      stop_command_loop();

      for (const {motor_controller} of opened_ports.values()) {
        if (!motor_controller) continue;
        await motor_controller.send_command({message_code: MessageCode.STREAM_READOUTS, stream_state: 0});
      }
    }],
  ],
  {label: "Read data"},
);
d3.select(data_request_buttons).selectAll("button").style("height", "4em");

const test_buttons = Inputs.button(
  [
    ["Test all permutation", async function(){
      let drive_schedule = [
        [0, 1.0],
        [0, 0.0],
        [Math.PI * 1 / 3, 1.0],
        [Math.PI * 1 / 3, 0.0],
        [Math.PI * 2 / 3, 1.0],
        [Math.PI * 2 / 3, 0.0],
      ].map(([angle, value]) => ({
        motor_flags: ENABLE_ALL_OUTPUTS,
        duration: HISTORY_SIZE / 6,
        target_value: command_drive_state.target_value * value,
        angle_value: radians_to_angle_units(angle),
        target_speed: 0,
      }));
      drive_schedule.message_code = MessageCode.SET_DRIVE_SCHEDULE;
      await motor_controller.send_command(drive_schedule);
    }],
    ["Test ground short", async function(){
      await motor_controller.send_command({
        message_code: MessageCode.SET_DRIVE_STATE,
        motor_flags: ENABLE_ALL_OUTPUTS,
        duration: HISTORY_SIZE,
        target_value: 0,
      });
    }],
    ["Test increasing steps", async function(){
      let drive_schedule = [0.0, 0.25, 0.5, 1.0, 0.75, 0.0].map(value => ({
        motor_flags: ENABLE_ALL_OUTPUTS,
        duration: HISTORY_SIZE / 6,
        target_value: command_drive_state.target_value * value,
        angle_value: command_drive_state.angle_value,
        target_speed: 0,
      }));
      drive_schedule.message_code = MessageCode.SET_DRIVE_SCHEDULE;
      await motor_controller.send_command(drive_schedule);
    }],
    ["HFI Saliency Pulses", async function(){
      // For the saliency detection pulses we will drive the motor open loop at positive
      // speed until we reach 45 degree electrical angle. Then negative speed until we reach
      // -45 degree electrical angle. And again positive speed back to 0.
      const common_state = {
        motor_flags: ENABLE_ALL_OUTPUTS,
        target_value: command_drive_state.target_value,
        angle_value: command_drive_state.angle_value,
      };

      // const half_duration = Math.floor(
      //   Math.PI / 4 / 
      //   speed_units_to_rotations_per_millisecond(command_angular_speed) * 
      //   (pwm_cycles_per_second / 1000.0)
      // );
      
      // Or we just rotate it for a fixed duration, the math works easier on chip so 
      // we don't have to do DIVISIOOOOOON.
      const half_duration = 8;

      let drive_schedule = [
        {
          ...common_state, 
          target_speed: command_angular_speed,
          duration: half_duration,
        },
        {
          ...common_state,
          target_speed: -command_angular_speed,
          duration: half_duration * 2,
        },
        {
          ...common_state,
          target_speed: command_angular_speed,
          duration: half_duration,
        }
      ];
      drive_schedule.message_code = MessageCode.SET_DRIVE_SCHEDULE;
      await motor_controller.send_command(drive_schedule);
    }],
  ],
  {label: "Test sequence"},
);
d3.select(test_buttons).selectAll("button").style("height", "4em");

const stop_buttons = Inputs.button(
  [
    ["Stop / Brake", async function(){
      stop_command_loop();

      await motor_controller.send_command({message_code: MessageCode.SET_STATE_OFF});
    }],
    ["Freewheel", async function(){
      stop_command_loop();

      await motor_controller.send_command({message_code: MessageCode.SET_STATE_FREEWHEEL});
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
    ["Drive open loop +", async function(){
      await motor_controller.send_command({
        message_code: MessageCode.SET_DRIVE_STATE,
        ...command_drive_state,
      });
    }],
    ["Drive open loop -", async function(){

      await motor_controller.send_command({
        message_code: MessageCode.SET_DRIVE_STATE,
        ...command_drive_state,
        target_speed: -command_drive_state.target_speed,
      });
      }],
      ["Drive closed loop +", async function(){
      await motor_controller.send_command({
        message_code: MessageCode.SET_DRIVE_STATE,
        ...command_drive_state,
        closed_loop: true,
        target_speed: +command_angular_speed,
        angle_value: +radians_to_angle_units(Math.PI/2),
        target_value: +command_torque_current,
      });
    }],
    ["Drive closed loop -", async function(){
      await motor_controller.send_command({
        message_code: MessageCode.SET_DRIVE_STATE,
        ...command_drive_state,
        closed_loop: true,
        target_speed: -command_angular_speed,
        angle_value: +radians_to_angle_units(Math.PI/2),
        target_value: +command_torque_current,
      });
    }],
    ["Drive closed loop 0", async function(){
      await motor_controller.send_command({
        message_code: MessageCode.SET_DRIVE_STATE,
        ...command_drive_state,
        closed_loop: true,
        target_speed: command_angular_speed,
        angle_value: 0,
        target_value: +command_torque_current,
      });
    }]
    // TODO: these need fixin' we want PWM always positive.
    // ["Drive 6S +", async function(){
    //   await motor_controller.send_command({
    //     message_code: MessageCode.SET_STATE_DRIVE_6_SECTOR, 
    //     ...command_drive_state});
    // }],
    // ["Drive 6S -", async function(){
    //   await motor_controller.send_command({
    //     message_code: MessageCode.SET_STATE_DRIVE_6_SECTOR, 
    //     ...command_drive_state,
    //     target_value: -command_drive_state.target_value
    //   });
    // }],
  ],
  {label: "Simple drive commands"},
);
d3.select(simple_drive_buttons).selectAll("button").style("height", "4em");





const seek_drive_buttons = Inputs.button(
  [
    ["Seek angle", async function(){
      await motor_controller.send_command({
        message_code: MessageCode.SET_STATE_SEEK_ANGLE,
        target_rotation: command_seek_rotation, 
        target_angle: command_angle,
        target_speed: command_angular_speed,
        target_current: command_torque_current,
        ...command_drive_state,
      });
    }],
    ["Go to zero", async function(){
      await motor_controller.send_command({
        message_code: MessageCode.SET_STATE_SEEK_ANGLE,
        target_rotation: 0, 
        target_angle: 0,
        target_speed: command_angular_speed,
        target_current: command_torque_current,
        ...command_drive_state,
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
            motor_flags: ENABLE_ALL_OUTPUTS,
            duration: command_duration,
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
            motor_flags: ENABLE_ALL_OUTPUTS,
            duration: command_duration,
            target_angle: command_angle,
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
            motor_flags: ENABLE_ALL_OUTPUTS,
            duration: command_duration,
            target_angle: command_angle,
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
Object.values(calibration_series_plots).forEach((plot) => plot.update({
  data: calibration_series,
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
    {y: "total_power_average", label: "Total Power (on chip average)", color: d3.color(colors.sum).darker(1)},
    {y: "resistive_power_average", label: "Resistive Power (on chip average)", color: d3.color(colors_categories[5]).darker(1)},
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
    {y: "average_buffer_fill_rate", label: "Average Buffer Fill Rate", color: colors.w},
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
    {y: "predicted_angle", label: "Predicted Angle", color: d3.color(colors.angle).darker(1)},
    {y: "saliency_angle", label: "Saliency Angle", color: colors_categories[5]},
    {y: (d) => d.saliency_angle * 0.5, label: "Half Saliency Angle", color: d3.color(colors_categories[5]).brighter(1)},
    {y: "hysteresis_angle", label: "Hysteresis Loss Angle", color: colors_categories[6]},
    {y: "current_angle", label: "Current Angle", color: colors_categories[3]},
    {y: "inductance_angle", label: "Inductance Angle", color: colors_categories[4]},
    {
      y: "emf_voltage_angle", label: "EMF Voltage Angle", color: colors.voltage_angle,
      draw_extra: setup_stdev_95({stdev: (d) => d.emf_angle_stdev}),
    },
    {y: "vibration_angle", label: "Vibration Angle", color: colors_categories[7]},
    {y: (d) => normalize_radians(2*d.emf_voltage_angle)*0.5, label: "EMF Voltage Axis", color: d3.color(colors.voltage_angle).brighter(1)},
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
    {y: "emf_angle_stdev", label: "EMF Angle stdev", color: colors.angle_driven},
    {y: "current_angle_offset", label: "Current Angle Offset", color: colors.current_angle},
    {y: "emf_voltage_angle_offset", label: "EMF Voltage Angle Offset", color: colors.voltage_angle},
    {y: "angle_adjustment", label: "Magnet Angle Correction", color: d3.color(colors.voltage_angle).brighter(1)},
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
    {y: "emf_voltage_angular_speed", label: "EMF Voltage Angular Speed", color: colors.voltage_angle},
    {y: "emf_angle_stdev", label: "EMF Angle stdev", color: colors.angle_driven},
  ],
  curve,
});


const plot_measured_voltage = plot_lines({
  subtitle: "VCC Voltage",
  description: "Measured voltage values for VCC.",
  width: 1200, height: 150,
  x: "time",
  // y_domain: [0, 24],
  x_label: "Time (ms)",
  y_label: "Voltage (V)",
  channels: [
    {y: "vcc_voltage", label: "VCC Voltage", color: colors.v},
    {y: "emf_speed_constant", label: "EMF Speed Constant", color: d3.color(colors.voltage_angle).brighter(1)},
    {y: "emf_speed_constant_stdev", label: "EMF Speed Constant stdev", color: d3.color(colors.voltage_angle).darker(1)},
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
  subtitle: "Phase Currents",
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

    {y: "u_current_zero", label: "Current U 0 offset", color: d3.color(colors.u).brighter(1)},
    {y: "v_current_zero", label: "Current V 0 offset", color: d3.color(colors.v).brighter(1)},
    {y: "w_current_zero", label: "Current W 0 offset", color: d3.color(colors.w).brighter(1)},

    {y: "battery_current", label: "Battery Current", color: colors.other},
    {y: "zero_current", label: "Current Sum/3", color: colors.sum},
    {y: "ref_readout", label: "Reference value", color: colors.ref_readout},
  ],
  curve,
});


const plot_voltages = plot_lines({
  subtitle: "Phase Voltages",
  description: "Measured voltage values for each phase.",
  width: 1200, height: 300,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Voltage (V)",
  channels: [
    {y: "u_drive_voltage", label: "Drive Voltage U", color: colors.u},
    {y: "v_drive_voltage", label: "Drive Voltage V", color: colors.v},
    {y: "w_drive_voltage", label: "Drive Voltage W", color: colors.w},
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
    {y: "x_current", label: "Current on X axis", color: colors.x_current},
    {y: "y_current", label: "Current on Y", color: colors.y_current},
    {y: "prev_x_current", label: "Previous Current on X axis", color: d3.color(colors.x_current).darker(1)},
    {y: "prev_y_current", label: "Previous Current on Y axis", color: d3.color(colors.y_current).darker(1)},
    {y: "prev_current_magnitude", label: "Previous Current Magnitude", color: d3.color(colors_categories[2]).darker(1)},
    {y: "current_magnitude", label: "Current Magnitude", color: colors_categories[2]},
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
    {y: "x_emf_voltage", label: "EMF Voltage on X", color: colors.x_current},
    {y: "y_emf_voltage", label: "EMF Voltage on Y", color: colors.y_current},
    {y: "emf_voltage_magnitude", label: "EMF Voltage Magnitude", color: colors_categories[5]},
    {y: "emf_magnitude_stdev", label: "EMF Magnitude stdev", color: d3.color(colors_categories[9]).darker(1)},
    {y: "drive_voltage_magnitude", label: "Drive Voltage Magnitude", color: colors_categories[2]},
    {y: "x_drive_voltage", label: "Drive Voltage on X axis", color: colors.u},
    {y: "y_drive_voltage", label: "Drive Voltage on Y axis", color: colors.v},
  ],
  curve,
});


const plot_inferred_voltages = plot_lines({
  subtitle: "Calibration Voltages",
  description: "Voltages deduced from calibration.",
  width: 1200, height: 300,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Voltage (V)",
  channels: [
    {y: "x_resistive_voltage", label: "X Resistive", color: colors_categories[0]},
    {y: "y_resistive_voltage", label: "Y Resistive", color: colors_categories[1]},
    {y: "x_inductance_saliency_voltage", label: "X Inductance Saliency", color: colors_categories[2]},
    {y: "y_inductance_saliency_voltage", label: "Y Inductance Saliency", color: colors_categories[3]},
    {y: "x_inductance_base_voltage", label: "X Inductance Base", color: colors_categories[4]},
    {y: "y_inductance_base_voltage", label: "Y Inductance Base", color: colors_categories[5]},
    {y: "x_hysteresis_loss_voltage", label: "X Hysteresis Loss", color: colors_categories[6]},
    {y: "y_hysteresis_loss_voltage", label: "Y Hysteresis Loss", color: colors_categories[7]},
    {y: "x_voltage_error", label: "X Voltage Error", color: colors_categories[8]},
    {y: "y_voltage_error", label: "Y Voltage Error", color: colors_categories[9]},
    {y: "voltage_error", label: "Voltage Error", color: colors_categories[10]},
    {y: "x_drive_voltage", label: "Drive Voltage on X axis", color: colors_categories[11]},
    {y: "y_drive_voltage", label: "Drive Voltage on Y axis", color: colors_categories[12]},
    {y: "x_vibration_voltage", label: "X Vibration", color: colors_categories[13]},
    {y: "y_vibration_voltage", label: "Y Vibration", color: colors_categories[14]},
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
    {y: "emf_fix", label: "EMF speed fix", color: colors_categories[4]},
    {y: "current_detected", label: "Current detected", color: colors_categories[6]},
    {y: "emf_counter", label: "EMF counter", color: colors_categories[7]},
    {y: "hall_u", label: "Hall U", color: colors.u},
    {y: "hall_v", label: "Hall V", color: colors.v},
    {y: "hall_w", label: "Hall W", color: colors.w},
  ],
  curve,
});

const plot_resistance_calibration = plot_lines({
  subtitle: "Resistance Calibration",
  description: "The calibration values for the motor's phase resistance.",
  width: 1200, height: 300,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Resistance (Ohms)",
  channels: [
    {y: "resistance", label: "Phase Resistance", color: colors.u},
    {y: "vibration_resistance", label: "Vibration 'Resistance'", color: colors_categories[7]},
  ],
  curve,
});

const plot_inductance_calibration = plot_lines({
  subtitle: "Inductance Calibration",
  description: "The calibration values for the motor's phase inductance.",
  width: 1200, height: 300,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Inductance (H)",
  channels: [
    {y: "inductance", label: "Phase Inductance", color: colors.v},
    {y: "inductance_saliency", label: "Inductance Saliency", color: colors.w},
    {y: "hysteresis_loss", label: "Hysteresis Loss", color: colors.w},
  ],
  curve,
});

const plot_angles_calibration = plot_lines({
  subtitle: "Angles Calibration",
  description: "Calibration of motor angles.",
  width: 1200, height: 300,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Angle (-pi to +pi)",
  y_domain: [-Math.PI, Math.PI],
  channels: [
    {y: "angle", label: "Angle", color: colors.u},
    {y: "saliency_angle", label: "Inductance Saliency Angle", color: colors.v},
    {y: "hysteresis_angle", label: "Hysteresis Loss Angle", color: colors.w},
    {y: "vibration_angle", label: "Vibration Angle", color: colors_categories[7]},
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
    {y: "readout_number_diff", label: "Readout Number Diff", color: colors_categories[0]},
    {y: "rotations", label: "Rotations", color: colors_categories[1]},
    {y: "resistance_sensitivity", label: "Resistance Sensitivity", color: colors_categories[2]},
    {y: "inductance_sensitivity", label: "Inductance Sensitivity", color: colors_categories[3]},
    {y: "hysteresis_loss_sensitivity", label: "Hysteresis Loss Sensitivity", color: colors_categories[6]},
    {y: "learning_rate_multiplier", label: "Learning Rate Multiplier", color: colors_categories[9]},
  ],
  curve,
});


const plot_gradients = plot_lines({
  subtitle: "Gradients",
  description: "Calibration gradients.",
  width: 1200, height: 400,
  x: "time",
  x_label: "Time (ms)",
  y_label: "Value",
  channels: [

    {y: "resistance_gradient", label: "Resistance Gradient", color: colors_categories[2]},
    {y: "vibration_resistance_gradient", label: "Vibration Resistance Gradient", color: colors_categories[4]},
    {y: "inductance_gradient", label: "Inductance Gradient", color: colors_categories[5]},
    {y: "inductance_saliency_gradient", label: "Inductance Saliency Gradient", color: colors_categories[6]},
    {y: "hysteresis_loss_gradient", label: "Hysteresis Loss Gradient", color: colors_categories[7]},
    {y: "gradients_dot", label: "Gradients Dot", color: colors_categories[8]},
    {y: "gradients_dot_variance", label: "Gradients Dot Variance", color: colors_categories[9]},
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
  plot_dq0_currents,
  plot_dq0_voltages,
  plot_inferred_voltages,
  plot_pwm_settings,
  plot_readout_flags,
  plot_resistance_calibration,
  plot_inductance_calibration,
  plot_angles_calibration,
  plot_motor_values,
  plot_gradients,
};

autosave_inputs(monitoring_plots);
```

```js
// Calibration
// -----------

function stringify_active_calibration() {
  return `motor_controller.calibration = ${JSON.stringify(motor_controller?.calibration, null, 2)}`;
}

const active_calibration_table =  Mutable(stringify_active_calibration());


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
  inductance_saliency: {
    label: "Inductance Saliency", 
    description: `The saliency of the motor's phase inductance.`
  },
  hysteresis_loss: {
    label: "Hysteresis Loss", 
    description: `The hysteresis loss from eddy currents.`
  },
  eddy_resistance: {
    label: "Eddy Current Resistance", 
    description: `The resistance due to iron eddy currents (specifically the back EMF from the change in eddy currents with time).`
  },
  vibration_resistance: {
    label: "Vibration Resistance", 
    description: `The fraction voltage stolen by vibration.`
  },
  motor_constant: {
    label: "Motor Constant", 
    description: `The motor constant, which relates torque and current, or speed and emf.`
  },
}

const calibration_input = Object.fromEntries(
  Object.entries(calibration_parameters).map(([key, {label, description}]) => {
    
    let parameter_input = Inputs.number([], {
      label,
      value: motor_controller?.calibration?.[key],
    });

    d3.select(parameter_input).style("width", "100%").style("margin", "1em 0em 1em 0em")
      .select("div").style("width", "50em")
        .append("span").text(description).style("margin", "0em 1em 0em 1em").style("width", "100em");

    return [key, parameter_input];
  })
);


function show_active_calibration() {
  active_calibration_table.value = stringify_active_calibration();
  Object.entries(calibration_input).forEach(([key, input]) => {
    input.value = motor_controller?.calibration?.[key];
  });
}

const calibration_data = Mutable(null);

const update_calibration_data = (new_value) => {
  calibration_data.value = new_value;
  if (new_value.is_stable) {
    Object.entries(calibration_input).forEach(([key, input]) => {
      input.value = new_value.calibration[key];
    });
  }
};
```

```js

const calibration_buttons = !motor_controller ? html`<p>Not connected to motor!</p>` : Inputs.button(
  [
    ["Run calibration", async function(){
      if (!motor_controller) return;
      await motor_controller.send_command({message_code: MessageCode.RUN_CALIBRATION});
    }],
    ["Upload to Driver", async function(){
      if (!motor_controller) return;
      const calibration = {
        ...motor_controller.calibration,
        ...Object.fromEntries(Object.entries(calibration_input).map(([key, input]) => [key, input.value])),
      };
      await motor_controller.upload_calibration(calibration);
      show_active_calibration();
      unsaved_changes();
    }],
    ["Reload from Driver", async function(){
      await motor_controller.load_calibration();
      show_active_calibration();
    }],
    ["Reset to Defaults", async function(){
      await motor_controller.reset_calibration();
      show_active_calibration();
      unsaved_changes();
    }],
  ],
  {
    label: "Calibration",
  },
);

d3.select(calibration_buttons).style("width", "100%");



const calibration_series_resistance_plot = plot_lines({
  subtitle: "Calibration Series - Optimizing resistance & inductance",
  description: "Calibration optimization results for each phase at various probing frequencies.",
  width: 1200, height: 300,
  x: "target_speed",
  x_label: "Probing Frequency",
  y_label: "Calibration Parameters",
  channels: [
    {y: "resistance", label: "Phase Resistance", color: colors_categories[0]},
    {y: "hysteresis_loss", label: "Hysteresis Loss", color: colors_categories[6]},
    {y: "vibration_resistance", label: "Vibration Resistance", color: colors_categories[7]},
  ],
  default_draw: draw_circles_of_size(function(d) { return 2 * Math.sqrt(d.target_pwm / 64.0); }),
});

const calibration_series_inductance_plot = plot_lines({
  subtitle: "Calibration Series - Optimizing resistance & inductance",
  description: "Calibration optimization results for each phase at various probing frequencies.",
  width: 1200, height: 300,
  x: "target_speed",
  x_label: "Probing Frequency",
  y_label: "Calibration Parameters",
  channels: [
    {y: "inductance", label: "Phase Inductance", color: colors_categories[2]},
    {y: "inductance_saliency", label: "Inductance Saliency", color: colors_categories[4]},

  ],
  default_draw: draw_circles_of_size(function(d) { return 2 * Math.sqrt(d.target_pwm / 64.0); }),
});


const calibration_series_angles_plot = plot_lines({
  subtitle: "Angles Calibration",
  description: "Calibration of motor angles.",
  width: 1200, height: 300,
  x: "target_speed",
  x_label: "Probing Frequency",
  y_label: "Angle (-pi to +pi)",
  y_domain: [-Math.PI, Math.PI],
  channels: [
    {y: "angle", label: "Angle", color: colors.u},
    {y: "saliency_angle", label: "Inductance Saliency Angle", color: colors.v},
    {y: "hysteresis_angle", label: "Hysteresis Loss Angle", color: colors.w},
    {y: "vibration_angle", label: "Vibration Angle", color: colors_categories[7]},
  ],
  default_draw: draw_circles_of_size(function(d) { return 2 * Math.sqrt(d.target_pwm / 64.0); }),
});

const calibration_series_stats_plot = plot_lines({
  subtitle: "Stats",
  description: "Calibration stats.",
  width: 1200, height: 300,
  x: "target_speed",
  x_label: "Probing Frequency",
  y_label: "Value",
  channels: [
    {y: "avg_current_magnitude", label: "Avg Current", color: colors_categories[4]},
    {y: "avg_drive_voltage", label: "Avg Drive Voltage", color: colors_categories[5]},
    {y: "avg_error", label: "Avg Error", color: colors_categories[6]},
  ],
  default_draw: draw_circles_of_size(function(d) { return 2 * Math.sqrt(d.target_pwm / 64.0); }),
});


const calibration_series_plots = {
  calibration_series_resistance_plot,
  calibration_series_inductance_plot,
  calibration_series_angles_plot,
  calibration_series_stats_plot,
};

autosave_inputs(calibration_series_plots);
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
    ["rotation_zero", {
      label: "Rotation Zero",
      description: "The zero position of the motor rotation.",
    }],
    ["angle_zero", {
      label: "Angle Zero",
      description: "The zero position of the motor angle.",
    }],
    ["motor_direction", {
      label: "Motor direction", 
      description: "Direction of the motor rotation (+1 for default, -1 to reverse rotation direction)."
    }],
    ["angle_probing_interval", {
      label: "Angle Probing Interval",
      description: "Interval at which we do the saliency angle probing (when we don't have EMF).",
    }],
    ["angle_probing_pwm", {
      label: "Angle Probing PWM",
      description: "PWM used during angle probing."
    }],
    ["angle_probing_sweep_duration", {
      label: "Angle Probing Sweep Duration",
      description: "Duration of the sweep during angle probing."
    }],
    ["angle_probing_angular_speed", {
      label: "Angle Probing Angular Speed",
      description: "Angular speed used during angle probing."
    }],
    ["u_pwm_floor", {
      label: "U PWM Floor",
      description: "Minimum PWM value for the voltage applied to the U motor phase (below this value the switch doesn't turn on).",
    }],
    ["v_pwm_floor", {
      label: "V PWM Floor",
      description: "Minimum PWM value for the voltage applied to the V motor phase (below this value the switch doesn't turn on).",
    }],
    ["w_pwm_floor", {
      label: "W PWM Floor",
      description: "Minimum PWM value for the voltage applied to the W motor phase (below this value the switch doesn't turn on).",
    }],
    ["z_pwm_floor", {
      label: "Z PWM Floor",
      description: "Minimum PWM value for the voltage applied to the Z motor phase (below this value the switch doesn't turn on).",
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
    ["angle_from_emf_ki", {
      label: "Angle from EMF KI",
      description: "Integral gain for the angle and angular speed estimated from EMF."
    }],
    ["loop_fast_ki", {
      label: "Loop Fast KI",
      description: "Integral gain for fast changing values in the tight control loop."
    }],
    ["loop_slow_ki", {
      label: "Loop Slow KI",
      description: "Integral gain for slow changing values in the control loop."
    }],
    ["angle_from_hall_ki", {
      label: "Position adjustment from hall angle KI", 
      description: "Integral gain for the angle adjustment based on the hall sensors."
    }],
    ["lead_angle_control_ki", {
      label: "Lead Angle Control KI",
      description: "Lead angle control gain. The lead angle controls how far ahead we drive the voltage for maximum torque/efficiency."
    }],
    ["angle_from_saliency_ki", {
      label: "Angle from Saliency KI",
      description: "Integral gain for the angle estimated from the saliency effect. Controls how fast we adjust the PWM to compensate for the saliency angle."
    }],
    ["current_measurement_minimum", {
      label: "Current Measurement Minimum",
      description: "Minimum current measurement value. Helps in filtering out spurious low current readings."
    }],
    ["voltage_measurement_variance", {
      label: "Voltage Measurement Variance",
      description: "Variance of the voltage measurement noise. Helps in filtering and estimating the true voltage accurately."
    }],
    ["resistance_current_minimum", {
      label: "Resistance Current Minimum",
      description: "Minimum current for resistance measurement. Helps in ensuring accurate resistance estimation."
    }],
    ["inductance_excitation_minimum_square", {
      label: "Inductance Excitation Minimum Square",
      description: "Minimum squared excitation current for inductance measurement. Ensures accurate inductance estimation."
    }],
    ["current_offset_maximum", {
      label: "Current Offset Maximum",
      description: "Maximum allowable current offset. Helps in filtering out systematic measurement errors."
    }],
    ["emf_min_angular_speed", {
      label: "EMF Minimum Angular Speed",
      description: `Minimum detectable angular speed for EMF detection. This value also determines
        the maximum reliable angular speed variance. It also determines how fast we probe
        for EMF when HFI is disabled and we must rely on spinning the motor to generate a
        detectable EMF signal.`
    }],
    ["emf_uncertainty_threshold", {
      label: "EMF Uncertainty Threshold",
      description: `When we detect incosistent EMF measurements (static EMF or high correlation with
        the drive voltage) we need to re-enable calibration in order to isolate differing
        resistance or inductance from the observed EMF measurements. We use a counter for
        each inconsistent reading and this threshold determines when we must act.`
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
    ["emf_speed_constant_ki", {
      label: "EMF Speed Constant KI",
      description: "EMF speed constant integral gain. Controls how fast we adjust for errors in the back EMF estimation."
    }],
    ["emf_angle_suspected_threshold", {
      label: "EMF Speed Constant Suspected Threshold",
      description: "Threshold for suspecting errors in the EMF speed constant estimation."
    }],
    ["emf_speed_constant_confirmed_threshold", {
      label: "EMF Speed Constant Confirmed Threshold",
      description: "Threshold for confirming the EMF speed constant estimation."
    }],
    ["zero_current_ki", {
      label: "Zero Current KI", 
      description: "Integral gain for the zero current observer; helps in accurately estimating the zero current offset for each motor phase."
    }],
    ["resistance_ki", {
      label: "Phase Resistance KI",
      description: "Integral gain for phase resistance estimation. Helps in accurately estimating the resistance of each motor phase in star configuration."
    }],
    ["inductance_ki", {
      label: "Phase Inductance KI",
      description: "Integral gain for phase inductance estimation. Helps in accurately estimating the inductance of each motor phase in star configuration."
    }],
    ["inductance_saliency_ki", {
      label: "Inductance Saliency KI",
      description: "Integral gain for the inductance saliency observer; helps in accurately estimating the saliency in the inductance measurement."
    }],
    ["saliency_angle_ki", {
      label: "Saliency Angle KI",
      description: "Integral gain for saliency angle estimation. To be explained later..."
    }],
    ["hysteresis_loss_ki", {
      label: "Hysteresis Loss KI",
      description: "Integral gain for hysteresis loss."
    }],
    ["hysteresis_angle_ki", {
      label: "Hysteresis Loss Angle KI",
      description: "Integral gain for hysteresis loss angle."
    }],
    ["vibration_resistance_ki", {
      label: "Vibration Resistance KI",
      description: "Integral gain for vibration resistance."
    }],
    ["vibration_angle_ki", {
      label: "Vibration Angle KI",
      description: "Integral gain for vibration angle."
    }],
    ["eddy_resistance_ki", {
      label: "Eddy Resistance KI",
      description: "Integral gain for eddy resistance."
    }],
    ["motor_constant_ki", {
      label: "Motor Constant KI", 
      description: "Integral gain for the motor constant observer; the relation between speed and EMF magnitude."
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
      await motor_controller.send_command({message_code: MessageCode.SAVE_SETTINGS_TO_FLASH});
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

// TODO: this needs redoing... we don't even have any unit tests.
// But also we are dropping `send_command_and_await_reply`, to do the unit
// test routine, we must first intercept "message" events from the motor controller
// then concatenate them to get the test output (this way we can also get longer
// test outputs).
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

import {plot_lines, plot_line, draw_circles_of_size, setup_faint_area, horizontal_step, setup_stdev_95, draw_line} from "./components/plotting_utils.js";

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
  PWM_BASE, HISTORY_SIZE, CURRENT_UNITS_PER_AMP, VOLTAGE_UNITS_PER_VOLT, ENABLE_ALL_OUTPUTS, DISABLE_ALL_OUTPUTS,
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
  z: "rgb(178, 228, 0)",
  angle: "rgb(39, 163, 185)",
  web_current_magnitude: "rgb(197, 152, 67)",
  current_angle: "rgb(102, 166, 30)",
  voltage_angle: "rgb(0, 185, 124)",
  angle_driven: "rgb(166, 30, 132)",
  angular_speed: "rgb(41, 194, 173)",
  web_angular_speed: "rgb(156, 196, 47)",
  x_current: "rgb(199, 0, 57)",
  y_current: "rgb(26, 82, 118)",
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
    console.warn("Motor connection error:", error);
    return html`<pre style="color: red">Connection lost; unknown error: ${error}</pre>`;
  }
}

```