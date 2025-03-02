---
title: Motor monitor
---

```js
// USB serial port commands
// ------------------------
const USBD_VID = 56987;
const USBD_PID_FS = 56988;

const READOUT = 0x80202020;
const GET_READOUTS = 0x80202021;
const GET_READOUTS_SNAPSHOT = 0x80202022;
const SET_STATE_OFF = 0x80202030;
const SET_STATE_DRIVE_POS = 0x80202031;
const SET_STATE_TEST_ALL_PERMUTATIONS = 0x80202032;
const SET_STATE_DRIVE_NEG = 0x80202033;
const SET_STATE_FREEWHEEL = 0x80202034;

const SET_STATE_TEST_GROUND_SHORT = 0x80202036;
const SET_STATE_TEST_POSITIVE_SHORT = 0x80202037;

const SET_STATE_TEST_U_DIRECTIONS = 0x80202039;
const SET_STATE_TEST_U_INCREASING = 0x8020203A;
const SET_STATE_TEST_U_DECREASING = 0x8020203B;
const SET_STATE_TEST_V_INCREASING = 0x8020203C;
const SET_STATE_TEST_V_DECREASING = 0x8020203D;
const SET_STATE_TEST_W_INCREASING = 0x8020203E;
const SET_STATE_TEST_W_DECREASING = 0x8020203F;

const SET_STATE_HOLD_U_POSITIVE = 0x80203020;
const SET_STATE_HOLD_V_POSITIVE = 0x80203021;
const SET_STATE_HOLD_W_POSITIVE = 0x80203022;
const SET_STATE_HOLD_U_NEGATIVE = 0x80203023;
const SET_STATE_HOLD_V_NEGATIVE = 0x80203024;
const SET_STATE_HOLD_W_NEGATIVE = 0x80203025;

// Other constants
const PWM_BASE = 1536;
const MAX_TIMEOUT = 0xFFFF; 
const HISTORY_SIZE = 420;

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

const phase_inductance = 0.000_21; // 0.21 mH
const phase_resistance = 2.0; // 2.0 Ohm
```


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

let motor_controller = Mutable(null);

let motor_controller_status = Mutable(html`<span>Not connected.</span>`);

async function grab_ports(){
  const ports = await navigator.serial.getPorts();
  return ports.filter((port) => {
    const info = port.getInfo();
    return info.usbVendorId === USBD_VID && info.usbProductId === USBD_PID_FS;
  });
}

async function maybe_prompt_port(){
  const ports = await grab_ports();
  if (ports.length == 1) return ports[0];
  try {
    return await navigator.serial.requestPort({filters: [{usbVendorId: USBD_VID, usbProductId: USBD_PID_FS}]});
  } catch (error) {
    if (error.name === "SecurityError") {
      motor_controller_status.value = html`<span style="color: red">Permission for port dialog denied.</span>`;
      return null;
    }
    throw error;
  }
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
    const port = await maybe_prompt_port();

    if (!port) return;

    let tries = 10;

    while(tries-- > 0){
      try {
        await port.open({baudRate: 115200, bufferSize: 16});
        break;
      } catch (error) {
        if (error.name === "InvalidStateError") {
          await wait(100);
          continue;
        }
        throw error;
      }
    }

    if (tries <= 0) throw new Error("Port open failed");

    if (!port.readable) throw new Error("Port unreadable");
    if (!port.writable) throw new Error("Port unwritable");

    motor_controller.value = new MotorController(port);

    motor_controller_status.value = html`<span>Connected, waiting for data.</span>`;

    for await (const data_received of motor_controller.value.reading_loop()) {
      motor_controller_status.value = html`<span>Connected; received: ${data_received}bytes.</span>`;
    }

  } catch (error) {
    motor_controller.value = null;

    if (error.message === "EOF") {
      motor_controller_status.value = html`<span style="color: red">End of connection.</span>`;
      return;
    }
    // Check for NetworkError due to disconnect.
    if (error.name === "NetworkError") {
      motor_controller_status.value = html`<span style="color: red">Connection lost.</span>`;
      return;
    }

    if (error.name === "NotFoundError") {
      motor_controller_status.value = html`<span style="color: red">No device found or nothing selected.</span>`;
      return;
    }

    motor_controller_status.value = html`<span style="color: red">Connection lost; unknown error: ${error}</span>`;
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

const command_timeout_slider = Inputs.range([0, MAX_TIMEOUT*time_conversion], {value: 2000, step: 100, label: "Command timeout (ms):"});

const command_timeout_millis = Generators.input(command_timeout_slider);
```

```js

// Control functions

let raw_readout_data = Mutable();

const max_data_points = HISTORY_SIZE;

const command_timeout = Math.floor(command_timeout_millis / time_conversion)
const command_value = Math.floor(command_value_fraction * PWM_BASE);


async function command_and_stream(command, options = {}){
  if (!motor_controller) return;
  
  await motor_controller.send_command({command, command_timeout, command_value, ...options});

  // Start reading the data stream.
  for await (const data_snapshot of motor_controller.stream_readouts(options)) {
    raw_readout_data.value = data_snapshot.length > max_data_points ? data_snapshot.slice(-max_data_points) : data_snapshot;
  }
}

async function command(command){
  if (!motor_controller) return;
  
  await motor_controller.send_command({command, command_timeout, command_value});
}


const data_stream_buttons = Inputs.button(
  [
    ["ADC SNAPSHOT", async function(){
      await command_and_stream(GET_READOUTS_SNAPSHOT, {expected_messages: HISTORY_SIZE, max_missed_messages: 0});
    }],
    ["ADC STREAM", async function(){
      await command_and_stream(GET_READOUTS, {max_missed_messages: 128});
    }],
    ["Test all permutations", async function(){
      await command_and_stream(SET_STATE_TEST_ALL_PERMUTATIONS);
    }],
    ["Test ground short", async function(){
      await command_and_stream(SET_STATE_TEST_GROUND_SHORT);
    }],
    ["Test positive short", async function(){
      await command_and_stream(SET_STATE_TEST_POSITIVE_SHORT);
    }],
    ["Test U directions", async function(){
      await command_and_stream(SET_STATE_TEST_U_DIRECTIONS);
    }],
    ["Test U increasing", async function(){
      await command_and_stream(SET_STATE_TEST_U_INCREASING);
    }],
    ["Test U decreasing", async function(){
      await command_and_stream(SET_STATE_TEST_U_DECREASING);
    }],
    ["Test V increasing", async function(){
      await command_and_stream(SET_STATE_TEST_V_INCREASING);
    }],
    ["Test V decreasing", async function(){
      await command_and_stream(SET_STATE_TEST_V_DECREASING);
    }],
    ["Test W increasing", async function(){
      await command_and_stream(SET_STATE_TEST_W_INCREASING);
    }],
    ["Test W decreasing", async function(){
      await command_and_stream(SET_STATE_TEST_W_DECREASING);
    }],
  ],
  {label: "Read data"},
);

const command_buttons = Inputs.button(
  [
    ["Stop", async function(){
      await command(SET_STATE_OFF);
    }],
    ["Drive+", async function(){
      await command(SET_STATE_DRIVE_POS);
    }],
    ["Drive-", async function(){
      await command(SET_STATE_DRIVE_NEG);
    }],
    ["Freewheel", async function(){
      await command(SET_STATE_FREEWHEEL);
    }],
    ["Hold U positive", async function(){
      await command(SET_STATE_HOLD_U_POSITIVE);
    }],
    ["Hold V positive", async function(){
      await command(SET_STATE_HOLD_V_POSITIVE);
    }],
    ["Hold W positive", async function(){
      await command(SET_STATE_HOLD_W_POSITIVE);
    }],
    ["Hold U negative", async function(){
      await command(SET_STATE_HOLD_U_NEGATIVE);
    }],
    ["Hold V negative", async function(){
      await command(SET_STATE_HOLD_V_NEGATIVE);
    }],
    ["Hold W negative", async function(){
      await command(SET_STATE_HOLD_W_NEGATIVE);
    }],
  ],
  {label: "Commands"},
);

```

Motor driver phase currents
----------------------------
<div class="card tight">
  <div>${checkboxes_inputs}</div>
  <div>${currents_plots}</div>
</div>

```js

const base_colors = {
  u: "cyan",
  v: "orangered",
  w: "purple",
  ref_diff: "gray",
  sum: "black",
};

const colors = {
  ...base_colors,
  u_pwm: base_colors.u,
  v_pwm: base_colors.v,
  w_pwm: base_colors.w,
  hall_3: base_colors.u, // Note colors are one permutation away
  hall_1: base_colors.v,
  hall_2: base_colors.w,
};

const data_to_plot = ["u", "v", "w", "ref_diff", "sum", "u_pwm", "v_pwm", "w_pwm", "hall_1", "hall_2", "hall_3", "stats"];

const checkboxes_inputs = Inputs.checkbox(data_to_plot, {
  label: "Display:", 
  value: data_to_plot, 
  format: (d) => html`<span style="color: ${colors[d]}">${d}</span>`,
});

const checkboxes = Generators.input(checkboxes_inputs);


const identity_calibration_factors = {
  u_positive: 1.0,
  u_negative: 1.0,
  v_positive: 1.0,
  v_negative: 1.0,
  w_positive: 1.0,
  w_negative: 1.0,
};

function load_calibration_factors(){
  try {
    const saved_calibration_string = localStorage.getItem("current_calibration");
    if (!saved_calibration_string) return identity_calibration_factors;
    return JSON.parse(localStorage.getItem("current_calibration"));
  } catch (error) {
    console.error("Error loading calibration:", error);
    return identity_calibration_factors;
  }
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

function calculate_data_stats(raw_readout_data){

  const ref_readout_mean = d3.mean(raw_readout_data, (d) => d.ref_readout);
  const start_readout_number = raw_readout_data[0].readout_number;

  const data = raw_readout_data.map((d) => {
    const readout_number = d.readout_number - start_readout_number;
    const time = readout_number * time_conversion;
  
    const u_readout = -current_conversion * (d.u_readout - d.ref_readout);
    const v_readout = +current_conversion * (d.v_readout - d.ref_readout);
    const w_readout = -current_conversion * (d.w_readout - d.ref_readout);
    const ref_diff = current_conversion * (d.ref_readout - ref_readout_mean);
  
    const u = u_readout * (u_readout >= 0 ? 
      calibration_factors.u_positive :
      calibration_factors.u_negative);

    const v = v_readout * (v_readout >= 0 ?
      calibration_factors.v_positive :
      calibration_factors.v_negative);

    const w = w_readout * (w_readout >= 0 ?
      calibration_factors.w_positive :
      calibration_factors.w_negative);

    // TODO: do I have to discard shunt readings for floating duty cycle?
    // const sum = (u_pwm === null ? 0 : u) + (v_pwm === null ? 0 : v) + (w_pwm === null ? 0 : w);
    const sum = u + v + w;
  
    return {...d, u_readout, v_readout, w_readout, u, v, w, ref_diff, time, sum};
  });

  return {data, ref_readout_mean, start_readout_number};
}

const {data, ref_readout_mean} = calculate_data_stats(raw_readout_data);


const current_lines = {
  u: Plot.line(data, {x: "time", y: 'u', stroke: colors.u, label: 'adc 0', curve: 'step'}),
  v: Plot.line(data, {x: "time", y: 'v', stroke: colors.v, label: 'adc 1', curve: 'step'}),
  w: Plot.line(data, {x: "time", y: 'w', stroke: colors.w, label: 'adc 2', curve: 'step'}),
  sum: Plot.line(data, {x: "time", y: 'sum', stroke: colors.sum, label: 'sum', curve: 'step'}),
  ref_diff: Plot.line(data, {x: "time", y: 'ref_diff', stroke: colors.ref_diff, label: 'ref', curve: 'step'}),
};

const derivative_start_index = 8;

function derivative_3_points(index, f){
  const left = data[derivative_start_index + index - 1];
  const mid = data[derivative_start_index + index];
  const right = data[derivative_start_index + index + 1];
  const left_derivative = (f(mid) - f(left)) / (mid.time - left.time) * 1000;
  const right_derivative = (f(right) - f(mid)) / (right.time - mid.time) * 1000;
  return (left_derivative + right_derivative) / 2;
}

const derivative_data = data.slice(derivative_start_index, -2).map((d, i) => {
  return {
    ...d,
    d_u: phase_inductance * derivative_3_points(i, (d) => d.u),
    d_v: phase_inductance * derivative_3_points(i, (d) => d.v),
    d_w: phase_inductance * derivative_3_points(i, (d) => d.w),
  };
});

const voltage_lines = {
  u: Plot.line(derivative_data, {x: "time", y: "d_u", stroke: colors.u, label: 'u', curve: 'step'}),
  v: Plot.line(derivative_data, {x: "time", y: "d_v", stroke: colors.v, label: 'v', curve: 'step'}),
  w: Plot.line(derivative_data, {x: "time", y: "d_w", stroke: colors.w, label: 'w', curve: 'step'}),
}

const pwm_lines = {
  u_pwm: Plot.line(data, {x: "time", y: 'u_pwm', stroke: colors.u, label: 'pwm 0', curve: 'step', strokeDasharray: "1 4", strokeWidth: 2}),
  v_pwm: Plot.line(data, {x: "time", y: 'v_pwm', stroke: colors.v, label: 'pwm 1', curve: 'step', strokeDasharray: "1 3", strokeWidth: 2}),
  w_pwm: Plot.line(data, {x: "time", y: 'w_pwm', stroke: colors.w, label: 'pwm 2', curve: 'step', strokeDasharray: "2 4", strokeWidth: 2}),
};

const hall_lines = {
  hall_3: Plot.line(data, {x: "time", y: 'hall_3', stroke: colors.u, label: 'hall 3', curve: 'step', strokeDasharray: "2 4", strokeWidth: 2}),
  hall_1: Plot.line(data, {x: "time", y: 'hall_1', stroke: colors.v, label: 'hall 1', curve: 'step', strokeDasharray: "1 4", strokeWidth: 2}),
  hall_2: Plot.line(data, {x: "time", y: 'hall_2', stroke: colors.w, label: 'hall 2', curve: 'step', strokeDasharray: "1 3", strokeWidth: 2}),
};

const selected_current_lines = Object.keys(current_lines).filter((key) => checkboxes.includes(key)).map((key) => current_lines[key]);
const selected_voltage_lines = Object.keys(current_lines).filter((key) => checkboxes.includes(key)).map((key) => voltage_lines[key]);

const selected_pwm_lines = Object.keys(pwm_lines).filter((key) => checkboxes.includes(key)).map((key) => pwm_lines[key]);
const selected_hall_lines = Object.keys(hall_lines).filter((key) => checkboxes.includes(key)).map((key) => hall_lines[key]);

const stats_table = data.length ? html`<div>Current stats:</div><table>
  <tr><td>U:</td><td>${d3.mean(data, (d) => d.u).toFixed(3)} A</td></tr>
  <tr><td>V:</td><td>${d3.mean(data, (d) => d.v).toFixed(3)} A</td></tr>
  <tr><td>W:</td><td>${d3.mean(data, (d) => d.w).toFixed(3)} A</td></tr>
  <tr><td>Sum:</td><td>${d3.mean(data, (d) => d.sum).toFixed(3)} A</td></tr>
  <tr><td>Ref:</td><td>${ref_readout_mean.toFixed(0)} ADC bits</td></tr>
</table>` : html`<div>No data</div>`;



const currents_plots = [
  ...(checkboxes.includes("stats") ? [stats_table] : []),
  Plot.plot({
    marks: [
      ...selected_current_lines,
      Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 2}),
      Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 2}),
      Plot.gridY({interval: 0.1, stroke: 'gray', strokeWidth : 1}),
      Plot.gridX({interval: 0.2, stroke: 'black', strokeWidth : 1}),
    ],
    x: {label: "Time (ms)"},
    y: {label: "Current (A)"},
    width: 1200, height: 500,
  }),
  Plot.plot({
    marks: [
      ...selected_hall_lines,
      Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 2}),
      Plot.gridX({interval: 0.2, stroke: 'black', strokeWidth : 1}),
    ],
    y: {label: "Hall state"},
    x: {label: "Time (ms)"},
    width: 1200, height: 150,
  }),
  Plot.plot({
    marks: [
      ...selected_voltage_lines,
      Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 2}),
      Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 2}),
      Plot.gridY({interval: 0.1, stroke: 'gray', strokeWidth : 1}),
      Plot.gridX({interval: 0.2, stroke: 'black', strokeWidth : 1}),
    ],
    x: {label: "Time (ms)", domain: [0, data[data.length - 1].time]},
    y: {label: "Voltage (V)"},
    width: 1200, height: 500,
  }),
  Plot.plot({
    marks: [
      ...selected_pwm_lines,
      Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 2}),
      Plot.gridX({interval: 0.2, stroke: 'black', strokeWidth : 1}),
      Plot.gridY({interval: 128, stroke: 'black', strokeWidth : 1}),
    ],
    y: {label: "PWM"},
    x: {label: "Time (ms)"},
    width: 1200, height: 150,
  }),
];

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
const short_duration = HISTORY_SIZE / 12 * time_conversion;

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
  const settle_strength = Math.floor(PWM_BASE * 2 / 10);

  console.info("Calibration starting");

  // Note: hold pwm is clamped by the motor driver

  await motor_controller.send_command({command: SET_STATE_HOLD_U_POSITIVE, command_timeout: settle_timeout, command_value: settle_strength});
  await wait(settle_time);
  await motor_controller.send_command({command: SET_STATE_TEST_U_INCREASING, command_timeout: 0, command_value: 0})

  const u_positive_data = calculate_data_stats(await motor_controller.get_readouts({expected_messages: HISTORY_SIZE, max_missed_messages: 0})).data;

  console.info("U positive done");

  await motor_controller.send_command({command: SET_STATE_HOLD_W_NEGATIVE, command_timeout: settle_timeout, command_value: settle_strength});
  await wait(settle_time);
  await motor_controller.send_command({command: SET_STATE_TEST_W_DECREASING, command_timeout: 0, command_value: 0})

  const w_negative_data = calculate_data_stats(await motor_controller.get_readouts({expected_messages: HISTORY_SIZE, max_missed_messages: 0})).data;

  console.info("W negative done");

  await motor_controller.send_command({command: SET_STATE_HOLD_V_POSITIVE, command_timeout: settle_timeout, command_value: settle_strength});
  await wait(settle_time);
  await motor_controller.send_command({command: SET_STATE_TEST_V_INCREASING, command_timeout: 0, command_value: 0})

  const v_positive_data = calculate_data_stats(await motor_controller.get_readouts({expected_messages: HISTORY_SIZE, max_missed_messages: 0})).data;

  console.info("V positive done");

  await motor_controller.send_command({command: SET_STATE_HOLD_U_NEGATIVE, command_timeout: settle_timeout, command_value: settle_strength});
  await wait(settle_time);
  await motor_controller.send_command({command: SET_STATE_TEST_U_DECREASING, command_timeout: 0, command_value: 0})

  const u_negative_data = calculate_data_stats(await motor_controller.get_readouts({expected_messages: HISTORY_SIZE, max_missed_messages: 0})).data;

  console.info("U negative done");

  await motor_controller.send_command({command: SET_STATE_HOLD_W_POSITIVE, command_timeout: settle_timeout, command_value: settle_strength});
  await wait(settle_time);
  await motor_controller.send_command({command: SET_STATE_TEST_W_INCREASING, command_timeout: 0, command_value: 0})

  const w_positive_data = calculate_data_stats(await motor_controller.get_readouts({expected_messages: HISTORY_SIZE, max_missed_messages: 0})).data;

  console.info("W positive done");

  await motor_controller.send_command({command: SET_STATE_HOLD_V_NEGATIVE, command_timeout: settle_timeout, command_value: settle_strength});
  await wait(settle_time);
  await motor_controller.send_command({command: SET_STATE_TEST_V_DECREASING, command_timeout: 0, command_value: 0})

  const v_negative_data = calculate_data_stats(await motor_controller.get_readouts({expected_messages: HISTORY_SIZE, max_missed_messages: 0})).data;

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


function wait(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function timeout_promise(promise, timeout) {
  let timeout_id;

  const timed_reject = new Promise((resolve, reject)=>{
    timeout_id = setTimeout(() => {
      reject(new Error("Timeout"));
    }, timeout);
  });

  return Promise.race([promise.finally(() => { clearTimeout(timeout_id); }), timed_reject]);
}

// USB serial port communication
// -----------------------------


class COMPort {
  constructor(port){
    this.port = port;
    this.buffer = new Uint8Array();
    this.reader = port.readable.getReader();
    this.writer = port.writable.getWriter();
  }

  async read(n_bytes){

    while (this.buffer.length < n_bytes){
      const {value, done} = await (this.reader.read());
      if (done) throw new Error("EOF");
      this.buffer = new Uint8Array([...this.buffer, ...value]);
    }
    const result = this.buffer.slice(0, n_bytes);
    this.buffer = this.buffer.slice(n_bytes);
    return result;
  }

  async write(buffer){
    await this.writer.write(buffer);
  }

  async forget(){
    await this.reader.cancel();
    this.reader.releaseLock();
    await this.writer.abort();
    this.writer.releaseLock();
    await this.port.close();
  }
}


// Motor control logic
// -------------------


class MotorController {
  constructor(port){
    this.com_port = new COMPort(port);
    this.data = [];
    this.notify = null;
    this.resolve = null;
    this.reject = null;
    this.expected_messages = 0;
    this.max_missed_messages = 0;
    this.timeout = 500;
  }

  async * reading_loop(){
    let data_received = 0;

    while(true){
      data_received += await this._read_message();
      yield data_received;

      // Nothing to do if we are not expecting messages.
      if (!this.expected_messages) {
        console.warn("Unexpected message received.");
        continue;
      }

      if (this.data.length > 2) {
        const last_index = this.data.length - 1;

        if ((this.data[last_index].readout_number - this.data[last_index - 1].readout_number) > (1 + this.max_missed_messages)){
          // Notify the observer and carry on.
          this._push_readouts(this.data.slice(0, -1));
          // Keep the last message.
          this.data = this.data.slice(-1);
          // Push an error to the deterministic readout.
          this._reject_readouts(new Error("Missed messages."));
        }
      }

      // Stream data as it's being transmitted.
      if (this.data.length && this.data.length % 1024) this._push_readouts(this.data);

      if (this.data.length == this.expected_messages) {
        this._resolve_readouts(this.data);
        this._push_readouts(this.data);
      }
    }
  }

  async send_command({command, command_timeout, command_value}) {
    if (this.com_port === null) return;

    let buffer = new Uint8Array(8);
    let view = new DataView(buffer.buffer);
    view.setUint32(0, command);
    view.setUint16(4, command_timeout);
    view.setUint16(6, command_value);
    await this.com_port.write(buffer);
  }

  _expect_readouts({expected_messages = HISTORY_SIZE, max_missed_messages = 0}) {
    this.data = [];
    this.expected_messages = expected_messages;
    this.max_missed_messages = max_missed_messages;
  }

  get_readouts(options={}) {
    this._clear_stream();
    this._reject_readouts(new Error("Overriding previous readout."));

    this._expect_readouts(options);

    return timeout_promise(new Promise((resolve, reject) => {
      this.resolve = resolve;
      this.reject = reject;
    }), this.timeout);
  }

  _resolve_readouts(data){
    if (this.resolve) {
      this.resolve(data);
      this.expected_messages = 0;
      this.resolve = null;
      this.reject = null;
    }
  }

  _reject_readouts(error){
    if (this.reject) {
      this.reject(error);
      this.expected_messages = 0;
      this.resolve = null;
      this.reject = null;
    }
  }

  stream_readouts(options={}) {
    this._expect_readouts(options);

    return Generators.observe((notify)=>{
      this.notify = notify;
    })
  }

  _push_readouts(data){
    if (this.notify) this.notify(data);
  }

  _clear_stream(last_data){
    if (this.notify) {
      if (last_data !== undefined) this.notify(last_data);
      this.expected_messages = 0;
      this.notify = null;
    }
  }



  async _read_message(){
    const message_header = await this.com_port.read(4)
    if (bytes_to_uint32(message_header) != READOUT) {
      console.warn("Unknown message header: ", message_header);
      return 4;
    }

    const data_bytes = await this.com_port.read(16);

    const data_view = new DataView(data_bytes.buffer);

    let offset = 0;
    let hall_and_readout_number = data_view.getUint32(0);
    // The first 3 bits are the hall sensor state.
    let hall_1 = (hall_and_readout_number >> 29) & 0b1;
    let hall_2 = (hall_and_readout_number >> 30) & 0b1;
    let hall_3 = (hall_and_readout_number >> 31) & 0b1;
    let readout_number = hall_and_readout_number & 0x1FFFFFFF;
    offset += 4;
    let pwm_commands = data_view.getUint32(offset);
    offset += 4;

    let u_pwm = Math.floor(pwm_commands / PWM_BASE / PWM_BASE) % PWM_BASE;
    let v_pwm = Math.floor(pwm_commands / PWM_BASE) % PWM_BASE;
    let w_pwm = pwm_commands % PWM_BASE;

    let u_readout = data_view.getUint16(offset);
    offset += 2;
    let v_readout = data_view.getUint16(offset);
    offset += 2;
    let w_readout = data_view.getUint16(offset);
    offset += 2;
    let ref_readout = data_view.getUint16(offset);
    offset += 2;

    // Only add messages if we are expecting them.
    if (this.expected_messages) this.data.push({
      readout_number,
      u_readout,
      v_readout,
      w_readout,
      ref_readout,
      u_pwm,
      v_pwm,
      w_pwm,
      hall_1,
      hall_2,
      hall_3,
    });

    return 20;
  }
}

```

```js
// Utils

// Uint8Array from uint32
function uint32_to_bytes(value) {
  let buffer = new Uint8Array(4);
  let view = new DataView(buffer.buffer);
  view.setUint32(0, value);
  return buffer;
}

function bytes_to_uint32(buffer) {
  // Check buffer is exactly 4 bytes long
  if (buffer.byteLength !== 4) throw new Error("Buffer must be 4 bytes long");
  let view = new DataView(buffer.buffer);
  return view.getUint32(0);
}


function round3(value) {
  return Number(value.toPrecision(3));
}

function even_spacing(max_value, n) {
  return d3.range(n).map((i) => i / (n - 1) * max_value);
}

function piecewise_linear({X, Y}) {
  const n = X.length;
  const m = Y.length;
  if (n !== m) throw new Error("X and Y must have the same length");
  if (n < 2) throw new Error("X and Y must have at least 2 points");

  // Check if X is sorted
  for (let i = 1; i < n; i++) {
    if (X[i] < X[i - 1]) throw new Error("X must be sorted");
  }

  const slopes = new Array(n - 1);
  const intercepts = new Array(n - 1);
  for (let i = 0; i < n - 1; i++) {
    slopes[i] = (Y[i + 1] - Y[i]) / (X[i + 1] - X[i]);
    intercepts[i] = Y[i] - slopes[i] * X[i];
  }

  return function(x) {
    let i = (x <= X[0]) ? 0 : (x > X[n - 1]) ? n - 2 : d3.bisectRight(X, x, 1, n - 1) - 1;
    return slopes[i] * x + intercepts[i];
  };
}

function even_piecewise_linear({x_min, x_max, Y}) {
  const n = Y.length;
  if (n < 2) throw new Error("Y must have at least 2 points");

  const x_period = (x_max - x_min) / (n - 1);

  const slopes = new Array(n - 1);
  const intercepts = new Array(n - 1);
  for (let i = 0; i < n - 1; i++) {
    slopes[i] = (Y[i + 1] - Y[i]) / x_period
    intercepts[i] = Y[i] - slopes[i] * (x_period * i + x_min);
  }

  return function(x) {
    let i = (x <= x_min) ? 0 : (x > x_max) ? n - 2 : Math.ceil((x - x_min) / x_period) - 1;
    return slopes[i] * x + intercepts[i];
  };
}

```

```js
// Imports
// -------

import {html} from "htl";

// Inline copy of https://observablehq.com/@mbostock/safe-local-storage
class MemoryStorage {
  constructor() {
    Object.defineProperties(this, {_: {value: new Map}});
  }
  get length() {
    return this._.size;
  }
  key(index) {
    return Array.from(this._.keys())[index | 0];
  }
  getItem(key) {
    return this._.has(key += "") ? this._.get(key) : null;
  }
  setItem(key, value) {
    this._.set(key + "", value + "");
  }
  removeItem(key) {
    this._.delete(key + "");
  }
  clear() {
    this._.clear();
  }
}

const localStorage = function(){
  try {
    const storage = window.localStorage;
    const key = "__storage_test__";
    storage.setItem(key, key);
    storage.removeItem(key);
    return storage;
  } catch (error) {
    return new MemoryStorage;
  }
}();

function get_default(key, default_func){
  let saved_value = localStorage.getItem(key);
  if (saved_value === null) return default_func();
  return JSON.parse(saved_value);
}

```