---
title: Motor monitor
---

```js
// USB serial port commands
// ------------------------

const STATE_READOUT = 0x80202020;
const GET_STATE_READOUTS = 0x80202021;
const SET_STATE_OFF = 0x80202030;
const SET_STATE_DRIVE = 0x80202031;
const SET_STATE_TEST_ALL_PERMUTATIONS = 0x80202032;

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
const SET_FLOATING_DUTY = PWM_BASE - 1;
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

const max_calibration_current = 1.0 * drive_voltage / drive_resistance;
```


Motor command dashboard
-----------------------

<div>${connect_buttons}</div>
<div>${data_stream_buttons}</div>
<div>${command_buttons}</div>
<div>
  ${command_value_slider}
  ${command_timeout_slider}
</div>


```js

let port = Mutable();

const connect_buttons = Inputs.button(
  [
    ["Connect", async function(){
      if (port.value) {
        console.log("Forgetting previous port");
        await port.value.forget();
      }
      try {
        const new_port = await navigator.serial.requestPort();
        await new_port.open({baudRate: 115200, bufferSize: 16});
        if (new_port.readable) port.value = new_port;
      } catch (error) {
        console.error("Error requesting port:", error);
      }
    }],
  ],
  {label: "Connect to COM"},
);
```

```js
const command_value_slider = Inputs.range([0, 1], {value: 0.2, step: 0.05, label: "Command value:"});

const command_value = Generators.input(command_value_slider);

const command_timeout_slider = Inputs.range([0, MAX_TIMEOUT*time_conversion], {value: 2000, step: 100, label: "Command timeout (ms):"});

const command_timeout = Generators.input(command_timeout_slider);
```

```js
let raw_readout_data = Mutable();

async function command_and_stream(command, timeout){
  await send_command(port, command, command_timeout, command_value);

  if (port.readable.locked) {
    console.error("Port is locked");
    return;
  }

  // Start reading the data stream.
  for await (const data_snapshot of stream_state_readouts({port, timeout})) {
    raw_readout_data.value = data_snapshot;
  }
}

async function command(command){
  await send_command(port, command, command_timeout, command_value);
}

const data_stream_buttons = Inputs.button(
  [
    ["ADC", async function(){
      await command_and_stream(GET_STATE_READOUTS, 50);
    }],
    ["Test all permutations", async function(){
      await command_and_stream(SET_STATE_TEST_ALL_PERMUTATIONS, 500);
    }],
    ["Test ground short", async function(){
      await command_and_stream(SET_STATE_TEST_GROUND_SHORT, 500);
    }],
    ["Test positive short", async function(){
      await command_and_stream(SET_STATE_TEST_POSITIVE_SHORT, 500);
    }],
    ["Test U directions", async function(){
      await command_and_stream(SET_STATE_TEST_U_DIRECTIONS, 500);
    }],
    ["Test U increasing", async function(){
      await command_and_stream(SET_STATE_TEST_U_INCREASING, 500);
    }],
    ["Test U decreasing", async function(){
      await command_and_stream(SET_STATE_TEST_U_DECREASING, 500);
    }],
    ["Test V increasing", async function(){
      await command_and_stream(SET_STATE_TEST_V_INCREASING, 500);
    }],
    ["Test V decreasing", async function(){
      await command_and_stream(SET_STATE_TEST_V_DECREASING, 500);
    }],
    ["Test W increasing", async function(){
      await command_and_stream(SET_STATE_TEST_W_INCREASING, 500);
    }],
    ["Test W decreasing", async function(){
      await command_and_stream(SET_STATE_TEST_W_DECREASING, 500);
    }],
  ],
  {label: "Read data"},
);

const command_buttons = Inputs.button(
  [
    ["Stop", async function(){
      await command(SET_STATE_OFF);
    }],
    ["Drive", async function(){
      await command(SET_STATE_DRIVE, command_timeout);
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

const colors = {
  u: "cyan",
  v: "orangered",
  w: "purple",
  ref_diff: "gray",
  sum: "black",
  u_pwm: "cyan",
  v_pwm: "orangered",
  w_pwm: "purple",
};

const data_to_plot = ["u", "v", "w", "ref_diff", "sum", "u_pwm", "v_pwm", "w_pwm", "stats"];

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
  
    const u_pwm = d.u_pwm == SET_FLOATING_DUTY ? null : d.u_pwm;
    const v_pwm = d.v_pwm == SET_FLOATING_DUTY ? null : d.v_pwm;
    const w_pwm = d.w_pwm == SET_FLOATING_DUTY ? null : d.w_pwm;
  
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
  
    return {...d, u_pwm, v_pwm, w_pwm, u_readout, v_readout, w_readout, u, v, w, ref_diff, time, sum};
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

const pwm_lines = {
  u_pwm: Plot.line(data, {x: "time", y: 'u_pwm', stroke: colors.u_pwm, label: 'pwm 0', curve: 'step', strokeDasharray: "1 4", strokeWidth: 2}),
  v_pwm: Plot.line(data, {x: "time", y: 'v_pwm', stroke: colors.v_pwm, label: 'pwm 1', curve: 'step', strokeDasharray: "1 3", strokeWidth: 2}),
  w_pwm: Plot.line(data, {x: "time", y: 'w_pwm', stroke: colors.w_pwm, label: 'pwm 2', curve: 'step', strokeDasharray: "2 4", strokeWidth: 2}),
};

const selected_current_lines = Object.keys(current_lines).filter((key) => checkboxes.includes(key)).map((key) => current_lines[key]);
const selected_pwm_lines = Object.keys(pwm_lines).filter((key) => checkboxes.includes(key)).map((key) => pwm_lines[key]);

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
      ...selected_pwm_lines,
      Plot.gridX({interval: 0.5, stroke: 'black', strokeWidth : 1}),
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
  {pwm: 0.1, start: short_duration * 1.4, end: short_duration * 2},
  {pwm: 0.2, start: short_duration * 2.4, end: short_duration * 3},
  {pwm: 0.3, start: short_duration * 3.4, end: short_duration * 4},
  {pwm: 0.4, start: short_duration * 4.4, end: short_duration * 5},
  {pwm: 0.5, start: short_duration * 5.4, end: short_duration * 6},
  {pwm: 0.6, start: short_duration * 6.4, end: short_duration * 7},
  {pwm: 0.7, start: short_duration * 7.4, end: short_duration * 8},
  {pwm: 0.8, start: short_duration * 8.4, end: short_duration * 9},
  {pwm: 0.9, start: short_duration * 9.4, end: short_duration * 10},
];


const calibration_points = 32;


async function command_and_return({command, timeout, command_timeout, command_value}) {
  await send_command(port, command, command_timeout, command_value);
  let data = [];
  for await (const data_snapshot of stream_state_readouts({port, timeout})) {
    data = data_snapshot;
  }
  return data;
}

async function run_calibration(){
  console.info("Calibration starting");

  // Note: hold pwm is clamped by the motor driver

  await command_and_return({command: SET_STATE_HOLD_U_POSITIVE, timeout: 500, command_timeout: 1000, command_value: 1.0});
  const u_positive_data = calculate_data_stats(
    await command_and_return({command: SET_STATE_TEST_U_INCREASING, timeout: 100, command_timeout: 0, command_value: 0})
  ).data;

  console.info("U positive done");

  await command_and_return({command: SET_STATE_HOLD_W_NEGATIVE, timeout: 500, command_timeout: 1000, command_value: 1.0});
  const w_negative_data = calculate_data_stats(
    await command_and_return({command: SET_STATE_TEST_W_DECREASING, timeout: 100, command_timeout: 0, command_value: 0})
  ).data;

  console.info("W negative done");

  await command_and_return({command: SET_STATE_HOLD_V_POSITIVE, timeout: 500, command_timeout: 1000, command_value: 1.0});
  const v_positive_data = calculate_data_stats(
    await command_and_return({command: SET_STATE_TEST_V_INCREASING, timeout: 100, command_timeout: 0, command_value: 0})
  ).data;

  console.info("V positive done");

  await command_and_return({command: SET_STATE_HOLD_U_NEGATIVE, timeout: 500, command_timeout: 1000, command_value: 1.0});
  const u_negative_data = calculate_data_stats(
    await command_and_return({command: SET_STATE_TEST_U_DECREASING, timeout: 100, command_timeout: 0, command_value: 0})
  ).data;

  console.info("U negative done");

  await command_and_return({command: SET_STATE_HOLD_W_POSITIVE, timeout: 500, command_timeout: 1000, command_value: 1.0});
  const w_positive_data = calculate_data_stats(
    await command_and_return({command: SET_STATE_TEST_W_INCREASING, timeout: 100, command_timeout: 0, command_value: 0})
  ).data;

  console.info("W positive done");

  await command_and_return({command: SET_STATE_HOLD_V_NEGATIVE, timeout: 500, command_timeout: 1000, command_value: 1.0});
  const v_negative_data = calculate_data_stats(
    await command_and_return({command: SET_STATE_TEST_V_DECREASING, timeout: 100, command_timeout: 0, command_value: 0})
  ).data;

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

// Stream motor driver data
// ------------------------

async function * stream_state_readouts({port, expected_messages = 420, timeout = 200, max_missed_messages = 1}) {

  let data = [];
  let buffer = new Uint8Array();
  let chunk = null;


  for (let i = 0; i < expected_messages; i++){
    ({chunk, buffer} = await read_from({port, n_bytes: 4, timeout, buffer}));
    if(chunk === null) break;
    if (bytes_to_uint32(chunk) != STATE_READOUT) break;

    ({chunk, buffer} = await read_from({port, n_bytes: 16, timeout, buffer}));
    if(chunk === null) break;

    const data_view = new DataView(chunk.buffer);

    let offset = 0;
    let readout_number = data_view.getUint32(0);
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


    data.push({
      readout_number,
      u_readout,
      v_readout,
      w_readout,
      ref_readout,
      u_pwm,
      v_pwm,
      w_pwm,
    });

    if (data.length > 2) {
      if ((data[data.length - 1].readout_number - data[data.length - 2].readout_number) > max_missed_messages){
        data = data.slice(0, -1);
        // Give up reading the rest.
        break;
      }
    }

    if (data.length && (data.length % 256 == 0)) yield data;
  }

  if (data.length) yield data;
}
```


```js

// USB serial port communication
// -----------------------------

async function send_command(port, command, timeout, value) {
  let writer = port.writable.getWriter();
  let buffer = new Uint8Array(8);
  let view = new DataView(buffer.buffer);
  view.setUint32(0, command);
  view.setUint16(4, Math.floor(timeout / time_conversion));
  view.setUint16(6, Math.floor(value * PWM_BASE));
  await writer.write(buffer);
  writer.releaseLock();
}

function wait(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function timeout_promise(promise, timeout) {
  return new Promise((resolve, reject) => {
    let timer = setTimeout(() => {
      reject(new Error("Timeout"));
    }, timeout);
    promise.then(resolve, reject).finally(() => {
      clearTimeout(timer);
    });
  });
}

async function read_from({port, n_bytes, timeout, buffer}) {
  if (buffer.length >= n_bytes) {
    const chunk = buffer.slice(0, n_bytes);
    buffer = buffer.slice(n_bytes);
    return {chunk, buffer};
  }

  const reader = port.readable.getReader();
  try {
    while (true) {
      const { value, done } = await timeout_promise(reader.read(), timeout);
      if (done) break;
      
      let concatenated_buffer = new Uint8Array(buffer.length + value.length);
      concatenated_buffer.set(buffer);
      concatenated_buffer.set(value, buffer.length);
      buffer = concatenated_buffer;

      if (buffer.length >= n_bytes) {
        const chunk = buffer.slice(0, n_bytes);
        buffer = buffer.slice(n_bytes);
        return {chunk, buffer};
      }
    }
  } catch (error) {
    if (error.message !== "Timeout") console.error("Error reading from port:", error);
    else console.info("Timeout reading from port");
  } finally {
    reader.releaseLock();
  }

  return {chunk: null, buffer};
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