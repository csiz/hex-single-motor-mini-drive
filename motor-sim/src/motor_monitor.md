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

const current_conversion = 0.004029304;
const time_conversion = 1/23400 * 1000;
```


Connect to COM port and read motor driver data.
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
let raw_data = Mutable();

async function command_and_stream(command, timeout){
  await send_command(port, command);
  for await (const data_snapshot of stream_state_readouts({port, timeout})) {
    raw_data.value = data_snapshot;
  }
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
      await command_and_stream(SET_STATE_OFF, 0);
    }],
    ["Drive", async function(){
      await command_and_stream(SET_STATE_DRIVE, 100 + command_timeout);
    }],
    ["Hold U positive", async function(){
      await command_and_stream(SET_STATE_HOLD_U_POSITIVE, 100 + command_timeout);
    }],
    ["Hold V positive", async function(){
      await command_and_stream(SET_STATE_HOLD_V_POSITIVE, 100 + command_timeout);
    }],
    ["Hold W positive", async function(){
      await command_and_stream(SET_STATE_HOLD_W_POSITIVE, 100 + command_timeout);
    }],
    ["Hold U negative", async function(){
      await command_and_stream(SET_STATE_HOLD_U_NEGATIVE, 100 + command_timeout);
    }],
    ["Hold V negative", async function(){
      await command_and_stream(SET_STATE_HOLD_V_NEGATIVE, 100 + command_timeout);
    }],
    ["Hold W negative", async function(){
      await command_and_stream(SET_STATE_HOLD_W_NEGATIVE, 100 + command_timeout);
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

<div class="card tight">
  <div>${calibration_plots}</div>
  <div>${calibration_buttons}</div>
  <div>${calibration_string}</div>
</div>


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


const calibration_points = 16;


const saved_calibration_string = `{"u_positive":{"x_min":0,"x_max":6.73,"Y":[0,0.449,0.875,1.24,1.61,2.03,2.45,2.87,3.31,3.76,4.23,4.71,5.21,5.75,6.33,6.98]},"u_negative":{"x_min":-7.42,"x_max":0,"Y":[-8.51,-7.74,-7.03,-6.37,-5.75,-5.16,-4.59,-4.04,-3.51,-2.99,-2.47,-1.96,-1.47,-0.988,-0.495,0]},"v_positive":{"x_min":0,"x_max":5.88,"Y":[0,0.392,0.783,1.18,1.58,2.01,2.44,2.88,3.33,3.79,4.27,4.77,5.29,5.85,6.45,7.13]},"v_negative":{"x_min":-5.76,"x_max":0,"Y":[-7.33,-6.63,-6.01,-5.43,-4.89,-4.38,-3.88,-3.4,-2.94,-2.48,-2.04,-1.6,-1.18,-0.768,-0.384,0]},"w_positive":{"x_min":0,"x_max":4.51,"Y":[0,0.301,0.601,0.891,1.18,1.49,1.8,2.12,2.44,2.78,3.13,3.48,3.86,4.26,4.7,5.18]},"w_negative":{"x_min":-4.46,"x_max":0,"Y":[-5.02,-4.56,-4.14,-3.75,-3.39,-3.05,-2.71,-2.38,-2.07,-1.76,-1.46,-1.16,-0.876,-0.593,-0.297,0]}}`;

const saved_calibration = Object.fromEntries(Object.entries(JSON.parse(saved_calibration_string)).map(([key, value]) => {
  return [key, even_piecewise_linear(value)];
}));


```

```js
const ref_readout_mean = d3.mean(raw_data, (d) => d.ref_readout);

const data = raw_data.map((d) => {
  const readout_number = d.readout_number - raw_data[0].readout_number;
  const time = readout_number * time_conversion;

  const u_readout = -current_conversion * (d.u_readout - d.ref_readout);
  const u = u_readout > 0 ? 
    saved_calibration.u_positive(u_readout) * 10 * 0.048 / 0.880 : 
    saved_calibration.u_negative(u_readout) * 10 * 0.048 / 0.990;

  const v_readout = +current_conversion * (d.v_readout - d.ref_readout);
  const v = v_readout > 0 ? 
    saved_calibration.v_positive(v_readout) * 10 * 0.048 / 0.818 : 
    saved_calibration.v_negative(v_readout) * 10 * 0.049 / 0.850;

  const w_readout = -current_conversion * (d.w_readout - d.ref_readout);
  const w = w_readout > 0 ? 
    saved_calibration.w_positive(w_readout) * 10 * 0.047 / 0.580 : 
    saved_calibration.w_negative(w_readout) * 10 * 0.049 / 0.610;
  
  const ref_diff = current_conversion * (d.ref_readout - ref_readout_mean);

  const u_pwm = d.u_pwm == SET_FLOATING_DUTY ? null : d.u_pwm;
  const v_pwm = d.v_pwm == SET_FLOATING_DUTY ? null : d.v_pwm;
  const w_pwm = d.w_pwm == SET_FLOATING_DUTY ? null : d.w_pwm;

  // const sum = (u_pwm === null ? 0 : u) + (v_pwm === null ? 0 : v) + (w_pwm === null ? 0 : w);
  const sum = u + v + w;

  return {...d, u_pwm, v_pwm, w_pwm, u_readout, v_readout, w_readout, u, v, w, ref_diff, time, sum};
});
```



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

const data_to_plot = ["u", "v", "w", "ref_diff", "sum", "u_pwm", "v_pwm", "w_pwm", "stats", "calibration_zones"];

const checkboxes_inputs = Inputs.checkbox(data_to_plot, {
  label: "Display:", 
  value: data_to_plot, 
  format: (d) => html`<span style="color: ${colors[d]}">${d}</span>`,
});

const checkboxes = Generators.input(checkboxes_inputs);

```


```js

const current_lines = {
  u: Plot.line(data, {x: "time", y: 'u', stroke: colors.u, label: 'adc 0', curve: 'step'}),
  v: Plot.line(data, {x: "time", y: 'v', stroke: colors.v, label: 'adc 1', curve: 'step'}),
  w: Plot.line(data, {x: "time", y: 'w', stroke: colors.w, label: 'adc 2', curve: 'step'}),
  sum: Plot.line(data, {x: "time", y: 'sum', stroke: colors.sum, label: 'sum', curve: 'step'}),
  ref_diff: Plot.line(data, {x: "time", y: 'ref_diff', stroke: colors.ref_diff, label: 'ref', curve: 'step'}),
  calibration_zones: Plot.rect(calibration_zones, {x1: "start", x2: "end", y1: -5.0, y2: 5.0, fill: "rgba(0, 0, 0, 0.1)"}),
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
      Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 1}),
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


```js

function cross_correlation(X, Y){
  const n = X.length;
  const m = Y.length;
  if (n !== m) throw new Error("X and Y must have the same length");
  if (n < 2) throw new Error("X and Y must have at least 2 points");

  const X_mean = d3.mean(X);
  const Y_mean = d3.mean(Y);

  const X_diff = X.map((x) => x - X_mean);
  const Y_diff = Y.map((y) => y - Y_mean);

  const X_diff_sq = X_diff.map((x) => x * x);
  const Y_diff_sq = Y_diff.map((y) => y * y);

  const X_diff_sq_sum = d3.sum(X_diff_sq);
  const Y_diff_sq_sum = d3.sum(Y_diff_sq);

  const X_diff_Y_diff = X_diff.map((x, i) => x * Y_diff[i]);
  const X_diff_Y_diff_sum = d3.sum(X_diff_Y_diff);

  return X_diff_Y_diff_sum / Math.sqrt(X_diff_sq_sum * Y_diff_sq_sum);
}

function compute_calibration({measurements, targets}){
  const last_measurement = measurements[measurements.length - 1];
  const is_positive = last_measurement > 0;

  
  const slow_calibration = is_positive ? piecewise_linear({
    X: [0.0, ...measurements], 
    Y: [0.0, ...targets],
  }) : piecewise_linear({
    X: [..._.reverse(measurements), 0.0], 
    Y: [..._.reverse(targets), 0.0],
  });

  const even_domain = is_positive ? 
    d3.range(calibration_points).map((i) => i / (calibration_points - 1) * last_measurement) :
    d3.range(calibration_points).map((i) => (calibration_points - 1 - i) / (calibration_points - 1) * last_measurement);

  const calibration_data = {
    x_min: is_positive ? 0.0 : last_measurement,
    x_max: is_positive ? last_measurement : 0.0,
    Y: even_domain.map(slow_calibration),
  };

  const calibration = even_piecewise_linear(calibration_data);
  const sample = even_domain.map((x) => ({reading: x, target: calibration(x)}));

  return {
    calibration_data,
    calibration,
    sample,
  };
} 


const calibration_means = calibration_zones.map((zone) => {
  const zone_data = data.filter((d) => d.time > zone.start && d.time < zone.end);
  return {
    ...zone,
    u: d3.mean(zone_data, (d) => d.u_readout),
    v: d3.mean(zone_data, (d) => d.v_readout),
    w: d3.mean(zone_data, (d) => d.w_readout),
  };
});



const calibration_ref = calibration_means[0];

const get_u_target = (c) => c.pwm / calibration_ref.pwm * calibration_ref.u;
const get_v_target = (c) => c.pwm / calibration_ref.pwm * calibration_ref.v;
const get_w_target = (c) => c.pwm / calibration_ref.pwm * calibration_ref.w;

const calibration_u = compute_calibration({
  measurements: calibration_means.map((c) => c.u),
  targets: calibration_means.map(get_u_target),
});

const calibration_v = compute_calibration({
  measurements: calibration_means.map((c) => c.v),
  targets: calibration_means.map(get_v_target),
});

const calibration_w = compute_calibration({
  measurements: calibration_means.map((c) => c.w),
  targets: calibration_means.map(get_w_target),
});

const calibration_plots = [
  Plot.plot({
    marks: [
      Plot.line(calibration_means, {x: "pwm", y: "u", stroke: colors.u, label: "u"}),
      Plot.line(calibration_means, {x: "pwm", y: get_u_target, stroke: colors.u, strokeDasharray: "2 5", label: "u linear"}),
      Plot.line(calibration_means, {x: "pwm", y: "v", stroke: colors.v, label: "v"}),
      Plot.line(calibration_means, {x: "pwm", y: get_v_target, stroke: colors.v, strokeDasharray: "2 5", label: "v linear"}),
      Plot.line(calibration_means, {x: "pwm", y: "w", stroke: colors.w, label: "w"}),
      Plot.line(calibration_means, {x: "pwm", y: get_w_target, stroke: colors.w, strokeDasharray: "2 5", label: "w linear"}),
    ],
    x: {label: "PWM"},
    y: {label: "Current (A)"},
    width: 1200, height: 300,
  }),

  Plot.plot({
    marks: [
      Plot.line(calibration_u.sample, {x: "reading", y: "target", stroke: colors.u, label: "u linear", marker: "circle"}),
      Plot.line(calibration_v.sample, {x: "reading", y: "target", stroke: colors.v, label: "v linear", marker: "circle"}),
      Plot.line(calibration_w.sample, {x: "reading", y: "target", stroke: colors.w, label: "w linear", marker: "circle"}),
    ],
    x: {label: "Current Reading (A)"},
    y: {label: "Current Target (A)"},
    width: 1200, height: 300,
  }),
];


const calibration_buttons = Inputs.button(
  [
    ["Calibrate U positive", function(){
      update_u_positive(calibration_u.calibration_data);
    }],
    ["Calibrate U negative", function(){
      update_u_negative(calibration_u.calibration_data);
    }],
    ["Calibrate V positive", function(){
      update_v_positive(calibration_v.calibration_data);
    }],
    ["Calibrate V negative", function(){
      update_v_negative(calibration_v.calibration_data);
    }],
    ["Calibrate W positive", function(){
      update_w_positive(calibration_w.calibration_data);
    }],
    ["Calibrate W negative", function(){
      update_w_negative(calibration_w.calibration_data);
    }],
],
  {label: "Collect calibration data"},
);

```

```js
let calibration_u_positive = Mutable("incomplete");
function update_u_positive(value){
  calibration_u_positive.value = value;
}
let calibration_u_negative = Mutable("incomplete");
function update_u_negative(value){
  calibration_u_negative.value = value;
}
let calibration_v_positive = Mutable("incomplete");
function update_v_positive(value){
  calibration_v_positive.value = value;
}
let calibration_v_negative = Mutable("incomplete");
function update_v_negative(value){
  calibration_v_negative.value = value;
}
let calibration_w_positive = Mutable("incomplete");
function update_w_positive(value){
  calibration_w_positive.value = value;
}
let calibration_w_negative = Mutable("incomplete");
function update_w_negative(value){
  calibration_w_negative.value = value;
}

```

```js
const calibration = {
  u_positive: calibration_u_positive,
  u_negative: calibration_u_negative,
  v_positive: calibration_v_positive,
  v_negative: calibration_v_negative,
  w_positive: calibration_w_positive,
  w_negative: calibration_w_negative,
};

const calibration_string = html`<div>Calibration:<pre style="display: inline-block; vertical-align: top; white-space: pre-wrap; word-wrap: break-word;">${stringify_nicely(calibration)}</pre></div>`;
```

```js

// Stream motor driver data
// ------------------------

async function * stream_state_readouts({port, timeout = 200, max_missed_messages = 1}) {
  let data = [];

  let messages = parse_with_delimiter(read_from(port, timeout), uint32_to_bytes(STATE_READOUT));

  for await (const message of messages) {
    if (message.buffer.byteLength != 16) continue;
    
    let data_view = new DataView(message.buffer);
    
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

    if (data.length % 64 === 0) yield data;
  }

  yield data;
}
```


```js

// USB serial port communication
// -----------------------------

async function send_command(port, command){
  let writer = port.writable.getWriter();
  let buffer = new Uint8Array(8);
  let view = new DataView(buffer.buffer);
  view.setUint32(0, command);
  view.setUint16(4, Math.floor(command_timeout / time_conversion));
  view.setUint16(6, Math.floor(command_value * PWM_BASE));
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

async function * read_from(port, timeout) {
  const reader = port.readable.getReader();
  try {
    while (true) {
      const { value, done } = await timeout_promise(reader.read(), timeout);
      if (done) break;
      yield value;
    }
  } catch (error) {
    console.error("Error reading from port:", error);
  } finally {
    reader.releaseLock();
  }
}


function find_substring_index(text, delimiter) {
  for (let i = 0; i <= text.length - delimiter.length; i++) {
    let match = true;
    for (let j = 0; j < delimiter.length; j++) {
      if (text[i + j] !== delimiter[j]) {
        match = false;
        break;
      }
    }
    if (match) {
      return i;
    }
  }
  return -1;
}



async function * parse_with_delimiter(message_generator, delimiter){
  if (typeof delimiter === "string") {
    delimiter = new TextEncoder().encode(delimiter);
  }

  let buffer = new Uint8Array();
  for await (const chunk of message_generator) {
    let tempBuffer = new Uint8Array(buffer.length + chunk.length);
    tempBuffer.set(buffer);
    tempBuffer.set(chunk, buffer.length);
    buffer = tempBuffer;

    let delimiterIndex;
    
    while ((delimiterIndex = find_substring_index(buffer, delimiter)) !== -1) {
      let part = buffer.slice(0, delimiterIndex);
      yield part;
      buffer = buffer.slice(delimiterIndex + delimiter.length);
    }
  }
  yield buffer;
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


function round3(value) {
  return Number(value.toPrecision(3));
}

function stringify_nicely(value) {
  return JSON.stringify(value, (key, value) => {
    if (typeof value === "number") return round3(value);
    return value;
  }, 0);
}

function piecewise_linear({X, Y}) {
  const n = X.length;
  const m = Y.length;
  if (n !== m) throw new Error("X and Y must have the same length");
  if (n < 2) throw new Error("X and Y must have at least 2 points");

  const slopes = new Array(n - 1);
  const intercepts = new Array(n - 1);
  for (let i = 0; i < n - 1; i++) {
    slopes[i] = (Y[i + 1] - Y[i]) / (X[i + 1] - X[i]);
    intercepts[i] = Y[i] - slopes[i] * X[i];
  }

  return function(x) {
    let i = (x < X[0]) ? 0 : (x >= X[n - 1]) ? n - 2 : d3.bisectLeft(X, x, 1, n - 1) - 1;
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
    let i = (x < x_min) ? 0 : (x >= x_max) ? n - 2 : Math.floor((x - x_min) / x_period);
    return slopes[i] * x + intercepts[i];
  };
}

```

```js
// Imports
// -------

import {html} from "htl";
```