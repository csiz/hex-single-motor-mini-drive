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
const SET_STATE_TEST_SINGLE_PHASE_POSITIVE = 0x80202033;
const SET_STATE_TEST_DOUBLE_PHASE_POSITIVE = 0x80202034;
const SET_STATE_TEST_ALL_SHORTED = 0x80202035;
const SET_STATE_TEST_LONG_GROUNDED_SHORT = 0x80202036;
const SET_STATE_TEST_LONG_POSITIVE_SHORT = 0x80202037;

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
```


Connect to COM port and read motor driver data.
<div>${connect_buttons}</div>
<div>${data_stream_buttons}</div>
<div>${command_buttons}</div>

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
    ["Test single phase positive", async function(){
      await command_and_stream(SET_STATE_TEST_SINGLE_PHASE_POSITIVE, 500);
    }],
    ["Test double phase positive", async function(){
      await command_and_stream(SET_STATE_TEST_DOUBLE_PHASE_POSITIVE, 500);
    }],
    ["Test all shorted", async function(){
      await command_and_stream(SET_STATE_TEST_ALL_SHORTED, 500);
    }],
    ["Test long grounded short", async function(){
      await command_and_stream(SET_STATE_TEST_LONG_GROUNDED_SHORT, 500);
    }],
    ["Test long positive short", async function(){
      await command_and_stream(SET_STATE_TEST_LONG_POSITIVE_SHORT, 500);
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
      await send_command(port, SET_STATE_OFF);
    }],
    ["Drive", async function(){
      await send_command(port, SET_STATE_DRIVE);
    }],
    ["Hold U positive", async function(){
      await send_command(port, SET_STATE_HOLD_U_POSITIVE);
    }],
    ["Hold V positive", async function(){
      await send_command(port, SET_STATE_HOLD_V_POSITIVE);
    }],
    ["Hold W positive", async function(){
      await send_command(port, SET_STATE_HOLD_W_POSITIVE);
    }],
    ["Hold U negative", async function(){
      await send_command(port, SET_STATE_HOLD_U_NEGATIVE);
    }],
    ["Hold V negative", async function(){
      await send_command(port, SET_STATE_HOLD_V_NEGATIVE);
    }],
    ["Hold W negative", async function(){
      await send_command(port, SET_STATE_HOLD_W_NEGATIVE);
    }],
  ],
  {label: "Commands"},
);
```

Motor driver phase currents
----------------------------
<div class="card tight">${checkboxes_inputs}</div>

<div class="card tight">${currents_plots}</div>

<div class="card tight">${calibration_plots}</div>


```js

const current_conversion = 0.004029304;
const time_conversion = 1/23400 * 1000;

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
```

```js
const ref_readout_mean = d3.mean(raw_data, (d) => d.ref_readout);

const data = raw_data.map((d) => {
  const readout_number = d.readout_number - raw_data[0].readout_number;
  const time = readout_number * time_conversion;

  let u_readout = -current_conversion * (d.u_readout - d.ref_readout);
  u_readout = 10*(u_readout > 0 ? u_readout * 0.048 / 0.747 : u_readout * 0.048 / 0.940);

  
  let v_readout = +current_conversion * (d.v_readout - d.ref_readout);
  v_readout = 10*(v_readout > 0 ? v_readout * 0.048 / 0.818 : v_readout * 0.049 / 0.853);
  let w_readout = -current_conversion * (d.w_readout - d.ref_readout);
  w_readout = 10*(w_readout > 0 ? w_readout * 0.047 / 0.533: w_readout * 0.049 / 0.560);
  const ref_diff = current_conversion * (d.ref_readout - ref_readout_mean);

  let u_pwm = d.u_pwm == SET_FLOATING_DUTY ? null : d.u_pwm;
  let v_pwm = d.v_pwm == SET_FLOATING_DUTY ? null : d.v_pwm;
  let w_pwm = d.w_pwm == SET_FLOATING_DUTY ? null : d.w_pwm;
  

  return {...d, u_pwm, v_pwm, w_pwm, u_readout, v_readout, w_readout, ref_diff, time};
});
```



```js
const colors = {
  u_readout: "cyan",
  v_readout: "orangered",
  w_readout: "purple",
  ref_diff: "gray",
  sum: "black",
  u_pwm: "cyan",
  v_pwm: "orangered",
  w_pwm: "purple",
};

const data_to_plot = ["u_readout", "v_readout", "w_readout", "ref_diff", "sum", "u_pwm", "v_pwm", "w_pwm", "stats", "calibration_zones"];

const checkboxes_inputs = Inputs.checkbox(data_to_plot, {
  label: "Display:", 
  value: data_to_plot, 
  format: (d) => html`<span style="color: ${colors[d]}">${d}</span>`,
});

const checkboxes = Generators.input(checkboxes_inputs);

```


```js
const currents_plots = function(){

  const current_lines = {
    u_readout: Plot.line(data, {x: "time", y: 'u_readout', stroke: colors.u_readout, label: 'adc 0', curve: 'step'}),
    v_readout: Plot.line(data, {x: "time", y: 'v_readout', stroke: colors.v_readout, label: 'adc 1', curve: 'step'}),
    w_readout: Plot.line(data, {x: "time", y: 'w_readout', stroke: colors.w_readout, label: 'adc 2', curve: 'step'}),
    sum: Plot.line(data, {x: "time", y: (d) => d.u_readout + d.v_readout + d.w_readout, stroke: colors.sum, label: 'sum', curve: 'step'}),
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

  const stats_table = html`<div>Current stats:</div><table>
    <tr><td>U:</td><td>${d3.mean(data, (d) => d.u_readout).toFixed(3)} A</td></tr>
    <tr><td>V:</td><td>${d3.mean(data, (d) => d.v_readout).toFixed(3)} A</td></tr>
    <tr><td>W:</td><td>${d3.mean(data, (d) => d.w_readout).toFixed(3)} A</td></tr>
    <tr><td>Sum:</td><td>${d3.mean(data, (d) => d.u_readout + d.v_readout + d.w_readout).toFixed(3)} A</td></tr>
    <tr><td>Ref:</td><td>${d3.mean(data, (d) => d.ref_readout).toFixed(0)} ADC bits</td></tr>
  </table>`;



  return [
    ...(checkboxes.includes("stats") ? [stats_table] : []),
    Plot.plot({
      marks: [
        ...selected_current_lines,
        Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 1}),
        Plot.gridX({interval: 0.2, stroke: 'black', strokeWidth : 1}),
      ],
      x: {label: "Time (ms)"},
      width: 1200, height: 500,
    }),
    Plot.plot({
      marks: [
        ...selected_pwm_lines,
        Plot.gridX({interval: 0.5, stroke: 'black', strokeWidth : 1}),
        Plot.gridY({interval: 128, stroke: 'black', strokeWidth : 1}),
      ],
      x: {label: "Time (ms)"},
      width: 1200, height: 150,
    }),
  ];

}();

```


```js
function round3(value) {
  return Number(value.toPrecision(3));
}


const calibration_plots = function(){

  let u_calibrations = calibration_zones.map((zone) => d3.mean(data.filter((d) => d.time > zone.start && d.time < zone.end), (d) => d.u_readout)/zone.pwm);
  let v_calibrations = calibration_zones.map((zone) => d3.mean(data.filter((d) => d.time > zone.start && d.time < zone.end), (d) => d.v_readout)/zone.pwm);
  let w_calibrations = calibration_zones.map((zone) => d3.mean(data.filter((d) => d.time > zone.start && d.time < zone.end), (d) => d.w_readout)/zone.pwm);

  u_calibrations = u_calibrations.map((value) => value / u_calibrations[0]);
  v_calibrations = v_calibrations.map((value) => value / v_calibrations[0]);
  w_calibrations = w_calibrations.map((value) => value / w_calibrations[0]);

  const calibration_array = html`<div>Calibration:</div><table>
    <tr><td>U:</td><td><raw>${JSON.stringify(u_calibrations.map((d) => round3(d)))}</raw></td></tr>
    <tr><td>V:</td><td><raw>${JSON.stringify(v_calibrations.map((d) => round3(d)))}</raw></td></tr>
    <tr><td>W:</td><td><raw>${JSON.stringify(w_calibrations.map((d) => round3(d)))}</raw></td></tr>
  </table>`;
  
  return calibration_array;

}();
```

```js
// Uint8Array from uint32
function uint32_to_bytes(value) {
  let buffer = new Uint8Array(4);
  let view = new DataView(buffer.buffer);
  view.setUint32(0, value);
  return buffer;
}
```

```js

const PWM_BASE = 1536;
const SET_FLOATING_DUTY = PWM_BASE - 1;
const HISTORY_SIZE = 384;


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

async function send_command(port, command){
  let writer = port.writable.getWriter();
  let buffer = new Uint8Array(8);
  let view = new DataView(buffer.buffer);
  view.setUint32(0, command);
  view.setUint32(4, 0);
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
import {html} from "htl";
```