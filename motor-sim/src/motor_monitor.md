---
title: Motor monitor
---

Connect to COM port and read motor driver data.

```js
const port = view(Inputs.button(
  [
    ["Connect", async function(prev_port){
      prev_port = await prev_port;
      if (prev_port) {
        console.log("Forgetting previous port");
        await prev_port.forget();
      }
      try {
        const port = await navigator.serial.requestPort();
        await port.open({baudRate: 115200, bufferSize: 16});
        return port;
      } catch (error) {
        console.error("Error requesting port:", error);
        return undefined;
      }
    }],
    ["Disconnect", async function(prev_port){
      prev_port = await prev_port;
      if (prev_port) {
        console.log("Forgetting previous port");
        await prev_port.forget();
      }
      return undefined;
    }],
  ],
  {label: "Connect to COM", value: undefined},
));
```


```js


const data_stream = function(){
  if(port && port.readable) {
    return view(Inputs.button(
      [
        ["Read ADC", async function(){
          await send_command(port, GET_STATE_READOUTS);
          return stream_state_readouts({port, timeout: 50});
        }],
        ["Stop", async function(){
          await send_command(port, SET_STATE_OFF);
          await wait(100);
          await send_command(port, GET_STATE_READOUTS);
          return stream_state_readouts({port, timeout: 50});
        }],
        ["Measure current", async function(){
          await send_command(port, SET_STATE_MEASURE_CURRENT);
          return stream_state_readouts({port, timeout: 500});
        }],
        ["Drive", async function(){
          await send_command(port, SET_STATE_DRIVE);
          await wait(100);
          await send_command(port, GET_STATE_READOUTS);
          return stream_state_readouts({port, timeout: 50});
        }],
      ],
      {label: "Send commands", value: undefined},
    ));
  }
  
  return undefined;
}();
```

```js
const data = async function*(){
  if (data_stream) {
    yield* data_stream;
  }

  return undefined;
}();

```

Motor driver phase currents
----------------------------

```js
const currents_plots = function(){
  if (!data) {
    return html`Not connected...`;
  }

  const ref_readout_mean = d3.mean(data, (d) => d.ref_readout);

  const computed_data = data.map((d) => {
    const readout_number = d.readout_number - data[0].readout_number;
    const time = readout_number * 1/23200 * 1000;
    const current_conversion = 0.004029304;

    let u_readout = -current_conversion * (d.u_readout - d.ref_readout);
    u_readout = u_readout > 0 ? u_readout * 1.0 : u_readout * 1.0;
    let v_readout = +current_conversion * (d.v_readout - d.ref_readout);
    v_readout = v_readout > 0 ? v_readout * 1.0 : v_readout * 1.0;
    let w_readout = -current_conversion * (d.w_readout - d.ref_readout);
    w_readout = w_readout > 0 ? w_readout * 1.0 : w_readout * 1.0;
    const ref_readout = current_conversion * (d.ref_readout - ref_readout_mean);

    let u_pwm = d.u_pwm == SET_FLOATING_DUTY ? null : d.u_pwm;
    let v_pwm = d.v_pwm == SET_FLOATING_DUTY ? null : d.v_pwm;
    let w_pwm = d.w_pwm == SET_FLOATING_DUTY ? null : d.w_pwm;
    

    return {...d, u_pwm, v_pwm, w_pwm, u_readout, v_readout, w_readout, ref_readout, time};
  });

  return [
    Plot.plot({
      marks: [
        Plot.line(computed_data, {x: "time", y: 'u_readout', stroke: 'cyan', label: 'adc 0', curve: 'step'}),
        Plot.line(computed_data, {x: "time", y: 'v_readout', stroke: 'orangered', label: 'adc 1', curve: 'step'}),
        Plot.line(computed_data, {x: "time", y: 'w_readout', stroke: 'purple', label: 'adc 2', curve: 'step'}),
        Plot.line(computed_data, {x: "time", y: (d) => d.u_readout + d.v_readout + d.w_readout, stroke: 'black', label: 'sum', curve: 'step'}),
        Plot.line(computed_data, {x: "time", y: 'ref_readout', stroke: 'gray', label: 'ref', curve: 'step'}),
        Plot.gridY({interval: 1, stroke: 'black', strokeWidth : 1}),
        Plot.gridX({interval: 0.5, stroke: 'black', strokeWidth : 1}),
      ],
      x: {label: "Time (ms)"},
      width: 1200, height: 500,
    }),
    Plot.plot({
      marks: [
        Plot.line(computed_data, {x: "time", y: 'u_pwm', stroke: 'cyan', label: 'pwm 0', curve: 'step', strokeDasharray: "2 9", strokeWidth: 3}),
        Plot.line(computed_data, {x: "time", y: 'v_pwm', stroke: 'orangered', label: 'pwm 1', curve: 'step', strokeDasharray: "2 7", strokeWidth: 3}),
        Plot.line(computed_data, {x: "time", y: 'w_pwm', stroke: 'purple', label: 'pwm 2', curve: 'step', strokeDasharray: "2 11", strokeWidth: 3}),
        Plot.gridX({interval: 0.5, stroke: 'black', strokeWidth : 1}),
      ],
      x: {label: "Time (ms)"},
      width: 1200, height: 150,
    }),
  ];

}();
```

<div class="card tight">${currents_plots}</div>

```js
const STATE_READOUT = 0x80202020;
const GET_STATE_READOUTS = 0x80202021;
const SET_STATE_OFF = 0x80202030;
const SET_STATE_MEASURE_CURRENT = 0x80202031;
const SET_STATE_DRIVE = 0x80202032;

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
        data = data.slice(-1);
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