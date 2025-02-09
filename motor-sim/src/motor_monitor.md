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
        ["Read ADC", function(){
          try {
            return stream_adc_readouts(port);
          } catch (error) {
            console.error("Error reading from port:", error);
          }
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
    return display(html`Not connected...`);
  }

  const ref_readout_mean = d3.mean(data, (d) => d.ref_readout);

  const computed_data = data.map((d) => {
    const readout_number = d.readout_number;
    let u_readout = +(d.u_readout - d.ref_readout);
    u_readout = u_readout > 0 ? u_readout * 1.0 : u_readout * 1.0;
    let v_readout = -(d.v_readout - d.ref_readout);
    v_readout = v_readout > 0 ? v_readout * 1.0 : v_readout * 1.0;
    let w_readout = +(d.w_readout - d.ref_readout);
    w_readout = w_readout > 0 ? w_readout * 1.0 : w_readout * 1.0;
    const ref_readout = d.ref_readout - ref_readout_mean;

    return {readout_number, u_readout, v_readout, w_readout, ref_readout};
  });


  return display(Plot.plot({
    marks: [
      Plot.lineY(computed_data, {x: "readout_number", y: 'u_readout', stroke: 'steelblue', label: 'adc 0', curve: 'step'}),
      Plot.lineY(computed_data, {x: "readout_number", y: 'v_readout', stroke: 'orangered', label: 'adc 1', curve: 'step'}),
      Plot.lineY(computed_data, {x: "readout_number", y: 'w_readout', stroke: 'purple', label: 'adc 2', curve: 'step'}),
      Plot.lineY(computed_data, {x: "readout_number", y: (d) => d.u_readout + d.v_readout + d.w_readout, stroke: 'black', label: 'sum', curve: 'step'}),
      Plot.lineY(computed_data, {x: "readout_number", y: 'ref_readout', stroke: 'gray', label: 'ref', curve: 'step'}),
      Plot.gridY({interval: 100, stroke: 'black', strokeWidth : 1}),
    ],
    width: 1200,
  }));

}();
```



```js
const ADC_READOUT = 0x80202020;
const GET_ADC_READOUTS = 0x80202021;
// Uint8Array from uint32
function uint32_to_bytes(value) {
  let buffer = new Uint8Array(4);
  let view = new DataView(buffer.buffer);
  view.setUint32(0, value);
  return buffer;
}
```

```js


async function * stream_adc_readouts(port) {
  const max_missed_messages = 1;
  const timeout = 200;

  let data = [];

  await request_adc_readings(port);

  let messages = parse_with_delimiter(read_from(port, timeout), uint32_to_bytes(ADC_READOUT));


  for await (const message of messages) {
    if (message.buffer.byteLength != 12) continue;
    
    let data_view = new DataView(message.buffer);
    
    let offset = 0;
    let readout_number = data_view.getUint32(0);
    offset += 4;


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


async function request_adc_readings(port){
  let writer = port.writable.getWriter();
  let buffer = new Uint8Array(8);
  let view = new DataView(buffer.buffer);
  view.setUint32(0, GET_ADC_READOUTS);
  view.setUint32(4, 0);
  await writer.write(buffer);
  writer.releaseLock();
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