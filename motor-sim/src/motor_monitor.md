---
title: Motor monitor
theme: dashboard
---


```js
const port = view(Inputs.button("Connect", {
  value: null, 
  reduce: open_port,
}));
```

```js
const data = port != null ? data_stream(parse_with_delimiter(read_from(port), "measurements 123456:")) : null;
```

```js
console.log(data);

display(data);

display(Plot.plot({
  marks: [
    Plot.lineY(data, {x: "adc_update_number", y: "adc_value_0", stroke: 'black', label: 'adc 0'}),
  ]
}));
```

```js
async function * data_stream(messages, max_entries = 400) {
  let data = [];
  const n = 20;

  for await (const message of messages) {
    if (message.buffer.byteLength != 4 + 8 * n) continue;
    
    let data_view = new DataView(message.buffer);
    
    let offset = 0;
    let adc_update_number = data_view.getUint32(0);
    offset += 4;

    let message_data = [];

    for (let i = 0; i < n; i++) {
      let adc_value_0 = data_view.getUint16(offset);
      offset += 2;
      let adc_value_1 = data_view.getUint16(offset);
      offset += 2;
      let adc_value_2 = data_view.getUint16(offset);
      offset += 2;
      let adc_value_3 = data_view.getUint16(offset);
      offset += 2;

      message_data.push({adc_update_number: adc_update_number - n + i + 1, adc_value_0, adc_value_1, adc_value_2, adc_value_3});

    }

    data.push(...message_data);
    data = data.slice(-max_entries);

    yield data;
  }
}
```


```js
async function open_port(){
  let port = await navigator.serial.requestPort();
  await port.open({baudRate: 115200, bufferSize: 512});
  return port;
}

async function * read_from(port){
  while (port.readable) {
    const reader = port.readable.getReader();
    try {
      while (true) {
        const { value, done } = await reader.read();
        if (done) break;
        yield value;
      }
    } catch (error) {
      // Handle error, maybe, maybe not?
    } finally {
      reader.releaseLock();
    }
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
}
```
