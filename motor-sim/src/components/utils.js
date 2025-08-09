import {html} from "htl";

/* Create a tooltip element. */
export function note (text) {
	return html`<div class="tooltip">*<span class="tooltiptext">${text}</span></div>`;
}

/* Create a link element. */
export function link (url) {
	return html`<a href="${url}" target="_blank">${url}</a>`;
}

/* Track update frequency compared to wall time. */
export class TimingStats {
  constructor() {
    this.last_time = Date.now();
    this.fps = 0.0;
    this.gamma = 0.9;
  }

  /* Update the FPS tracker after each frame is done rendereing. */
  update() {
    const now = Date.now();
    const wall_dt = now - this.last_time;
    this.last_time = now;
    this.fps = this.gamma * this.fps + (1.0-this.gamma) * (1000.0 / wall_dt);
  };
}

/* Create Uint8Array from uint32. */
export function uint32_to_bytes(value) {
  let buffer = new Uint8Array(4);
  let view = new DataView(buffer.buffer);
  view.setUint32(0, value);
  return buffer;
}

/* Convert 4 byte buffer to uint32. */
export function bytes_to_uint32(buffer) {
  // Check buffer is exactly 4 bytes long
  if (buffer.byteLength !== 4) throw new Error("Buffer must be 4 bytes long");
  let view = new DataView(buffer.buffer);
  return view.getUint32(0);
}

/* Convert 2 byte buffer to uint16. */
export function bytes_to_uint16(buffer) {
  // Check buffer is exactly 2 bytes long
  if (buffer.byteLength !== 2) throw new Error("Buffer must be 2 bytes long");
  let view = new DataView(buffer.buffer);
  return view.getUint16(0);
}

/* Round to `n` decimals. */
export function round(value, n = 3) {
  return Number(value.toPrecision(n));
}

/* Wait milliseconds. */
export function wait(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

/* Make promise timeout after milliseconds. */
export function timeout_promise(promise, timeout) {
  let timeout_id;

  const timed_reject = new Promise((resolve, reject)=>{
    timeout_id = setTimeout(() => {
      reject(new Error("Timeout"));
    }, timeout);
  });

  return Promise.race([promise.finally(() => { clearTimeout(timeout_id); }), timed_reject]);
}

/* Clean a string so it can be used as an id. */
export function clean_id(id) {
  return id.replace(/[^a-zA-Z0-9_]/g, "_");
}

export function valid_number(value) {
  return (typeof value === "number" && !isNaN(value) && isFinite(value));
}
