import {bytes_to_uint16, timeout_promise, wait} from "./utils.js";

import {Generators} from "observablehq:stdlib";

// USB serial port commands
// ------------------------
export const USBD_VID = 56987;
export const USBD_PID_FS = 56988;

export const READOUT = 0x2020;
export const GET_READOUTS = 0x2021;
export const GET_READOUTS_SNAPSHOT = 0x2022;
export const SET_STATE_OFF = 0x2030;
export const SET_STATE_DRIVE_POS = 0x2031;
export const SET_STATE_TEST_ALL_PERMUTATIONS = 0x2032;
export const SET_STATE_DRIVE_NEG = 0x2033;
export const SET_STATE_FREEWHEEL = 0x2034;

export const SET_STATE_TEST_GROUND_SHORT = 0x2036;
export const SET_STATE_TEST_POSITIVE_SHORT = 0x2037;
export const SET_STATE_TEST_U_DIRECTIONS = 0x2039;
export const SET_STATE_TEST_U_INCREASING = 0x203A;
export const SET_STATE_TEST_U_DECREASING = 0x203B;
export const SET_STATE_TEST_V_INCREASING = 0x203C;
export const SET_STATE_TEST_V_DECREASING = 0x203D;
export const SET_STATE_TEST_W_INCREASING = 0x203E;
export const SET_STATE_TEST_W_DECREASING = 0x203F;

export const SET_STATE_HOLD_U_POSITIVE = 0x3020;
export const SET_STATE_HOLD_V_POSITIVE = 0x3021;
export const SET_STATE_HOLD_W_POSITIVE = 0x3022;
export const SET_STATE_HOLD_U_NEGATIVE = 0x3023;
export const SET_STATE_HOLD_V_NEGATIVE = 0x3024;
export const SET_STATE_HOLD_W_NEGATIVE = 0x3025;

export const SET_STATE_DRIVE_SMOOTH_POS = 0x4030;
export const SET_STATE_DRIVE_SMOOTH_NEG = 0x4031;


// Other constants
export const PWM_BASE = 1536; // 0x0600
export const MAX_TIMEOUT = 0xFFFF; 
export const HISTORY_SIZE = 420;

export const READOUT_BASE = 0x10000;


// USB serial port communication
// -----------------------------

export class COMPort {
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


export class MotorController {
  constructor(port){
    this.com_port = new COMPort(port);
    this.data = [];
    this.notify = null;
    this.resolve = null;
    this.reject = null;
    this.expected_messages = 0;
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

      // Stream data as it's being transmitted.
      if (this.data.length && (this.data.length % 1024 === 0)) this._push_readouts(this.data);

      if (this.data.length == this.expected_messages) {
        this._resolve_readouts(this.data);
        this._push_readouts(this.data);
      }
    }
  }

  async send_command({command, command_timeout, command_pwm, command_leading_angle = 0}) {
    if (this.com_port === null) return;
    
    let buffer = new Uint8Array(8);
    let view = new DataView(buffer.buffer);
    let offset = 0;
    view.setUint16(offset, command);
    offset += 2;
    view.setUint16(offset, command_timeout);
    offset += 2;
    view.setUint16(offset, command_pwm);
    offset += 2;
    view.setUint16(offset, command_leading_angle);
    offset += 2;
    await this.com_port.write(buffer);
  }

  _expect_readouts({expected_messages = HISTORY_SIZE}) {
    this.data = [];
    this.expected_messages = expected_messages;
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
    const message_header = await this.com_port.read(2)
    if (bytes_to_uint16(message_header) != READOUT) {
      console.warn("Unknown message header: ", message_header);
      return 2;
    }

    const data_bytes = await this.com_port.read(16);

    const data_view = new DataView(data_bytes.buffer);

    let offset = 0;
    const readout_number = data_view.getUint16(offset);
    offset += 2;
    const position = data_view.getUint16(offset);
    offset += 2;
    // The first 3 bits are the hall sensor state.
    const hall_u = (position >> 13) & 0b1;
    const hall_v = (position >> 14) & 0b1;
    const hall_w = (position >> 15) & 0b1;
    const motor_angle_valid = (position >> 12) & 0b1;
    const motor_angle = position & 0xFF;
    
    const pwm_commands = data_view.getUint32(offset);
    offset += 4;

    const u_pwm = Math.floor(pwm_commands / PWM_BASE / PWM_BASE) % PWM_BASE;
    const v_pwm = Math.floor(pwm_commands / PWM_BASE) % PWM_BASE;
    const w_pwm = pwm_commands % PWM_BASE;


    const u_readout = data_view.getUint16(offset);
    offset += 2;
    const v_readout = data_view.getUint16(offset);
    offset += 2;
    const w_readout = data_view.getUint16(offset);
    offset += 2;
    const ref_readout = data_view.getUint16(offset);
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
      hall_u,
      hall_v,
      hall_w,
      motor_angle_valid,
      motor_angle,
    });

    return offset + 2;
  }
}


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
  return await navigator.serial.requestPort({filters: [{usbVendorId: USBD_VID, usbProductId: USBD_PID_FS}]});

}

export async function connect_usb_motor_controller(){
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

  if (tries <= 0) throw new Error("Port open failed; InvalidStateError after all retries.");

  if (!port.readable) throw new Error("Port unreadable");
  if (!port.writable) throw new Error("Port unwritable");

  return new MotorController(port);
}

