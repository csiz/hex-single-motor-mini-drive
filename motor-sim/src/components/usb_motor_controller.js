// USB serial port commands

import { exp } from "three/tsl";

// ------------------------
export const USBD_VID = 56987;
export const USBD_PID_FS = 56988;

export const READOUT = 0x2020;
export const STREAM_FULL_READOUTS = 0x2021;
export const GET_READOUTS_SNAPSHOT = 0x2022;
export const FULL_READOUT = 0x2023;

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


// Serial Port Management
// ----------------------

/* Get all ports that report as our motor driver. */
async function grab_ports(){
  const ports = await navigator.serial.getPorts();
  return ports.filter((port) => {
    const info = port.getInfo();
    return info.usbVendorId === USBD_VID && info.usbProductId === USBD_PID_FS;
  });
}

/* Prompt for a motor driver port if we can't get it automatically. */
async function maybe_prompt_port(){
  const ports = await grab_ports();
  if (ports.length == 1) return ports[0];
  return await navigator.serial.requestPort({filters: [{usbVendorId: USBD_VID, usbProductId: USBD_PID_FS}]});

}

/* Request a COM port and try to connect to the motor driver. */
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



// USB serial port communication
// -----------------------------

/* Convenience class to work with COM ports. */
export class COMPort {
  constructor(port){
    this.port = port;
    this.buffer = new Uint8Array();
    this.reader = port.readable.getReader();
    this.writer = port.writable.getWriter();
  }

  /* Read exactly n_bytes from the USB line. */
  async read(n_bytes){

    while (this.buffer.length < n_bytes){
      const {value, done} = await (this.reader.read());
      if (done) throw new Error("EOF");
      this.buffer = new Uint8Array([...this.buffer, ...value]);
    }
    const result = this.buffer.slice(0, n_bytes);
    this.buffer = this.buffer.slice(n_bytes);
    
    return new DataView(result.buffer);
  }

  /* Write to the USB line. */
  async write(buffer){
    await this.writer.write(buffer);
  }

  /* Close the COM port. */
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
    this.onmessage = () => {};
    this.onerror = () => {};
  }

  /* Stop receiving messages close the USB line. */
  async forget(){
    this.onerror(new Error("EOF"));

    try {
      await this.com_port.forget();
    } catch (error) {
      // Ignore network errors when forgetting, likely due to previous disconnect.
      if (error.name != "NetworkError" && error.name != "InvalidStateError") throw error;
    }
  }

  /* Start reading messages from the motor driver.
    * Only one reading loop should be active per controller instance.
    * Keep the generator running so it polls the USB line to receive messages.
    * The generator yields the number of bytes received.
  */
  async * reading_loop(){
    try {
      let bytes_received = 0;
      while(true){
        bytes_received += await this._receive_data();
        yield bytes_received;
      }
    } catch (error) {
      this.onerror(error);
    } finally {
      await this.forget();
    }
  }

  /* Send a command to the motor driver. */
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

  /* Wait for a single message from the motor driver; async alternative to onmessage. */
  async get_message({expected_code, response_timeout = 500}) {
    // We're going to hijack the onmessage and onerror handlers to wait for a message.

    // Chuck an error to the previous listener; it will be ignored if none exists.
    this.onerror(new Error("Overriding Request"));

    // Save the current handlers so we can restore them after the message.
    const onmessage = this.onmessage;
    const onerror = this.onerror;
    try {
      const message = await timeout_promise(new Promise((resolve, reject) => {
        this.onmessage = resolve;
        this.onerror = reject;
      }), response_timeout);
      
      if (message.code != expected_code) throw new Error(`Unexpected code: ${message.code} != ${expected_code}`);

      return message;
    } finally {
      this.onmessage = onmessage;
      this.onerror = onerror;
    }
  }

  /* Get a snapshot of the last N messages from the motor driver. */
  async get_readouts({expected_code = READOUT, expected_messages = HISTORY_SIZE, response_timeout = 500}) {
    let data = [];
    for (let i = 0; i < expected_messages; i++) {
      data.push(await this.get_message({expected_code, response_timeout}));
    }

    return data;
  }

  /* Stream messages from the motor driver. 

    This generator will stop if another message function is called; or if the
    driver takes too long to respond. The generator yields an array of messages.
  */
  async * stream_readouts({expected_code = READOUT, response_timeout = 500}) {
    let data = [];
    while (true) {
      try {
        data.push(await this.get_message({expected_code, response_timeout}));
      } catch (error) {
        if (error.message == "Timeout") return data;
        if (error.message == "EOF") return data;
        if (error.message == "Overriding Request") return data;
        throw error;
      }

      if (data.length % HISTORY_SIZE == 0) {
        yield data;
      }
    }

    return data;
  }


  /* Read data from the USB line. */
  async _receive_data(){

    const message_header = await this.com_port.read(2);
    const code = message_header.getUint16(0);
    const {parse_func, data_size} = parser_mapping[code] || {parse_func: null, data_size: 0};
    if (parse_func === null) {
      console.warn("Unknown message header: ", message_header);
      return 2;
    }

    const data_view = await this.com_port.read(data_size);
    const response = parse_func(data_view);

    this.onmessage(response);

    return data_size + 2;
  }
}


const readout_size = 16;

function parse_readout(data_view){
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


  return {
    code: READOUT,
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
  };
}

const full_readout_size = 16+8;

function parse_full_readout(data_view){
  const readout = parse_readout(data_view);
  let offset = readout_size;
  const tick_rate = data_view.getUint16(offset);
  offset += 2;
  const adc_update_rate = data_view.getUint16(offset);
  offset += 2;
  const hall_unobserved_rate = data_view.getUint16(offset);
  offset += 2;
  const hall_observed_rate = data_view.getUint16(offset);
  offset += 2;

  return {
    ...readout,
    code: FULL_READOUT,
    tick_rate,
    adc_update_rate,
    hall_unobserved_rate,
    hall_observed_rate,
  };
}

const parser_mapping = {
  [READOUT]: {parse_func: parse_readout, data_size: readout_size},
  [FULL_READOUT]: {parse_func: parse_full_readout, data_size: full_readout_size},
};


// Utility functions
// -----------------

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

  return Promise.race([promise, timed_reject]).finally(() => { clearTimeout(timeout_id); });
}