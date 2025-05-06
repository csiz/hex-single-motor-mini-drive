import {PWM_BASE} from "./motor_driver_constants.js";

// USB serial port commands
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


// Other internal constants

const MAX_WRONG_CODE = 256;


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
      // I don't know what florControl does! Unfortunately open doesn't have a `highWaterMark` option
      // to would slow the internal USB reader when the webcode is slow. Thus data appears to keep 
      // coming after the driver stops sending it. SerialPort is just reading from the internal buffer.
      await port.open({baudRate: 115200, bufferSize: 4096, flowControl: "hardware"});
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

function exponential_average(new_value, old_value, time_since_last, alpha_time) {
  const alpha = Math.exp(-time_since_last / alpha_time);
  return new_value * (1 - alpha) + old_value * alpha;
}


// Motor control logic
// -------------------


export class MotorController {
  constructor(port){
    this._port = port;
    this._writer = port.writable.getWriter();
    this._reader = port.readable.getReader();
    this._promised_readouts = null;
    this._onmessage = null;
    this._onerror = null;

    this._expected_code = 0;

    this.receive_rate_timescale = 0.5; // seconds
    this.receive_rate_min_period = 0.050; // seconds
  }


  /* Stop receiving messages close the USB line. */
  async forget(){
    try {
      if (this._port.readable) {
        await this._reader.cancel();
        this._reader.releaseLock();
      }
      if (this._port.writable) {
        await this._writer.abort();
        this._writer.releaseLock();
      }
      await this._port.close();
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
  async reading_loop(status_callback = () => {}) {
    let bytes_received = 0;

    let wrong_code_count = 0;
    let bytes_discarded = 0;

    let last_bytes_received = 0;
    let last_received_time = Date.now();
    let receive_rate = 0.0;

    let byte_array = new Uint8Array();

    while (true) {
      const {value: chunk, done} = await this._reader.read();
      if (done) {
        if (this._promised_readouts != null) this._onerror(new Error("EOF"));
        break;
      }

      // If we're not expecting any messages, just ignore the data.
      if (this._promised_readouts == null || this._expected_code == 0) {
        bytes_discarded += chunk.length;
        byte_array = new Uint8Array();
        continue;
      }

      byte_array = byte_array.length == 0 ? chunk : new Uint8Array([...byte_array, ...chunk]);
      let offset = 0;

      while (byte_array.length >= offset + 2) {
        const message_header = new DataView(byte_array.buffer, offset, 2);

        const code = message_header.getUint16(0);
        if (code != this._expected_code) {
          // Search each byte until we find the expected code.
          offset += 1;
          wrong_code_count += 1;
          bytes_discarded += 1;
          if (wrong_code_count > MAX_WRONG_CODE) {
            this._onerror(new Error(`Could not find expected message code: ${this._expected_code}`));
            break;
          }
          continue;
        }

        const {parse_func, data_size} = parser_mapping[code] || {parse_func: null, data_size: 0};
        if (parse_func === null) {
          const error = new Error(`Unknown message code: ${code}`);
          console.error(error.message);
          this._onerror(error);
          throw error;
        }

        // Wait for enough data to parse the message.
        if (byte_array.length < offset + 2 + data_size) break;
        
        const response = parse_func(new DataView(byte_array.buffer, offset + 2, data_size));
        offset += data_size + 2;

        this._onmessage(response);
      }

      byte_array = byte_array.slice(offset);

      bytes_received += chunk.length;
      last_bytes_received += chunk.length;

      const now = Date.now();
      // Cap the minimum time since last message to avoid division by zero.
      const time_since_last = (now - last_received_time) / 1000.0;

      if(time_since_last < this.receive_rate_min_period) {

        // Wait for more data before updating the rate.
        status_callback({bytes_received, bytes_discarded, receive_rate});

      } else {
        // Update the rate only if we have received data since the last update. 
        receive_rate = last_bytes_received == 0 ? receive_rate :
          exponential_average(last_bytes_received / time_since_last, receive_rate, time_since_last, this.receive_rate_timescale);

        last_bytes_received = 0;
        last_received_time = now;

        status_callback({bytes_received, bytes_discarded, receive_rate});
      }
    }

    await this.forget();
  }

  /* Send a command to the motor driver. */
  async send_command({command, command_timeout, command_pwm, command_leading_angle = 0}) {
    if (!this._port.writable) return;
    
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

    await this._writer.write(buffer);
  }

  async cancel_previous_request() {

    if (this._promised_readouts != null) {
      // Chuck an error to the previous listener to resolve the 
      this._onerror(new Error("Overriding Request"));
      try {
        await this._promised_readouts;
      } catch (error) {
        if (error.message != "Overriding Request") throw error;
      }
    }
  }

  /* Get a snapshot of the last N messages from the motor driver. */
  async get_readouts({expected_code = READOUT, expected_messages = HISTORY_SIZE, response_timeout = 500}) {

    this.cancel_previous_request();

    let timeout_id;
    
    const data_promise = new Promise((resolve, reject) => {

      let data = [];

      this._expected_code = expected_code;

      this._onerror = reject;

      timeout_id = setTimeout(() => {
        reject(new Error("Timeout"));
      }, response_timeout);

      this._onmessage = (message) => {
        data.push(message);
        if (data.length >= expected_messages) {
          resolve(data);
          this._expected_code = 0;
        }
      };
    });

    this._promised_readouts = data_promise;
    
    try {
      return await data_promise;
    } finally {
      clearTimeout(timeout_id);
      this._promised_readouts = null;
      this._onmessage = null;
      this._onerror = null;
    }
  }

  /* Stream messages from the motor driver. 

    This generator will stop if another message function is called; or if the
    driver takes too long to respond. The generator yields an array of messages.
  */
  async stream_readouts({readout_callback, expected_code = READOUT, response_timeout = 500}) {
    this.cancel_previous_request();

    let timeout_id;
    
    const data_promise = new Promise((resolve, reject) => {

      this._expected_code = expected_code;

      this._onerror = reject;

      timeout_id = setTimeout(resolve, response_timeout);

      this._onmessage = (message) => {
        clearTimeout(timeout_id);
        timeout_id = setTimeout(resolve, response_timeout);
        readout_callback(message);
      };
    });

    this._promised_readouts = data_promise;
    
    try {
      return await data_promise;
    } catch (error) { 
      if (error.message != "Overriding Request") throw error;
    } finally {
      clearTimeout(timeout_id);
      this._promised_readouts = null;
      this._expected_code = 0;
      this._onmessage = null;
      this._onerror = null;
    }
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

const full_readout_size = 16+16;

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
  const temperature_readout = data_view.getUint16(offset);
  offset += 2;
  const vcc_readout = data_view.getUint16(offset);
  offset += 2;
  const cycle_start_tick = data_view.getInt16(offset);
  offset += 2;
  const cycle_end_tick = data_view.getInt16(offset);
  offset += 2;

  return {
    ...readout,
    code: FULL_READOUT,
    tick_rate,
    adc_update_rate,
    hall_unobserved_rate,
    hall_observed_rate,
    temperature_readout,
    vcc_readout,
    cycle_start_tick,
    cycle_end_tick,
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