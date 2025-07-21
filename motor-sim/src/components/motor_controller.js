import {serial as serial_polyfill} from "web-serial-polyfill";

import {wait} from "./async_utils.js";
import {exponential_average} from "./math_utils.js";
import {parser_mapping, command_codes, serialise_command, header_size, default_command_options} from "./motor_interface.js";
import {default_current_calibration, history_size} from "./motor_constants.js";

export { command_codes };

// USB serial port commands
// ------------------------

export const USBD_VID = 56987;
export const USBD_PID_FS = 56988;


// Other internal constants

const max_wrong_code = history_size + 256;

const serial = navigator.serial ?? serial_polyfill;

// Serial Port Management
// ----------------------

/* Get all ports that report as our motor driver. */
async function grab_ports(){
  const ports = await serial.getPorts();
  return ports.filter((port) => {
    const info = port.getInfo();
    return info.usbVendorId === USBD_VID && info.usbProductId === USBD_PID_FS;
  });
}

/* Prompt for a motor driver port if we can't get it automatically. */
async function maybe_prompt_port(){
  const ports = await grab_ports();
  if (ports.length == 1) return ports[0];
  return await serial.requestPort({filters: [{usbVendorId: USBD_VID, usbProductId: USBD_PID_FS}]});

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
    this._last_message = null;

    this._expected_code = 0;

    this.receive_rate_timescale = 0.5; // seconds
    this.receive_rate_min_period = 0.050; // seconds

    this.current_calibration = default_current_calibration;
    this.control_parameters = {};
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

    this._last_message = null;
    let last_message_time = Date.now();

    let byte_array = new Uint8Array();

    while (true) {
      const {value: chunk, done} = await this._reader.read();
      if (done) {
        if (this._promised_readouts != null) this._onerror(new Error("EOF"));
        break;
      }

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

      // If we're not expecting any messages, just ignore the data.
      if (this._promised_readouts == null || this._expected_code == 0) {
        bytes_discarded += chunk.length;
        byte_array = new Uint8Array();
        this._last_message = null;
        last_message_time = Date.now();

        console.debug("Ignoring data received while not expecting messages:", chunk.length, "bytes");
        continue;
      }

      byte_array = byte_array.length == 0 ? chunk : new Uint8Array([...byte_array, ...chunk]);
      let offset = 0;

      while (byte_array.length >= offset + header_size) {
        const message_header = new DataView(byte_array.buffer, offset, header_size);

        const code = message_header.getUint16(0);

        const {parse_func, message_size} = parser_mapping[code] ?? {parse_func: null, message_size: null};

        // Handle an unexpected message code.
        if (code != this._expected_code) {
          
          
          if (message_size != null) {
            // Check if it's a valid message code different than the expected; quietly discard the message.
            
            offset += message_size;
            bytes_discarded += message_size;
            console.debug("Ignoring unexpected message code:", code, " expected:", this._expected_code);

          } else {
            // Unrecognised message code; we must have read the middle of a message. Iterate over each
            // byte until we find the expected code indicating the start of our expected message.

            offset += 1;
            bytes_discarded += 1;
          }

          wrong_code_count += 1;

          if (wrong_code_count > max_wrong_code) {
            this._onerror(new Error(`Could not find expected message code: 0x${this._expected_code.toString(16)}`));
            break;
          }
          continue;
        }

        // We found the expected message code; reset the wrong code count.
        wrong_code_count = 0;

        // Ensure we have a parsing function for the message code.
        if (!parse_func) {
          // We might reach this point if the user requested an unknown code; and we somehow received it.
          const error = new Error(`Unknown message code: ${code}`);
          
          console.error(error.message);
          this._onerror(error);
          throw error;
        }

        // Wait for enough data to parse the message; break inner loop and await another read.
        if (byte_array.length < offset + message_size) break;

        // We have enough data to parse the message; parse it.
        const message = parse_func.call(this, new DataView(byte_array.buffer, offset, message_size), this._last_message);

        // Advance the offset once we parsed the message.
        offset += message_size;

        // Keep track of receive rate statistics.
        last_message_time = Date.now();

        if(message) {
          // If we have a valid message, report it to the listener.
          this._last_message = message;
          this._onmessage(message);
        } else {
          // Something was wrong with the message; discard it.
          bytes_discarded += message_size;
          console.warn("Message was discarded due to invalid data:", code, byte_array.slice(offset, offset + message_size));
        }
      }
      
      // Slice our buffer once we have processed all the messages.
      if (offset) byte_array = byte_array.slice(offset);
    }

    await this.forget();
  }

  /* Send a command to the motor driver. */
  async send_command({command, ...command_options}) {
    if (!this._port.writable) return;
    const buffer = serialise_command({...default_command_options,command, ...command_options});
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

  reset_request() {
    this._expected_code = 0;
    this._last_message = null;
    this._promised_readouts = null;
    this._onmessage = null;
    this._onerror = null;
  }

  /* Get a snapshot of the last N messages from the motor driver. */
  async command_and_read(command_options, {expected_code = command_codes.READOUT, expected_messages = HISTORY_SIZE, response_timeout = 500}) {

    await this.cancel_previous_request();

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

    // Wait for the command to be sent before waiting for the response.
    await this.send_command(command_options);
    
    try {
      return await data_promise;
    } finally {
      clearTimeout(timeout_id);
      this.reset_request();
    }
  }

  /* Stream messages from the motor driver. 

    This generator will stop if another message function is called; or if the
    driver takes too long to respond. The generator yields an array of messages.
  */
  async command_and_stream(command_options, {readout_callback, expected_code = command_codes.READOUT, response_timeout = 500}) {
    await this.cancel_previous_request();

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
    
    // Wait for the command to be sent before waiting for the response.
    await this.send_command(command_options);

    try {
      return await data_promise;
    } catch (error) { 
      if (error.message != "Overriding Request") throw error;
    } finally {
      clearTimeout(timeout_id);
      this.reset_request();
    }
  }

  async load_current_calibration(){
    try {
      const data = await this.command_and_read(
        {command: command_codes.GET_CURRENT_FACTORS}, 
        {expected_code: command_codes.CURRENT_FACTORS, expected_messages: 1},
      );
      if (data.length != 1) throw new Error("Invalid current calibration data");
      this.current_calibration = data[0];
    } catch (error) {
      console.error("Error loading current calibration:", error);
    }
  }

  async load_pid_parameters(){
    try {
      const data = await this.command_and_read(
        {command: command_codes.GET_PID_PARAMETERS}, 
        {expected_code: command_codes.PID_PARAMETERS, expected_messages: 1},
      );
      if (data.length != 1) throw new Error("Invalid PID parameters data");
      this.pid_parameters = data[0];
    } catch (error) {
      console.error("Error loading PID parameters:", error);
    }
  }

  async load_control_parameters(){
    try {
      const data = await this.command_and_read(
        {command: command_codes.GET_CONTROL_PARAMETERS}, 
        {expected_code: command_codes.CONTROL_PARAMETERS, expected_messages: 1},
      );
      if (data.length != 1) throw new Error("Invalid observer parameters data");
      this.control_parameters = data[0];
    } catch (error) {
      console.error("Error loading observer parameters:", error);
    }
  }

  async upload_current_calibration(current_calibration){
    try {
      const data = await this.command_and_read(
        {command: command_codes.SET_CURRENT_FACTORS, additional_data: current_calibration}, 
        {expected_code: command_codes.CURRENT_FACTORS, expected_messages: 1},
      );
      if (data.length != 1) throw new Error("Invalid current calibration data");
      this.current_calibration = data[0];
    } catch (error) {
      console.error("Error uploading current calibration:", error);
    }
  }

  async upload_pid_parameters(pid_parameters){
    try {
      const data = await this.command_and_read(
        {command: command_codes.SET_PID_PARAMETERS, additional_data: pid_parameters}, 
        {expected_code: command_codes.PID_PARAMETERS, expected_messages: 1},
      );
      if (data.length != 1) throw new Error("Invalid PID parameters data");
      this.pid_parameters = data[0];
    } catch (error) {
      console.error("Error uploading PID parameters:", error);
    }
  }

  async upload_control_parameters(control_parameters){
    try {
      const data = await this.command_and_read(
        {command: command_codes.SET_CONTROL_PARAMETERS, additional_data: control_parameters}, 
        {expected_code: command_codes.CONTROL_PARAMETERS, expected_messages: 1},
      );
      if (data.length != 1) throw new Error("Invalid control parameters data");
      this.control_parameters = data[0];
    } catch (error) {
      console.error("Error uploading control parameters:", error);
    }
  }

  async reset_control_parameters(){
    try {
      const data = await this.command_and_read(
        {command: command_codes.RESET_CONTROL_PARAMETERS}, 
        {expected_code: command_codes.CONTROL_PARAMETERS, expected_messages: 1},
      );
      if (data.length != 1) throw new Error("Invalid control parameters data");
      this.control_parameters = data[0];
    } catch (error) {
      console.error("Error resetting control parameters:", error);
    }
  }
}

