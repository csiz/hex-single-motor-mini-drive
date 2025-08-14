import {serial as serial_polyfill} from "web-serial-polyfill";

import {exponential_average} from "./math_utils.js";
import {parser_mapping, command_codes, serialise_command, header_size, default_command_options} from "./interface.js";

// Re-export the codes from the interface.
export { command_codes };

// USB serial port commands
// ------------------------

export const USBD_VID = 56987;
export const USBD_PID_FS = 56988;


// Serial Port Management
// ----------------------

const serial = navigator.serial ?? serial_polyfill;

/* Get all ports that report as our motor driver. */
async function grab_ports(){
  const ports = await serial.getPorts();
  return ports.filter((port) => {
    const info = port.getInfo();
    return info.usbVendorId === USBD_VID && info.usbProductId === USBD_PID_FS;
  });
}

/* Prompt for a motor driver port if we can't get it automatically. */
export async function maybe_prompt_port(){
  const ports = await grab_ports();
  if (ports.length == 1) return ports[0];
  return await serial.requestPort({filters: [{usbVendorId: USBD_VID, usbProductId: USBD_PID_FS}]});

}

/* Request a COM port and try to connect to the motor driver. */
export async function open_usb_com_port(port){
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
        await new Promise((resolve) => setTimeout(resolve, 100));
        continue;
      }
      throw error;
    }
  }

  if (tries <= 0) throw new Error("Port open failed; InvalidStateError after all retries.");

  return port;
}


// Motor driver control
// --------------------

export async function start_motor_controller_loop(options){
  const controller = new MotorController(options);
  await controller.start_reading_loop();
}

/* Motor driver controller using a javascript webapp. 

SerialPort works on Chrome, however Firefox needs an addon. Unfortunately it doesn't work on 
mobile, need to check how to turn this page into an app to access the USB line on mobile.

Parameters:
- `writable`: A `WriteableStream` that sends data to the motor driver.
- `readable`: A `ReadableStream` that receives data from the motor driver.
- `onstatus`: Callback function to handle status updates from the motor driver.
- `onmessage`: Callback function to handle incoming messages from the motor driver.
- `onerror`: Callback function to handle errors that occur during communication.
*/
export class MotorController {
  constructor({
    opened_port,
    onstatus = () => {},
    onmessage = () => {},
    onerror = () => {},
    onready = () => {},
    onclose = () => {},
  }){
    if (!opened_port) throw new Error("Invalid port provided");
    if (!opened_port.readable) throw new Error("Port unreadable");
    if (!opened_port.writable) throw new Error("Port unwritable");


    this.port = opened_port;
    this.writer = opened_port.writable.getWriter();
    this.reader = opened_port.readable.getReader();

    this.onmessage = onmessage;
    this.onerror = onerror;
    this.onstatus = onstatus;
    this.onready = onready;
    this.onclose = onclose;

    // Time duration for averaging the `receive_rate`.
    this.receive_rate_timescale = 0.5; // seconds
    // Minimum time period for receiving messages to update the `receive_rate`.
    this.receive_rate_min_period = 0.050; // seconds

    this.current_calibration = null;
    this.control_parameters = null;
    this.position_calibration = null;

    this.awaiting_reply = new Map();
    this.last_message = new Map();
  }


  /* Stop receiving messages close the USB line. */
  async forget(){
    try {
      if (this.reader) {
        await this.reader.cancel();
        this.reader = null;
      }
      if (this.writer) {
        await this.writer.abort();
        this.writer = null;
      }
      
      await this.port.readable.cancel();

      await this.port.close();

      await this.onclose();

    } catch (error) {
      // Ignore network errors when forgetting, likely due to previous disconnect.
      if (error.name != "NetworkError" && error.name != "InvalidStateError") {
        this.onerror(error);
        throw error;
      }
    }
  }

  /* Start the reading loop after loading configuration parameters from the driver. */
  async start_reading_loop(){
    try {
      // Start the reading loop and then the parameter requests in parallel.
      return await Promise.all([
        this.reading_loop(),
        (async () => {
          await this.load_current_calibration();
          await this.load_position_calibration();
          await this.load_control_parameters();
          console.info("Motor controller is ready.");
          this.onready(this);
        })(),
      ]);

    } catch (error) {
      // If we have an error, we need to forget the port and report the error.
      await this.forget();
      this.onerror(error);
    }
  }


  /* Start reading messages from the motor driver.
    * Only one reading loop should be active per controller instance.
    * Keep the generator running so it polls the USB line to receive messages.
    * The generator yields the number of bytes received.
  */
  async reading_loop() {
    let bytes_received = 0;

    let bytes_discarded = 0;

    let last_bytes_received = 0;
    let last_received_time = Date.now();

    let receive_rate = 0.0;

    let byte_array = new Uint8Array();

    while (true) {
      // Grab all buffered data from the driver serial line.
      const {value: byte_chunk, done} = await this.reader.read();

      // Release the locks when done.
      if (done) return await this.forget();
      
      // Count connection statistics.

      bytes_received += byte_chunk.length;
      last_bytes_received += byte_chunk.length;

      // Get the current time in milliseconds.
      const now = Date.now();

      // Cap the minimum time since last message to avoid division by zero.
      const time_since_last = (now - last_received_time) / 1000.0;

      // Compute connection speed in byte_chunks of a minimum time period.
      if(time_since_last > this.receive_rate_min_period) {

        // Update the rate only if we have received data since the last update.
        receive_rate = last_bytes_received == 0 ? receive_rate :
          exponential_average(last_bytes_received / time_since_last, receive_rate, time_since_last, this.receive_rate_timescale);

        last_bytes_received = 0;
        last_received_time = now;
      }

      // Feedback for connection status on every chunk of data received.
      this.onstatus({bytes_received, bytes_discarded, receive_rate});


      // Concatenate the new byte chunk to our message buffer array.
      byte_array = byte_array.length == 0 ? byte_chunk : new Uint8Array([...byte_array, ...byte_chunk]);

      // We start from the beginning at every chunk. We might receive multiple messages per chunk.
      let offset = 0;

      // Process all messages in the buffer as long as we received enough data for the header code.
      while (byte_array.length >= offset + header_size) {

        // The message code indicates the type of message and the expected size.
        const message_code = new DataView(byte_array.buffer, offset, header_size).getUint16(0);

        // Get the parser and expected size for the message; if we can't find it, we must've received a spurious code.
        const {parse_function, message_size} = parser_mapping[message_code] ?? {parse_function: null, message_size: null};

        // Check if we have a parsing function for the message code.
        if (parse_function === null) {
          // Unrecognised message code; we must have read the middle of a message. Iterate over each
          // byte until we find the expected code indicating the start of a valid message.

          offset += 1;
          bytes_discarded += 1;

          // Continue parsing from the next byte.
          continue;
        }

        // Wait for enough data to parse the message; break inner loop and await another read.
        if (byte_array.length < offset + message_size) break;

        // We have enough data to parse the message; parse it.
        const message = parse_function(
          new DataView(byte_array.buffer, offset, message_size), 
          // We compute running averages and current index relative to previous messages of the same type.
          this.last_message.get(message_code), 
          // Pass the controller instance for so we can read the calibration parameters.
          this,
        );

        // Advance the offset once we parsed the message.
        offset += message_size;

        // If we have a valid message, report it to the user.
        if(message) {
          // Remember for the next message so we can increment the message index.
          this.last_message.set(message_code, message);

          // Intercept awaited messages and resolve the reply promise.
          if (this.awaiting_reply.has(message_code)) {
            const {resolve, data, expected_messages} = this.awaiting_reply.get(message_code);
            data.push(message);
            if (data.length === expected_messages) {
              resolve(data);
            }
          } else {
            // Otherwise stream all replies through the `onmessage` callback.
            this.onmessage(message);
          }
        } else {
          // Something was wrong parsing the message; discard it.
          bytes_discarded += message_size;
          console.warn("Message was discarded due to invalid data:", message_code, byte_array.slice(offset, offset + message_size));
        }
      }
      
      // Slice our buffer once we have processed all the messages; we can have leftover data.
      if (offset) byte_array = byte_array.slice(offset);
    }
  }

  reset_history(){
    this.last_message.clear();
  }

  /* Send a command to the motor driver. */
  async send_command({command, ...command_options}) {
    if (!this.writer) return;

    const buffer = serialise_command({...default_command_options, command, ...command_options});

    await this.writer.write(buffer);
  }

  /* Send a command and capture the reply messages. */
  async send_command_and_await_reply({
    command, 
    expected_code = command_codes.READOUT, 
    expected_messages = 1, 
    response_timeout = 500,
    ...command_options
  }) {
    if (expected_messages <= 0) {
      throw new Error("Must expect positive number of messages.");
    }

    // Setup the reply promise and message code indexed reply queue.

    // It's a singleton queue, only process one reply at a time.
    if (this.awaiting_reply.has(expected_code)) {
      throw new Error(`Already awaiting reply for message code: ${expected_code}`);
    }

    // Reset the message history so we start from index 0.
    this.last_message.delete(expected_code);

    const { promise, resolve, reject } = Promise.withResolvers();

    const data = [];

    this.awaiting_reply.set(expected_code, {resolve, reject, data, expected_messages});

    // Reject after a short timeout if we don't get a reply.
    const timeout_id = setTimeout(() => {reject(new Error("Timeout"));}, response_timeout);
    
    // Send the command, return the data and do any cleanup necessary.
    try {
      // Wait for the command to be sent before waiting for the response.
      await this.send_command({command, ...command_options});
      
      // Wait for the reading loop to resolve the promise with the expected data.
      const data = await promise;
      
      // Return the promised data.
      return expected_messages === 1 ? data[0] : data;
    } finally {
      // Clear the timeout and remove the queued reply.
      clearTimeout(timeout_id);
      // Remove the reply from the queue.
      this.awaiting_reply.delete(expected_code);
    }
  }


  async load_current_calibration(){
    try {
      this.current_calibration = await this.send_command_and_await_reply({
        command: command_codes.GET_CURRENT_FACTORS, 
        expected_code: command_codes.CURRENT_FACTORS, 
        expected_messages: 1,
      });
    } catch (error) {
      // Log then ignore the error.
      console.error("Error loading current calibration:", error);
    }
  }

  async upload_current_calibration(current_calibration){
    if(!current_calibration) return;

    try {
      this.current_calibration = await this.send_command_and_await_reply({
        command: command_codes.SET_CURRENT_FACTORS, 
        additional_data: current_calibration,
        expected_code: command_codes.CURRENT_FACTORS, 
        expected_messages: 1,
      });
    } catch (error) {
      console.error("Error uploading current calibration:", error);
      throw error;
    }
  }

  async reset_current_calibration(){
    try {
      this.current_calibration = await this.send_command_and_await_reply({
        command: command_codes.RESET_CURRENT_FACTORS,
        expected_code: command_codes.CURRENT_FACTORS,
        expected_messages: 1,
      });
    } catch (error) {
      console.error("Error resetting current calibration:", error);
      throw error;
    }
  }


  async load_position_calibration(){
    try {
      this.position_calibration = await this.send_command_and_await_reply({
        command: command_codes.GET_HALL_POSITIONS,
        expected_code: command_codes.HALL_POSITIONS,
        expected_messages: 1,
      });
    } catch (error) {
      console.error("Error loading position calibration:", error);
    }
  }

  async upload_position_calibration(position_calibration){
    if(!position_calibration) return;

    try {
      this.position_calibration = await this.send_command_and_await_reply({
        command: command_codes.SET_HALL_POSITIONS,
        additional_data: position_calibration,
        expected_code: command_codes.HALL_POSITIONS,
        expected_messages: 1
      });
    } catch (error) {
      console.error("Error uploading position calibration:", error);
      throw error;
    }
  }

  async reset_position_calibration(){
    try {
      this.position_calibration = await this.send_command_and_await_reply({
        command: command_codes.RESET_HALL_POSITIONS,
        expected_code: command_codes.HALL_POSITIONS,
        expected_messages: 1
      });
    } catch (error) {
      console.error("Error resetting position calibration:", error);
      throw error;
    }
  }

  async load_control_parameters(){
    try {
      this.control_parameters = await this.send_command_and_await_reply({
        command: command_codes.GET_CONTROL_PARAMETERS,
        expected_code: command_codes.CONTROL_PARAMETERS,
        expected_messages: 1
      });
    } catch (error) {
      console.error("Error loading control parameters:", error);
    }
  }

  async upload_control_parameters(control_parameters){
    if(!control_parameters) return;

    try {
      this.control_parameters = await this.send_command_and_await_reply({
        command: command_codes.SET_CONTROL_PARAMETERS,
        additional_data: control_parameters,
        expected_code: command_codes.CONTROL_PARAMETERS,
        expected_messages: 1
      });
    } catch (error) {
      console.error("Error uploading control parameters:", error);
      throw error;
    }
  }

  async reset_control_parameters(){
    try {
      this.control_parameters = await this.send_command_and_await_reply({
        command: command_codes.RESET_CONTROL_PARAMETERS,
        expected_code: command_codes.CONTROL_PARAMETERS,
        expected_messages: 1
      });
    } catch (error) {
      console.error("Error resetting control parameters:", error);
      throw error;
    }
  }
}