import {exponential_average} from "./math_utils.js";
import {parser_mapping, make_position_calibration, make_current_calibration} from "./interface.js";

import {COBS_Buffer, MessageCode, serialise, deserialise} from "hex-mini-drive-interface";

// Re-export the codes from the interface.
export { MessageCode };


// Serial Port Management
// ----------------------

export async function maybe_prompt_port(){
  const ports = await navigator.serial.getPorts();
  if (ports.length == 1) return ports[0 ];
  return await navigator.serial.requestPort();
}

export async function get_port_index(port){
  const ports = await navigator.serial.getPorts();
  return ports.findIndex((p) => p === port);
}



// Motor driver control
// --------------------


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
    port_index,
    onstatus = () => {},
    onmessage = () => {},
    onerror = () => {},
    onready = () => {},
    onclose = () => {},
  }){
    this.connection_worker = new Worker(new URL("./usb_connection_worker.js", import.meta.url), {type: "module"});

    this.encoding_buffer = new COBS_Buffer();

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

    let bytes_received = 0;
    
    let bytes_discarded = 0;
    
    let last_bytes_received = 0;
    let last_received_time = Date.now();
    
    let receive_rate = 0.0;
    
    let receive_message = (message_data) => {

      let bare_message = deserialise(message_data);
      
      // If we have a valid message, report it to the user.
      if(!bare_message) {
        // Something was wrong parsing the message; discard it.
        bytes_discarded += message_data.length;
        console.warn("Message was discarded due to invalid data:", message_data);
        return;
      }
      
      const message_code = bare_message.message_code;
      
      // We compute running averages and current index relative to previous messages of the same type.
      const last_message = this.last_message.get(message_code);

      // Add additional data to readout messages.
      const message = parser_mapping[message_code] ? parser_mapping[message_code](bare_message, last_message, this) : bare_message;

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
    };



    this.connection_worker.addEventListener("message", async (event) => {
      const {type} = event.data;

      if (type === "opened") {
        await this.load_current_calibration();
        await this.load_position_calibration();
        await this.load_control_parameters();
        console.info("Motor controller is ready.");
        this.onready(this);
      }
      else if (type === "closed") this.onclose();
      else if (type === "received") {

        const {byte_chunk} = event.data;
        
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

        this.encoding_buffer.decode_chunk(byte_chunk, receive_message);
      }
    });

    this.connection_worker.addEventListener("error", (event) => {
      this.onerror(event.error);
    });

    this.connection_worker.postMessage({type: "open", port_index});

  }

  forget(){
    this.connection_worker.postMessage({type: "close"});
  }

  reset_history(){
    this.last_message.clear();
  }

  /* Send a command to the motor driver. */
  async send_command(message) {
    console.log("Sending message:", message);
    // Send messages or message codes.
    if (typeof message === "number") {
      message = {message_code: message};
    } else if (typeof message !== "object") {
      throw new Error("Message must be an object or a message code number.");
    } else if (!("message_code" in message)) {
      throw new Error("Message object must contain a message_code property.");
    }
    
    const message_data = serialise(message);

    const buffer = this.encoding_buffer.encode_message(message_data);

    this.connection_worker.postMessage({type: "send", buffer}, {transfer: [buffer.buffer]});
  }

  /* Send a command and capture the reply messages. */
  async send_command_and_await_reply({
    message,
    expected_code = MessageCode.Readout, 
    expected_messages = 1, 
    response_timeout = 500,
  }) {
    if (expected_messages <= 0) {
      throw new Error("Must expect positive number of messages.");
    }

    // Setup the reply promise and message code indexed reply queue.

    // Don't allow repeated messages with the same reply. User needs to await the first result or 
    // timeout before sending another command with the same expected reply code to avoid confusion.
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
      await this.send_command(message);
      
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
        message: MessageCode.GetCurrentCalibration,
        expected_code: MessageCode.CurrentCalibration, 
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
        message: {message_code: MessageCode.SetCurrentCalibration, current_calibration: make_current_calibration(current_calibration)},
        expected_code: MessageCode.CurrentCalibration,
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
        message: MessageCode.ResetCurrentCalibration,
        expected_code: MessageCode.CurrentCalibration,
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
        message: MessageCode.GetHallPositions,
        expected_code: MessageCode.HallPositions,
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
        message: {message_code: MessageCode.SetHallPositions, ...make_position_calibration(position_calibration)},
        expected_code: MessageCode.HallPositions,
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
        message: MessageCode.ResetHallPositions,
        expected_code: MessageCode.HallPositions,
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
        message: MessageCode.GetControlParameters,
        expected_code: MessageCode.ControlParameters,
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
        message: {message_code: MessageCode.SetControlParameters, ...control_parameters},
        expected_code: MessageCode.ControlParameters,
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
        message: MessageCode.ResetControlParameters,
        expected_code: MessageCode.ControlParameters,
        expected_messages: 1
      });
    } catch (error) {
      console.error("Error resetting control parameters:", error);
      throw error;
    }
  }
}