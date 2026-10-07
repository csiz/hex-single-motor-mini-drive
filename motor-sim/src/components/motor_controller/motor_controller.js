import {exponential_average} from "./math_utils.js";
import {parser_mapping, make_current_calibration} from "./interface.js";

import {ConsistentOverheadByteStuffing, MessageCode, write_message, read_message, MAX_MESSAGE_SIZE} from "hex-mini-drive-interface";

// Re-export the codes from the interface.
export { MessageCode };


// Serial Port Management
// ----------------------

export async function prompt_com_port_get_index(){
  let ports = await navigator.serial.getPorts();
  const port = (ports.length == 1) ? ports[0] : await navigator.serial.requestPort();
  // Get ports again.
  ports = await navigator.serial.getPorts();
  
  if (ports.length === 0) return null;

  const port_index = ports.findIndex((p) => p === port);

  if (port_index === -1) return null;
  
  return port_index;
}

// Websocket
// ---------

export const default_ws_uri = "ws://hex-mini-drive.local/ws";


// Motor driver control
// --------------------


/* Motor driver controller using a javascript webapp. 

SerialPort works on Chrome, however Firefox needs an addon. Unfortunately it doesn't work on 
mobile, need to check how to turn this page into an app to access the USB line on mobile.

Parameters:
- `motor_uri`: The URI of the motor to connect to.
- `onstatus`: Callback function to handle status updates from the motor driver.
- `onmessage`: Callback function to handle incoming messages from the motor driver.
- `onerror`: Callback function to handle errors that occur during communication.
*/
export class MotorController {
  constructor({
    motor_uri,
    onstatus = () => {},
    onmessage = () => {},
    onerror = () => {},
    onready = () => {},
    onclose = () => {},
  }){
    this.motor_uri = motor_uri;

    this.current_calibration = null;
    this.control_parameters = null;

    this.last_message = new Map();

    this.onmessage = onmessage;
    this.onerror = onerror;
    this.onstatus = onstatus;
    this.onready = onready;
    this.onclose = onclose;

    // Time duration for averaging the `receive_rate`.
    this.receive_rate_timescale = 0.5; // seconds
    // Minimum time period for receiving messages to update the `receive_rate`.
    this.receive_rate_min_period = 0.050; // seconds

    this.bytes_received = 0;
    this.bytes_discarded = 0;
    this.last_bytes_received = 0;
    this.last_received_time = Date.now();
    this.receive_rate = 0.0;

    this.encoding_buffer = new ConsistentOverheadByteStuffing({max_message_size: MAX_MESSAGE_SIZE});

    // Start the motor connection
    // --------------------------

    this.connection_type = motor_uri.split(":")[0];

    if (this.connection_type === "usb") {
      const port_index = parseInt(motor_uri.split(":")[1]);
      this.start_usb_connection(port_index);
    } else if (this.connection_type === "ws") {
      this.start_websocket_connection(motor_uri);
    } else {
      throw new Error(`Unsupported connection type: ${this.connection_type}`);
    }
  }

  when_opened(){
    this.load_current_calibration();
    this.load_control_parameters();
    this.onready(this);
    console.info(`Motor controller is ready: ${this.motor_uri}`);
  }

  when_closed(){
    this.onclose(this);
    console.info(`Motor controller is closed: ${this.motor_uri}`);
  }

  start_usb_connection(port_index){
    this.connection_worker = new Worker(new URL("./usb_connection_worker.js", import.meta.url), {type: "module"});

    this.connection_worker.addEventListener("message", (event) => {
      const {type} = event.data;

      if (type === "opened") this.when_opened();
      else if (type === "closed") this.when_closed();
      else if (type === "received") {

        const {byte_chunk} = event.data;
        
        this.count_data_received(byte_chunk.length);

        this.encoding_buffer.decode_chunk(byte_chunk, this.receive_message.bind(this));
      }
    });

    this.connection_worker.addEventListener("error", (event) => {
      this.onerror(event.error);
    });

    this.connection_worker.postMessage({type: "open", port_index});
  }

  start_websocket_connection(uri){
    // Setup a websocket connection to `ws://hex-mini-drive.local/ws` and send a message, log any replies.

    this.ws = new WebSocket(uri);
    this.ws.binaryType = "arraybuffer";

    this.ws.addEventListener('open', (event) => this.when_opened() );

    this.ws.addEventListener('message', (event) => {
      const message_data = new Uint8Array(event.data);
      this.count_data_received(message_data.length);
      this.encoding_buffer.decode_chunk(message_data, this.receive_message.bind(this));
    });

    this.ws.addEventListener('close', (event) => this.when_closed());

    this.ws.addEventListener('error', (event) => this.onerror(event.error));
  }


  count_data_received(length){
    // Count connection statistics.

    this.bytes_received += length;
    this.last_bytes_received += length;

    // Get the current time in milliseconds.
    const now = Date.now();

    // Cap the minimum time since last message to avoid division by zero.
    const time_since_last = (now - this.last_received_time) / 1000.0;

    // Compute connection speed in byte_chunks of a minimum time period.
    if(time_since_last > this.receive_rate_min_period) {

      // Update the rate only if we have received data since the last update.
      this.receive_rate = this.last_bytes_received == 0 ? this.receive_rate :
        exponential_average(this.last_bytes_received / time_since_last, this.receive_rate, time_since_last, this.receive_rate_timescale);

      this.last_bytes_received = 0;
      this.last_received_time = now;
    }

    // Feedback for connection status on every chunk of data received.
    this.onstatus({bytes_received: this.bytes_received, bytes_discarded: this.bytes_discarded, receive_rate: this.receive_rate});
  }

  receive_message(message_data) {
    let bare_message = read_message(message_data);
    
    // If we have a valid message, report it to the user.
    if(!bare_message) {
      // Something was wrong parsing the message; discard it.
      this.bytes_discarded += message_data.length;
      console.warn("Message was discarded due to invalid data:", message_data);
      return;
    }
    
    const message_code = bare_message.message_code;
    
    // We compute running averages and current index relative to previous messages of the same type.
    const last_message = this.last_message.get(message_code);

    // Add additional data to readout messages.
    const message = parser_mapping[message_code] ? 
      parser_mapping[message_code](bare_message, last_message, this) : 
      // Otherwise just return the bare message.
      bare_message;

    // Remember for the next message so we can increment the message index.
    this.last_message.set(message_code, message);

    // Intercept motor parameters and forward all other messages.
    if (message_code == MessageCode.CURRENT_CALIBRATION){
      this.current_calibration = message;
    } else if (message_code == MessageCode.CONTROL_PARAMETERS) {
      this.control_parameters = message;
    } else {
      // Stream all replies through the `onmessage` callback.
      this.onmessage(message);
    }
  }

  forget(){
    this.connection_worker.postMessage({type: "close"});
  }

  reset_history(){
    this.last_message.clear();
  }

  /* Send a command to the motor driver. */
  send_command(message) {
    console.debug("Sending message:", message);
    // Send messages or message codes.
    if (typeof message === "number") {
      message = {message_code: message};
    } else if (typeof message !== "object") {
      throw new Error("Message must be an object or a message code number.");
    } else if (!("message_code" in message)) {
      throw new Error("Message object must contain a message_code property.");
    }
    
    const message_data = write_message(message);
    this.encoding_buffer.encode_reset();
    const buffer = this.encoding_buffer.encode_message(message_data);

    if (this.connection_type === "ws") {
      this.ws.send(buffer);
    } else if (this.connection_type === "usb") {
      this.connection_worker.postMessage({type: "send", buffer}, {transfer: [buffer.buffer]});
    } else {
      throw new Error(`Unsupported connection type: ${this.connection_type}`);
    }
  }


  load_current_calibration(){
    this.send_command(MessageCode.GET_CURRENT_CALIBRATION);
  }

  upload_current_calibration(current_calibration){
    if(!current_calibration) return;

    this.send_command({
      message_code: MessageCode.SET_CURRENT_CALIBRATION,
      ...make_current_calibration(current_calibration),
    });
  }

  reset_current_calibration(){
    this.send_command(MessageCode.RESET_CURRENT_CALIBRATION);
  }

  load_control_parameters(){
    this.send_command(MessageCode.GET_CONTROL_PARAMETERS);
  }

  upload_control_parameters(control_parameters){
    if(!control_parameters) return;

    this.send_command({
      message_code: MessageCode.SET_CONTROL_PARAMETERS,
      ...control_parameters,
    });
  }

  reset_control_parameters(){
    this.send_command(MessageCode.RESET_CONTROL_PARAMETERS);
  }
}
