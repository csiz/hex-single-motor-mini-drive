import {exponential_average} from "./math_utils.js";
import {
  parse_readout,
  parse_full_readout,
  parse_unit_test_output,
  parse_current_calibration,
  make_current_calibration,
} from "./interface.js";

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

// Data management
// ---------------

// Maximum time between readouts to consider them part of the same series.
const readout_series_timeout = 500;

const max_data_length = 16384;

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

    this.current_calibration = {};
    this.control_parameters = {};

    this.data = [];

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
    bare_message.local_time = Date.now();

    // Reset data if it's been too long since the last message.
    if (this.data?.length && this.data[this.data.length - 1].local_time + readout_series_timeout < bare_message.local_time) {
      this.data = [];
    }

    // We compute running averages and current index relative to previous messages of the same type.
    const last_message = this.data?.length ? this.data[this.data.length - 1] : null;

    // Start with the bare message by default but we might add more info.
    let message = bare_message;
    
    // Intercept motor parameters and forward all other messages.
    if (message_code == MessageCode.CURRENT_CALIBRATION){
      message = parse_current_calibration(bare_message);
      this.current_calibration = message;
    } else if (message_code == MessageCode.CONTROL_PARAMETERS) {
      this.control_parameters = message;
    } else if (message_code == MessageCode.READOUT) {
      // Add the current_calibration data to the bare message and parse it all together. 
      // This way we can also track how the calibration parameters change over time. And if
      // transmission order is preserved, we should receive the new calibration data just ahead
      // of the corresponding readout.
      message = parse_readout({...this.current_calibration, ...bare_message}, last_message);
      // Store readout data up to some maximum size. Allows us to easily plot and analyse data samples.
      this.data = capped_push(this.data, message, max_data_length);
    } else if (message_code == MessageCode.FULL_READOUT){
      // TODO: now we can drop full readouts in favour of simpler readouts + concatenated calibration data
      message = parse_full_readout({...this.current_calibration,...bare_message}, last_message);
      this.data = capped_push(this.data, message, max_data_length);
    } else if (message_code == MessageCode.UNIT_TEST_OUTPUT) {
      message = parse_unit_test_output(bare_message);
    }

    // Stream all replies through the `onmessage` callback.
    this.onmessage(message);
  }

  forget(){
    this.connection_worker.postMessage({type: "close"});
  }

  reset_history(){
    this.data = [];
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







// TODO: We need to integrate more of the motor controller functionality
// ---------------------------------------------------------------------
// 








// Connection info formatting utils
// --------------------------------

// Format a floating number of bytes with the common endings KiB, MiB, GiB (products of 1024).
function format_bytes(bytes){
  if (bytes < 1024) return `${bytes.toFixed(2)} bytes`;
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(2)} KiB`;
  if (bytes < 1024 * 1024 * 1024) return `${(bytes / (1024 * 1024)).toFixed(2)} MiB`;
  return `${(bytes / (1024 * 1024 * 1024)).toFixed(2)} GiB`;
}

// Format the receiving data rate from the connection status.
function format_data_rate({bytes_received, bytes_discarded, receive_rate}){
  bytes_received = `received: ${format_bytes(bytes_received).padStart(12)}`;
  bytes_discarded = `discarded: ${format_bytes(bytes_discarded).padStart(12)}`;
  receive_rate = `download rate: ${format_bytes(receive_rate).padStart(12)}/s`;

  return `${bytes_received} | ${bytes_discarded} | ${receive_rate}`;
}

// Format the connection error message as an html element.
function displayable_connection_error(error){
  if (error.message === "EOF") {
    return `End of connection.`;
  } else if (error.name === "NotFoundError") {
    return `No device found or nothing selected.`;
  } else if (error.name === "SecurityError") {
    return `Permission for port dialog denied.`;
  } else if (error.name === "NetworkError") {
    return `Transmission error to usb device.`;
  } else {
    console.warn("Motor connection error:", error);
    return `Connection lost; unknown error: ${error}`;
  }
}


// Memory consumption capping
// --------------------------

// Push to array, but keep array size capped to a limit.
function capped_push(data, value, target_size){
  // Maximum amount of data readouts per motor driver to keep in memory, is
  // used so we don't resize the array with every new readout. When the max
  // cap is reached we resize to the target size, amortizing resize ops.
  const max_size = 2 * target_size;
  
  let new_data = (data.length > max_size) ? data.slice(-target_size) : data;

  new_data.push(value);

  return new_data;
}