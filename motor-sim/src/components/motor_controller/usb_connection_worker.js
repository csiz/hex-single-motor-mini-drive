// Serial Port Management
// ----------------------
// 
// We need a web worker to continuously read from the serial port without blocking
// for UI updates, otherwise the browser implementation of the SerialPort will
// silently drop data when the buffer fills up. Flow control doesn't propagate
// to the device so data just puffs away.


// USB COM Port
// 
// Open a SerialPort connection and continuously read data from it inside a web worker thread.
class USB_COM_Port {
  constructor() {
    this.port = null;
    this.reader = null;
    this.writer = null;
  }

  /* Request a COM port and try to connect to the motor driver. */
  async open_usb_com_port(port){
    if (!port) throw new Error("No port specified.");

    let tries_left = 10;

    while(tries_left-- > 0){
      try {
        // Note: SerialPort uses an internal buffer that has unreliable flow control.
        await port.open({baudRate: 115200, bufferSize: 4096, flowControl: "hardware"});

        if (!port.readable) throw new Error("Port unreadable");
        if (!port.writable) throw new Error("Port unwritable");

        this.port = port;
        this.writer = port.writable.getWriter();
        this.reader = port.readable.getReader();

        break;
      } catch (error) {
        if (error.name === "InvalidStateError") {
          await new Promise((resolve) => setTimeout(resolve, 100));
          continue;
        }
        throw error;
      }
    }

    if (tries_left <= 0) throw new Error("Port open failed; InvalidStateError after all retries.");
  }

  
  // Stop receiving messages close the USB line.
  async forget_port(){
    if (!this.port) return;

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

      this.port = null;

    } catch (error) {
      // Ignore network errors when forgetting, likely due to previous disconnect.
      if (error.name != "NetworkError" && error.name != "InvalidStateError") {
        // Rethrow other errors.
        throw error;
      }
    }
  }
}

// Each worker will manage a single COM port.
self.com_port = new USB_COM_Port();

// Handle messages from the main thread.
// 
// The worker handles these actions:
//  - "open": Open the specified COM port index (we cannot send the port over a worker message, we need to request it again inside the worker).
//  - "close": Close the current COM port.
//  - "send": Send a byte buffer to the motor driver.
// And in turns replies with these messages:
// - "opened": The COM port has been opened.
// - "closed": The COM port has been closed.
// - "received": A byte chunk has been received from the motor driver.
self.onmessage = async (e) => {
  const {type} = e.data;
  if (type === "open") {
    const available_ports = await navigator.serial.getPorts();
    
    const {port_index} = e.data;

    if (available_ports.length <= port_index) throw new Error(`No port at index ${port_index}.`);
      
    await self.com_port.open_usb_com_port(available_ports[port_index]);
    
    self.postMessage({type: "opened", port_index});

    while (true) {
      // Grab all buffered data from the driver serial line.
      const {value: byte_chunk, done} = await self.com_port.reader.read();

      // Release the locks when done.
      if (done) return await self.com_port.forget_port();

      if (byte_chunk && byte_chunk.length > 0) {
        self.postMessage({type: "received", byte_chunk}, {transfer: [byte_chunk.buffer]});
      }
    }

  } else if (type === "close") {

    await self.com_port.forget_port();
    self.postMessage({type: "closed"});

    self.close();

  } else if (type === "send") {

    if (self.com_port.writer) {
      const {buffer} = e.data;
      await self.com_port.writer.write(buffer);
    }
  }
};