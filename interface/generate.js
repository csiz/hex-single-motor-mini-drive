// Interface Generator
// -------------------
// 
// We will autogenerate interface files for each language based on the spec defined
// in `interface.yaml`.
// 
// At the top level of the spec config are:
//   - Constants: A dictionary of named constant values.
//   - MessageCodes: A dictionary of named message codes.
// 
// 
// MessageCodes is a dictionary of messages that can be sent or received 
// over a generic serial wire. Messages start with a uint16 to indentify 
// the message `code`. The message spec also contains the following fields:
// - code: The uint16 code identifying the message type.
// - struct: An optional dictionary defining other data fields in the message.
// - extends: This message appends more data to the end of another message type, extending it.
// - returns: Some commands expect a response from the device, the spec encodes
// the expected response message type; or a dictionary with `stream` or `array` typed follow ups.
// - doc: Documentation for the message type.
//
// 
// The struct field is itself a dictionary of fields, the keys are the named
// fields of the message data structure. On the wire each message will have its
// fields written one after the other in binary format according to the type.
// Each field has the properties:
// - type: The data type of the field.
// - doc: Documentation for the field.
// 
// 
// The field types are:
// - uint8, uint16, uint32: Unsigned integers of 8, 16 or 32 bits.
// - int8, int16, int32: Signed integers of 8, 16 or 32 bits.
// - float32: 32 bit floating point values.
// - array: An array of items of the same type. The field must also specify
//   `len` (the number of items) and `item` (the type of each item).
// 
// Constants contains a dictionary of named constant values and their value
// or if a dictionary, the typed valued. With the fields `type` and `value`.

import { readFileSync } from "fs";
import yaml from "js-yaml";

// Run this script if executed directly, to generate interface files.
if (import.meta.main) {
  const interface_spec = yaml.load(readFileSync("./interface.yaml", "utf8"));


  console.log("Loaded interface specification:");
  
  const cpp_interface = generate_cpp_interface(interface_spec)

  console.log("Generated C++ interface:");
  console.log(cpp_interface);
}


function generate_cpp_interface(spec) {
  
}