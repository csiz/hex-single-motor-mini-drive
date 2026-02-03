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
import { writeFileSync, mkdirSync } from "fs";
import { dirname } from "path";


function assemble_literal(strings, ...values) {
  // Reassemble or process the template; or just use the string if
  // strings is not an array (normal function call).
  return Array.isArray(strings)
    ? strings.reduce((acc, str, i) => acc + str + (values[i] ?? ""), "")
    : strings;
}

class CodeWriter {
  constructor() {
    this.code = '';
    this.indent_level = 0;
  }
  write(strings, ...values) {
    const string = assemble_literal(strings, ...values);

    this.code += '  '.repeat(this.indent_level) + string + '\n';
  }
  comment(strings, ...values) {
    const string = assemble_literal(strings, ...values);
    const comment_lines = string.trim().split('\n');
    for (const line of comment_lines) {
      this.code += '  '.repeat(this.indent_level) + `// ${line}` + '\n';
    }
  }
  indent() {
    this.indent_level += 1;
  }
  dedent() {
    this.indent_level = Math.max(0, this.indent_level - 1);
  }
  get() {
    return this.code;
  }
}

const plain_types = {
  'uint8': {
    cpp: 'uint8_t',
    size: 1,
  },
  'uint16': {
    cpp: 'uint16_t',
    size: 2,
  },
  'uint32': {
    cpp: 'uint32_t',
    size: 4,
  },
  'int8': {
    cpp: 'int8_t',
    size: 1,
  },
  'int16': {
    cpp: 'int16_t',
    size: 2,
  },
  'int32': {
    cpp: 'int32_t',
    size: 4,
  },
  'float32': {
    cpp: 'float',
    size: 4,
  }
}

// Helper function to convert type to C++ type.
const to_cpp_type = (type_spec) => {
  // The type can be a plain string;
  if (typeof type_spec === 'string') {
    // Check if it's a valid plain type.
    if (plain_types[type_spec] === undefined) throw new Error(`Unknown type: ${type_spec}`);
    return plain_types[type_spec].cpp;
  // Or it's dictionary with the type field; in this case we include collection types.
  } else if (type_spec.type === 'array') {
    const item_type = to_cpp_type(type_spec.item);
    return `std::array<${item_type}, ${type_spec.len}>`;
  // Finally check if it's a plain type again.
  } else if (plain_types[type_spec.type] !== undefined) {
    return plain_types[type_spec.type].cpp;
  } else {
    throw new Error(`Unknown type spec: ${JSON.stringify(type_spec)}`);
  }
};


function generate_cpp_interface(spec) {
  let code = new CodeWriter();
  
  // Header guards and includes
  code.comment(spec.doc);
  code.write``;
  code.write`#pragma once`;
  code.write``;
  code.write`#include <cstddef>`;
  code.write`#include <cstdint>`;
  code.write`#include <array>`;
  code.write``;
  
  // Generate constants
  code.write`// Constants`;
  code.write`// ---------`;
  code.write``;

  for (const [name, value] of Object.entries(spec.Constants)) {
    if (typeof value === 'object' && value.type && value.value) {
      code.write`constexpr ${value.type}_t ${name} = ${value.value};`;
    } else {
      code.write`constexpr auto ${name} = ${value};`;
    }
  }
  code.write``;
  
  // Generate message codes enum
  code.write`// Message Codes`;
  code.write`enum MessageCode : uint16_t {`;
  code.indent();

  const message_codes = Object.entries(spec.MessageCodes);
  for (let i = 0; i < message_codes.length; i++) {
    const [name, msg] = message_codes[i];
    const comma = i < message_codes.length - 1 ? ',' : '';
    code.write`${name} = ${msg.code}${comma}`;
  }
  code.dedent();
  code.write`};`;
  code.write``;
  
  
  // Generate structs for messages with struct field
  code.write`// Message Structures`;
  code.write`// ------------------`;
  code.write``;

  for (const [name, msg] of Object.entries(spec.MessageCodes)) {
    if (msg.struct) {
      // Add message doc if exists
      if (msg.doc) code.comment(msg.doc);
      
      // Define the struct, including a single extends as the parent class.
      code.write`struct ${name}${msg.extends ? ` : ${msg.extends}` : ''} {`;
      code.write``;
      code.indent();
      
      // Add message code field
      code.write`static constexpr uint16_t message_code = ${msg.code};`;
      code.write``;
      
      // Add fields from this struct
      for (const [field_name, field] of Object.entries(msg.struct)) {
        if (field.doc) code.comment(field.doc);
        code.write`${to_cpp_type(field)} ${field_name};`;
        code.write``;
      }
      
      code.dedent();
      code.write`};`;
      code.write``;
      code.write``;
    }
  }
  
  return code.get();
}


// Run this script if executed directly, to generate interface files.
if (import.meta.main) {
  const interface_spec = yaml.load(readFileSync("./interface.yaml", "utf8"));


  console.log("Loaded interface specification:");
  
  const cpp_interface = generate_cpp_interface(interface_spec)
  
  // Write the generated C++ interface.
  const cpp_output_path = "./dist/hex_mini_drive/interface.hpp";
  mkdirSync(dirname(cpp_output_path), { recursive: true });
  writeFileSync(cpp_output_path, cpp_interface);

  console.log("Generated C++ interface:", cpp_output_path);
}