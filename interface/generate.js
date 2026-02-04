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
    js: 'Uint8',
    size: 1,
  },
  'uint16': {
    cpp: 'uint16_t',
    js: 'Uint16',
    size: 2,
  },
  'uint32': {
    cpp: 'uint32_t',
    js: 'Uint32',
    size: 4,
  },
  'int8': {
    cpp: 'int8_t',
    js: 'Int8',
    size: 1,
  },
  'int16': {
    cpp: 'int16_t',
    js: 'Int16',
    size: 2,
  },
  'int32': {
    cpp: 'int32_t',
    js: 'Int32',
    size: 4,
  },
  'float32': {
    cpp: 'float',
    js: 'Float32',
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

// Helper function to calculate the size of a type in bytes.
const get_type_size = (type_spec) => {
  // The type can be a plain string.
  if (typeof type_spec === 'string') {
    if (plain_types[type_spec] === undefined) throw new Error(`Unknown type: ${type_spec}`);
    return plain_types[type_spec].size;
  // Or it's a dictionary with the type field; in this case we include collection types.
  } else if (type_spec.type === 'array') {
    const item_size = get_type_size(type_spec.item);
    return item_size * type_spec.len;
  // Finally check if it's a plain type again.
  } else if (plain_types[type_spec.type] !== undefined) {
    return plain_types[type_spec.type].size;
  } else {
    throw new Error(`Unknown type spec: ${JSON.stringify(type_spec)}`);
  }
};

// Calculate message sizes (including all inherited fields)
const calculate_message_size = (spec, msg) => {
  let size = 2; // 2 bytes for message code
  
  // Add size from extended message
  if (msg.extends) {
    const base_msg = spec.MessageCodes[msg.extends];
    if (base_msg && base_msg.struct) {
      for (const field of Object.values(base_msg.struct)) {
        size += get_type_size(field);
      }
    }
  }
  
  // Add size from this message's fields
  if (msg.struct) {
    for (const field of Object.values(msg.struct)) {
      size += get_type_size(field);
    }
  }
  
  return size;
};

// Helper function to convert PascalCase to snake_case.
const to_snake_case = (str) => {
  return str.replace(/[A-Z]/g, (letter, index) => {
    return index === 0 ? letter.toLowerCase() : '_' + letter.toLowerCase();
  });
};



function generate_cpp_interface(spec) {
  let code = new CodeWriter();
  
  // Header guards and includes
  code.comment(spec.doc);
  code.write``;
  
  // Autogenerated note.
  code.comment`Note, this file is autogenerated. Use 'npm run build' to regenerate from the 'interface.yaml' spec.`;
  code.write``;

  code.write`#pragma once`;
  code.write``;
  code.write`#include <cstddef>`;
  code.write`#include <cstdint>`;
  code.write`#include <array>`;
  code.write``;
  code.write`#include "hex_mini_drive/byte_handling.hpp"`;
  code.write``;

  code.write`namespace hex_mini_drive {`;
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
      
      // Add message size field
      const message_size = calculate_message_size(spec, msg);
      code.write`static constexpr size_t message_size = ${message_size};`;
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

  // Need a struct of the MessageCode and the union of all message structs.
  code.write`// Generic Message Structure`;
  code.write`// -------------------------`;
  code.write``;
  code.write`struct Message {`;
  code.indent();
  code.write`MessageCode message_code;`;
  code.write``;
  code.write`union {`;
  code.indent();
  for (const [name, msg] of Object.entries(spec.MessageCodes)) {
    if (msg.struct) {
      code.write`${name} ${to_snake_case(name)};`;
    }
  }
  code.dedent();
  code.write`};`;
  code.dedent();
  code.write`};`;
  code.write``;
  
  // Helper to generate field serialization code
  const generate_field_serialization = (code, field_name, field, offset_var) => {
    if (typeof field === 'string' || (field.type && plain_types[field.type])) {
      // Plain type
      const type_name = typeof field === 'string' ? field : field.type;
      code.write`write_${type_name}(buffer + ${offset_var}, ${field_name});`;
      code.write`${offset_var} += ${get_type_size(field)};`;
    } else if (field.type === 'array') {
      // Array type
      const item_type = typeof field.item === 'string' ? field.item : field.item.type;
      const item_size = get_type_size(field.item);
      code.write`for (size_t i = 0; i < ${field.len}; ++i) {`;
      code.indent();
      
      if (typeof field.item === 'string' || (field.item.type && plain_types[field.item.type])) {
        // Array of plain types
        code.write`write_${item_type}(buffer + ${offset_var}, ${field_name}[i]);`;
        code.write`${offset_var} += ${item_size};`;
      } else if (field.item.type === 'array') {
        // Nested array
        const nested_item_type = typeof field.item.item === 'string' ? field.item.item : field.item.item.type;
        const nested_item_size = get_type_size(field.item.item);
        code.write`for (size_t j = 0; j < ${field.item.len}; ++j) {`;
        code.indent();
        code.write`write_${nested_item_type}(buffer + ${offset_var}, ${field_name}[i][j]);`;
        code.write`${offset_var} += ${nested_item_size};`;
        code.dedent();
        code.write`}`;
      }
      code.dedent();
      code.write`}`;
    }
  };
  
  // Helper to generate field deserialization code
  const generate_field_deserialization = (code, field_name, field, offset_var, struct_var) => {
    if (typeof field === 'string' || (field.type && plain_types[field.type])) {
      // Plain type
      const type_name = typeof field === 'string' ? field : field.type;
      code.write`${struct_var}.${field_name} = read_${type_name}(buffer + ${offset_var});`;
      code.write`${offset_var} += ${get_type_size(field)};`;
    } else if (field.type === 'array') {
      // Array type
      const item_type = typeof field.item === 'string' ? field.item : field.item.type;
      const item_size = get_type_size(field.item);
      code.write`for (size_t i = 0; i < ${field.len}; ++i) {`;
      code.indent();
      
      if (typeof field.item === 'string' || (field.item.type && plain_types[field.item.type])) {
        // Array of plain types
        code.write`${struct_var}.${field_name}[i] = read_${item_type}(buffer + ${offset_var});`;
        code.write`${offset_var} += ${item_size};`;
      } else if (field.item.type === 'array') {
        // Nested array
        const nested_item_type = typeof field.item.item === 'string' ? field.item.item : field.item.item.type;
        const nested_item_size = get_type_size(field.item.item);
        code.write`for (size_t j = 0; j < ${field.item.len}; ++j) {`;
        code.indent();
        code.write`${struct_var}.${field_name}[i][j] = read_${nested_item_type}(buffer + ${offset_var});`;
        code.write`${offset_var} += ${nested_item_size};`;
        code.dedent();
        code.write`}`;
      }
      code.dedent();
      code.write`}`;
    }
  };
  
  // Generate serialize and deserialize functions
  code.write`// Serialization Functions`;
  code.write`// -----------------------`;
  code.write``;
  
  for (const [name, msg] of Object.entries(spec.MessageCodes)) {
    if (msg.struct) {
      // Serialize function
      code.write`static inline bool serialise(${name} const& value, uint8_t * buffer) {`;
      code.indent();
      code.write`if (buffer == nullptr) return false;`;
      code.write``;
      code.write`size_t offset = 0;`;
      code.write``;
      code.comment`Write message code`;
      code.write`write_uint16(buffer + offset, ${name}::message_code);`;
      code.write`offset += 2;`;
      code.write``;
      
      // Serialize fields from base message if extends
      if (msg.extends) {
        const base_msg = spec.MessageCodes[msg.extends];
        if (base_msg && base_msg.struct) {
          code.comment`Fields from ${msg.extends}`;
          for (const [field_name, field] of Object.entries(base_msg.struct)) {
            generate_field_serialization(code, 'value.' + field_name, field, 'offset');
          }
          code.write``;
        }
      }
      
      // Serialize fields from this message
      for (const [field_name, field] of Object.entries(msg.struct)) {
        generate_field_serialization(code, 'value.' + field_name, field, 'offset');
      }
      code.write`return true;`;
      code.dedent();
      code.write`}`;
      code.write``;
      
      // Deserialize function
      code.write`static inline bool deserialise(${name} & result, uint8_t const* buffer, size_t length) {`;
      code.indent();
      code.comment`Validate length`;
      code.write`if (length != ${name}::message_size) {`;
      code.indent();
      code.write`return false;`;
      code.dedent();
      code.write`}`;
      code.write``;
      code.comment`Validate message code`;
      code.write`uint16_t code = read_uint16(buffer);`;
      code.write`if (code != ${name}::message_code) {`;
      code.indent();
      code.write`return false;`;
      code.dedent();
      code.write`}`;
      code.write``;
      code.write`size_t offset = 2;`;
      code.write``;
      
      // Deserialize fields from base message if extends
      if (msg.extends) {
        const base_msg = spec.MessageCodes[msg.extends];
        if (base_msg && base_msg.struct) {
          code.comment`Fields from ${msg.extends}`;
          for (const [field_name, field] of Object.entries(base_msg.struct)) {
            generate_field_deserialization(code, field_name, field, 'offset', 'result');
          }
          code.write``;
        }
      }
      
      // Deserialize fields from this message
      for (const [field_name, field] of Object.entries(msg.struct)) {
        generate_field_deserialization(code, field_name, field, 'offset', 'result');
      }
      code.write`return true;`;
      code.dedent();
      code.write`}`;
      code.write``;
    }
  }

  // And now the generic serialize/deserialize functions for Message.
  code.write`// Generic Serialization Functions`;
  code.write`// -------------------------------`;
  code.write``;

  // Need a helper function to write a bare message code.
  code.write`static inline bool write_code(uint8_t * (*reserve_buffer)(size_t length), MessageCode code) {`;
  code.indent();
  code.write`uint8_t * buffer = reserve_buffer(2);`;
  code.write`if (buffer == nullptr) return false;`;
  code.write`write_uint16(buffer, static_cast<uint16_t>(code));`;
  code.write`return true;`;
  code.dedent();
  code.write`}`;
  code.write``;

  code.write`static inline bool serialise(Message const& message, uint8_t * (*reserve_buffer)(size_t length)) {`;
  code.indent();
  code.write`switch (message.message_code) {`;
  code.indent();
  for (const [name, msg] of Object.entries(spec.MessageCodes)) {
    if (msg.struct) {
      code.write`case MessageCode::${name}: {`;
      code.indent();
      code.write`return serialise(message.${to_snake_case(name)}, reserve_buffer(${name}::message_size));`;
      code.dedent();
      code.write`}`;
    } else {
      code.write`case MessageCode::${name}: return write_code(reserve_buffer, MessageCode::${name});`;
    }
  }
  code.dedent();
  code.write`}`;
  code.dedent();
  code.write`}`;
  code.write``;

  code.write`static inline bool deserialise(uint8_t const* buffer, size_t length, bool (*handle_message)(Message const& message)) {`;
  code.indent();
  code.write`if (length < 2) return false; // Need at least message code`;
  code.write`Message message;`;
  code.write`message.message_code = static_cast<MessageCode>(read_uint16(buffer));`;
  code.write``;
  code.write`switch (message.message_code) {`;
  code.indent();
  for (const [name, msg] of Object.entries(spec.MessageCodes)) {
    if (msg.struct) {
      code.write`case MessageCode::${name}: {`;
      code.indent();
      code.write`if (not deserialise(message.${to_snake_case(name)}, buffer, length)) return false;`;
      code.write`return handle_message(message);`;
      code.dedent();
      code.write`}`;
    } else {
      code.write`case MessageCode::${name}: return handle_message(message);`;
    }
  }
  code.dedent();
  code.write`}`;
  code.dedent();
  code.write`}`;
  code.write``;

  code.write`} // end namespace hex_mini_drive`;
  
  return code.get();
}

function generate_js_interface(spec) {
  let code = new CodeWriter();
  
  // Header comment
  code.comment(spec.doc);
  code.write``;

  // Autogenerated note.
  code.comment`Note, this file is autogenerated. Use 'npm run build' to regenerate from the 'interface.yaml' spec.`;
  code.write``;

  code.write`export { COBS_Buffer } from './cobs_encoding.js';`;
  code.write``;

  // Constants
  code.write`// Constants`;
  for (const [name, value] of Object.entries(spec.Constants)) {
    if (typeof value === 'object' && value.type && value.value) {
      code.write`export const ${name} = ${value.value};`;
    } else {
      code.write`export const ${name} = ${value};`;
    }
  }
  code.write``;

  // Message codes
  code.write`// Message Codes`;
  code.write`export const MessageCode = {`;
  code.indent();
  for (const [name, msg] of Object.entries(spec.MessageCodes)) {
    code.write`${name}: ${msg.code},`;
  }
  code.dedent();
  code.write`};`;
  code.write``;
  
  // Function to write just the code for simple messages.
  code.write`// Helper function to write just the message code`;
  code.write`function write_code(message_code) {`;
  code.indent();
  code.write`const buffer = new Uint8Array(2);`;
  code.write`const view = new DataView(buffer.buffer);`;
  code.write`view.setUint16(0, message_code);`;
  code.write`return buffer;`;
  code.dedent();
  code.write`}`;
  code.write``;
  
  // Map type names to DataView methods
  const get_dataview_method = (type_name) => {
    return plain_types[type_name].js;
  };

  // Helper to generate field serialization for JS
  const generate_js_field_serialization = (code, field_name, field, offset_var, value_var) => {
    if (typeof field === 'string' || (field.type && plain_types[field.type])) {
      const type_name = typeof field === 'string' ? field : field.type;
      const method = get_dataview_method(type_name);
      code.write`view.set${method}(${offset_var}, ${value_var}.${field_name});`;
      code.write`${offset_var} += ${get_type_size(field)};`;
    } else if (field.type === 'array') {
      const item_type = typeof field.item === 'string' ? field.item : field.item.type;
      const item_size = get_type_size(field.item);
      code.write`for (let i = 0; i < ${field.len}; i++) {`;
      code.indent();
      
      if (typeof field.item === 'string' || (field.item.type && plain_types[field.item.type])) {
        const method = get_dataview_method(item_type);
        code.write`view.set${method}(${offset_var}, ${value_var}.${field_name}[i]);`;
        code.write`${offset_var} += ${item_size};`;
      } else if (field.item.type === 'array') {
        const nested_item_type = typeof field.item.item === 'string' ? field.item.item : field.item.item.type;
        const nested_item_size = get_type_size(field.item.item);
        const method = get_dataview_method(nested_item_type);
        code.write`for (let j = 0; j < ${field.item.len}; j++) {`;
        code.indent();
        code.write`view.set${method}(${offset_var}, ${value_var}.${field_name}[i][j]);`;
        code.write`${offset_var} += ${nested_item_size};`;
        code.dedent();
        code.write`}`;
      }
      code.dedent();
      code.write`}`;
    }
  };

  // Helper to generate field deserialization for JS
  const generate_js_field_deserialization = (code, field_name, field, offset_var, result_var) => {
    if (typeof field === 'string' || (field.type && plain_types[field.type])) {
      const type_name = typeof field === 'string' ? field : field.type;
      const method = get_dataview_method(type_name);
      code.write`${result_var}.${field_name} = view.get${method}(${offset_var});`;
      code.write`${offset_var} += ${get_type_size(field)};`;
    } else if (field.type === 'array') {
      const item_type = typeof field.item === 'string' ? field.item : field.item.type;
      const item_size = get_type_size(field.item);
      code.write`${result_var}.${field_name} = [];`;
      code.write`for (let i = 0; i < ${field.len}; i++) {`;
      code.indent();
      
      if (typeof field.item === 'string' || (field.item.type && plain_types[field.item.type])) {
        const method = get_dataview_method(item_type);
        code.write`${result_var}.${field_name}[i] = view.get${method}(${offset_var});`;
        code.write`${offset_var} += ${item_size};`;
      } else if (field.item.type === 'array') {
        const nested_item_type = typeof field.item.item === 'string' ? field.item.item : field.item.item.type;
        const nested_item_size = get_type_size(field.item.item);
        const method = get_dataview_method(nested_item_type);
        code.write`${result_var}.${field_name}[i] = [];`;
        code.write`for (let j = 0; j < ${field.item.len}; j++) {`;
        code.indent();
        code.write`${result_var}.${field_name}[i][j] = view.get${method}(${offset_var});`;
        code.write`${offset_var} += ${nested_item_size};`;
        code.dedent();
        code.write`}`;
      }
      code.dedent();
      code.write`}`;
    }
  };

  // Serialize function
  code.write`// Serialize a message object to a Uint8Array`;
  code.write`export function serialise(message) {`;
  code.indent();
  code.write`if (!message || typeof message.message_code !== 'number') {`;
  code.indent();
  code.write`return null;`;
  code.dedent();
  code.write`}`;
  code.write``;
  code.write`switch (message.message_code) {`;
  code.indent();
  
  for (const [name, msg] of Object.entries(spec.MessageCodes)) {
    if (msg.struct) {
      const message_size = calculate_message_size(spec, msg);
      code.write`case MessageCode.${name}: {`;
      code.indent();
      code.write`const buffer = new Uint8Array(${message_size});`;
      code.write`const view = new DataView(buffer.buffer);`;
      code.write`let offset = 0;`;
      code.write`view.setUint16(offset, message.message_code);`;
      code.write`offset += 2;`;
      
      // Serialize base fields if extends
      if (msg.extends) {
        const base_msg = spec.MessageCodes[msg.extends];
        if (base_msg && base_msg.struct) {
          for (const [field_name, field] of Object.entries(base_msg.struct)) {
            generate_js_field_serialization(code, field_name, field, 'offset', 'message');
          }
        }
      }
      
      // Serialize this message's fields
      for (const [field_name, field] of Object.entries(msg.struct)) {
        generate_js_field_serialization(code, field_name, field, 'offset', 'message');
      }
      
      code.write`return buffer;`;
      code.dedent();
      code.write`}`;
    } else {
      // Message with no struct, just code
      code.write`case MessageCode.${name}: return write_code(MessageCode.${name});`;
    }
  }
  
  code.write`default:`;
  code.indent();
  code.write`return null;`;
  code.dedent();
  code.dedent();
  code.write`}`;
  code.dedent();
  code.write`}`;
  code.write``;

  // Deserialize function
  code.write`// Deserialize a Uint8Array to a message object`;
  code.write`export function deserialise(buffer) {`;
  code.indent();
  code.write`if (!buffer || buffer.length < 2) {`;
  code.indent();
  code.write`return null;`;
  code.dedent();
  code.write`}`;
  code.write``;
  code.write`view = new DataView(buffer.buffer, buffer.byteOffset, buffer.byteLength);`;
  code.write`const message_code = view.getUint16(0);`;
  code.write``;
  code.write`switch (message_code) {`;
  code.indent();
  
  for (const [name, msg] of Object.entries(spec.MessageCodes)) {
    if (msg.struct) {
      const message_size = calculate_message_size(spec, msg);
      code.write`case MessageCode.${name}: {`;
      code.indent();
      code.write`if (buffer.length !== ${message_size}) return null;`;
      code.write`const result = { message_code };`;
      code.write`let offset = 2;`;
      
      // Deserialize base fields if extends
      if (msg.extends) {
        const base_msg = spec.MessageCodes[msg.extends];
        if (base_msg && base_msg.struct) {
          for (const [field_name, field] of Object.entries(base_msg.struct)) {
            generate_js_field_deserialization(code, field_name, field, 'offset', 'result');
          }
        }
      }
      
      // Deserialize this message's fields
      for (const [field_name, field] of Object.entries(msg.struct)) {
        generate_js_field_deserialization(code, field_name, field, 'offset', 'result');
      }
      
      code.write`return result;`;
      code.dedent();
      code.write`}`;
    } else {
      // Message with no struct
      code.write`case MessageCode.${name}:`;
      code.indent();
      code.write`return buffer.length === 2 ? { message_code } : null;`;
      code.dedent();
    }
  }
  
  code.write`default:`;
  code.indent();
  code.write`return null;`;
  code.dedent();
  code.dedent();
  code.write`}`;
  code.dedent();
  code.write`}`;

  return code.get();
}


// Run this script if executed directly, to generate interface files.
if (import.meta.main) {
  const interface_spec = yaml.load(readFileSync("./interface.yaml", "utf8"));

  console.log("Loaded interface specification:");

  const output_dir = "./dist/hex_mini_drive/";
  mkdirSync(output_dir, { recursive: true });
  
  const cpp_interface = generate_cpp_interface(interface_spec)
  
  // Write the generated C++ interface.
  const cpp_output_path = `${output_dir}/interface.hpp`;
  writeFileSync(cpp_output_path, cpp_interface);

  // Also copy the helping header files:
  writeFileSync(`${output_dir}/byte_handling.hpp`, readFileSync("./byte_handling.hpp"));
  writeFileSync(`${output_dir}/cobs_encoding.hpp`, readFileSync("./cobs_encoding.hpp"));

  console.log("Generated C++ interface:", cpp_output_path);
  
  // Generate JavaScript interface
  const js_interface = generate_js_interface(interface_spec);
  const js_output_path = `${output_dir}/interface.js`;
  writeFileSync(js_output_path, js_interface);

  // Also copy the helping JS files:
  writeFileSync(`${output_dir}/cobs_encoding.js`, readFileSync("./cobs_encoding.js"));
  
  console.log("Generated JavaScript interface:", js_output_path);
}