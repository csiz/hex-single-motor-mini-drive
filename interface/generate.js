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
  
  
  // Calculate message sizes (including all inherited fields)
  const calculate_message_size = (name, msg) => {
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
      const message_size = calculate_message_size(name, msg);
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

  console.log("Generated C++ interface:", cpp_output_path);
}