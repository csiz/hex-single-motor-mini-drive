// Interface Generator
// -------------------
// 
// We will autogenerate interface files for each language based on the spec defined
// in `interface.yaml`.
// 
// At the top level of the spec config are:
//   - doc: Documentation overview of the interface.
//   - definitions: A dictionary of constants and named type definitions that can be used in messages.
//   - message_codes: A dictionary of named message codes.
// 
// 
// `message_codes` is a dictionary of messages that can be sent or received 
// over a generic serial wire. Messages start with a uint16 to indentify 
// the `message_code` followed by the message data. Each message contains the following fields:
// - message_code: The uint16 code identifying the message type.
// - type: The name of the type definition for this message; this determined how the data is 
// serialized and deserialized. When type is missing then it's a bare message containing just the
// 2 byte identifier, and no other data.
// - returns: Some commands expect a response from the device, the spec encodes
// the expected response message type; or a dictionary with `array` typed follow ups.
//
// `definitions` is an ordered dictionary of named types and constants. Type definitions 
// contain a `struct` and/or `struct_base` to extend a previous type. Constants on are defined as
// a `const` with a `type`. The definition spec also contains the following fields:
// - doc: Documentation for the message type.
// - struct: An optional dictionary defining other data fields in the message.
// - struct_base: This message appends more data to the end of another message type, extending it.
// - type: The type of the constant.
// - const: The value of the constant.
// - array_len: For array types, the length of the array; can be a number or a reference to a defined constant.
// 
// The struct field is itself a dictionary of fields, the keys are the named
// fields of the message data structure. On the wire each message will have its
// fields written one after the other in binary format according to the type.
// Each field has the properties:
// - type: The data type of the field.
// - doc: Documentation for the field.
// 
// The field types are:
// - uint8, uint16, uint32: Unsigned integers of 8, 16 or 32 bits.
// - int8, int16, int32: Signed integers of 8, 16 or 32 bits.
// - float32: 32 bit floating point values.
// 

// TODO: set max_message_size in the config so we can populate both js and cpp interfaces with it.


import { readFileSync } from "fs";
import yaml from "js-yaml";
import { writeFileSync, mkdirSync } from "fs";
import { parse } from "path";
import { assert } from "console";

import { 
  CodeWriter, 
  ConstantSpec,
  TypeSpec,
  BasicType,
  ArraySpec,
  StructSpec,
  FieldSpec,
  MessageSpec,
  Spec, 
} from "./spec_types.js";

import { basic_types } from "./basic_types.js";

// Helper function to convert PascalCase to snake_case. Also treat numbers as separate words, Set6Sector->set_6_sector.
const to_snake_case = (str) => {
  return str.replace(/([a-zA-Z])([0-9A-Z])/g, '$1_$2').replace(/([0-9])([a-zA-Z])/g, '$1_$2').replace(/([A-Z])([A-Z])/g, '$1_$2').toLowerCase();
};


function parse_type(type_spec, definitions) {
  if (typeof type_spec !== 'string') throw new Error(`Invalid type spec: ${JSON.stringify(type_spec)}: must be a string referring to a basic type or a defined type.`);

  if (basic_types[type_spec]) {
    return basic_types[type_spec];
  } else if (definitions[type_spec]) {
    const definition = definitions[type_spec];
    if (!(definition instanceof TypeSpec)) {
      throw new Error(`Invalid type definition for ${type_spec}: must have either struct, struct_base, or array_len.`);
    }
    return definition;
  } else {
    throw new Error(`Unknown type: ${type_spec}`);
  }
}

// Parse definitions in order, all referenced definitions must have been defined earlier in the file.
// 
// Also pre-calculate the size of each type.
function parse_definitions(definitions_spec, definitions = {}) {
  // Copy the definitions object.
  definitions = {...definitions};

  for (const [name, def] of Object.entries(definitions_spec)) {


    // Or we define a type using a struct and/or struct_base field.
    if (def.struct || def.struct_base) {
      const struct = Object.fromEntries(Object.entries(def.struct || {}).map(([field_name, field_def]) => {
        const type = parse_type(field_def.type, definitions);
        return [field_name, new FieldSpec({
          name: field_name,
          type,
          doc: field_def.doc || '',
          size: type.size,
        })];
      }));

      const struct_base = def.struct_base ? parse_type(def.struct_base, definitions) : undefined;
      
      const size = Object.values(struct || {}).reduce((acc, field) => acc + field.size, 0) + (struct_base ? struct_base.size : 0);

      definitions[name] = new StructSpec({
        name,
        doc: def.doc || '',
        struct,
        struct_base,
        size,
      }); 

    // Define an array type.
    } else if (def.array_len && def.type) {
      let array_len = def.array_len;
      if (typeof array_len === 'string') {
        if (definitions[array_len] && Number.isFinite(definitions[array_len].const)) {
          array_len = definitions[array_len].const;
        } else {
          throw new Error(`Invalid array_len for ${name}: ${def.array_len} is not a defined constant.`);
        }
      } else if (typeof array_len !== 'number') throw new Error(`Invalid array_len for ${name}: must be a number or a defined constant.`);

      const type = parse_type(def.type, definitions);
      const size = type.size * array_len;
      definitions[name] = new ArraySpec({
        name,
        doc: def.doc || '',
        array_len,
        type,
        size,
      });

    // Finally it must be a const.
    } else if (def.const !== undefined && def.type) {
      const type = parse_type(def.type, definitions);
      definitions[name] = new ConstantSpec({
        name,
        doc: def.doc || '',
        const: def.const,
        type,
        size: type.size,
      });
    } else {
      throw new Error(`Invalid definition for ${name}: must have either const (for constants), struct/struct_base (for types), or array_type/array_len (for arrays).`);
    }
  }

  return definitions;
}


function parse_message_codes(message_codes_spec, definitions) {
  return Object.fromEntries(Object.entries(message_codes_spec).map(([name, msg]) => {
    const message_code = msg.message_code;

    if (message_code === undefined || typeof message_code !== 'number') {
      throw new Error(`Message ${name} is missing required field 'message_code'.`);
    }

    const type = msg.type ? parse_type(msg.type, definitions) : undefined;

    const size = 2 + (type !== undefined ? type.size : 0);

    return [name, new MessageSpec({
      name,
      message_code,
      type,
      size,
      doc: msg.doc || '',
    })];
  }));
}

// Load spec file, verify dependency correctness, calculate message sizes, and return a typified Spec object.
function load_spec(file_path){
  const loaded_spec = yaml.load(readFileSync(file_path, "utf8"));

  const doc = loaded_spec.doc || '';
  const definitions = parse_definitions(loaded_spec.definitions || {}, {});
  const message_codes = parse_message_codes(loaded_spec.message_codes || {}, definitions);

  return new Spec({
    doc,
    definitions,
    message_codes,
  });
}

function cpp_typename(type){
  if (type instanceof BasicType) {
    return type.cpp_type;
  } else if (type instanceof StructSpec || type instanceof ArraySpec) {
    return type.name;
  } else {
    throw new Error(`Unknown type: ${type}`);
  }
}

function write_cpp_definition(code, def) {
  if (def instanceof ConstantSpec) {
    code.write`constexpr ${cpp_typename(def.type)} ${def.name} = ${def.const};`;
  } else if (def instanceof StructSpec) {
    if (def.doc) code.comment(def.doc);
    code.write`struct ${def.name}${def.struct_base ? ` : ${cpp_typename(def.struct_base)}` : ''} {`;
    code.indent();
    for (const [field_name, field] of Object.entries(def.struct || {})) {
      if (field.doc) code.comment(field.doc);
      code.write`${cpp_typename(field.type)} ${field_name};`;
    }
    code.dedent();
    code.write`};`;
  } else if (def instanceof ArraySpec) {
    if (def.doc) code.comment(def.doc);
    code.write`using ${def.name} = std::array<${cpp_typename(def.type)}, ${def.array_len}>;`;
  }
  code.write``;
}

function serialize_cpp_field(buf, offset, name, type) {
  if (type instanceof BasicType || type instanceof StructSpec || type instanceof ArraySpec) {
    return `write_${type.name}(${buf} + ${offset}, ${name});`;
  } else {
    throw new Error(`Unknown type for serialization: ${type}`);
  }
}

function deserialize_cpp_field(buf, offset, type) {
  if (type instanceof BasicType || type instanceof StructSpec || type instanceof ArraySpec) {
    return `read_${type.name}(${buf} + ${offset})`;
  } else {
    throw new Error(`Unknown type for deserialization: ${type}`);
  }
}

function write_cpp_serialization(code, def) {
  if (def instanceof StructSpec) {
    code.write`static inline void write_${def.name}(uint8_t * buffer, ${def.name} const& value) {`;
    code.indent();

    code.write`size_t offset = 0;`;

    // Serialize base fields if struct_base
    if (def.struct_base) {
      code.write`${serialize_cpp_field(`buffer`, `offset`, `value`, def.struct_base)};`;
      code.write`offset += ${def.struct_base.size};`;
    }

    // Now serialize own fields.
    for (const field of Object.values(def.struct || {})) {
      code.write`${serialize_cpp_field(`buffer`, `offset`, `value.${field.name}`, field.type)};`;
      code.write`offset += ${field.type.size};`;
    }
    code.dedent();
    code.write`}`;

  } else if (def instanceof ArraySpec) {
    code.write`static inline void write_${def.name}(uint8_t * buffer, ${def.name} const& value) {`;
    code.indent();
    code.write`size_t offset = 0;`;
    code.write`for (size_t i = 0; i < ${def.array_len}; ++i) {`;
    code.indent();
    code.write`${serialize_cpp_field(`buffer`, `offset`, `value[i]`, def.type)};`;
    code.write`offset += ${def.type.size};`;
    code.dedent();
    code.write`}`;
    code.dedent();
    code.write`}`;
    code.write``;
  } else if (def instanceof ConstantSpec) {
    // Nothing to serialize for constants.
  } else {
    throw new Error(`Unknown definition type: ${def}`);
  }
}

function write_cpp_deserialization(code, def) {
  if (def instanceof StructSpec) {
    code.write`static inline ${def.name} read_${def.name}(uint8_t const* buffer) {`;
    code.indent();
    code.write`size_t offset = 0;`;
    code.write``;
    // Deserialize base fields if struct_base
    if (def.struct_base) {
      code.write`${def.name} result {${deserialize_cpp_field(`buffer`, `offset`, def.struct_base)}};`;
      code.write`offset += ${def.struct_base.size};`;
    } else {
      code.write`${def.name} result;`;
    }

    code.write``;
    // Now deserialize own fields.
    for (const field of Object.values(def.struct || {})) {
      code.write`result.${field.name} = ${deserialize_cpp_field(`buffer`, `offset`, field.type)};`;
      code.write`offset += ${field.type.size};`;
    }
    code.write`return result;`;
    code.dedent();
    code.write`}`;
  } else if (def instanceof ArraySpec) {
    code.write`static inline ${def.name} read_${def.name}(uint8_t const* buffer) {`;
    code.indent();
    code.write`${def.name} result;`;
    code.write`size_t offset = 0;`;
    code.write`for (size_t i = 0; i < ${def.array_len}; ++i) {`;
    code.indent();
    code.write`result[i] = ${deserialize_cpp_field(`buffer`, `offset`, def.type)};`;
    code.write`offset += ${def.type.size};`;
    code.dedent();
    code.write`}`;
    code.write`return result;`;
    code.dedent();
    code.write`}`;
    code.write``;
  } else if (def instanceof ConstantSpec) {
    // Nothing to deserialize for constants.
  } else {
    throw new Error(`Unknown definition type: ${def}`);
  }
}

// Create the generic message and serialization functions for the messaging protocol.
function write_cpp_message_protocol(code, message_codes) {

  code.comment`Message Codes`;
  code.write`enum MessageCode : uint16_t {`;
  code.indent();

  for (const message of Object.values(message_codes)) {
    code.write`${message.name} = ${message.message_code},`;
  }
  code.dedent();
  code.write`};`;
  code.write``;


  // Get all unique types referenced by messages.
  let message_types = new Set();
  for (const message of Object.values(message_codes)) {
    if (message.type) message_types.add(message.type);
  }
  message_types = [...message_types];


  code.comment`Generic Message Structure`;
  code.write`struct Message {`;
  code.indent();
  code.write`MessageCode message_code = static_cast<MessageCode>(0);`;
  code.write``;
  code.write`std::variant<`;
  code.indent();
  for (const message_type of message_types) {
    code.write`${message_type.name},`;
  }
  code.write`std::monostate`;
  code.dedent();
  code.write`> message_data;`;
  code.dedent();
  code.write`};`;
  code.write``;

  // Helper for message sizes.
  code.write`constexpr size_t message_size(MessageCode code) {`;
  code.indent();
  code.write`switch (code) {`;
  code.indent();
  for (const message of Object.values(message_codes)) {
    code.write`case MessageCode::${message.name}: return ${message.size};`;
  }
  code.dedent();
  code.write`}`;
  code.write`return 0; // Unknown message code`;
  code.dedent();
  code.write`}`;
  code.write``;
  

  // Lastly we need to write the generic serialize and deserialize functions for Message.
  code.comment`Generic Serialization Functions`;
  code.write`static inline size_t write_message(uint8_t * buffer, const size_t max_size, Message const& message) {`;
  code.indent();
  code.write`if (max_size < 2) return 0;`;
  code.write`switch (message.message_code) {`;
  code.indent();
  for (const message of Object.values(message_codes)) {
    code.write`case MessageCode::${message.name}: {`;
    code.indent();
    code.write`write_uint16(buffer, static_cast<uint16_t>(MessageCode::${message.name}));`;
    if (message.type) {
      code.write`if (max_size < 2 + ${message.type.size}) return 0;`;
      code.write`write_${message.type.name}(buffer + 2, std::get<${message.type.name}>(message.message_data));`;
    }
    code.write`return ${message.type ? 2 + message.type.size : 2};`;
    code.dedent();
    code.write`}`;
  }
  code.dedent();
  code.write`}`;
  code.write`return 0;`;
  code.dedent();
  code.write`}`;
  code.write``;

  code.comment`Generic Deserialization Function`;
  code.write`static inline bool read_message(Message & message, uint8_t const* buffer, size_t size) {`;
  code.indent();
  code.comment`Need at least 2 bytes for the message code.`;
  code.write`if (size < 2) return false;`;
  code.write``;
  code.write`message.message_code = static_cast<MessageCode>(read_uint16(buffer));`;
  code.write``;
  code.write`switch (message.message_code) {`;
  code.indent();
  for (const message of Object.values(message_codes)) {
    code.write`case MessageCode::${message.name}: {`;
    code.indent();
    if (message.type) {
      code.write`if (size != 2 + ${message.type.size}) return false;`;
      code.write`message.message_data = read_${message.type.name}(buffer + 2);`;
    } else {
      code.write`if (size != 2) return false;`;
      code.write`message.message_data = std::monostate{};`;
    }
    code.write`return true;`;
    code.dedent();
    code.write`}`;
  }
  code.dedent();
  code.write`}`;
  code.write``;
  code.comment`Unknown message code`;
  code.write`return false;`;
  code.dedent();
  code.write`}`;
  code.write``;
}


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
  code.write`#include <variant>`;
  code.write``;
  code.write`namespace hex_mini_drive {`;
  code.write``;

  // First write out the basic type serialization functions, since the generated code relies on them.
  code.comment`Basic Type Serialization Functions`;
  code.comment`----------------------------------`;
  code.write``;
  for (const basic_type of Object.values(basic_types)) {
    basic_type.write_cpp_serialization(code);
    code.write``;
    basic_type.write_cpp_deserialization(code);
    code.write``;
  }

  // Write out the definitions as C++ constants, structs and array defs.
  code.comment`Constants and Definitions`;
  code.comment`-------------------------`;
  code.write``;
  for (const def of Object.values(spec.definitions)) {
    write_cpp_definition(code, def);
    write_cpp_serialization(code, def);
    write_cpp_deserialization(code, def);
  }

  write_cpp_message_protocol(code, spec.message_codes);

  code.write`} // namespace hex_mini_drive`;
  code.write``;

  return (
    code.get() + 
    // Also copy the helping header files:
    readFileSync("./cobs_encoding.hpp")
  );
}

function write_js_definition(code, def) {
  if (def instanceof ConstantSpec) {
    if (def.doc) code.comment(def.doc);
    code.write`export const ${def.name} = ${def.const};`;

  } else if (def instanceof StructSpec) {
    if (def.doc) code.comment(def.doc);
    code.write`export class ${def.name}${def.struct_base ? ` extends ${def.struct_base.name}` : ''} {`;
    code.indent();
    for (const field of Object.values(def.struct || {})) {
      if (field.doc) code.comment(field.doc);
      code.write`${field.name};`;
    }

    code.write``;
    code.write`constructor(init) {${def.struct_base ? `super(init);` : ''}Object.assign(this, init);}`;
    code.dedent();
    code.write`}`;

  } else if (def instanceof ArraySpec) {
    if (def.doc) code.comment(def.doc);
    code.write`export class ${def.name} extends Array {`;
    code.indent();
    code.write`constructor(init) {`;
    code.indent();
    code.write`if (Array.isArray(init)) {`;
    code.indent();
    code.write`super(...init);`;
    code.dedent();
    code.write`} else {`;
    code.indent();
    code.write`super(${def.array_len});`;
    code.dedent();
    code.write`}`;
    code.dedent();
    code.write`}`;
    code.dedent();
    code.write`}`;
    code.write``;

  } else {
    throw new Error(`Unknown definition type: ${def}`);
  }
  code.write``;
}

function serialize_js_field(view, offset, name, type) {
  if (type instanceof BasicType) {
    return `${type.js_serialization(view, offset, name)}`;
  } else if (type instanceof StructSpec || type instanceof ArraySpec) {
    return `write_${type.name}(${name})`;
  } else {
    throw new Error(`Unknown type for serialization: ${type}`);
  }
}

function deserialize_js_field(view, offset, type) {
  if (type instanceof BasicType) {
    return `${type.js_deserialization(view, offset)}`;
  } else if (type instanceof StructSpec || type instanceof ArraySpec) {
    return `read_${type.name}(${view}, ${offset})`;
  } else {
    throw new Error(`Unknown type for deserialization: ${type}`);
  }
}

// We will generate separate serialization functions for each type, and then a generic serialize function for messages that uses those.
function write_js_serialization(code, def) {
  if (def instanceof StructSpec) {
    code.write`function write_${def.name}(value) {`;
    code.indent();
    code.write`const buffer = new Uint8Array(${def.size});`;
    code.write`const view = new DataView(buffer.buffer);`;
    code.write`let offset = 0;`;

    // Serialize base fields if struct_base
    if (def.struct_base) {
      code.write`const base_buffer = ${serialize_js_field(`view`, `offset`, `value`, def.struct_base)}`;
      code.write`buffer.set(base_buffer, offset);`;
      code.write`offset += ${def.struct_base.size};`;
    }

    // Now serialize own fields.
    for (const field of Object.values(def.struct || {})) {
      code.write`${serialize_js_field(`view`, `offset`, `value.${field.name}`, field.type)}`;
      code.write`offset += ${field.type.size};`;
    }
    code.write`return buffer;`;
    code.dedent();
    code.write`}`;

  } else if (def instanceof ArraySpec) {
    code.write`function write_${def.name}(value) {`;
    code.indent();
    code.write`const buffer = new Uint8Array(${def.size});`;
    code.write`const view = new DataView(buffer.buffer);`;
    code.write`let offset = 0;`;
    code.write`for (let i = 0; i < ${def.array_len}; i++) {`;
    code.indent();
    code.write`${serialize_js_field(`view`, `offset`, `value[i]`, def.type)}`;
    code.write`offset += ${def.type.size};`;
    code.dedent();
    code.write`}`;
    code.write`return buffer;`;
    code.dedent();
    code.write`}`;
    code.write``;
  } else if (def instanceof ConstantSpec) {
    // Nothing to serialize for constants.
  } else {
    throw new Error(`Unknown definition type: ${def}`);
  }
}

function write_js_deserialization(code, def) {
  if (def instanceof StructSpec) {
    code.write`function read_${def.name}(view, offset = 0) {`;
    code.indent();

    code.write`let result = new ${def.name}();`;
    code.write``;

    if (def.struct_base) {
      code.write`Object.assign(result, ${deserialize_js_field(`view`, `offset`, def.struct_base)});`;
      code.write`offset += ${def.struct_base.size};`;
      code.write``;
    }
    
    // Deserialize own fields.
    for (const field of Object.values(def.struct || {})) {
      code.write`result.${field.name} = ${deserialize_js_field(`view`, `offset`, field.type)};`;
      code.write`offset += ${field.type.size};`;
    }
    code.write`return result;`;
    code.dedent();
    code.write`}`;
  } else if (def instanceof ArraySpec) {
    code.write`function read_${def.name}(view, offset = 0) {`;
    code.indent();

    code.write`let result = new ${def.name}();`;
    code.write`for (let i = 0; i < ${def.array_len}; i++) {`;
    code.indent();
    code.write`result[i] = ${deserialize_js_field(`view`, `offset`, def.type)};`;
    code.write`offset += ${def.type.size};`;
    code.dedent();
    code.write`}`;
    code.write`return result;`;
    code.dedent();
    code.write`}`;
    code.write``;
  } else if (def instanceof ConstantSpec) {
    // Nothing to deserialize for constants.
  } else {
    throw new Error(`Unknown definition type: ${def}`);
  }
}

function write_js_message_protocol(code, message_codes) {
  code.comment`Message Codes`;
  for (const message of Object.values(message_codes)) {
    if (message.doc) code.comment(message.doc);
    code.write`const ${message.name} = ${message.message_code};`;
  }
  code.write``;

  code.write`export const MessageCode = {`;
  code.indent();
  for (const message of Object.values(message_codes)) {
    code.write`${message.name},`;
  }
  code.dedent();
  code.write`};`;
  code.write``;

  // Get all unique types referenced by messages.
  let message_types = new Set();
  for (const message of Object.values(message_codes)) {
    if (message.type) message_types.add(message.type);
  }

  // Generic serialize function.
  code.comment`Generic Serialize Function`;
  code.write`export function write_message(message) {`;
  code.indent();
  code.write`switch (message.message_code) {`;
  code.indent();
  for (const message of Object.values(message_codes)) {
    code.write`case ${message.name}: {`;
    code.indent();
    if (message.type) {
      code.write`const message_buffer = write_${message.type.name}(message);`;
      code.write`const buffer = new Uint8Array(2 + message_buffer.length);`;
      code.write`const view = new DataView(buffer.buffer);`;
      code.write`view.setUint16(0, message.message_code);`;
      code.write`buffer.set(message_buffer, 2);`;
      code.write`return buffer;`;
    } else {
      code.write`const buffer = new Uint8Array(2);`;
      code.write`const view = new DataView(buffer.buffer);`;
      
      code.write`view.setUint16(0, message.message_code);`;
      code.write`return buffer;`;
    }
    code.dedent();
    code.write`}`;
  }
  code.dedent();
  code.write`}`;
  code.dedent();
  code.write`}`;
  code.write``;

  // Generic deserialize function.
  code.comment`Generic Deserialize Function`;
  code.write`export function read_message(buffer) {`;
  code.indent();
  code.write`if (buffer.length < 2) return null;`;
  code.write`const view = new DataView(buffer.buffer);`;
  code.write`const message_code = view.getUint16(0);`;
  code.write`switch (message_code) {`;
  code.indent();
  for (const message of Object.values(message_codes)) {
    code.write`case ${message.name}: {`;
    code.indent();
    if (message.type) {
      code.write`if (buffer.length !== 2 + ${message.type.size}) return null;`;
      code.write`let message = read_${message.type.name}(view, 2);`;
      code.write`message.message_code = ${message.name};`;
      code.write`return message;`;
    } else {
      code.write`if (buffer.length !== 2) return null;`;
      code.write`return {message_code};`;
    }
    code.dedent();
    code.write`}`;
  }
  code.dedent();
  code.write`}`;
  code.write``;
  code.comment`Unknown message code`;
  code.write`return null;`;
  code.dedent();
  code.write`}`;
  code.write``;
}
  
function generate_js_interface(spec) {
  let code = new CodeWriter();

  // Header comment
  code.comment(spec.doc);
  code.write``;

  // Autogenerated note.
  code.comment`Note, this file is autogenerated. Use 'npm run build' to regenerate from the 'interface.yaml' spec.`;
  code.write``;

  
  for (const def of Object.values(spec.definitions)) {
    write_js_definition(code, def);
    write_js_serialization(code, def);
    write_js_deserialization(code, def);
  }

  write_js_message_protocol(code, spec.message_codes);

  return (
    code.get() + 
    readFileSync("./cobs_encoding.js")
  );
}


// Run this script if executed directly, to generate interface files.
if (import.meta.main) {
  const spec = load_spec("./interface.yaml");

  console.log("Loaded interface specification:");

  const output_dir = "./dist/";
  mkdirSync(output_dir, { recursive: true });
  
  const cpp_interface = generate_cpp_interface(spec);
  
  // Write the generated C++ interface.
  const cpp_output_path = `${output_dir}/hex_mini_drive_interface.hpp`;
  writeFileSync(cpp_output_path, cpp_interface);

  console.log("Generated C++ interface:", cpp_output_path);
  
  // Generate JavaScript interface
  const js_interface = generate_js_interface(spec);
  const js_output_path = `${output_dir}/hex_mini_drive_interface.js`;
  writeFileSync(js_output_path, js_interface);

  console.log("Generated JavaScript interface:", js_output_path);

  // And don't forget to copy the interface spec itself.
  writeFileSync(`${output_dir}/hex_mini_drive_interface.yaml`, readFileSync("./interface.yaml"));
  console.log("Copied interface specification:", `${output_dir}/hex_mini_drive_interface.yaml`);
}