import { CodeWriter, BasicType } from "./spec_types.js";

export const basic_types = {
  uint8: new BasicType({
    size: 1, 
    name: 'uint8', 
    cpp_type: 'uint8_t', 
    js_type: 'Uint8',
    write_cpp_serialization: (code) => {
      code.write`static inline void write_uint8(uint8_t * buffer, uint8_t const& value) {`;
      code.indent();
      code.write`buffer[0] = value;`;
      code.dedent();
      code.write`}`;
    },
    write_cpp_deserialization: (code) => {
      code.write`static inline uint8_t read_uint8(uint8_t const* buffer) {`;
      code.indent();
      code.write`return buffer[0];`;
      code.dedent();
      code.write`}`;
    },
    js_serialization: (view, offset, val) => `${view}.setUint8(${offset}, ${val})`,
    js_deserialization: (view, offset) => `${view}.getUint8(${offset})`,
  }),
  uint16: new BasicType({
    size: 2, 
    name: 'uint16', 
    cpp_type: 'uint16_t', 
    js_type: 'Uint16',
    write_cpp_serialization: (code) => {
      code.write`static inline void write_uint16(uint8_t * buffer, uint16_t const& value) {`;
      code.indent();
      code.write`buffer[0] = (value >> 8) & 0xFF;`;
      code.write`buffer[1] = value & 0xFF;`;
      code.dedent();
      code.write`}`;
    },
    write_cpp_deserialization: (code) => {
      code.write`static inline uint16_t read_uint16(uint8_t const* buffer) {`;
      code.indent();
      code.write`uint16_t value = 0;`;
      code.write`value |= buffer[0] << 8;`;
      code.write`value |= buffer[1];`;
      code.write`return value;`;
      code.dedent();
      code.write`}`;
    },
    js_serialization: (view, offset, val) => `${view}.setUint16(${offset}, ${val})`,
    js_deserialization: (view, offset) => `${view}.getUint16(${offset})`,
  }),
  uint32: new BasicType({
    size: 4, 
    name: 'uint32', 
    cpp_type: 'uint32_t', 
    js_type: 'Uint32',
    write_cpp_serialization: (code) => {
      code.write`static inline void write_uint32(uint8_t * buffer, uint32_t const& value) {`;
      code.indent();
      code.write`buffer[0] = (value >> 24) & 0xFF;`;
      code.write`buffer[1] = (value >> 16) & 0xFF;`;
      code.write`buffer[2] = (value >> 8) & 0xFF;`;
      code.write`buffer[3] = value & 0xFF;`;
      code.dedent();
      code.write`}`;
    },
    write_cpp_deserialization: (code) => {
      code.write`static inline uint32_t read_uint32(uint8_t const* buffer) {`;
      code.indent();
      code.write`uint32_t value = 0;`;
      code.write`value |= buffer[0] << 24;`;
      code.write`value |= buffer[1] << 16;`;
      code.write`value |= buffer[2] << 8;`;
      code.write`value |= buffer[3];`;
      code.write`return value;`;
      code.dedent();
      code.write`}`;
    },
    js_serialization: (view, offset, val) => `${view}.setUint32(${offset}, ${val})`,
    js_deserialization: (view, offset) => `${view}.getUint32(${offset})`,
  }),
  int8: new BasicType({
    size: 1, 
    name: 'int8', 
    cpp_type: 'int8_t', 
    js_type: 'Int8',
    write_cpp_serialization: (code) => {
      code.write`static inline void write_int8(uint8_t * buffer, int8_t const& value) {`;
      code.indent();
      code.write`buffer[0] = value;`;
      code.dedent();
      code.write`}`;
    },
    write_cpp_deserialization: (code) => {
      code.write`static inline int8_t read_int8(uint8_t const* buffer) {`;
      code.indent();
      code.write`return buffer[0];`;
      code.dedent();
      code.write`}`;
    },
    js_serialization: (view, offset, val) => `${view}.setInt8(${offset}, ${val})`,
    js_deserialization: (view, offset) => `${view}.getInt8(${offset})`,
  }),
  int16: new BasicType({
    size: 2, 
    name: 'int16', 
    cpp_type: 'int16_t', 
    js_type: 'Int16',
    write_cpp_serialization: (code) => {
      code.write`static inline void write_int16(uint8_t * buffer, int16_t const& value) {`;
      code.indent();
      code.write`buffer[0] = (value >> 8) & 0xFF;`;
      code.write`buffer[1] = value & 0xFF;`;
      code.dedent();
      code.write`}`;
    },
    write_cpp_deserialization: (code) => {
      code.write`static inline int16_t read_int16(uint8_t const* buffer) {`;
      code.indent();
      code.write`int16_t value = 0;`;
      code.write`value |= buffer[0] << 8;`;
      code.write`value |= buffer[1];`;
      code.write`return value;`;
      code.dedent();
      code.write`}`;
    },
    js_serialization: (view, offset, val) => `${view}.setInt16(${offset}, ${val})`,
    js_deserialization: (view, offset) => `${view}.getInt16(${offset})`,
  }),
  int32: new BasicType({
    size: 4, 
    name: 'int32', 
    cpp_type: 'int32_t', 
    js_type: 'Int32',
    write_cpp_serialization: (code) => {
      code.write`static inline void write_int32(uint8_t * buffer, int32_t const& value) {`;
      code.indent();
      code.write`buffer[0] = (value >> 24) & 0xFF;`;
      code.write`buffer[1] = (value >> 16) & 0xFF;`;
      code.write`buffer[2] = (value >> 8) & 0xFF;`;
      code.write`buffer[3] = value & 0xFF;`;
      code.dedent();
      code.write`}`;
    },
    write_cpp_deserialization: (code) => {
      code.write`static inline int32_t read_int32(uint8_t const* buffer) {`;
      code.indent();
      code.write`int32_t value = 0;`;
      code.write`value |= buffer[0] << 24;`;
      code.write`value |= buffer[1] << 16;`;
      code.write`value |= buffer[2] << 8;`;
      code.write`value |= buffer[3];`;
      code.write`return value;`;
      code.dedent();
      code.write`}`;
    },
    js_serialization: (view, offset, val) => `${view}.setInt32(${offset}, ${val})`,
    js_deserialization: (view, offset) => `${view}.getInt32(${offset})`,
  }),
  float32: new BasicType({
    size: 4, 
    name: 'float32', 
    cpp_type: 'float', 
    js_type: 'Float32',
    write_cpp_serialization: (code) => {
      code.write`static inline void write_float32(uint8_t * buffer, float const& value) {`;
      code.indent();
      code.write`uint8_t const* value_pointer = reinterpret_cast<uint8_t const*>(&value);`;
      code.write`buffer[0] = value_pointer[0];`;
      code.write`buffer[1] = value_pointer[1];`;
      code.write`buffer[2] = value_pointer[2];`;
      code.write`buffer[3] = value_pointer[3];`;
      code.dedent();
      code.write`}`;
    },
    write_cpp_deserialization: (code) => {
      code.write`static inline float read_float32(uint8_t const* buffer) {`;
      code.indent();
      code.write`float value;`;
      code.write`uint8_t * value_pointer = reinterpret_cast<uint8_t*>(&value);`;
      code.write`value_pointer[0] = buffer[0];`;
      code.write`value_pointer[1] = buffer[1];`;
      code.write`value_pointer[2] = buffer[2];`;
      code.write`value_pointer[3] = buffer[3];`;
      code.write`return value;`;
      code.dedent();
      code.write`}`;
    },
    js_serialization: (view, offset, val) => `${view}.setFloat32(${offset}, ${val})`,
    js_deserialization: (view, offset) => `${view}.getFloat32(${offset})`,
  }),
};