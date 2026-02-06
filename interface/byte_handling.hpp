#pragma once

#include <cstdint>
#include <cstddef>

namespace hex_mini_drive {

// Alias for a generic function that reads a buffer of some size.
using BufferFunction = void (*)(uint8_t * buffer, size_t size);

// Maximum message size (in bytes) for a message.
const size_t max_message_size = 256;

// A statically allocated buffer for messages.
struct MessageBuffer {
    uint8_t data[max_message_size];
    size_t size = 0;
};

// Write 1 byte to the buffer.
static inline void write_uint8(uint8_t * buf, uint8_t value) {
    buf[0] = value;
}

// Read 1 byte from the buffer.
static inline uint8_t read_uint8(uint8_t const * buf) {
    return buf[0];
}

// Write 2 byte unsigned integer to the buffer.
static inline void write_uint16(uint8_t * buf, uint16_t value) {
    buf[0] = (value >> 8) & 0xFF;
    buf[1] = value & 0xFF;
}

// Read 2 byte unsigned integer from the buffer.
static inline uint16_t read_uint16(uint8_t const * buf) {
    uint16_t value = 0;
    value |= buf[0] << 8;
    value |= buf[1];
    return value;
}

// Write 2 byte signed integer to the buffer.
static inline void write_int16(uint8_t * buf, int16_t value) {
    buf[0] = (value >> 8) & 0xFF;
    buf[1] = value & 0xFF;
}

// Read 2 byte signed integer from the buffer.
static inline int16_t read_int16(uint8_t const * buf) {
    int16_t value = 0;
    value |= buf[0] << 8;
    value |= buf[1];
    return value;
}

// Write 4 byte unsigned integer to the buffer.
static inline void write_uint32(uint8_t * buf, uint32_t value){
    buf[0] = (value >> 24) & 0xFF;
    buf[1] = (value >> 16) & 0xFF;
    buf[2] = (value >> 8) & 0xFF;
    buf[3] = value & 0xFF;
}

// Read 4 byte unsigned integer from the buffer.
static inline uint32_t read_uint32(uint8_t const * buf) {
    uint32_t value = 0;
    value |= buf[0] << 24;
    value |= buf[1] << 16;
    value |= buf[2] << 8;
    value |= buf[3];
    return value;
}

// Write 4 byte float to the buffer.
static inline void write_float32(uint8_t * buf, float value) {
    uint8_t * value_pointer = reinterpret_cast<uint8_t*>(&value);
    buf[0] = value_pointer[0];
    buf[1] = value_pointer[1];
    buf[2] = value_pointer[2];
    buf[3] = value_pointer[3];
}

// Read 4 byte float from the buffer.
static inline float read_float32(uint8_t const * buf) {
    float value = 0.0f;
    uint8_t * value_pointer = reinterpret_cast<uint8_t*>(&value);
    value_pointer[0] = buf[0];
    value_pointer[1] = buf[1];
    value_pointer[2] = buf[2];
    value_pointer[3] = buf[3];
    return value;
}

} // end namespace hex_mini_drive
