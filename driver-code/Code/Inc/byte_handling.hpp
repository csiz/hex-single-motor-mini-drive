#pragma once

#include <cstdint>

static inline void write_uint32(uint8_t * buf, uint32_t value){
    buf[0] = (value >> 24) & 0xFF;
    buf[1] = (value >> 16) & 0xFF;
    buf[2] = (value >> 8) & 0xFF;
    buf[3] = value & 0xFF;
}

static inline uint32_t read_uint32(uint8_t const * buf) {
    uint32_t value = 0;
    value |= buf[0] << 24;
    value |= buf[1] << 16;
    value |= buf[2] << 8;
    value |= buf[3];
    return value;
}

static inline void write_uint16(uint8_t * buf, uint16_t value) {
    buf[0] = (value >> 8) & 0xFF;
    buf[1] = value & 0xFF;
}

static inline void write_int16(uint8_t * buf, int16_t value) {
    buf[0] = (value >> 8) & 0xFF;
    buf[1] = value & 0xFF;
}

static inline uint16_t read_uint16(uint8_t const * buf) {
    uint16_t value = 0;
    value |= buf[0] << 8;
    value |= buf[1];
    return value;
}

static inline int16_t read_int16(uint8_t const * buf) {
    int16_t value = 0;
    value |= buf[0] << 8;
    value |= buf[1];
    return value;
}