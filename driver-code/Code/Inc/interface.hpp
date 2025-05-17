#pragma once

#include "type_definitions.hpp"

#include <cstdint>
#include <cstring>


// Interface command codes
// -----------------------

// These are the command codes that are used to identify the command type sent over the wire.
enum CommandCode : uint16_t {
    NULL_COMMAND = 0x0000,

    READOUT = 0x2020,
    STREAM_FULL_READOUTS = 0x2021,
    GET_READOUTS_SNAPSHOT = 0x2022,
    FULL_READOUT = 0x2023,

    SET_STATE_OFF = 0x2030,
    SET_STATE_DRIVE_POS = 0x2031,
    SET_STATE_TEST_ALL_PERMUTATIONS = 0x2032,
    SET_STATE_DRIVE_NEG = 0x2033,
    SET_STATE_FREEWHEEL = 0x2034,

    SET_STATE_TEST_GROUND_SHORT = 0x2036,
    SET_STATE_TEST_POSITIVE_SHORT = 0x2037,

    SET_STATE_TEST_U_DIRECTIONS = 0x2039,
    SET_STATE_TEST_U_INCREASING = 0x203A,
    SET_STATE_TEST_U_DECREASING = 0x203B,
    SET_STATE_TEST_V_INCREASING = 0x203C,
    SET_STATE_TEST_V_DECREASING = 0x203D,
    SET_STATE_TEST_W_INCREASING = 0x203E,
    SET_STATE_TEST_W_DECREASING = 0x203F,

    SET_STATE_HOLD_U_POSITIVE = 0x3020,
    SET_STATE_HOLD_V_POSITIVE = 0x3021,
    SET_STATE_HOLD_W_POSITIVE = 0x3022,
    SET_STATE_HOLD_U_NEGATIVE = 0x3023,
    SET_STATE_HOLD_V_NEGATIVE = 0x3024,
    SET_STATE_HOLD_W_NEGATIVE = 0x3025,

    SET_STATE_DRIVE_SMOOTH_POS = 0x4030,
    SET_STATE_DRIVE_SMOOTH_NEG = 0x4031,

    SET_CURRENT_FACTORS = 0x4040,
    SET_TRIGGER_ANGLES = 0x4041,
    CURRENT_FACTORS = 0x4042,
    TRIGGER_ANGLES = 0x4043,
    GET_CURRENT_FACTORS = 0x4044,
    GET_TRIGGER_ANGLES = 0x4045,
};



// Command buffer
// --------------

// Buffer parts of commands until they are complete.

const size_t command_header_size = sizeof(CommandHeader);
const size_t max_command_size = 128;

struct CommandBuffer {
    uint8_t data[max_command_size] = {};
    size_t index = 0;
    int bytes_expected = command_header_size;
};

static inline void reset_command_buffer(CommandBuffer & buffer) {
    // Clear the buffer with memset to avoid stale data.
    memset(buffer.data, 0, sizeof(buffer.data));
    buffer.index = 0;
    buffer.bytes_expected = command_header_size;
}


// Receiving commands
// ------------------

// Receive a command on the data buffer (or part of command or nothing).
// 
// Use a data stream with a receive function that takes a buffer and a length as
// arguments and returns the number of bytes received. Can receive partial commands.
bool buffer_command(CommandBuffer & buffer, int receive_function(uint8_t * buf, uint16_t len));

CommandHeader parse_command_header(CommandBuffer const & buffer);
CurrentCalibration parse_current_calibration(CommandBuffer const & buffer);
PositionCalibration parse_position_calibration(CommandBuffer const & buffer);


// Sending data
// ------------


const size_t readout_size = 2 + sizeof(Readout);
void write_readout(uint8_t * buffer, Readout const & readout);

const size_t full_readout_size = 2 + sizeof(FullReadout);
void write_full_readout(uint8_t * buffer, FullReadout const & full_readout);

const size_t current_calibration_size = 2 + sizeof(CurrentCalibration);
void write_current_calibration(uint8_t * buffer, CurrentCalibration const & factors);

const size_t position_calibration_size = 2 + sizeof(PositionCalibration);
void write_position_calibration(uint8_t * buffer, PositionCalibration const & position_calibration);