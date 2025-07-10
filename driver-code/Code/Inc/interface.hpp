#pragma once

#include "type_definitions.hpp"

#include <cstdint>


// Interface command codes
// -----------------------


// These are the command codes that are used to identify the command type sent over the wire.
enum MessageCode : uint16_t {
    NULL_COMMAND = 0x0000,

    READOUT = 0x2020,
    STREAM_FULL_READOUTS = 0x2021,
    GET_READOUTS_SNAPSHOT = 0x2022,
    FULL_READOUT = 0x2023,

    SET_STATE_OFF = 0x2030,
    SET_STATE_DRIVE_6_SECTOR = 0x2031,
    SET_STATE_TEST_ALL_PERMUTATIONS = 0x2032,
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

    SET_STATE_DRIVE_PERIODIC = 0x3040,
    SET_STATE_DRIVE_SMOOTH = 0x4030,
    SET_STATE_DRIVE_TORQUE = 0x4031,
    SET_STATE_DRIVE_BATTERY_POWER = 0x4032,

    CURRENT_FACTORS = 0x4040,
    GET_CURRENT_FACTORS = 0x4041,
    SET_CURRENT_FACTORS = 0x4042,

    PID_PARAMETERS = 0x4046,
    GET_PID_PARAMETERS = 0x4047,
    SET_PID_PARAMETERS = 0x4048,

    OBSERVER_PARAMETERS = 0x4049,
    SET_OBSERVER_PARAMETERS = 0x404A,
    GET_OBSERVER_PARAMETERS = 0x404B,

    SAVE_SETTINGS_TO_FLASH = 0x4080,

    UNIT_TEST_OUTPUT = 0x5040,
    RUN_UNIT_TEST_FUNKY_ATAN = 0x5042,
    RUN_UNIT_TEST_FUNKY_ATAN_PART_2 = 0x5043,
    RUN_UNIT_TEST_FUNKY_ATAN_PART_3 = 0x5044,
};



// Expected data sizes
// -------------------

// Command header size (the command code).
const size_t header_size = 2;

const size_t basic_command_size = sizeof(BasicCommand); // Note; basic command includes the header code.

const size_t readout_size = header_size + sizeof(Readout);
const size_t full_readout_size = header_size + sizeof(FullReadout);
const size_t current_calibration_size = header_size + sizeof(CurrentCalibration);
const size_t pid_parameters_size = header_size + sizeof(PIDParameters);
const size_t observer_parameters_size = header_size + sizeof(ObserverParameters);

const size_t max_message_size = 256;
const size_t min_message_size = 8;



// Command buffer
// --------------

// Buffer parts of commands until they are complete.
struct MessageBuffer {
    uint8_t data[max_message_size] = {};
    size_t write_index = 0;
    int bytes_expected = min_message_size;
};

static inline void reset_command_buffer(MessageBuffer & buffer) {
    buffer.write_index = 0;
    buffer.bytes_expected = min_message_size;
}

// Receiving commands
// ------------------

// Receive a command on the data buffer (or part of command or nothing).
// 
// Use a data stream with a receive function that takes a buffer and a length as
// arguments and returns the number of bytes received. Can receive partial commands.
bool buffer_command(MessageBuffer & buffer, int receive_function(uint8_t * buf, uint16_t len));

BasicCommand parse_basic_command(uint8_t const * data, size_t size);
CurrentCalibration parse_current_calibration(uint8_t const * data, size_t size);
PIDParameters parse_pid_parameters(uint8_t const * data, size_t size);
ObserverParameters parse_observer_parameters(uint8_t const * data, size_t size);

// Sending data
// ------------


void write_readout(uint8_t * buffer, Readout const & readout);
void write_full_readout(uint8_t * buffer, FullReadout const & full_readout);
void write_current_calibration(uint8_t * data, CurrentCalibration const & factors);
void write_pid_parameters(uint8_t * buffer, PIDParameters const & parameters);
void write_observer_parameters(uint8_t * buffer, ObserverParameters const & parameters);

// The unit test will write directly to the buffer.
const size_t unit_test_size = max_message_size;
