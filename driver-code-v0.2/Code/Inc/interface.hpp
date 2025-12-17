#pragma once

#include "type_definitions.hpp"
#include "byte_handling.hpp"

#include <cstddef>
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
    SET_STATE_DRIVE_SPEED = 0x4033,
    SET_STATE_SEEK_ANGLE_WITH_POWER = 0x4034,
    SET_STATE_SEEK_ANGLE_WITH_TORQUE = 0x4035,
    SET_STATE_SEEK_ANGLE_WITH_SPEED = 0x4036,

    CURRENT_FACTORS = 0x4040,
    GET_CURRENT_FACTORS = 0x4041,
    SET_CURRENT_FACTORS = 0x4042,
    RESET_CURRENT_FACTORS = 0x4043,

    HALL_POSITIONS = 0x4044,
    GET_HALL_POSITIONS = 0x4045,
    SET_HALL_POSITIONS = 0x4046,
    RESET_HALL_POSITIONS = 0x4047,

    CONTROL_PARAMETERS = 0x4049,
    SET_CONTROL_PARAMETERS = 0x404A,
    GET_CONTROL_PARAMETERS = 0x404B,
    RESET_CONTROL_PARAMETERS = 0x404C,

    SET_ANGLE = 0x4050,

    SAVE_SETTINGS_TO_FLASH = 0x4080,

    UNIT_TEST_OUTPUT = 0x5040,
    RUN_UNIT_TEST_FUNKY_ATAN = 0x5042,
    RUN_UNIT_TEST_FUNKY_ATAN_PART_2 = 0x5043,
    RUN_UNIT_TEST_FUNKY_ATAN_PART_3 = 0x5044,
};

// Bit pattern used to mark the end of a message.
const uint16_t END_OF_MESSAGE = 0b0101'0101'0101'0101;

// Expected data sizes
// -------------------

// The maximum size of a message that we send or receive.
const size_t max_message_size = 256;

// Command header size (the command code).
const size_t header_size = 2;

// Size of the cyclic redundancy check (CRC) at the end of the message.
const size_t crc_size = 4;

// Size of the end of message marker. We output a fixed bit pattern at the end of each message.
const size_t end_of_message_size = sizeof(END_OF_MESSAGE);

// The size of the tail of the message; CRC and end of message marker.
const size_t tail_size = crc_size + end_of_message_size;

// Size of the generic command message.
const size_t basic_command_size = header_size + sizeof(BasicCommand) + tail_size;

// Size of the compact readout message.
const size_t readout_size = header_size + sizeof(Readout) + tail_size;

// Size of the complete state readout message.
const size_t full_readout_size = header_size + sizeof(FullReadout) + tail_size;

// Size of the current calibration message.
const size_t current_calibration_size = header_size + sizeof(CurrentCalibration) + tail_size;

// Size of the position calibration message.
const size_t position_calibration_size = header_size + sizeof(PositionCalibration) + tail_size;

// Size of the control parameters message.
const size_t control_parameters_size = header_size + sizeof(ControlParameters) + tail_size;

// Size of the unit test output message. We set the message size to the maximum and adjust
// the test output buffer size to account for the header and tail.
const size_t unit_test_size = max_message_size;

// The basic command is the smallest message that we send or receive.
const size_t min_message_size = basic_command_size;


// Message buffer
// --------------

// Buffer bits of command data until we receive a complete message.
struct MessageBuffer {
    uint8_t data[max_message_size] = {};
    size_t write_index = 0;
};

// Receiving commands
// ------------------

// Receive a command on the data buffer (or part of command or nothing).
// 
// Use a data stream with a receive function that takes a buffer and a length as
// arguments and returns the number of bytes received. Can receive partial commands.
bool buffer_command(MessageBuffer & buffer, int receive_function(uint8_t * buf, uint16_t len));

// Run the cyclic redundancy check (CRC) on the message data and check if the message is valid.
bool check_message_for_errors(uint8_t const * data, size_t size);


BasicCommand parse_basic_command(uint8_t const * data, size_t size);
CurrentCalibration parse_current_calibration(uint8_t const * data, size_t size);
ControlParameters parse_control_parameters(uint8_t const * data, size_t size);
PositionCalibration parse_position_calibration(uint8_t const * data, size_t size);

// Sending data
// ------------


size_t write_readout(uint8_t * buffer, Readout const& readout);
size_t write_full_readout(uint8_t * buffer, FullReadout const& full_readout);
size_t write_current_calibration(uint8_t * buffer, CurrentCalibration const& factors);
size_t write_position_calibration(uint8_t * data, PositionCalibration const & position_calibration);
size_t write_control_parameters(uint8_t * buffer, ControlParameters const& control_parameters);
size_t write_unit_test(uint8_t * buffer, UnitTestFunction test_function);


// Message size for each command code that we can receive.
static inline size_t get_message_size(uint16_t code) {
    switch (static_cast<MessageCode>(code)) {
        case NULL_COMMAND: return 0;

        case STREAM_FULL_READOUTS: return min_message_size;
        case GET_READOUTS_SNAPSHOT: return min_message_size;

        case SET_STATE_OFF: return min_message_size;
        case SET_STATE_DRIVE_6_SECTOR: return min_message_size;
        case SET_STATE_TEST_ALL_PERMUTATIONS: return min_message_size;
        case SET_STATE_FREEWHEEL: return min_message_size;

        case SET_STATE_TEST_GROUND_SHORT: return min_message_size;
        case SET_STATE_TEST_POSITIVE_SHORT: return min_message_size;

        case SET_STATE_TEST_U_DIRECTIONS: return min_message_size;
        case SET_STATE_TEST_U_INCREASING: return min_message_size;
        case SET_STATE_TEST_U_DECREASING: return min_message_size;
        case SET_STATE_TEST_V_INCREASING: return min_message_size;
        case SET_STATE_TEST_V_DECREASING: return min_message_size;
        case SET_STATE_TEST_W_INCREASING: return min_message_size;
        case SET_STATE_TEST_W_DECREASING: return min_message_size;

        case SET_STATE_HOLD_U_POSITIVE: return min_message_size;
        case SET_STATE_HOLD_V_POSITIVE: return min_message_size;
        case SET_STATE_HOLD_W_POSITIVE: return min_message_size;
        case SET_STATE_HOLD_U_NEGATIVE: return min_message_size;
        case SET_STATE_HOLD_V_NEGATIVE: return min_message_size;
        case SET_STATE_HOLD_W_NEGATIVE: return min_message_size;

        case SET_STATE_DRIVE_PERIODIC: return min_message_size;
        case SET_STATE_DRIVE_SMOOTH: return min_message_size;
        case SET_STATE_DRIVE_TORQUE: return min_message_size;
        case SET_STATE_DRIVE_BATTERY_POWER: return min_message_size;
        case SET_STATE_DRIVE_SPEED: return min_message_size;
        case SET_STATE_SEEK_ANGLE_WITH_POWER: return min_message_size;
        case SET_STATE_SEEK_ANGLE_WITH_TORQUE: return min_message_size;
        case SET_STATE_SEEK_ANGLE_WITH_SPEED: return min_message_size;

        case GET_CURRENT_FACTORS: return min_message_size;
        case RESET_CURRENT_FACTORS: return min_message_size;
        case GET_HALL_POSITIONS: return min_message_size;
        case RESET_HALL_POSITIONS: return min_message_size;
        case GET_CONTROL_PARAMETERS: return min_message_size;
        case RESET_CONTROL_PARAMETERS: return min_message_size;
        case SET_ANGLE: return min_message_size;

        case SAVE_SETTINGS_TO_FLASH: return min_message_size;

        case RUN_UNIT_TEST_FUNKY_ATAN: return min_message_size;
        case RUN_UNIT_TEST_FUNKY_ATAN_PART_2: return min_message_size;
        case RUN_UNIT_TEST_FUNKY_ATAN_PART_3: return min_message_size;
        
        case SET_CURRENT_FACTORS: return current_calibration_size;
        case SET_HALL_POSITIONS: return position_calibration_size;
        case SET_CONTROL_PARAMETERS: return control_parameters_size;

        case READOUT: return readout_size;
        case FULL_READOUT: return full_readout_size;
        case CURRENT_FACTORS: return current_calibration_size;
        case HALL_POSITIONS: return position_calibration_size;
        case CONTROL_PARAMETERS: return control_parameters_size;

        case UNIT_TEST_OUTPUT: return unit_test_size;
    }

    // Unknown message.
    return 0;
}