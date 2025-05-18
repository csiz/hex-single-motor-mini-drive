#include "interface.hpp"

#include "byte_handling.hpp"
#include "error_handler.hpp"


// Serialize data
// --------------

void write_readout(uint8_t * buffer, Readout const & readout) {
    size_t offset = 0;
    write_uint16(buffer + offset, READOUT);
    offset += 2;
    write_uint32(buffer + offset, readout.pwm_commands);
    offset += 4;
    write_uint16(buffer + offset, readout.readout_number);
    offset += 2;
    write_uint16(buffer + offset, readout.u_readout);
    offset += 2;
    write_uint16(buffer + offset, readout.v_readout);
    offset += 2;
    write_uint16(buffer + offset, readout.w_readout);
    offset += 2;
    write_uint16(buffer + offset, readout.ref_readout);
    offset += 2;
    write_uint16(buffer + offset, readout.position);
    offset += 2;
    write_int16(buffer + offset, readout.angular_speed);
    offset += 2;
    write_uint16(buffer + offset, readout.vcc_voltage);
    offset += 2;
    write_int16(buffer + offset, readout.torque);
    offset += 2;
    write_int16(buffer + offset, readout.hold);
    offset += 2;
    write_int16(buffer + offset, readout.total_power);
    offset += 2;
    write_int16(buffer + offset, readout.resistive_power);
    offset += 2;

    // Check if we wrote the correct number of bytes.
    if (offset != readout_size) error();
}

void write_full_readout(uint8_t * buffer, FullReadout const & readout) {
    size_t offset = 0;
    write_readout(buffer + offset, static_cast<Readout>(readout));
    offset += readout_size;

    // Overwrite command code.
    write_uint16(buffer + 0, FULL_READOUT);

    // Write the additional data.
    write_uint16(buffer + offset, readout.tick_rate);
    offset += 2;
    write_uint16(buffer + offset, readout.adc_update_rate);
    offset += 2;
    write_uint16(buffer + offset, readout.hall_unobserved_rate);
    offset += 2;
    write_uint16(buffer + offset, readout.hall_observed_rate);
    offset += 2;

    write_uint16(buffer + offset, readout.temperature);
    offset += 2;
    write_uint16(buffer + offset, readout.vcc_voltage);
    offset += 2;
    write_int16(buffer + offset, readout.cycle_start_tick);
    offset += 2;
    write_int16(buffer + offset, readout.cycle_end_tick);
    offset += 2;

    write_int16(buffer + offset, readout.current_angle);
    offset += 2;
    write_uint16(buffer + offset, readout.current_angle_variance);
    offset += 2;

    write_uint16(buffer + offset, readout.angle_variance);
    offset += 2;
    write_uint16(buffer + offset, readout.angular_speed_variance);
    offset += 2;


    // Check if we wrote the correct number of bytes.
    if (offset != full_readout_size) error();
}

void write_current_calibration(uint8_t * data, CurrentCalibration const & factors) {
    size_t offset = 0;
    write_uint16(data + offset, CURRENT_FACTORS);
    offset += 2;
    write_int16(data + offset, factors.u_factor);
    offset += 2;
    write_int16(data + offset, factors.v_factor);
    offset += 2;
    write_int16(data + offset, factors.w_factor);
    offset += 2;

    // Check if we wrote the correct number of bytes.
    if (offset != current_calibration_size) error();
}


void write_position_calibration(uint8_t * buffer, PositionCalibration const & position_calibration) {
    size_t offset = 0;
    write_uint16(buffer + offset, TRIGGER_ANGLES);
    offset += 2;

    for (int i = 0; i < 6; i++){
        write_uint16(buffer + offset, position_calibration.sector_transition_angles[i][0]);
        offset += 2;
        write_uint16(buffer + offset, position_calibration.sector_transition_angles[i][1]);
        offset += 2;
    }

    for (int i = 0; i < 6; i++){
        write_uint16(buffer + offset, position_calibration.sector_transition_variances[i][0]);
        offset += 2;
        write_uint16(buffer + offset, position_calibration.sector_transition_variances[i][1]);
        offset += 2;
    }
    for (int i = 0; i < 6; i++){
        write_uint16(buffer + offset, position_calibration.sector_center_angles[i]);
        offset += 2;
    }
    for (int i = 0; i < 6; i++){
        write_uint16(buffer + offset, position_calibration.sector_center_variances[i]);
        offset += 2;
    }

    write_uint16(buffer + offset, position_calibration.initial_angular_speed_variance);
    offset += 2;
    write_uint16(buffer + offset, position_calibration.angular_acceleration_div_2_variance);
    offset += 2;

    // Check if we wrote the correct number of bytes.
    if (offset != position_calibration_size) error();
}

// Receive data
// ------------

static inline int get_message_size(uint16_t code) {
    switch (code) {
        case STREAM_FULL_READOUTS:
        case GET_READOUTS_SNAPSHOT:
        case SET_STATE_OFF:
        case SET_STATE_DRIVE_POS:
        case SET_STATE_TEST_ALL_PERMUTATIONS:
        case SET_STATE_DRIVE_NEG:
        case SET_STATE_FREEWHEEL:
        case SET_STATE_TEST_GROUND_SHORT:
        case SET_STATE_TEST_POSITIVE_SHORT:
        case SET_STATE_TEST_U_DIRECTIONS:
        case SET_STATE_TEST_U_INCREASING:
        case SET_STATE_TEST_U_DECREASING:
        case SET_STATE_TEST_V_INCREASING:
        case SET_STATE_TEST_V_DECREASING:
        case SET_STATE_TEST_W_INCREASING:
        case SET_STATE_TEST_W_DECREASING:
        case SET_STATE_HOLD_U_POSITIVE:
        case SET_STATE_HOLD_V_POSITIVE:
        case SET_STATE_HOLD_W_POSITIVE:
        case SET_STATE_HOLD_U_NEGATIVE:
        case SET_STATE_HOLD_V_NEGATIVE:
        case SET_STATE_HOLD_W_NEGATIVE:
        case SET_STATE_DRIVE_SMOOTH_POS:
        case SET_STATE_DRIVE_SMOOTH_NEG:
        case GET_CURRENT_FACTORS:
        case GET_TRIGGER_ANGLES:
            return min_message_size;
        
        case SET_CURRENT_FACTORS:
            return current_calibration_size;
        case SET_TRIGGER_ANGLES:
            return position_calibration_size;

        default:
            // Unknown message; we don't know how many bytes to expect.
            return -1;
    }
}


bool buffer_command(MessageBuffer & buffer, int receive_function(uint8_t *buf, uint16_t len)){
    // Make sure the buffer is ready to receive data.
    if (buffer.bytes_expected == 0) error();

    const int bytes_received = receive_function(buffer.data + buffer.write_index, buffer.bytes_expected);

    // We've received too much data.
    if (bytes_received > buffer.bytes_expected) error();

    buffer.write_index += bytes_received;

    // Wait for more data if we don't have a command header yet.
    if (buffer.write_index < header_size) return false;

    // The first number is the command code, the remainder is the command data.
    const uint16_t code = read_uint16(buffer.data);

    const int expected_size = get_message_size(code);

    if (expected_size < 0) {
        // Unknown command. Discard the buffer.
        reset_command_buffer(buffer);
        // Notify the caller that we have an error.
        return true;
    }

    buffer.bytes_expected = expected_size - buffer.write_index;

    if (buffer.bytes_expected < 0) error();

    return false;
}

BasicCommand parse_basic_command(uint8_t const * data, size_t size) {
    if(size < basic_command_size) error();

    size_t offset = 0;

    const uint16_t code = read_uint16(data + offset);
    offset += 2;
    const uint16_t timeout = read_uint16(data + offset);
    offset += 2;
    const uint16_t pwm = read_uint16(data + offset);
    offset += 2;
    const uint16_t leading_angle = read_uint16(data + offset);
    offset += 2;

    if (offset != basic_command_size) error();

    return BasicCommand { code, timeout, pwm, leading_angle };
}

CurrentCalibration parse_current_calibration(uint8_t const * data, size_t size) {
    if(size < current_calibration_size) error();

    CurrentCalibration current_calibration = {};

    size_t offset = header_size;

    current_calibration.u_factor = read_int16(data + offset);
    offset += 2;
    current_calibration.v_factor = read_int16(data + offset);
    offset += 2;
    current_calibration.w_factor = read_int16(data + offset);
    offset += 2;

    if (offset != current_calibration_size) error();

    return current_calibration;
}


PositionCalibration parse_position_calibration(uint8_t const * data, size_t size) {
    if(size < position_calibration_size) error();

    size_t offset = header_size;

    PositionCalibration position_calibration = {};

    for (int i = 0; i < 6; i++){
        position_calibration.sector_transition_angles[i][0] = read_uint16(data + offset);
        offset += 2;
        position_calibration.sector_transition_angles[i][1] = read_uint16(data + offset);
        offset += 2;
    }

    for (int i = 0; i < 6; i++){
        position_calibration.sector_transition_variances[i][0] = read_uint16(data + offset);
        offset += 2;
        position_calibration.sector_transition_variances[i][1] = read_uint16(data + offset);
        offset += 2;
    }
    for (int i = 0; i < 6; i++){
        position_calibration.sector_center_angles[i] = read_uint16(data + offset);
        offset += 2;
    }
    for (int i = 0; i < 6; i++){
        position_calibration.sector_center_variances[i] = read_uint16(data + offset);
        offset += 2;
    }

    position_calibration.initial_angular_speed_variance = read_uint16(data + offset);
    offset += 2;
    position_calibration.angular_acceleration_div_2_variance = read_uint16(data + offset);
    offset += 2;

    if (offset != position_calibration_size) error();

    return position_calibration;
}




