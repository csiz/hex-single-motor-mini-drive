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


// Receive data
// ------------


bool buffer_command(CommandBuffer & buffer, int receive_function(uint8_t *buf, uint16_t len)){
    // Make sure the buffer is ready to receive data.
    if (buffer.bytes_expected == 0) error();

    const int bytes_received = receive_function(buffer.data + buffer.index, buffer.bytes_expected);

    // We've received too much data.
    if (bytes_received > buffer.bytes_expected) error();

    buffer.index += bytes_received;
    buffer.bytes_expected -= bytes_received;

    // Wait for more data if we don't have a command header yet.
    if (buffer.index < command_header_size) return false;

    const int bytes_extra = buffer.index - command_header_size;
        
    // The first number is the command code, the remainder is the command data.
    const uint16_t code = read_uint16(buffer.data);

    // Expect additional bytes for the commands below.
    switch (code){
        case SET_CURRENT_FACTORS:
            buffer.bytes_expected = sizeof(CurrentCalibration) - bytes_extra;
            break;
        case SET_TRIGGER_ANGLES:
            buffer.bytes_expected = sizeof(PositionCalibration) - bytes_extra;
            break;
        default:
            // ALl other commands fit into the command header; and were therefore fully received.
            break;
    }

    // We can now check if we have received a complete command.
    return buffer.bytes_expected == 0;
}

CommandHeader parse_command_header(CommandBuffer const & buffer) {
    if(buffer.index < command_header_size) error();

    uint8_t const * data = buffer.data;

    const uint16_t code = read_uint16(data);
    data += 2;
    const uint16_t timeout = read_uint16(data);
    data += 2;
    const uint16_t pwm = read_uint16(data);
    data += 2;
    const uint16_t leading_angle = read_uint16(data);
    data += 2;

    if (data - buffer.data != command_header_size) error();

    return CommandHeader { code, timeout, pwm, leading_angle };
}

CurrentCalibration parse_current_calibration(CommandBuffer const & buffer) {
    if(buffer.index < command_header_size + sizeof(CurrentCalibration)) error();

    // Skip the command header.
    uint8_t const * data = buffer.data + command_header_size;

    CurrentCalibration factors = {};

    factors.u_factor = read_int16(data);
    data += 2;
    factors.v_factor = read_int16(data);
    data += 2;
    factors.w_factor = read_int16(data);
    data += 2;

    if (data - buffer.data != command_header_size + sizeof(CurrentCalibration)) error();

    return factors;
}

void write_current_calibration(uint8_t * buffer, CurrentCalibration const & factors) {
    size_t offset = 0;
    write_uint16(buffer + offset, CURRENT_FACTORS);
    offset += 2;
    write_int16(buffer + offset, factors.u_factor);
    offset += 2;
    write_int16(buffer + offset, factors.v_factor);
    offset += 2;
    write_int16(buffer + offset, factors.w_factor);
    offset += 2;

    // Check if we wrote the correct number of bytes.
    if (offset != current_calibration_size) error();
}

PositionCalibration parse_position_calibration(CommandBuffer const & buffer) {
    if(buffer.index < command_header_size + sizeof(PositionCalibration)) error();

    // Skip the command header.
    uint8_t const * data = buffer.data + command_header_size;

    PositionCalibration position_calibration = {};

    for (int i = 0; i < 6; i++){
        position_calibration.sector_transition_angles[i][0] = read_uint16(data);
        data += 2;
        position_calibration.sector_transition_angles[i][1] = read_uint16(data);
        data += 2;
    }

    for (int i = 0; i < 6; i++){
        position_calibration.sector_transition_variances[i][0] = read_uint16(data);
        data += 2;
        position_calibration.sector_transition_variances[i][1] = read_uint16(data);
        data += 2;
    }
    for (int i = 0; i < 6; i++){
        position_calibration.sector_center_angles[i] = read_uint16(data);
        data += 2;
    }
    for (int i = 0; i < 6; i++){
        position_calibration.sector_center_variances[i] = read_uint16(data);
        data += 2;
    }

    position_calibration.initial_angular_speed_variance = read_uint16(data);
    data += 2;
    position_calibration.angular_acceleration_div_2_variance = read_uint16(data);
    data += 2;

    if (data - buffer.data != command_header_size + sizeof(PositionCalibration)) error();

    return position_calibration;
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


