#include "interface.hpp"

#include "byte_handling.hpp"
#include "error_handler.hpp"


// Serialize data
// --------------

void write_state_readout(uint8_t * buffer, StateReadout const & readout) {
    size_t offset = 0;
    write_uint16(buffer + offset, READOUT);
    offset += 2;
    write_uint16(buffer + offset, readout.readout_number);
    offset += 2;
    write_uint16(buffer + offset, readout.position);
    offset += 2;
    write_uint32(buffer + offset, readout.pwm_commands);
    offset += 4;
    write_uint16(buffer + offset, readout.u_readout);
    offset += 2;
    write_uint16(buffer + offset, readout.v_readout);
    offset += 2;
    write_uint16(buffer + offset, readout.w_readout);
    offset += 2;
    write_uint16(buffer + offset, readout.ref_readout);
    offset += 2;

    // Check if we wrote the correct number of bytes.
    if (offset != state_readout_size) error();
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
            buffer.bytes_expected = sizeof(CurrentFactors) - bytes_extra;
            break;
        case SET_TRIGGER_ANGLES:
            buffer.bytes_expected = sizeof(TriggerAngles) - bytes_extra;
            break;
        default:
            // ALl other commands fit into the command header; and were therefore received.
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

    return { code, timeout, pwm, leading_angle };
}

CurrentFactors parse_current_factors(CommandBuffer const & buffer) {
    if(buffer.index < command_header_size + sizeof(CurrentFactors)) error();

    // Skip the command header.
    uint8_t const * data = buffer.data + command_header_size;

    CurrentFactors factors = {};

    factors.u_pos_factor = read_uint16(data);
    data += 2;
    factors.u_neg_factor = read_uint16(data);
    data += 2;
    factors.v_pos_factor = read_uint16(data);
    data += 2;
    factors.v_neg_factor = read_uint16(data);
    data += 2;
    factors.w_pos_factor = read_uint16(data);
    data += 2;
    factors.w_neg_factor = read_uint16(data);
    data += 2;

    if (data - (buffer.data + command_header_size) != sizeof(CurrentFactors)) error();
    
    return factors;
}

TriggerAngles parse_trigger_angles(CommandBuffer const & buffer) {
    if(buffer.index < command_header_size + sizeof(TriggerAngles)) error();

    // Skip the command header.
    uint8_t const * data = buffer.data + command_header_size;

    TriggerAngles angles = {};

    for (int i = 0; i < 6; i++){
        angles.trigger_angle[i][0] = read_uint16(data);
        data += 2;
        angles.trigger_angle[i][1] = read_uint16(data);
        data += 2;
    }

    for (int i = 0; i < 6; i++){
        angles.trigger_angle_variance[i][0] = read_uint16(data);
        data += 2;
        angles.trigger_angle_variance[i][1] = read_uint16(data);
        data += 2;
    }

    if (data - (buffer.data + command_header_size) != sizeof(TriggerAngles)) error();
    
    return angles;
}


