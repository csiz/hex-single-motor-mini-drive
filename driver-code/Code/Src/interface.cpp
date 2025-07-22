#include "interface.hpp"

#include "byte_handling.hpp"
#include "error_handler.hpp"

#include <cstring>

// Serialize data
// --------------

static inline void write_message_tail(uint8_t * buffer, size_t & offset){

    // TODO: do the CRC calculation.
    const uint32_t crc = 0;

    write_uint32(buffer + offset, crc);
    offset += 4;

    write_uint16(buffer + offset, END_OF_MESSAGE);
    offset += 2;
}

// Write the bare values so we can re-use the code for the full readout.
static inline size_t write_readout_values(uint8_t * buffer, Readout const& readout) {
    size_t offset = 0;
    
    write_uint32(buffer + offset, readout.pwm_commands);
    offset += 4;
    write_uint16(buffer + offset, readout.readout_number);
    offset += 2;
    write_uint16(buffer + offset, readout.state_flags);
    offset += 2;
    write_int16(buffer + offset, readout.u_readout);
    offset += 2;
    write_int16(buffer + offset, readout.v_readout);
    offset += 2;
    write_int16(buffer + offset, readout.w_readout);
    offset += 2;
    write_uint16(buffer + offset, readout.ref_readout);
    offset += 2;
    write_int16(buffer + offset, readout.u_readout_diff);
    offset += 2;
    write_int16(buffer + offset, readout.v_readout_diff);
    offset += 2;
    write_int16(buffer + offset, readout.w_readout_diff);
    offset += 2;
    write_uint16(buffer + offset, readout.angle);
    offset += 2;
    write_int16(buffer + offset, readout.angular_speed);
    offset += 2;
    write_uint16(buffer + offset, readout.instant_vcc_voltage);
    offset += 2;

    return offset;
}

size_t write_readout(uint8_t * buffer, Readout const& readout) {
    size_t offset = 0;
    write_uint16(buffer + offset, READOUT);
    offset += 2;

    offset += write_readout_values(buffer + offset, readout);

    write_message_tail(buffer, offset);

    // Check if we wrote the correct number of bytes.
    if (offset != readout_size) error();

    return offset;
}

size_t write_full_readout(uint8_t * buffer, FullReadout const& readout) {
    size_t offset = 0;

    // Overwrite command code.
    write_uint16(buffer + offset, FULL_READOUT);
    offset += 2;
    
    offset += write_readout_values(buffer + offset, readout);

    // Write the additional data.
    write_uint16(buffer + offset, readout.main_loop_rate);
    offset += 2;
    write_uint16(buffer + offset, readout.adc_update_rate);
    offset += 2;
    write_uint16(buffer + offset, readout.temperature);
    offset += 2;
    write_uint16(buffer + offset, readout.vcc_voltage);
    offset += 2;
    
    write_int16(buffer + offset, readout.cycle_start_tick);
    offset += 2;
    write_int16(buffer + offset, readout.cycle_end_tick);
    offset += 2;


    write_int16(buffer + offset, readout.alpha_current);
    offset += 2;
    write_int16(buffer + offset, readout.beta_current);
    offset += 2;
    write_int16(buffer + offset, readout.alpha_emf_voltage);
    offset += 2;
    write_int16(buffer + offset, readout.beta_emf_voltage);
    offset += 2;


    write_int16(buffer + offset, readout.total_power);
    offset += 2;
    write_int16(buffer + offset, readout.resistive_power);
    offset += 2;
    write_int16(buffer + offset, readout.emf_power);
    offset += 2;
    write_int16(buffer + offset, readout.inductive_power);
    offset += 2;

    write_int16(buffer + offset, readout.motor_constant);
    offset += 2;
    write_int16(buffer + offset, readout.inductor_angle);
    offset += 2;
    write_int16(buffer + offset, readout.emf_voltage_magnitude);
    offset += 2;
    write_int16(buffer + offset, readout.rotor_acceleration);
    offset += 2;

    
    write_int16(buffer + offset, readout.angle_error);
    offset += 2;
    write_int16(buffer + offset, readout.phase_resistance);
    offset += 2;
    write_int16(buffer + offset, readout.phase_inductance);
    offset += 2;
    write_int16(buffer + offset, readout.emf_voltage_variance);
    offset += 2;
    write_int16(buffer + offset, readout.debug_1);
    offset += 2;
    write_int16(buffer + offset, readout.debug_2);
    offset += 2;

    write_message_tail(buffer, offset);

    // Check if we wrote the correct number of bytes.
    if (offset != full_readout_size) error();

    return offset;
}

size_t write_current_calibration(uint8_t * buffer, CurrentCalibration const& factors) {
    size_t offset = 0;
    write_uint16(buffer + offset, CURRENT_FACTORS);
    offset += 2;
    write_int16(buffer + offset, factors.u_factor);
    offset += 2;
    write_int16(buffer + offset, factors.v_factor);
    offset += 2;
    write_int16(buffer + offset, factors.w_factor);
    offset += 2;
    write_int16(buffer + offset, factors.inductance_factor);
    offset += 2;

    write_message_tail(buffer, offset);
    
    // Check if we wrote the correct number of bytes.
    if (offset != current_calibration_size) error();

    return offset;
}

size_t write_unit_test(uint8_t * buffer, UnitTestFunction test_function){
    if (test_function == nullptr) error();

    // Fully 0 out the buffer so we can write the test output safely.
    memset(buffer, 0, unit_test_size);

    write_uint16(buffer, MessageCode::UNIT_TEST_OUTPUT);

    const size_t test_data_size = unit_test_size - header_size - tail_size;

    // Run the test function and write the results to the USB buffer.
    test_function(reinterpret_cast<char *>(buffer + header_size), test_data_size);

    size_t offset = header_size + test_data_size;

    write_message_tail(buffer, offset);

    // Check if we wrote the correct number of bytes.
    if (offset != unit_test_size) error();

    return offset;
}

size_t write_control_parameters(uint8_t * buffer, ControlParameters const& control_parameters) {
    size_t offset = 0;

    write_uint16(buffer + offset, CONTROL_PARAMETERS);
    offset += 2;

    write_int16(buffer + offset, control_parameters.rotor_angle_ki);
    offset += 2;
    write_int16(buffer + offset, control_parameters.rotor_angular_speed_ki);
    offset += 2;
    write_int16(buffer + offset, control_parameters.rotor_acceleration_ki);
    offset += 2;
    write_int16(buffer + offset, control_parameters.motor_constant_ki);
    offset += 2;
    write_int16(buffer + offset, control_parameters.resistance_ki);
    offset += 2;
    write_int16(buffer + offset, control_parameters.inductance_ki);
    offset += 2;
    write_int16(buffer + offset, control_parameters.max_pwm_change);
    offset += 2;
    write_int16(buffer + offset, control_parameters.max_angle_change);
    offset += 2;
    write_int16(buffer + offset, control_parameters.min_emf_voltage);
    offset += 2;
    write_int16(buffer + offset, control_parameters.min_emf_speed);
    offset += 2;
    write_int16(buffer + offset, control_parameters.lead_angle_control_ki);
    offset += 2;
    write_int16(buffer + offset, control_parameters.torque_control_ki);
    offset += 2;
    write_int16(buffer + offset, control_parameters.battery_power_control_ki);
    offset += 2;
    write_int16(buffer + offset, control_parameters.speed_control_ki);
    offset += 2;
    write_int16(buffer + offset, control_parameters.probing_angular_speed);
    offset += 2;
    write_int16(buffer + offset, control_parameters.probing_max_pwm);
    offset += 2;
    
    write_message_tail(buffer, offset);

    // Check if we wrote the correct number of bytes.
    if (offset != control_parameters_size) error();

    return offset;
}

// Receive data
// ------------

bool check_message_for_errors(uint8_t const * data, size_t size) {
    if (size < min_message_size) return true;

    // TODO: Run the cyclic redundancy check (CRC).
    // const uint32_t crc = read_uint32(data + size - tail_size);
    // if (crc != calculate_crc(data, size - crc_size)) return true;

    const uint16_t end_of_message = read_uint16(data + size - end_of_message_size);

    if (end_of_message != END_OF_MESSAGE) return true;

    return false;
}


// Message size for each command code that we can receive.
static inline int get_message_size(uint16_t code) {
    switch (static_cast<MessageCode>(code)) {
        case NULL_COMMAND: return -1;

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

        case GET_CURRENT_FACTORS: return min_message_size;
        case GET_CONTROL_PARAMETERS: return min_message_size;
        case RESET_CONTROL_PARAMETERS: return min_message_size;
        case SET_ANGLE: return min_message_size;

        case SAVE_SETTINGS_TO_FLASH: return min_message_size;

        case RUN_UNIT_TEST_FUNKY_ATAN: return min_message_size;
        case RUN_UNIT_TEST_FUNKY_ATAN_PART_2: return min_message_size;
        case RUN_UNIT_TEST_FUNKY_ATAN_PART_3: return min_message_size;
        
        case SET_CURRENT_FACTORS: return current_calibration_size;
        case SET_CONTROL_PARAMETERS: return control_parameters_size;

        case READOUT: return readout_size;
        case FULL_READOUT: return full_readout_size;
        case CURRENT_FACTORS: return current_calibration_size;
        case CONTROL_PARAMETERS: return control_parameters_size;

        case UNIT_TEST_OUTPUT: return unit_test_size;
    }

    // Unknown message.
    return -1;
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
        reset_message_buffer(buffer);
        // Notify the caller that we have an error.
        return true;
    }

    buffer.bytes_expected = expected_size - buffer.write_index;

    if (buffer.bytes_expected < 0) error();

    return false;
}

BasicCommand parse_basic_command(uint8_t const * data, size_t size) {
    if(size != basic_command_size) error();

    // Skip the code check for basic commands as it can have multiple values.
    size_t offset = header_size;

    const uint16_t timeout = read_uint16(data + offset);
    offset += 2;
    const int16_t value = read_int16(data + offset);
    offset += 2;
    const int16_t second = read_int16(data + offset);
    offset += 2;
    const int16_t third = read_int16(data + offset);
    offset += 2;

    if (offset + tail_size != basic_command_size) error();

    return BasicCommand { 
        .timeout = timeout, 
        .value = value, 
        .second = second, 
        .third = third,
    };
}

CurrentCalibration parse_current_calibration(uint8_t const * data, size_t size) {
    if(size < current_calibration_size) error();

    CurrentCalibration current_calibration = {};

    // Yep, skip this too.
    size_t offset = header_size;

    current_calibration.u_factor = read_int16(data + offset);
    offset += 2;
    current_calibration.v_factor = read_int16(data + offset);
    offset += 2;
    current_calibration.w_factor = read_int16(data + offset);
    offset += 2;
    current_calibration.inductance_factor = read_int16(data + offset);
    offset += 2;

    if (offset + tail_size != current_calibration_size) error();

    return current_calibration;
}


ControlParameters parse_control_parameters(uint8_t const * data, size_t size) {
    if(size < control_parameters_size) error();

    // And skip check again.
    size_t offset = header_size;

    ControlParameters control_parameters = {};

    control_parameters.rotor_angle_ki = read_int16(data + offset);
    offset += 2;
    control_parameters.rotor_angular_speed_ki = read_int16(data + offset);
    offset += 2;
    control_parameters.rotor_acceleration_ki = read_int16(data + offset);
    offset += 2;
    control_parameters.motor_constant_ki = read_int16(data + offset);
    offset += 2;
    control_parameters.resistance_ki = read_int16(data + offset);
    offset += 2;
    control_parameters.inductance_ki = read_int16(data + offset);
    offset += 2;
    control_parameters.max_pwm_change = read_int16(data + offset);
    offset += 2;
    control_parameters.max_angle_change = read_int16(data + offset);
    offset += 2;
    control_parameters.min_emf_voltage = read_int16(data + offset);
    offset += 2;
    control_parameters.min_emf_speed = read_int16(data + offset);
    offset += 2;
    control_parameters.lead_angle_control_ki = read_int16(data + offset);
    offset += 2;
    control_parameters.torque_control_ki = read_int16(data + offset);
    offset += 2;
    control_parameters.battery_power_control_ki = read_int16(data + offset);
    offset += 2;
    control_parameters.speed_control_ki = read_int16(data + offset);
    offset += 2;
    control_parameters.probing_angular_speed = read_int16(data + offset);
    offset += 2;
    control_parameters.probing_max_pwm = read_int16(data + offset);
    offset += 2;

    if (offset + tail_size != control_parameters_size) error();

    return control_parameters;
}