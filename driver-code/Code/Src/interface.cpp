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

    write_int16(buffer + offset, readout.position);
    offset += 2;
    write_uint16(buffer + offset, readout.angle_bytes);
    offset += 2;
    write_int16(buffer + offset, readout.angular_speed);
    offset += 2;
    write_uint16(buffer + offset, readout.instant_vcc_voltage);
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
    write_uint16(buffer + offset, readout.temperature);
    offset += 2;
    write_uint16(buffer + offset, readout.vcc_voltage);
    offset += 2;
    
    write_int16(buffer + offset, readout.cycle_start_tick);
    offset += 2;
    write_int16(buffer + offset, readout.cycle_end_tick);
    offset += 2;
    write_uint16(buffer + offset, readout.angle_variance);
    offset += 2;
    write_uint16(buffer + offset, readout.angular_speed_variance);
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

    write_int16(buffer + offset, readout.angle_error);
    offset += 2;
    write_int16(buffer + offset, readout.angle_error_variance);
    offset += 2;
    write_int16(buffer + offset, readout.angular_speed_error);
    offset += 2;
    write_int16(buffer + offset, readout.angular_speed_error_variance);
    offset += 2;

    write_int16(buffer + offset, readout.inductor_angle);
    offset += 2;
    write_int16(buffer + offset, readout.inductor_angle_variance);
    offset += 2;
    write_int16(buffer + offset, readout.inductor_angle_error);
    offset += 2;
    write_int16(buffer + offset, readout.inductor_angle_error_variance);
    offset += 2;

    write_int16(buffer + offset, readout.inductor_angular_speed);
    offset += 2;
    write_int16(buffer + offset, readout.inductor_angular_speed_variance);
    offset += 2;
    write_int16(buffer + offset, readout.inductor_angular_speed_error);
    offset += 2;
    write_int16(buffer + offset, readout.inductor_angular_speed_error_variance);
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
    write_int16(data + offset, factors.inductance_factor);
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

void write_pid_parameters(uint8_t * buffer, PIDParameters const & parameters) {
    size_t offset = 0;

    write_uint16(buffer + offset, PID_PARAMETERS);
    offset += 2;
    
    write_int16(buffer + offset, parameters.current_angle_gains.kp);
    offset += 2;
    write_int16(buffer + offset, parameters.current_angle_gains.ki);
    offset += 2;
    write_int16(buffer + offset, parameters.current_angle_gains.kd);
    offset += 2;
    write_int16(buffer + offset, parameters.current_angle_gains.max_output);
    offset += 2;

    write_int16(buffer + offset, parameters.torque_gains.kp);
    offset += 2;
    write_int16(buffer + offset, parameters.torque_gains.ki);
    offset += 2;
    write_int16(buffer + offset, parameters.torque_gains.kd);
    offset += 2;
    write_int16(buffer + offset, parameters.torque_gains.max_output);
    offset += 2;

    write_int16(buffer + offset, parameters.battery_power_gains.kp);
    offset += 2;
    write_int16(buffer + offset, parameters.battery_power_gains.ki);
    offset += 2;
    write_int16(buffer + offset, parameters.battery_power_gains.kd);
    offset += 2;
    write_int16(buffer + offset, parameters.battery_power_gains.max_output);
    offset += 2;

    write_int16(buffer + offset, parameters.angular_speed_gains.kp);
    offset += 2;
    write_int16(buffer + offset, parameters.angular_speed_gains.ki);
    offset += 2;
    write_int16(buffer + offset, parameters.angular_speed_gains.kd);
    offset += 2;
    write_int16(buffer + offset, parameters.angular_speed_gains.max_output);
    offset += 2;

    write_int16(buffer + offset, parameters.position_gains.kp);
    offset += 2;
    write_int16(buffer + offset, parameters.position_gains.ki);
    offset += 2;
    write_int16(buffer + offset, parameters.position_gains.kd);
    offset += 2;
    write_int16(buffer + offset, parameters.position_gains.max_output);
    offset += 2;

    // Check if we wrote the correct number of bytes.
    if (offset != pid_parameters_size) error();
}

void write_observer_parameters(uint8_t * buffer, ObserverParameters const & observers) {
    size_t offset = 0;

    write_uint16(buffer + offset, OBSERVER_PARAMETERS);
    offset += 2;

    write_int16(buffer + offset, observers.rotor_angle_ki);
    offset += 2;
    write_int16(buffer + offset, observers.rotor_angular_speed_ki);
    offset += 2;
    write_int16(buffer + offset, observers.inductor_angle_ki);
    offset += 2;
    write_int16(buffer + offset, observers.inductor_angular_speed_ki);
    offset += 2;
    write_int16(buffer + offset, observers.resistance_ki);
    offset += 2;
    write_int16(buffer + offset, observers.inductance_ki);
    offset += 2;
    write_int16(buffer + offset, observers.motor_constant_ki);
    offset += 2;
    write_int16(buffer + offset, observers.magnetic_resistance_ki);
    offset += 2;
    write_int16(buffer + offset, observers.rotor_mass_ki);
    offset += 2;
    write_int16(buffer + offset, observers.rotor_torque_ki);
    offset += 2;
    
    // Check if we wrote the correct number of bytes.
    if (offset != observer_parameters_size) error();
}

// Receive data
// ------------

// Message size for each command code that we can receive.
static inline int get_message_size(uint16_t code) {
    switch (static_cast<MessageCode>(code)) {
        case NULL_COMMAND:
            return -1;
        case STREAM_FULL_READOUTS:
        case GET_READOUTS_SNAPSHOT:
        case SET_STATE_OFF:
        case SET_STATE_DRIVE_6_SECTOR:
        case SET_STATE_TEST_ALL_PERMUTATIONS:
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
        case SET_STATE_DRIVE_PERIODIC:
        case SET_STATE_DRIVE_SMOOTH:
        case SET_STATE_DRIVE_TORQUE:
        case SET_STATE_DRIVE_BATTERY_POWER:
        case GET_CURRENT_FACTORS:
        case GET_TRIGGER_ANGLES:
        case GET_PID_PARAMETERS:
        case GET_OBSERVER_PARAMETERS:
        case SAVE_SETTINGS_TO_FLASH:
        case RUN_UNIT_TEST_ATAN:
        case RUN_UNIT_TEST_FUNKY_ATAN:
        case RUN_UNIT_TEST_FUNKY_ATAN_PART_2:
        case RUN_UNIT_TEST_FUNKY_ATAN_PART_3:
            return min_message_size;
        
        case SET_CURRENT_FACTORS:
            return current_calibration_size;
        case SET_TRIGGER_ANGLES:
            return position_calibration_size;
        case SET_PID_PARAMETERS:
            return pid_parameters_size;
        case SET_OBSERVER_PARAMETERS:
            return observer_parameters_size;

        case READOUT:
            return readout_size;
        case FULL_READOUT:
            return full_readout_size;
        case CURRENT_FACTORS:
            return current_calibration_size;
        case TRIGGER_ANGLES:
            return position_calibration_size;
        case PID_PARAMETERS:
            return pid_parameters_size;
        case OBSERVER_PARAMETERS:
            return observer_parameters_size;

        case UNIT_TEST_OUTPUT:
            return unit_test_size;
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
    const int16_t pwm = read_int16(data + offset);
    offset += 2;
    const int16_t leading_angle = read_int16(data + offset);
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
    current_calibration.inductance_factor = read_int16(data + offset);
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


PIDParameters parse_pid_parameters(uint8_t const * data, size_t size) {
    if(size < pid_parameters_size) error();

    size_t offset = header_size;

    PIDParameters parameters = {};

    parameters.current_angle_gains.kp = read_int16(data + offset);
    offset += 2;
    parameters.current_angle_gains.ki = read_int16(data + offset);
    offset += 2;
    parameters.current_angle_gains.kd = read_int16(data + offset);
    offset += 2;
    parameters.current_angle_gains.max_output = read_int16(data + offset);
    offset += 2;

    parameters.torque_gains.kp = read_int16(data + offset);
    offset += 2;
    parameters.torque_gains.ki = read_int16(data + offset);
    offset += 2;
    parameters.torque_gains.kd = read_int16(data + offset);
    offset += 2;
    parameters.torque_gains.max_output = read_int16(data + offset);
    offset += 2;

    parameters.battery_power_gains.kp = read_int16(data + offset);
    offset += 2;
    parameters.battery_power_gains.ki = read_int16(data + offset);
    offset += 2;
    parameters.battery_power_gains.kd = read_int16(data + offset);
    offset += 2;
    parameters.battery_power_gains.max_output = read_int16(data + offset);
    offset += 2;

    parameters.angular_speed_gains.kp = read_int16(data + offset);
    offset += 2;
    parameters.angular_speed_gains.ki = read_int16(data + offset);
    offset += 2;
    parameters.angular_speed_gains.kd = read_int16(data + offset);
    offset += 2;
    parameters.angular_speed_gains.max_output = read_int16(data + offset);
    offset += 2;

    parameters.position_gains.kp = read_int16(data + offset);
    offset += 2;
    parameters.position_gains.ki = read_int16(data + offset);
    offset += 2;
    parameters.position_gains.kd = read_int16(data + offset);
    offset += 2;
    parameters.position_gains.max_output = read_int16(data + offset);
    offset += 2;

    if (offset != pid_parameters_size) error();

    return parameters;
}

ObserverParameters parse_observer_parameters(uint8_t const * data, size_t size) {
    if(size < observer_parameters_size) error();

    size_t offset = header_size;

    ObserverParameters observers = {};

    observers.rotor_angle_ki = read_int16(data + offset);
    offset += 2;
    observers.rotor_angular_speed_ki = read_int16(data + offset);
    offset += 2;
    observers.inductor_angle_ki = read_int16(data + offset);
    offset += 2;
    observers.inductor_angular_speed_ki = read_int16(data + offset);
    offset += 2;
    observers.resistance_ki = read_int16(data + offset);
    offset += 2;
    observers.inductance_ki = read_int16(data + offset);
    offset += 2;
    observers.motor_constant_ki = read_int16(data + offset);
    offset += 2;
    observers.magnetic_resistance_ki = read_int16(data + offset);
    offset += 2;
    observers.rotor_mass_ki = read_int16(data + offset);
    offset += 2;
    observers.rotor_torque_ki = read_int16(data + offset);
    offset += 2;

    if (offset != observer_parameters_size) error();

    return observers;
}