#include "app_main.hpp"

#include "parameters_store.hpp"
#include "test_schedules.hpp"
#include "interrupts.hpp"
#include "interrupts_data.hpp"
#include "interface.hpp"


#include "byte_handling.hpp"
#include "integer_math.hpp"
#include "error_handler.hpp"
#include "constants.hpp"
#include "io.hpp"
#include "usb_com.hpp"


#include <cstddef>
#include <cstdint>
#include <stm32g4xx_ll_adc.h>
#include <stm32g4xx_ll_tim.h>
#include <stm32g4xx_hal.h>

#include <cstdlib>
#include <cmath>
#include <cstring>



uint32_t main_loop_number = 0;
uint32_t last_loop_number = 0;
uint16_t last_readout_number = 0;


const uint32_t min_timing_period_millis = 50;
uint32_t last_update_time_millis = 0;

float main_loop_rate = 0.0f;
float adc_update_rate = 0.0f;

MessageBuffer usb_receive_buffer = {};
uint32_t usb_chunk_receive_time = 0;

const uint32_t usb_partial_message_timeout_ms = 500;

MessageBuffer usb_response_buffer = {};

uint16_t usb_stream_state = 0;
uint16_t usb_stream_last_sent = 0;

uint16_t usb_readouts_to_send = 0;
bool usb_wait_full_history = false;

bool usb_reply_current_factors = false;
bool usb_reply_control_parameters = false;
bool usb_reply_hall_positions = false;
bool usb_reply_unit_test = false;

UnitTestFunction usb_unit_test_function = nullptr;

// Time of last USB packet sent; used to detect timeouts.
uint32_t usb_last_send = 0;

// Time of the last USB packet received; used to detect timeouts.
uint32_t usb_last_receive_time = 0;

// Index of the last partial message received; used to detect timeouts.
size_t usb_last_receive_index = 0;


int32_t motor_constant_observer = 0;




void app_init() {

    io_init();

    set_GREEN_LED(0xFF);

    // Get initial hall sensor state and initialize poisition tracking.
    initialize_angle_tracking();
}

static inline void motor_start_test(PWMSchedule const& schedule, int16_t value, bool take_snapshot) {
    // Clear the readouts buffer of old data.
    readout_history_reset();
    
    // Stop emptying the readouts queue; we want to keep the test data.
    usb_wait_full_history = take_snapshot;

    usb_readouts_to_send = take_snapshot ? history_size : 0;

    // Start the test schedule.
    set_motor_command(DriverState{ 
        .mode = DriverMode::SCHEDULE,
        .target_pwm = value,
        .schedule = DriveSchedule{ .pointer = &schedule } 
    });
}

// Run a unit test that takes a function pointer to a test function (which itself takes a buffer); returns whether error occurred.
inline bool run_unit_test(UnitTestFunction test_function) {
    if (usb_reply_unit_test) return true; // We are already running a unit test.

    // Remember which test to run when the USB queue is ready.
    usb_unit_test_function = test_function;

    // Set the flag to indicate that we have a unit test result ready to send.
    usb_reply_unit_test = true;

    return false;
}

// Handle the command on the buffer; returns whether there was an error.
bool handle_command(MessageBuffer const& buffer) {
    // Check the message has a header and tail and passes the CRC check.
    if (check_message_for_errors(buffer.data, buffer.write_index)) return true;

    // Get the message code from the data header.
    const uint16_t code = read_uint16(buffer.data);

    const BasicCommand command = parse_basic_command(buffer.data, buffer.write_index);


    switch (static_cast<MessageCode>(code)) {
        case NULL_COMMAND:
            // No command received; ignore it.
            return true;

        case STREAM_FULL_READOUTS: {
            // Cotinuously stream data if timeout > 0.
            usb_stream_state = command.timeout;
            // Also stop the motor if we stop the stream.
            if (not usb_stream_state) set_motor_command(DriverState{.mode = DriverMode::OFF});
            return false;
        }
        case GET_READOUTS_SNAPSHOT:
            // Cancel streaming; so we can take a data snapshot without interruptions.
            usb_stream_state = 0;
            
            readout_history_reset();
            // Dissalow sending until we fill the queue, so it doesn't interrupt commutation.
            usb_wait_full_history = true;
            usb_readouts_to_send = history_size;
            return false;

        // Turn off the motor driver.
        case SET_STATE_OFF:
            // Repeat command, with the interrupt guards.
            set_motor_command(DriverState{.mode = DriverMode::OFF});
            return false;
            
        // Measure the motor phase currents.
        
        case SET_STATE_TEST_ALL_PERMUTATIONS:
            motor_start_test(test_all_permutations, command.value, command.timeout > 0);
            return false;
        case SET_STATE_TEST_GROUND_SHORT:
            motor_start_test(test_ground_short, command.value, command.timeout > 0);
            return false;
        case SET_STATE_TEST_POSITIVE_SHORT:
            motor_start_test(test_positive_short, command.value, command.timeout > 0);
            return false;
        case SET_STATE_TEST_U_DIRECTIONS:
            motor_start_test(test_u_directions, command.value, command.timeout > 0);
            return false;

        case SET_STATE_TEST_U_INCREASING:
            motor_start_test(test_u_increasing, command.value, command.timeout > 0);
            return false;
        case SET_STATE_TEST_U_DECREASING:
            motor_start_test(test_u_decreasing, command.value, command.timeout > 0);
            return false;
        case SET_STATE_TEST_V_INCREASING:
            motor_start_test(test_v_increasing, command.value, command.timeout > 0);
            return false;
        case SET_STATE_TEST_V_DECREASING:
            motor_start_test(test_v_decreasing, command.value, command.timeout > 0);
            return false;
        case SET_STATE_TEST_W_INCREASING:
            motor_start_test(test_w_increasing, command.value, command.timeout > 0);
            return false;
        case SET_STATE_TEST_W_DECREASING:
            motor_start_test(test_w_decreasing, command.value, command.timeout > 0);
            return false;

        // Drive the motor.
        case SET_STATE_DRIVE_6_SECTOR: {
            set_motor_command(DriverState{ 
                .mode = DriverMode::DRIVE_6_SECTOR, 
                .duration = command.timeout, 
                .active_pwm = clip_to_short(-pwm_max, +pwm_max, command.value * control_parameters.motor_direction),
            });
            return false;
        }

        case SET_STATE_DRIVE_PERIODIC: {
            set_motor_command(DriverState{
                .mode = DriverMode::DRIVE_PERIODIC, 
                .duration = command.timeout,
                .active_angle = command.third,
                .active_pwm = clip_to_short(0, +pwm_max, command.value),
                .angular_speed = clip_to_short(-max_angular_speed, +max_angular_speed, command.second * control_parameters.motor_direction),
            });
            return false;
        }

        case SET_STATE_DRIVE_SMOOTH: {
            set_motor_command(DriverState{
                .mode = DriverMode::DRIVE_SMOOTH, 
                .duration = command.timeout, 
                .target_pwm = clip_to_short(-pwm_max, +pwm_max, command.value * control_parameters.motor_direction),
            });
            return false;
        }

        case SET_STATE_DRIVE_TORQUE: {
            set_motor_command(DriverState{
                .mode = DriverMode::DRIVE_TORQUE, 
                .duration = command.timeout,
                .secondary_target = clip_to_short(-max_drive_current, +max_drive_current, command.value * control_parameters.motor_direction),
            });
            return false;
        }

        case SET_STATE_DRIVE_BATTERY_POWER: {
            set_motor_command(DriverState{
                .mode = DriverMode::DRIVE_BATTERY_POWER, 
                .duration = command.timeout,
                .secondary_target = clip_to_short(-max_drive_power, +max_drive_power, command.value * control_parameters.motor_direction),
            });
            return false;
        }

        case SET_STATE_DRIVE_SPEED: {
            set_motor_command(DriverState{
                .mode = DriverMode::DRIVE_SPEED, 
                .duration = command.timeout,
                .secondary_target = clip_to_short(-max_angular_speed, +max_angular_speed, command.value * control_parameters.motor_direction),
            });
            return false;
        }

        case SET_STATE_SEEK_ANGLE_WITH_POWER: {
            set_motor_command(DriverState{
                .mode = DriverMode::SEEK_ANGLE_POWER, 
                .duration = command.timeout, 
                .seek_angle = SeekAngle{
                    .target_rotation = clip_to_short(-max_16bit, +max_16bit, command.value * control_parameters.motor_direction),
                    .target_angle = static_cast<int16_t>(normalize_angle(command.second * control_parameters.motor_direction)),
                    .max_secondary_target = clip_to_short(0, max_drive_power, command.third),
                }
            });
            return false;
        }

        case SET_STATE_SEEK_ANGLE_WITH_TORQUE: {
            set_motor_command(DriverState{
                .mode = DriverMode::SEEK_ANGLE_TORQUE, 
                .duration = command.timeout, 
                .seek_angle = SeekAngle{
                    .target_rotation = clip_to_short(-max_16bit, +max_16bit, command.value * control_parameters.motor_direction),
                    .target_angle = static_cast<int16_t>(normalize_angle(command.second * control_parameters.motor_direction)),
                    .max_secondary_target = clip_to_short(0, max_drive_current, command.third),
                }
            });
            return false;
        }

        case SET_STATE_SEEK_ANGLE_WITH_SPEED: {
            set_motor_command(DriverState{
                .mode = DriverMode::SEEK_ANGLE_SPEED, 
                .duration = command.timeout, 
                .seek_angle = SeekAngle{
                    .target_rotation = clip_to_short(-max_16bit, +max_16bit, command.value * control_parameters.motor_direction),
                    .target_angle = static_cast<int16_t>(normalize_angle(command.second * control_parameters.motor_direction)),
                    .max_secondary_target = clip_to_short(0, max_angular_speed, command.third),
                }
            });
            return false;
        }

        // Freewheel the motor.
        case SET_STATE_FREEWHEEL:
            set_motor_command(DriverState{ .mode = DriverMode::FREEWHEEL });
            return false;

        case SET_STATE_HOLD_U_POSITIVE: {
            set_motor_command(DriverState{ 
                .motor_outputs = MotorOutputs{ 
                    .enable_flags = enable_flags_all, 
                    .u_duty = static_cast<uint16_t>(faster_abs(command.value))
                },
                .mode = DriverMode::HOLD,
                .duration = command.timeout, 
            });
            return false;
        }
        case SET_STATE_HOLD_V_POSITIVE: {
            set_motor_command(DriverState{ 
                .motor_outputs = MotorOutputs{ 
                    .enable_flags = enable_flags_all, 
                    .v_duty = static_cast<uint16_t>(faster_abs(command.value))
                },
                .mode = DriverMode::HOLD,
                .duration = command.timeout, 
            });
            return false;
        }
        case SET_STATE_HOLD_W_POSITIVE: {
            set_motor_command(DriverState{ 
                .motor_outputs = MotorOutputs{ 
                    .enable_flags = enable_flags_all, 
                    .w_duty = static_cast<uint16_t>(faster_abs(command.value))
                },
                .mode = DriverMode::HOLD,
                .duration = command.timeout, 
            });
            return false;
        }
        case SET_STATE_HOLD_U_NEGATIVE: {
            set_motor_command(DriverState{ 
                .motor_outputs = MotorOutputs{ 
                    .enable_flags = enable_flags_all, 
                    .v_duty = static_cast<uint16_t>(faster_abs(command.value)),
                    .w_duty = static_cast<uint16_t>(faster_abs(command.value))
                },
                .mode = DriverMode::HOLD,
                .duration = command.timeout, 
            });
            return false;
        }
        case SET_STATE_HOLD_V_NEGATIVE: {
            set_motor_command(DriverState{ 
                .motor_outputs = MotorOutputs{ 
                    .enable_flags = enable_flags_all, 
                    .u_duty = static_cast<uint16_t>(faster_abs(command.value)),
                    .w_duty = static_cast<uint16_t>(faster_abs(command.value))
                },
                .mode = DriverMode::HOLD,
                .duration = command.timeout, 
            });
            return false;
        }
        case SET_STATE_HOLD_W_NEGATIVE: {

            set_motor_command(DriverState{
                .motor_outputs = MotorOutputs{ 
                    .enable_flags = enable_flags_all, 
                    .u_duty = static_cast<uint16_t>(faster_abs(command.value)),
                    .v_duty = static_cast<uint16_t>(faster_abs(command.value))
                },
                .mode = DriverMode::HOLD,
                .duration = command.timeout, 
            });
            return false;
        }
        case SET_CURRENT_FACTORS:
            current_calibration = parse_current_calibration(buffer.data, buffer.write_index);
            usb_reply_current_factors = true;
            return false;
            
        case RESET_CURRENT_FACTORS:
            // Reset the current factors to the default values.
            current_calibration = default_current_calibration;
            usb_reply_current_factors = true;
            return false;

        case SET_HALL_POSITIONS:
            position_calibration = parse_position_calibration(buffer.data, buffer.write_index);
            usb_reply_hall_positions = true;
            return false;

        case RESET_HALL_POSITIONS:
            // Reset the hall positions to the default values.
            position_calibration = default_position_calibration;
            usb_reply_hall_positions = true;
            return false;

        case SET_CONTROL_PARAMETERS:
            control_parameters = parse_control_parameters(buffer.data, buffer.write_index);
            usb_reply_control_parameters = true;
            return false;

        case RESET_CONTROL_PARAMETERS:
            control_parameters = default_control_parameters;
            usb_reply_control_parameters = true;
            return false;

        case GET_CURRENT_FACTORS:
            usb_reply_current_factors = true;
            return false;
        case GET_HALL_POSITIONS:
            usb_reply_hall_positions = true;
            return false;

        case GET_CONTROL_PARAMETERS:
            usb_reply_control_parameters = true;
            return false;

        case SET_ANGLE: {
            set_angle(command.value);
            return false;
        }

        case SAVE_SETTINGS_TO_FLASH:
            if(is_motor_safed()){
                save_settings_to_flash(current_calibration, position_calibration, control_parameters);

                current_calibration = get_current_calibration();
                position_calibration = get_position_calibration();
                control_parameters = get_control_parameters();
                
                return false;
            } else {
                return true;
            }

        // We shouldn't receive these messages; the driver only sends them.
        case CURRENT_FACTORS:
        case CONTROL_PARAMETERS:
        case HALL_POSITIONS:
        case READOUT:
        case FULL_READOUT:
        case UNIT_TEST_OUTPUT:
            return true;

        case RUN_UNIT_TEST_FUNKY_ATAN:
            return run_unit_test(unit_test_funky_atan);
        case RUN_UNIT_TEST_FUNKY_ATAN_PART_2:
            return run_unit_test(unit_test_funky_atan_part_2);
        case RUN_UNIT_TEST_FUNKY_ATAN_PART_3:
            return run_unit_test(unit_test_funky_atan_part_3);
    }

    // If we get here, we have an unknown command code.
    return true;
}


inline Readout adjust_direction(Readout && readout){
    readout.angle = normalize_angle(control_parameters.motor_direction * readout.angle);
    readout.angle_adjustment = control_parameters.motor_direction * readout.angle_adjustment;
    readout.angular_speed = control_parameters.motor_direction * readout.angular_speed;
    return readout;
}

void usb_onreset(){
    usb_response_buffer = {};
    usb_receive_buffer = {};
    usb_chunk_receive_time = 0;
    usb_stream_state = 0;
}

void usb_serialize_response(MessageBuffer & usb_response_buffer, FullReadout const& readout) {
    if (usb_response_buffer.write_index != 0) {
        // We have unsent data in the response buffer; don't send more data until that is done.
        return;
    }
    
    // Check if we have to wait for the queue to fill before sending readouts.
    // This is used to prevent sending readouts while the motor is commutating.
    if (usb_wait_full_history) {
        if(readout_history_full()){
            // The queue is full; we can start sending readouts.
            usb_wait_full_history = false;
        } else {
            // We have not filled the queue yet; don't send readouts.
            return;
        }
    }

    // Stream readouts but only once per readout number.
    if(usb_stream_state and usb_stream_last_sent != readout.readout_number){
        usb_response_buffer.write_index = write_full_readout(usb_response_buffer.data, readout);
        usb_stream_last_sent = readout.readout_number;
    }

    // Send unit test response if requested.
    if (usb_reply_unit_test) {
        usb_response_buffer.write_index = write_unit_test(usb_response_buffer.data, usb_unit_test_function);
        usb_reply_unit_test = false;
    }

    // Send control parameters if requested.
    if (usb_reply_control_parameters) {
        usb_response_buffer.write_index = write_control_parameters(usb_response_buffer.data, control_parameters);
        usb_reply_control_parameters = false;
    }

    // Send current factors if requested.
    if (usb_reply_current_factors) {
        usb_response_buffer.write_index = write_current_calibration(usb_response_buffer.data, current_calibration);
        usb_reply_current_factors = false;
    }

    // Send trigger angles if requested.
    if (usb_reply_hall_positions) {
        usb_response_buffer.write_index = write_position_calibration(usb_response_buffer.data, position_calibration);
        usb_reply_hall_positions = false;
    }

    // Queue the readout history to the send buffer.
    if(usb_readouts_to_send > 0){
        // Stop if we have caught up to the readout history.
        if (not readout_history_available()) return readout_history_reset();
        
        // Send the readout to the host.
        usb_response_buffer.write_index = write_readout(usb_response_buffer.data, adjust_direction(readout_history_pop()));

        // Readout added to the USB buffer.
        usb_readouts_to_send -= 1;
    }
}

void usb_receive_data(uint8_t * rx_buffer, int rx_size) {
    // We need to read at least the message header to determine the size.
    const size_t total_data_received = usb_receive_buffer.write_index + rx_size;

    // We have a partial message that has timed out; reset the buffer.
    if (usb_receive_buffer.write_index > 0 and 
        (HAL_GetTick() - usb_chunk_receive_time) > usb_partial_message_timeout_ms) {
        usb_onreset();
        return;
    }
    
    // Copy the partial header if we don't have enough data yet.
    if (total_data_received < header_size) {
        // Not enough data to determine message size; just copy the data.
        std::memcpy(
            &usb_receive_buffer.data[usb_receive_buffer.write_index],
            rx_buffer, 
            rx_size
        );
        usb_chunk_receive_time = HAL_GetTick();
        usb_receive_buffer.write_index += rx_size;
        return;
    }

    // Copy until we have a full header.
    const size_t header_bytes_needed = header_size - usb_receive_buffer.write_index;
    std::memcpy(
        &usb_receive_buffer.data[usb_receive_buffer.write_index], 
        rx_buffer, 
        header_bytes_needed
    );
    usb_chunk_receive_time = HAL_GetTick();
    usb_receive_buffer.write_index = header_size;

    // The first number is the command code, the remainder is the command data.
    const uint16_t code = read_uint16(usb_receive_buffer.data);

    const size_t expected_size = get_message_size(code);

    if (not expected_size) {
        // Invalid command code; reset the USB buffers.
        usb_onreset();
        return;
    }
    
    if (total_data_received < expected_size) {
        std::memcpy(
            &usb_receive_buffer.data[usb_receive_buffer.write_index], 
            &rx_buffer[header_bytes_needed], 
            rx_size - header_bytes_needed
        );
        usb_chunk_receive_time = HAL_GetTick();
        usb_receive_buffer.write_index += rx_size - header_bytes_needed;
        return;
    }

    // We have a complete message, copy the remaining data.
    const size_t remaining_message_bytes = expected_size - usb_receive_buffer.write_index;
    std::memcpy(
        &usb_receive_buffer.data[usb_receive_buffer.write_index], 
        &rx_buffer[header_bytes_needed], 
        remaining_message_bytes
    );
    usb_receive_buffer.write_index += remaining_message_bytes;

    // Handle the complete command.
    if(handle_command(usb_receive_buffer)){
        // Invalid command; reset the USB buffers.
        usb_onreset();
        return;
    }
    
    // Clear the USB receive buffer for the next message.
    usb_receive_buffer.write_index = 0;
    usb_chunk_receive_time = 0;
        
    const size_t remaining_rx_bytes = rx_size - (header_bytes_needed + remaining_message_bytes);
    if (remaining_rx_bytes > 0) {
        // There is more data in the RX buffer; process it recursively.
        usb_receive_data(&rx_buffer[header_bytes_needed + remaining_message_bytes], remaining_rx_bytes);
    }
}


void app_tick() {
    main_loop_number += 1;

    FullReadout readout = get_readout();

    // Timing
    // ------

    // Update timing information.
    const uint32_t milliseconds = HAL_GetTick();

    const uint32_t duration_since_timing_update = milliseconds - last_update_time_millis;
    if (duration_since_timing_update > min_timing_period_millis) {
        last_update_time_millis = milliseconds;
        
        float seconds = duration_since_timing_update / 1000.f;

        main_loop_rate = (main_loop_number - last_loop_number) / seconds;
        adc_update_rate = ((readout_number_base + readout.readout_number - last_readout_number) % readout_number_base) / seconds;

        last_loop_number = main_loop_number;
        last_readout_number = readout.readout_number;
    }

    
    // Calculate the motor constant
    // ----------------------------
    // 
    // The motor constant is the ratio of the EMF voltage to the angular speed (in radians per second).
    // 
    // It is also the ratio between the torque produced by the motor and the quadrature current. We 
    // can compute the motor constant from the a spinning motor and use it to estimate our torque.
    // 
    // We calculate the motor constant by gradient descent using the configured integral gain.

    // The voltage magnitude is always positive, also use the positive angular speed.
    const int abs_angular_speed = faster_abs(readout.angular_speed);

    const int predicted_emf_voltage = abs_angular_speed * readout.motor_constant / emf_motor_constant_conversion;

    const bool angle_fix = readout.state_flags & angle_fix_bit_mask;

    // Only compute the motor constant if we have a valid angle and the EMF voltage is above the threshold where noise is low.
    const bool compute_motor_constant = angle_fix and (readout.emf_voltage_magnitude > control_parameters.min_emf_for_motor_constant);

    // Get the error (gradient) for the motor constant observer.
    const int motor_constant_error = compute_motor_constant * (readout.emf_voltage_magnitude - predicted_emf_voltage);

    // Adjust the high resolution observer for the motor constant.
    motor_constant_observer += motor_constant_error * control_parameters.motor_constant_ki;

    const int motor_constant = motor_constant_observer / hires_fixed_point;


    // Write all values
    // ----------------
    
    readout.main_loop_rate = static_cast<int>(main_loop_rate);
    readout.adc_update_rate = static_cast<int>(adc_update_rate);
    readout.motor_constant = motor_constant;

    // Adjust direction
    // ----------------

    readout.angle = normalize_angle(control_parameters.motor_direction * readout.angle);
    readout.angle_adjustment = control_parameters.motor_direction * readout.angle_adjustment;
    readout.angular_speed = control_parameters.motor_direction * readout.angular_speed;

    readout.rotations = control_parameters.motor_direction * readout.rotations;
    readout.rotor_acceleration = control_parameters.motor_direction * readout.rotor_acceleration;


    // USB comms
    // ---------

    // Queue the state readouts on the USB buffer.
    usb_serialize_response(usb_response_buffer, readout);

    if(usb_update(
        usb_response_buffer.data, usb_response_buffer.write_index, 
        usb_receive_data, usb_onreset)
    ){
        // Data was sent; clear the response buffer.
        usb_response_buffer.write_index = 0;
    };
    
}



