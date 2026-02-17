#include "app_main.hpp"

#include "hex_mini_drive/byte_handling.hpp"
#include "hex_mini_drive/interface.hpp"
#include "hex_mini_drive/cobs_encoding.hpp"

#include "parameters_store.hpp"
#include "test_schedules.hpp"
#include "interrupts.hpp"
#include "interrupts_data.hpp"


#include "integer_math.hpp"
#include "error_handler.hpp"
#include "constants.hpp"
#include "io.hpp"
#include "type_definitions.hpp"
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

hex_mini_drive::COBS_Buffer usb_encoding_buffer = {};

uint32_t usb_chunk_receive_time = 0;
uint32_t usb_bytes_discarded = 0;
uint32_t usb_last_sent_time = 0;

const uint32_t usb_partial_message_timeout_ms = 500;
const uint32_t usb_sending_timeout_ms = 500;

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
void handle_message(hex_mini_drive::Message const& message) {

    using namespace hex_mini_drive;

    switch (message.message_code) {
        case NULL_MESSAGE_CODE:
            // No command received; ignore it.
            return;

        case STREAM_FULL_READOUTS: {
            // Continuously stream data if timeout > 0.
            usb_stream_state = std::get<StreamFullReadouts>(message.message_data).stream_state;
            // Also stop the motor if we stop the stream.
            if (not usb_stream_state) set_motor_command(DriverState{.mode = DriverMode::OFF});
            return;
        }
        case GET_READOUTS_SNAPSHOT:
            // Cancel streaming; so we can take a data snapshot without interruptions.
            usb_stream_state = 0;
            
            readout_history_reset();
            // Dissalow sending until we fill the queue, so it doesn't interrupt commutation.
            usb_wait_full_history = true;
            usb_readouts_to_send = history_size;
            return;

        // Turn off the motor driver.
        case SET_STATE_OFF:
            // Repeat command, with the interrupt guards.
            set_motor_command(DriverState{.mode = DriverMode::OFF});
            return;
            
        // Measure the motor phase currents.
        
        case SET_STATE_TEST_ALL_PERMUTATIONS:
            motor_start_test(
                test_all_permutations, 
                std::get<TestCommand>(message.message_data).pwm_value, 
                std::get<TestCommand>(message.message_data).take_snapshot > 0
            );
            return;
        case SET_STATE_TEST_GROUND_SHORT:
            motor_start_test(
                test_ground_short, 
                std::get<TestCommand>(message.message_data).pwm_value, 
                std::get<TestCommand>(message.message_data).take_snapshot > 0
            );
            return;
        case SET_STATE_TEST_POSITIVE_SHORT:
            motor_start_test(
                test_positive_short, 
                std::get<TestCommand>(message.message_data).pwm_value, 
                std::get<TestCommand>(message.message_data).take_snapshot > 0
            );
            return;
        case SET_STATE_TEST_U_DIRECTIONS:
            motor_start_test(
                test_u_directions,
                std::get<TestCommand>(message.message_data).pwm_value,
                std::get<TestCommand>(message.message_data).take_snapshot > 0
            );
            return;

        case SET_STATE_TEST_U_INCREASING:
            motor_start_test(
                test_u_increasing, 
                std::get<TestCommand>(message.message_data).pwm_value, 
                std::get<TestCommand>(message.message_data).take_snapshot > 0
            );
            return;
        case SET_STATE_TEST_U_DECREASING:
            motor_start_test(
                test_u_decreasing, 
                std::get<TestCommand>(message.message_data).pwm_value, 
                std::get<TestCommand>(message.message_data).take_snapshot > 0
            );
            return;
        case SET_STATE_TEST_V_INCREASING:
            motor_start_test(
                test_v_increasing, 
                std::get<TestCommand>(message.message_data).pwm_value, 
                std::get<TestCommand>(message.message_data).take_snapshot > 0
            );
            return;
        case SET_STATE_TEST_V_DECREASING:
            motor_start_test(
                test_v_decreasing, 
                std::get<TestCommand>(message.message_data).pwm_value, 
                std::get<TestCommand>(message.message_data).take_snapshot > 0
            );
            return;
        case SET_STATE_TEST_W_INCREASING:
            motor_start_test(
                test_w_increasing, 
                std::get<TestCommand>(message.message_data).pwm_value, 
                std::get<TestCommand>(message.message_data).take_snapshot > 0
            );
            return;
        case SET_STATE_TEST_W_DECREASING:
            motor_start_test(
                test_w_decreasing, 
                std::get<TestCommand>(message.message_data).pwm_value, 
                std::get<TestCommand>(message.message_data).take_snapshot > 0
            );
            return;

        // Drive the motor.
        case SET_STATE_DRIVE_6_SECTOR: {
            set_motor_command(DriverState{ 
                .mode = DriverMode::DRIVE_6_SECTOR, 
                .duration = std::get<BasicDriveCommand>(message.message_data).timeout, 
                .active_pwm = clip_to_short(-pwm_max, +pwm_max, std::get<BasicDriveCommand>(message.message_data).pwm_value * control_parameters.motor_direction),
            });
            return;
        }

        case SET_STATE_DRIVE_PERIODIC: {
            set_motor_command(DriverState{
                .mode = DriverMode::DRIVE_PERIODIC, 
                .duration = std::get<SetStateDrivePeriodic>(message.message_data).timeout,
                .active_angle = std::get<SetStateDrivePeriodic>(message.message_data).angle,
                .active_pwm = clip_to_short(0, +pwm_max, std::get<SetStateDrivePeriodic>(message.message_data).pwm_value),
                .angular_speed = clip_to_short(-max_angular_speed, +max_angular_speed, std::get<SetStateDrivePeriodic>(message.message_data).angular_speed * control_parameters.motor_direction),
            });
            return;
        }

        case SET_STATE_DRIVE_SMOOTH: {
            set_motor_command(DriverState{
                .mode = DriverMode::DRIVE_SMOOTH, 
                .duration = std::get<SetStateDriveSmooth>(message.message_data).timeout, 
                .target_pwm = clip_to_short(-pwm_max, +pwm_max, std::get<SetStateDriveSmooth>(message.message_data).pwm_value * control_parameters.motor_direction),
            });
            return;
        }

        case SET_STATE_DRIVE_TORQUE: {
            set_motor_command(DriverState{
                .mode = DriverMode::DRIVE_TORQUE, 
                .duration = std::get<SetStateDriveTorque>(message.message_data).timeout,
                .secondary_target = clip_to_short(-max_drive_current, +max_drive_current, std::get<SetStateDriveTorque>(message.message_data).target_current * control_parameters.motor_direction),
            });
            return;
        }

        case SET_STATE_DRIVE_BATTERY_POWER: {
            set_motor_command(DriverState{
                .mode = DriverMode::DRIVE_BATTERY_POWER, 
                .duration = std::get<SetStateDriveBatteryPower>(message.message_data).timeout,
                .secondary_target = clip_to_short(-max_drive_power, +max_drive_power, std::get<SetStateDriveBatteryPower>(message.message_data).target_power * control_parameters.motor_direction),
            });
            return;
        }

        case SET_STATE_DRIVE_SPEED: {
            set_motor_command(DriverState{
                .mode = DriverMode::DRIVE_SPEED, 
                .duration = std::get<SetStateDriveSpeed>(message.message_data).timeout,
                .secondary_target = clip_to_short(-max_angular_speed, +max_angular_speed, std::get<SetStateDriveSpeed>(message.message_data).target_speed * control_parameters.motor_direction),
            });
            return;
        }

        case SET_STATE_SEEK_ANGLE_WITH_POWER: {
            set_motor_command(DriverState{
                .mode = DriverMode::SEEK_ANGLE_POWER, 
                .duration = std::get<SetStateSeekAngleWithPower>(message.message_data).timeout,
                .seek_angle = SeekAngle{
                    .target_rotation = clip_to_short(-max_16bit, +max_16bit, std::get<SetStateSeekAngleWithPower>(message.message_data).target_rotation * control_parameters.motor_direction),
                    .target_angle = static_cast<int16_t>(normalize_angle(std::get<SetStateSeekAngleWithPower>(message.message_data).target_angle * control_parameters.motor_direction)),
                    .max_secondary_target = clip_to_short(0, max_drive_power, std::get<SetStateSeekAngleWithPower>(message.message_data).max_drive_power),
                }
            });
            return;
        }

        case SET_STATE_SEEK_ANGLE_WITH_TORQUE: {
            set_motor_command(DriverState{
                .mode = DriverMode::SEEK_ANGLE_TORQUE, 
                .duration = std::get<SetStateSeekAngleWithTorque>(message.message_data).timeout,
                .seek_angle = SeekAngle{
                    .target_rotation = clip_to_short(-max_16bit, +max_16bit, std::get<SetStateSeekAngleWithTorque>(message.message_data).target_rotation * control_parameters.motor_direction),
                    .target_angle = static_cast<int16_t>(normalize_angle(std::get<SetStateSeekAngleWithTorque>(message.message_data).target_angle * control_parameters.motor_direction)),
                    .max_secondary_target = clip_to_short(0, max_drive_current, std::get<SetStateSeekAngleWithTorque>(message.message_data).max_drive_current),
                }
            });
            return;
        }

        case SET_STATE_SEEK_ANGLE_WITH_SPEED: {
            set_motor_command(DriverState{
                .mode = DriverMode::SEEK_ANGLE_SPEED, 
                .duration = std::get<SetStateSeekAngleWithSpeed>(message.message_data).timeout,
                .seek_angle = SeekAngle{
                    .target_rotation = clip_to_short(-max_16bit, +max_16bit, std::get<SetStateSeekAngleWithSpeed>(message.message_data).target_rotation * control_parameters.motor_direction),
                    .target_angle = static_cast<int16_t>(normalize_angle(std::get<SetStateSeekAngleWithSpeed>(message.message_data).target_angle * control_parameters.motor_direction)),
                    .max_secondary_target = clip_to_short(0, max_angular_speed, std::get<SetStateSeekAngleWithSpeed>(message.message_data).max_drive_speed),
                }
            });
            return;
        }

        // Freewheel the motor.
        case SET_STATE_FREEWHEEL:
            set_motor_command(DriverState{ .mode = DriverMode::FREEWHEEL });
            return;

        case SET_STATE_HOLD_U_POSITIVE: {
            set_motor_command(DriverState{ 
                .motor_outputs = MotorOutputs{ 
                    .enable_flags = enable_flags_all, 
                    .u_duty = static_cast<uint16_t>(faster_abs(std::get<HoldCommand>(message.message_data).pwm_value))
                },
                .mode = DriverMode::HOLD,
                .duration = std::get<HoldCommand>(message.message_data).timeout, 
            });
            return;
        }
        case SET_STATE_HOLD_V_POSITIVE: {
            set_motor_command(DriverState{ 
                .motor_outputs = MotorOutputs{ 
                    .enable_flags = enable_flags_all, 
                    .v_duty = static_cast<uint16_t>(faster_abs(std::get<HoldCommand>(message.message_data).pwm_value))
                },
                .mode = DriverMode::HOLD,
                .duration = std::get<HoldCommand>(message.message_data).timeout, 
            });
            return;
        }
        case SET_STATE_HOLD_W_POSITIVE: {
            set_motor_command(DriverState{ 
                .motor_outputs = MotorOutputs{ 
                    .enable_flags = enable_flags_all, 
                    .w_duty = static_cast<uint16_t>(faster_abs(std::get<HoldCommand>(message.message_data).pwm_value))
                },
                .mode = DriverMode::HOLD,
                .duration = std::get<HoldCommand>(message.message_data).timeout, 
            });
            return;
        }
        case SET_STATE_HOLD_U_NEGATIVE: {
            set_motor_command(DriverState{ 
                .motor_outputs = MotorOutputs{ 
                    .enable_flags = enable_flags_all, 
                    .v_duty = static_cast<uint16_t>(faster_abs(std::get<HoldCommand>(message.message_data).pwm_value)),
                    .w_duty = static_cast<uint16_t>(faster_abs(std::get<HoldCommand>(message.message_data).pwm_value))
                },
                .mode = DriverMode::HOLD,
                .duration = std::get<HoldCommand>(message.message_data).timeout, 
            });
            return;
        }
        case SET_STATE_HOLD_V_NEGATIVE: {
            set_motor_command(DriverState{ 
                .motor_outputs = MotorOutputs{ 
                    .enable_flags = enable_flags_all, 
                    .u_duty = static_cast<uint16_t>(faster_abs(std::get<HoldCommand>(message.message_data).pwm_value)),
                    .w_duty = static_cast<uint16_t>(faster_abs(std::get<HoldCommand>(message.message_data).pwm_value))
                },
                .mode = DriverMode::HOLD,
                .duration = std::get<HoldCommand>(message.message_data).timeout, 
            });
            return;
        }
        case SET_STATE_HOLD_W_NEGATIVE: {

            set_motor_command(DriverState{
                .motor_outputs = MotorOutputs{ 
                    .enable_flags = enable_flags_all, 
                    .u_duty = static_cast<uint16_t>(faster_abs(std::get<HoldCommand>(message.message_data).pwm_value)),
                    .v_duty = static_cast<uint16_t>(faster_abs(std::get<HoldCommand>(message.message_data).pwm_value))
                },
                .mode = DriverMode::HOLD,
                .duration = std::get<HoldCommand>(message.message_data).timeout, 
            });
            return;
        }
        case SET_CURRENT_CALIBRATION:
            current_calibration = std::get<CurrentCalibration>(message.message_data);
            usb_reply_current_factors = true;
            return;
            
        case RESET_CURRENT_CALIBRATION:
            // Reset the current factors to the default values.
            current_calibration = default_current_calibration;
            usb_reply_current_factors = true;
            return;

        case SET_HALL_POSITIONS:
            position_calibration = std::get<HallPositions>(message.message_data);
            usb_reply_hall_positions = true;
            return;

        case RESET_HALL_POSITIONS:
            // Reset the hall positions to the default values.
            position_calibration = default_position_calibration;
            usb_reply_hall_positions = true;
            return;

        case SET_CONTROL_PARAMETERS:
            control_parameters = std::get<ControlParameters>(message.message_data);
            usb_reply_control_parameters = true;
            return;

        case RESET_CONTROL_PARAMETERS:
            control_parameters = default_control_parameters;
            usb_reply_control_parameters = true;
            return;

        case GET_CURRENT_CALIBRATION:
            usb_reply_current_factors = true;
            return;
        case GET_HALL_POSITIONS:
            usb_reply_hall_positions = true;
            return;

        case GET_CONTROL_PARAMETERS:
            usb_reply_control_parameters = true;
            return;

        case SET_ANGLE: {
            set_angle(std::get<SetAngle>(message.message_data).angle);
            return;
        }

        case SAVE_SETTINGS_TO_FLASH:
            if(is_motor_safed()){
                save_settings_to_flash(current_calibration, position_calibration, control_parameters);

                current_calibration = get_current_calibration();
                position_calibration = get_position_calibration();
                control_parameters = get_control_parameters();
                
                return;
            }

        // We shouldn't receive these messages; the driver only sends them.
        case CURRENT_CALIBRATION:
        case CONTROL_PARAMETERS:
        case HALL_POSITIONS:
        case READOUT:
        case FULL_READOUT:
        case UNIT_TEST_OUTPUT:
            return;

        case RUN_UNIT_TEST_FUNKY_ATAN:
            run_unit_test(unit_test_funky_atan);
            return;
        case RUN_UNIT_TEST_FUNKY_ATAN_PART2:
            run_unit_test(unit_test_funky_atan_part_2);
            return;
        case RUN_UNIT_TEST_FUNKY_ATAN_PART3:
            run_unit_test(unit_test_funky_atan_part_3);
            return;

        }
}


inline hex_mini_drive::Readout adjust_direction(hex_mini_drive::Readout && readout){
    readout.angle = normalize_angle(control_parameters.motor_direction * readout.angle);
    readout.angle_adjustment = control_parameters.motor_direction * readout.angle_adjustment;
    readout.angular_speed = control_parameters.motor_direction * readout.angular_speed;
    return readout;
}

void usb_reset_buffers(){
    usb_encoding_buffer.encode_reset();
    usb_encoding_buffer.decode_reset();
    usb_chunk_receive_time = 0;
    usb_stream_state = 0;
}


void usb_queue_message(hex_mini_drive::Message const& message) {
    hex_mini_drive::MessageBuffer buffer;
    buffer.size = hex_mini_drive::write_message(buffer.data, hex_mini_drive::max_message_size, message);

    if (buffer.size) usb_encoding_buffer.encode_message(buffer);
}

void usb_serialize_response(hex_mini_drive::COBS_Buffer & usb_encoding_buffer, hex_mini_drive::FullReadout const& readout) {
    // Wait until the previous message has been sent or buffers reset before encoding a new message.
    if (usb_encoding_buffer.is_message_encoded()) return;

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

    // Queue the readout history to the send buffer.
    if(usb_readouts_to_send > 0){
        // Stop if we have caught up to the readout history.
        if (not readout_history_available()) return readout_history_reset();


        usb_queue_message(hex_mini_drive::Message{
            .message_code = hex_mini_drive::MessageCode::READOUT,
            .message_data = adjust_direction(readout_history_pop())
        });
        
        // Readout added to the USB buffer.
        usb_readouts_to_send -= 1;

        return;
    }

    // Stream readouts but only once per readout number.
    if(usb_stream_state and usb_stream_last_sent != readout.readout_number){
        usb_queue_message(hex_mini_drive::Message{
            .message_code = hex_mini_drive::MessageCode::FULL_READOUT,
            .message_data = readout
        });

        usb_stream_last_sent = readout.readout_number;
        return;
    }

    // Send unit test response if requested.
    if (usb_reply_unit_test) {
        hex_mini_drive::Message message = {
            .message_code = hex_mini_drive::MessageCode::UNIT_TEST_OUTPUT,
            .message_data = hex_mini_drive::UnitTestOutput{}
        };
        usb_unit_test_function(
            reinterpret_cast<char*>(std::get<hex_mini_drive::UnitTestOutput>(message.message_data).data()), 
            hex_mini_drive::UNIT_TEST_OUTPUT_SIZE
        );
        usb_queue_message(message);
        usb_reply_unit_test = false;
        return;
    }

    // Send control parameters if requested.
    if (usb_reply_control_parameters) {
        usb_queue_message(hex_mini_drive::Message{
            .message_code = hex_mini_drive::MessageCode::CONTROL_PARAMETERS,
            .message_data = control_parameters
        });
        usb_reply_control_parameters = false;
        return;
    }

    // Send current factors if requested.
    if (usb_reply_current_factors) {
        usb_queue_message(hex_mini_drive::Message{
            .message_code = hex_mini_drive::MessageCode::CURRENT_CALIBRATION,
            .message_data = current_calibration
        });
        usb_reply_current_factors = false;
        return;
    }

    // Send trigger angles if requested.
    if (usb_reply_hall_positions) {
        usb_queue_message(hex_mini_drive::Message{
            .message_code = hex_mini_drive::MessageCode::HALL_POSITIONS,
            .message_data = position_calibration
        });
        usb_reply_hall_positions = false;
        return;
    }
}

void usb_receive_serialised_message(uint8_t * buffer, size_t size){
    hex_mini_drive::Message message;
    if(hex_mini_drive::read_message(message, buffer, size)){
        // We have a valid message; handle it.
        handle_message(message);
    } else {
        // Invalid message; ignore it.
        usb_bytes_discarded += size;
    }
} 

void usb_receive_data(uint8_t * rx_buffer, size_t rx_size) {
    // We have a partial message that has timed out; reset the buffer and continue 
    // reading as a fresh message.
    if (usb_encoding_buffer.decode_ongoing() and 
        (HAL_GetTick() - usb_chunk_receive_time) > usb_partial_message_timeout_ms) {
        usb_reset_buffers();
    }

    usb_encoding_buffer.decode_chunk(rx_buffer, rx_size, usb_receive_serialised_message);
    usb_chunk_receive_time = HAL_GetTick();
}


void app_tick() {
    main_loop_number += 1;

    hex_mini_drive::FullReadout readout = get_readout();

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
    usb_serialize_response(usb_encoding_buffer, readout);

    if(usb_update(
        usb_encoding_buffer.encoding_buffer.data, usb_encoding_buffer.encoding_buffer.size, 
        usb_receive_data)
    ){
        // Data was sent; clear the response buffer.
        usb_encoding_buffer.encode_reset();
        usb_last_sent_time = HAL_GetTick();
    } else if (usb_encoding_buffer.is_message_encoded()) {
        // We have data to send but the USB queue is busy.
        if(HAL_GetTick() - usb_last_sent_time > usb_sending_timeout_ms){
            // Timeout waiting for USB to send; reset the USB buffers.
            usb_reset_buffers();
            // And the other USB buffers.
            usb_reset();
        }
    }
    
}



