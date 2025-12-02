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

#include "usbd_cdc_if.h"

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

// Maximum time to wait for a USB packet to be sent.
const uint32_t USB_TIMEOUT = 500;


int32_t motor_constant_observer = 0;


// Setup ADC for reading motor phase currents.
void adc_init(){

    // Enable the ADC interrupt for the end of the injected sequence.
    LL_ADC_EnableIT_JEOS(ADC1);
    // Disable the ADC interrupt for the end of the regular sequence; we are not using it.
    LL_ADC_DisableIT_EOS(ADC1);

    // Enable the ADCs and wait for them to settle.
    LL_ADC_Enable(ADC1);
    LL_ADC_Enable(ADC2);
    HAL_Delay(100);

    // Calibrate the ADCs.
    LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC1)) {}
    LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC2)) {}

    // Start ADC conversions from the external trigger in TIM1.
    // TODO: recheck timing of conversions; it was set on the rising trigger
    LL_ADC_REG_StartConversion(ADC1);
    LL_ADC_REG_StartConversion(ADC2);

    // Gotta enable ext triggers for the second channel to work in dual mode.
    LL_ADC_REG_StartConversion(ADC2);
}

// Select which interrupts to handle.
void interrupts_init(){

    // TIM1 is used to generate the PWM signals for the motor phases, and to trigger the ADC.
    // 
    // Note that we don't need to set dead-time at this point; the gate driver FD6288T&Q has a 
    // built-in dead-time of 100-300ns. The MOSFETs AO4266E have turn on rise time of 8ns and 
    // turn off fall time of 22ns so we should be covered.

    // Disable TIM1 update interrupt; occuring every PWM cycle. We will use the ADC interrupt to update the motor control.
    LL_TIM_DisableIT_UPDATE(TIM1);
    // Disable TIM1 trigger interrupt; it's only triggered during resets.
    LL_TIM_DisableIT_TRIG(TIM1);
    // Disable commutation interrupts; 
    LL_TIM_DisableIT_COM(TIM1);


    // Disable TIM2 update interrupt. Count updates of TIM2 to measure slow motor 
    // speed in case the timer overflows before a toggle in the hall sensor.
    LL_TIM_DisableIT_UPDATE(TIM2);
    // Disable TIM2 trigger interrupt. It is triggered every reset and hall sensor toggle.
    LL_TIM_DisableIT_TRIG(TIM2);
    // Enable TIM2 channel 1 interrupt. It is triggered every hall sensor toggle; we can 
    // read the time from the last time the input changed.
    LL_TIM_DisableIT_CC1(TIM2);
    // Disable TIM2 channel 2 interrupt. We are ignoring the delayed trigger output from 
    // TIM2 as we will commutate faster than the hall sensor toggles. To use this effectively
    // we need to use the prescaler to slow the timer down; but then we lose resolution.
    LL_TIM_DisableIT_CC2(TIM2);

    // Disable TIM1 capture/compare update selection preload. This is used to update the
    // output modes of all channels at the same time when a commutation event is triggered.
    // We don't need this as we always enable PWM mode and update the compare register to 
    // modulate the PWM duty cycle.
    // 
    // This setting is mostly here to document that in order to trigger commutations on TRGI 
    // we also need to enable preload for the motor ouput channel mode registers.
    LL_TIM_CC_DisablePreload(TIM1);

    // Don't allow TIM1 commutations to be triggered by TIM2 hall sensor toggles.
    LL_TIM_CC_SetUpdate(TIM1, LL_TIM_CCUPDATESOURCE_COMG_ONLY);
    
}

// Enable the timers to start the ADC and motor control loops.
void enable_timers(){
    // We use the ADC1 and ADC2 in simulatenous mode to read the motor phase currents
    // around the PWM cycle up to down transition (when the ground mosfet is on). When 
    // the ADC is triggered in injected mode both ADC modules read the current for the 
    // U and V phases first then W and reference voltage second.
    // 
    // The ADC sample time is 20cycles, so the total sampling period is 20/12MHz = 1.67us.
    // 
    // Use the TIM1 channel 4 to generate an event a short time before the counter reaches
    // the auto reload value. This event triggers the ADC to read the motor phase currents.
    const uint16_t injected_conversion_start = pwm_base - sample_lead_time;
    LL_TIM_OC_SetCompareCH4(TIM1, injected_conversion_start);

    // It appears that the countermode is being ignored for the external ADC triggering.
    if (LL_TIM_GetCounterMode(TIM1) != LL_TIM_COUNTERMODE_CENTER_DOWN) error();
    // Invert the PWM mode of channel 4 instead so we emit a rising edge during up counting.
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM2);

    // Reinitialize the timers; reset the counters and update registers. Because the timers
    // are setup with preload registers the values we write to them are stored in shadow registers
    // and only applied when the update event occurs (to any individual PWM cycle consistent). We
    // need to generate an update event to force the timers to load the new values.
    LL_TIM_GenerateEvent_UPDATE(TIM1);
    LL_TIM_GenerateEvent_UPDATE(TIM2);
    LL_TIM_GenerateEvent_UPDATE(TIM3);

    // Start the timers.
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_EnableCounter(TIM2);
    LL_TIM_EnableCounter(TIM3);
    
    // Enable TIM1 channel 4 used to trigger the ADC.
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);
    
    // Enable the TIM1 outputs following a break or clock issue event.
    LL_TIM_EnableAllOutputs(TIM1);
    
    // Enable TIM2 channels 1 as the hall sensor timer between commutations.
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
}

void app_init() {

    // Setup PWM settings.
    LL_TIM_SetAutoReload(TIM1, pwm_autoreload); // 72MHz / 1536 / 2 = 23.4KHz

    // Set which interrupts to handle.
    interrupts_init();
    
    // Setup the ADC for reading motor phase currents, temperature and voltage.
    // This function will run an initial calibration of the chip's ADCs (takes a few ms).
    adc_init();
    
    // Enable the timers: TIM1, TIM2, TIM3;
    // and ouputs: TIM1_CH1, TIM1_CH1N, TIM1_CH2, TIM1_CH2N, TIM1_CH3, TIM1_CH3N, TIM1_CH4, TIM2_CH1.
    enable_timers();

    // Enable LED outputs: TIM2_CH4, TIM3_CH1, TIM3_CH2.
    enable_LED_channels();

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

inline bool usb_check_queue(size_t len_to_send) {
    if(not usb_com_queue_check(len_to_send)) {
        // The USB buffer is full; stop sending until there's space.
        if (HAL_GetTick() > usb_last_send + USB_TIMEOUT) {
            // The USB controller is not reading data; stop sending and clear the USB buffer.
            usb_readouts_to_send = 0;
            usb_stream_state = 0;
            usb_com_reset();
        }
        // Stop sending until the USB buffer is free again.
        return false;
    }

    return true;
}

inline void usb_queue_send(MessageBuffer & buffer) {
    // Send the data to the USB buffer.
    if(not usb_com_queue_send(buffer.data, buffer.write_index)) {
        // We should have checked whether we can send, so we should always succeed.
        error();
    }

    reset_message_buffer(buffer);

    usb_last_send = HAL_GetTick();
}

inline Readout adjust_direction(Readout && readout){
    readout.angle = normalize_angle(control_parameters.motor_direction * readout.angle);
    readout.angle_adjustment = control_parameters.motor_direction * readout.angle_adjustment;
    readout.angular_speed = control_parameters.motor_direction * readout.angular_speed;
    return readout;
}

void usb_queue_response(FullReadout const& readout) {
    if(usb_stream_state){
        if (not usb_check_queue(full_readout_size)) return;
        
        // We have already sent this readout; don't send it again.
        if (usb_stream_last_sent == readout.readout_number) return;

        usb_response_buffer.write_index = write_full_readout(usb_response_buffer.data, readout);
        usb_queue_send(usb_response_buffer);

        usb_stream_last_sent = readout.readout_number;
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

    // Send unit test response if requested.
    if (usb_reply_unit_test) {
        if (not usb_check_queue(unit_test_size)) return;

        // Run and write the results of the unit test.
        usb_response_buffer.write_index = write_unit_test(usb_response_buffer.data, usb_unit_test_function);
        usb_queue_send(usb_response_buffer);

        usb_reply_unit_test = false;
    }

    // Send control parameters if requested.
    if (usb_reply_control_parameters) {
        if (not usb_check_queue(control_parameters_size)) return;

        // Send the control parameters to the host.
        usb_response_buffer.write_index = write_control_parameters(usb_response_buffer.data, control_parameters);
        usb_queue_send(usb_response_buffer);

        usb_reply_control_parameters = false;
    }

    // Send current factors if requested.
    if (usb_reply_current_factors) {
        if(not usb_check_queue(current_calibration_size)) return;
        
        // Send the current factors to the host.

        usb_response_buffer.write_index = write_current_calibration(usb_response_buffer.data, current_calibration);
        usb_queue_send(usb_response_buffer);

        usb_reply_current_factors = false;
    }

    // Send trigger angles if requested.
    if (usb_reply_hall_positions) {
        if(not usb_check_queue(position_calibration_size)) return;
        
        // Send the trigger angles to the host.
        usb_response_buffer.write_index = write_position_calibration(usb_response_buffer.data, position_calibration);
        usb_queue_send(usb_response_buffer);

        usb_reply_hall_positions = false;
    }

    // Queue the readout history to the send buffer.
    while(usb_readouts_to_send > 0){
        // Check if we can enqueue the readout to the USB buffer.
        if(not usb_check_queue(readout_size)) return;
        
        // Stop if we have caught up to the readout history.
        if (not readout_history_available()) return readout_history_reset();
        
        // Send the readout to the host.
        usb_response_buffer.write_index = write_readout(usb_response_buffer.data, adjust_direction(readout_history_pop()));
        usb_queue_send(usb_response_buffer);

        // Readout added to the USB buffer.
        usb_readouts_to_send -= 1;
        
    }


}

void usb_receive_command(){
    // Buffer the command from USB and handle it when it's complete.
    if (buffer_command(usb_receive_buffer, usb_com_recv)){
        // Invalid command; reset the USB buffers.
        usb_com_reset();
    }

    // Checck if we received any command data at all, skip if not.
    if (usb_receive_buffer.write_index == 0) return;

    // Check if we received a complete command in the buffer.
    if (usb_receive_buffer.bytes_expected == 0) {
        // Handle the complete command.
        if(handle_command(usb_receive_buffer)){
            // Invalid command; reset the USB buffers.
            usb_com_reset();
        }

        // Reset the command buffer for the next command.
        reset_message_buffer(usb_receive_buffer);

        // Reset the receive index to 0 indicating we have no partial command.
        usb_last_receive_index = 0;

    } else if (usb_receive_buffer.write_index > usb_last_receive_index) {
        // We have received new data for a partial command.
        usb_last_receive_time = HAL_GetTick();
        usb_last_receive_index = usb_receive_buffer.write_index;
    } else if (HAL_GetTick() - usb_last_receive_time > USB_TIMEOUT) {
        // We have not received new data for a while; reset the command buffer.
        usb_com_reset();
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

    // Send USB data from the buffer.
    usb_com_send();

    // Read and execute commands from USB.
    usb_receive_command();
    
    // Queue the state readouts on the USB buffer.
    usb_queue_response(readout);
    
    // Send USB data from the buffer, twice per loop, hopefully the 
    // USB module sends 64 byte packets while we computed stuff.
    usb_com_send();
}



