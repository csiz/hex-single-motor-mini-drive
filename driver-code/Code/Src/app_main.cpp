#include "app_main.hpp"

#include "user_data.hpp"
#include "motor_control.hpp"
#include "interrupts.hpp"
#include "interrupts_data.hpp"
#include "interface.hpp"


#include "byte_handling.hpp"
#include "integer_math.hpp"
#include "error_handler.hpp"
#include "constants.hpp"
#include "io.hpp"

#include "usbd_cdc_if.h"

#include <stm32f1xx_ll_adc.h>
#include <stm32f1xx_ll_tim.h>
#include <stm32f1xx_hal.h>

#include <cstdlib>
#include <cmath>
#include <cstring>



uint32_t tick_number = 0;
uint32_t last_tick = 0;
uint16_t last_readout_number = 0;


const uint32_t min_timing_period_millis = 50;
uint32_t last_tick_time_millis = 0;

float tick_rate = 0.0f;
float adc_update_rate = 0.0f;

MessageBuffer usb_receive_buffer = {};

uint8_t usb_response_buffer[max_message_size] = {0};

uint16_t usb_stream_state = 0;
uint16_t usb_stream_last_sent = 0;

uint16_t usb_readouts_to_send = 0;
bool usb_wait_full_history = false;

bool usb_reply_current_factors = false;
bool usb_reply_trigger_angles = false;
bool usb_reply_pid_parameters = false;
bool usb_reply_observer_parameters = false;

uint8_t usb_unit_test_buffer[unit_test_size] = {0};
bool usb_reply_unit_test = false;


// Time of last USB packet sent; used to detect timeouts.
uint32_t usb_last_send = 0;

// Time of the last USB packet received; used to detect timeouts.
uint32_t usb_last_receive_time = 0;

// Index of the last partial message received; used to detect timeouts.
size_t usb_last_receive_index = 0;

// Maximum time to wait for a USB packet to be sent.
const uint32_t USB_TIMEOUT = 500;

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
    LL_ADC_StartCalibration(ADC1);
    while (LL_ADC_IsCalibrationOnGoing(ADC1)) {}
    LL_ADC_StartCalibration(ADC2);
    while (LL_ADC_IsCalibrationOnGoing(ADC2)) {}

    // Start ADC conversions from the external trigger in TIM1.
    LL_ADC_INJ_StartConversionExtTrig(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);
    LL_ADC_INJ_StartConversionExtTrig(ADC2, LL_ADC_INJ_TRIG_EXT_RISING);

    // Gotta enable ext triggers for the second channel to work in dual mode.
    LL_ADC_REG_StartConversionExtTrig(ADC2, LL_ADC_REG_TRIG_EXT_RISING);
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

static inline void motor_start_test(PWMSchedule const& schedule){
    // Clear the readouts buffer of old data.
    readout_history_reset();
    
    // Stop emptying the readouts queue; we want to keep the test data.
    usb_wait_full_history = true;

    // Stop adding readouts to the queue until the test starts in the interrupt handler.
    usb_readouts_to_send = history_size;

    // Start the test schedule.
    set_motor_command(DriverState::SCHEDULE, DriverParameters{ .schedule = &schedule });
}

// Run a unit test that takes a function pointer to a test function (which itself takes a buffer); returns whether error occurred.
inline bool run_unit_test( void (*test_function)(char * buffer, size_t max_size)) {
    if (usb_reply_unit_test) return true; // We are already running a unit test.

    // Clear the USB buffer.
    memset(usb_unit_test_buffer, 0, unit_test_size);

    write_uint16(usb_unit_test_buffer, MessageCode::UNIT_TEST_OUTPUT); // Write the command code to the buffer.

    // Run the test function and write the results to the USB buffer.
    test_function(reinterpret_cast<char *>(usb_unit_test_buffer + header_size), unit_test_size - header_size);

    // Set the flag to indicate that we have a unit test result ready to send.
    usb_reply_unit_test = true;

    return false;
}

// Handle the command on the buffer; returns whether there was an error.
bool handle_command(MessageBuffer const& buffer) {
    const BasicCommand command = parse_basic_command(buffer.data, buffer.write_index);

    switch (static_cast<MessageCode>(command.code)) {
        case NULL_COMMAND:
            // No command received; ignore it.
            return true;

        case STREAM_FULL_READOUTS:
            // Cotinuously stream data if timeout > 0.
            usb_stream_state = command.timeout;
            // Also stop the motor if we stop the stream.
            if (not usb_stream_state) set_motor_command(DriverState::OFF, DriverParameters{});
            return false;

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
            set_motor_command(DriverState::OFF, DriverParameters{});
            return false;
            
        // Measure the motor phase currents.
        
        case SET_STATE_TEST_ALL_PERMUTATIONS:
            motor_start_test(test_all_permutations);
            return false;
        case SET_STATE_TEST_GROUND_SHORT:
            motor_start_test(test_ground_short);
            return false;
        case SET_STATE_TEST_POSITIVE_SHORT:
            motor_start_test(test_positive_short);
            return false;
        case SET_STATE_TEST_U_DIRECTIONS:
            motor_start_test(test_u_directions);
            return false;

        case SET_STATE_TEST_U_INCREASING:
            motor_start_test(test_u_increasing);
            return false;
        case SET_STATE_TEST_U_DECREASING:
            motor_start_test(test_u_decreasing);
            return false;
        case SET_STATE_TEST_V_INCREASING:
            motor_start_test(test_v_increasing);
            return false;
        case SET_STATE_TEST_V_DECREASING:
            motor_start_test(test_v_decreasing);
            return false;
        case SET_STATE_TEST_W_INCREASING:
            motor_start_test(test_w_increasing);
            return false;
        case SET_STATE_TEST_W_DECREASING:
            motor_start_test(test_w_decreasing);
            return false;

        // Drive the motor.
        case SET_STATE_DRIVE_6_SECTOR:
            set_motor_command(
                DriverState::DRIVE_6_SECTOR,
                DriverParameters{ .sector = Drive6Sector{ 
                    .duration = command.timeout, 
                    .pwm_target = command.value, 
                }}
            );
            return false;

        case SET_STATE_DRIVE_PERIODIC:
            set_motor_command(
                DriverState::DRIVE_PERIODIC, 
                DriverParameters{ .periodic = DrivePeriodic{ 
                    .duration = command.timeout,
                    .zero_offset = 0,
                    .pwm_target = command.value,
                    .angular_speed = command.secondary
                }}
            );
            return false;

        case SET_STATE_DRIVE_SMOOTH:
            set_motor_command(
                DriverState::DRIVE_SMOOTH, 
                DriverParameters{ .smooth = DriveSmooth{ 
                    .duration = command.timeout, 
                    .pwm_target = command.value,
                    .leading_angle = command.secondary 
                }}
            );
            return false;

        case SET_STATE_DRIVE_TORQUE:
            set_motor_command(
                DriverState::DRIVE_TORQUE, 
                DriverParameters{ .torque = DriveTorque{ 
                    .duration = command.timeout, 
                    .current_target = static_cast<int16_t>(max_drive_current * command.value / pwm_max),
                    .leading_angle = command.secondary
                }}
            );
            return false;

        case SET_STATE_DRIVE_BATTERY_POWER:
            set_motor_command(
                DriverState::DRIVE_BATTERY_POWER, 
                DriverParameters{ .battery_power = DriveBatteryPower{ 
                    .duration = command.timeout, 
                    .power_target = static_cast<int16_t>(max_drive_power * command.value / pwm_max),
                    .leading_angle = command.secondary
                }}
            );
            return false;

        // Freewheel the motor.
        case SET_STATE_FREEWHEEL:
            set_motor_command(DriverState::FREEWHEEL, DriverParameters{});
            return false;

        case SET_STATE_HOLD_U_POSITIVE:
            set_motor_command(DriverState::HOLD, DriverParameters{ .hold = PWMStage{ 
                .duration = command.timeout, 
                .u_duty = static_cast<uint16_t>(command.value)
            }});
            return false;

        case SET_STATE_HOLD_V_POSITIVE:
            set_motor_command(DriverState::HOLD, DriverParameters{ .hold = PWMStage{ 
                .duration = command.timeout, 
                .v_duty = static_cast<uint16_t>(command.value)
            }});
            return false;

        case SET_STATE_HOLD_W_POSITIVE:
            set_motor_command(DriverState::HOLD, DriverParameters{ .hold = PWMStage{ 
                .duration = command.timeout, 
                .w_duty = static_cast<uint16_t>(command.value)
            }});
            return false;

        case SET_STATE_HOLD_U_NEGATIVE:
            set_motor_command(DriverState::HOLD, DriverParameters{ .hold = PWMStage{ 
                .duration = command.timeout, 
                .v_duty = static_cast<uint16_t>(command.value),
                .w_duty = static_cast<uint16_t>(command.value)
            }});
            return false;

        case SET_STATE_HOLD_V_NEGATIVE:
            set_motor_command(DriverState::HOLD, DriverParameters{ .hold = PWMStage{ 
                .duration = command.timeout, 
                .u_duty = static_cast<uint16_t>(command.value), 
                .w_duty = static_cast<uint16_t>(command.value)
            }});
            return false;

        case SET_STATE_HOLD_W_NEGATIVE:
            set_motor_command(DriverState::HOLD, DriverParameters{ .hold = PWMStage{ 
                .duration = command.timeout, 
                .u_duty = static_cast<uint16_t>(command.value),
                .v_duty = static_cast<uint16_t>(command.value)
            }});
            return false;

        case SET_CURRENT_FACTORS:
            current_calibration = parse_current_calibration(buffer.data, buffer.write_index);
            usb_reply_current_factors = true;
            return false;

        case SET_TRIGGER_ANGLES:
            position_calibration = parse_position_calibration(buffer.data, buffer.write_index);
            usb_reply_trigger_angles = true;
            return false;

        case SET_PID_PARAMETERS:
            pid_parameters = parse_pid_parameters(buffer.data, buffer.write_index);
            usb_reply_pid_parameters = true;
            return false;
        case SET_OBSERVER_PARAMETERS:
            observer_parameters = parse_observer_parameters(buffer.data, buffer.write_index);
            usb_reply_observer_parameters = true;
            return false;

        case GET_CURRENT_FACTORS:
            usb_reply_current_factors = true;
            return false;
        case GET_TRIGGER_ANGLES:
            usb_reply_trigger_angles = true;
            return false;

        case GET_PID_PARAMETERS:
            usb_reply_pid_parameters = true;
            return false;

        case GET_OBSERVER_PARAMETERS:
            usb_reply_observer_parameters = true;
            return false;

        case SAVE_SETTINGS_TO_FLASH:
            if(is_motor_safed()){
                save_settings_to_flash(current_calibration, position_calibration, pid_parameters);

                current_calibration = get_current_calibration();
                position_calibration = get_position_calibration();
                pid_parameters = get_pid_parameters();
                observer_parameters = get_observer_parameters();
                
                return false;
            } else {
                return true;
            }

        // We shouldn't receive these messages; the driver only sends them.
        case CURRENT_FACTORS:
        case TRIGGER_ANGLES:
        case PID_PARAMETERS:
        case OBSERVER_PARAMETERS:
        case READOUT:
        case FULL_READOUT:
        case UNIT_TEST_OUTPUT:
            return true;

        case RUN_UNIT_TEST_ATAN:
            return run_unit_test(unit_test_atan);
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

inline void usb_queue_send(uint8_t * data, size_t len_to_send) {
    // Send the data to the USB buffer.
    if(not usb_com_queue_send(data, len_to_send)) error();

    usb_last_send = HAL_GetTick();
}


void usb_queue_response(FullReadout const& readout) {

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
        // The unit test should have filled the buffer with data; send it all.
        usb_queue_send(usb_unit_test_buffer, unit_test_size);

        // Reset the unit test buffer; using C memory functions.
        std::memset(usb_unit_test_buffer, 0, unit_test_size);
        usb_reply_unit_test = false;
    }

    // Send PID parameters if requested.
    if (usb_reply_pid_parameters) {
        if (not usb_check_queue(pid_parameters_size)) return;
        
        // Send the PID parameters to the host.
        write_pid_parameters(usb_response_buffer, pid_parameters);
        usb_queue_send(usb_response_buffer, pid_parameters_size);

        usb_reply_pid_parameters = false;
    }

    // Send observer parameters if requested.
    if (usb_reply_observer_parameters) {
        if (not usb_check_queue(observer_parameters_size)) return;
        
        // Send the observer parameters to the host.
        write_observer_parameters(usb_response_buffer, observer_parameters);
        usb_queue_send(usb_response_buffer, observer_parameters_size);

        usb_reply_observer_parameters = false;
    }

    // Send current factors if requested.
    if (usb_reply_current_factors) {
        if(not usb_check_queue(current_calibration_size)) return;
        
        // Send the current factors to the host.

        write_current_calibration(usb_response_buffer, current_calibration);

        usb_queue_send(usb_response_buffer, current_calibration_size);

        usb_reply_current_factors = false;
    }

    // Send trigger angles if requested.
    if (usb_reply_trigger_angles) {
        if(not usb_check_queue(position_calibration_size)) return;
        
        // Send the trigger angles to the host.
        write_position_calibration(usb_response_buffer, position_calibration);
        usb_queue_send(usb_response_buffer, position_calibration_size);

        usb_reply_trigger_angles = false;
    }

    // Queue the readout history to the send buffer.
    while(usb_readouts_to_send > 0){
        // Check if we can enqueue the readout to the USB buffer.
        if(not usb_check_queue(readout_size)) return;
        
        // Stop if we have caught up to the readout history.
        if (not readout_history_available()) return readout_history_reset();
        
        // Send the readout to the host.
        write_readout(usb_response_buffer, readout_history_pop());

        // We checked whether we can send, we should always succeed.
        usb_queue_send(usb_response_buffer, readout_size);

        // Readout added to the USB buffer.
        usb_readouts_to_send -= 1;
        
    }

    if(usb_stream_state){
        if (not usb_check_queue(full_readout_size)) return;
        
        // We have already sent this readout; don't send it again.
        if (usb_stream_last_sent == readout.readout_number) return;

        write_full_readout(usb_response_buffer, readout);
        usb_queue_send(usb_response_buffer, full_readout_size);

        usb_stream_last_sent = readout.readout_number;
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
        reset_command_buffer(usb_receive_buffer);

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
    tick_number += 1;

    FullReadout readout = get_readout();

    // Timing
    // ------

    // Update timing information.
    const uint32_t milliseconds = HAL_GetTick();

    const uint32_t duration_since_timing_update = milliseconds - last_tick_time_millis;
    if (duration_since_timing_update > min_timing_period_millis) {
        last_tick_time_millis = milliseconds;
        
        float seconds = duration_since_timing_update / 1000.f;

        tick_rate = (tick_number - last_tick) / seconds;
        adc_update_rate = ((readout_number_base + readout.readout_number - last_readout_number) % readout_number_base) / seconds;

        last_tick = tick_number;
        last_readout_number = readout.readout_number;
    }

    readout.tick_rate = static_cast<int>(tick_rate);
    readout.adc_update_rate = static_cast<int>(adc_update_rate);

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



