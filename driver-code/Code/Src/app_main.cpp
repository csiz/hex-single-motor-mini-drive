#include "app_main.hpp"

#include "user_data.hpp"
#include "motor_control.hpp"
#include "interrupts.hpp"
#include "interrupts_data.hpp"
#include "interface.hpp"
#include "interrupts_angle.hpp"
#include "interrupts_motor.hpp"

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
uint32_t last_adc_update = 0;
uint32_t last_hall_unobserved = 0;
uint32_t last_hall_observed = 0;

const uint32_t min_timing_period_millis = 50;
uint32_t last_tick_time_millis = 0;

float tick_rate = 0.0f;
float adc_update_rate = 0.0f;
float hall_unobserved_rate = 0.0f;
float hall_observed_rate = 0.0f;

MessageBuffer usb_receive_buffer = {};

uint8_t usb_response_buffer[max_message_size] = {0};

uint16_t usb_stream_state = 0;
uint16_t usb_stream_last_sent = 0;

uint16_t usb_readouts_to_send = 0;
bool usb_wait_full_history = false;

bool usb_reply_current_factors = false;
bool usb_reply_trigger_angles = false;

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
    motor_start_schedule(schedule);
}

// Run a unit test that takes a function pointer to a test function (which itself takes a buffer).
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


bool handle_command(MessageBuffer const & buffer) {
    const BasicCommand command = parse_basic_command(buffer.data, buffer.write_index);

    switch (static_cast<MessageCode>(command.code)) {
        case NULL_COMMAND:
            // No command received; ignore it.
            return true;

        case STREAM_FULL_READOUTS:
            // Cotinuously stream data if timeout > 0.
            usb_stream_state = command.timeout;
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
            motor_break();
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
        case SET_STATE_DRIVE_POS:
            motor_drive_pos(command.pwm, command.timeout);
            return false;
        case SET_STATE_DRIVE_NEG:
            motor_drive_neg(command.pwm, command.timeout);
            return false;
        case SET_STATE_DRIVE_SMOOTH_POS:
            motor_drive_smooth_pos(command.pwm, command.timeout, command.leading_angle);
            return false;
        case SET_STATE_DRIVE_SMOOTH_NEG:
            motor_drive_smooth_neg(command.pwm, command.timeout, command.leading_angle);
            return false;

        // Freewheel the motor.
        case SET_STATE_FREEWHEEL:
            motor_freewheel();
            return false;

        case SET_STATE_HOLD_U_POSITIVE:
            motor_hold(command.pwm, 0, 0, command.timeout);
            return false;

        case SET_STATE_HOLD_V_POSITIVE:
            motor_hold(0, command.pwm, 0, command.timeout);
            return false;

        case SET_STATE_HOLD_W_POSITIVE:
            motor_hold(0, 0, command.pwm, command.timeout);
            return false;

        case SET_STATE_HOLD_U_NEGATIVE:
            motor_hold(0, command.pwm, command.pwm, command.timeout);
            return false;

        case SET_STATE_HOLD_V_NEGATIVE:
            motor_hold(command.pwm, 0, command.pwm, command.timeout);
            return false;

        case SET_STATE_HOLD_W_NEGATIVE:
            motor_hold(command.pwm, command.pwm, 0, command.timeout);
            return false;

        case SET_CURRENT_FACTORS:
            current_calibration = parse_current_calibration(buffer.data, buffer.write_index);
            usb_reply_current_factors = true;
            return false;

        case SET_TRIGGER_ANGLES:
            position_calibration = parse_position_calibration(buffer.data, buffer.write_index);
            usb_reply_trigger_angles = true;
            return false;

        case GET_CURRENT_FACTORS:
            usb_reply_current_factors = true;
            return false;
        case GET_TRIGGER_ANGLES:
            usb_reply_trigger_angles = true;
            return false;

        case SAVE_SETTINGS_TO_FLASH:
            if(is_motor_stopped()){
                save_settings_to_flash(current_calibration, position_calibration);

                current_calibration = get_current_calibration();
                position_calibration = get_position_calibration();
                
                return false;
            } else {
                return true;
            }

        // We shouldn't receive these messages; the driver only sends them.
        case CURRENT_FACTORS:
        case TRIGGER_ANGLES:
        case READOUT:
        case FULL_READOUT:
        case UNIT_TEST_OUTPUT:
            return true;

        case RUN_UNIT_TEST_ATAN:
            return run_unit_test(unit_test_atan);
        case RUN_UNIT_TEST_INTEGER_ARITHMETIC:
            return run_unit_test(unit_test_integer_arithmetic);

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


void usb_queue_response(){

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

    // Send readouts if requested; up to an arbitrary number of readouts so we don't block for long.
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
        
        // Disable the ADC interrupt while we read the latest readout.
        NVIC_DisableIRQ(ADC1_2_IRQn);
        FullReadout full_readout = get_readout();
        NVIC_EnableIRQ(ADC1_2_IRQn);
        
        // We have already sent this readout; don't send it again.
        if (usb_stream_last_sent == full_readout.readout_number) return;
        
        full_readout.tick_rate = static_cast<int>(tick_rate);
        full_readout.adc_update_rate = static_cast<int>(adc_update_rate);
        full_readout.hall_unobserved_rate = static_cast<int>(hall_unobserved_rate);
        full_readout.hall_observed_rate = static_cast<int>(hall_observed_rate);

        write_full_readout(usb_response_buffer, full_readout);
        usb_queue_send(usb_response_buffer, full_readout_size);

        usb_stream_last_sent = full_readout.readout_number;
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

    // Show the current hall sensor state on the LEDs.
    const uint8_t hall_state = (get_readout().position >> 13 & 0b111);
    set_LED_RGB_colours(hall_state & 0b001 ? 0x80 : 0, hall_state & 0b010 ? 0x40 : 0, hall_state & 0b100 ? 0x80 : 0);


    // Timing
    // ------

    // Update timing information.
    const uint32_t milliseconds = HAL_GetTick();

    const uint32_t duration_since_timing_update = milliseconds - last_tick_time_millis;
    if (duration_since_timing_update > min_timing_period_millis) {
        last_tick_time_millis = milliseconds;
        
        float seconds = duration_since_timing_update / 1000.f;

        const uint32_t adc_update_number = get_adc_update_number();
        const uint32_t hall_unobserved_number = get_hall_unobserved_number();
        const uint32_t hall_observed_number = get_hall_observed_number();
        
        tick_rate = (tick_number - last_tick) / seconds;
        adc_update_rate = (adc_update_number - last_adc_update) / seconds;
        hall_unobserved_rate = (hall_unobserved_number - last_hall_unobserved) / seconds;
        hall_observed_rate = (hall_observed_number - last_hall_observed) / seconds;

        last_tick = tick_number;
        last_adc_update = adc_update_number;
        last_hall_unobserved = hall_unobserved_number;
        last_hall_observed = hall_observed_number;
    } 

    

    // USB comms
    // ---------

    // Send USB data from the buffer.
    usb_com_send();

    // Read and execute commands from USB.
    usb_receive_command();
    
    // Queue the state readouts on the USB buffer.
    usb_queue_response();
    
    // Send USB data from the buffer, twice per loop, hopefully the 
    // USB module sends 64 byte packets while we computed stuff.
    usb_com_send();
}



