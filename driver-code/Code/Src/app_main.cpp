#include "app_main.hpp"


#include "motor_control.hpp"
#include "interrupts.hpp"
#include "interface.hpp"

#include "error_handler.hpp"
#include "constants.hpp"
#include "io.hpp"

#include "usbd_cdc_if.h"

#include <stm32f1xx_ll_adc.h>
#include <stm32f1xx_ll_tim.h>

#include <cstdlib>
#include <cmath>



uint32_t main_loop_update_number = 0;


float main_loop_update_rate = 0.0f;
float adc_update_rate = 0.0f;
float hall_unobserved_rate = 0.0f;
float hall_observed_rate = 0.0f;

CommandBuffer usb_command_buffer = {};


uint32_t usb_readouts_to_send = 0;
bool usb_wait_full_history = false;


// Time of last USB packet sent; used to detect timeouts.
uint32_t usb_last_send = 0;

// Maximum time to wait for a USB packet to be sent.
const uint32_t USB_TIMEOUT = 500;



// TODO: enable DMA channel 1 for ADC1 reading temperature and voltage.
// TODO: also need to set a minium on time for MOSFET driving, too little 
// won't spin the motor at all (lies! we can use a small on time to
// bias the motor to make it easy to turn by the load).

// Phase Currents
// --------------

float current_u = 0;
float current_v = 0;
float current_w = 0;

// Compute motor phase currents using latest ADC readouts. 
void calculate_motor_phase_currents_gated(){
    // TODO: rework
    return;

    // Get the latest readout; we have to gate the ADC interrupt so we copy a consistent readout.
    NVIC_DisableIRQ(ADC1_2_IRQn);
    const StateReadout readout = get_latest_readout();
    NVIC_EnableIRQ(ADC1_2_IRQn);

    const int32_t readout_diff_u = readout.u_readout - readout.ref_readout;
    const int32_t readout_diff_v = readout.v_readout - readout.ref_readout;
    const int32_t readout_diff_w = readout.w_readout - readout.ref_readout;

    // The amplifier voltage output is specified by the formula:
    //     Vout = (Iload * Rsense * GAIN) + Vref
    // Therefore:
    //     Iload = (Vout - Vref) / (Rsense * GAIN)
    // Where:
    //     Vout = adc_current_readout / adc_max_value * adc_voltage_reference;

    current_u = readout_diff_u * current_conversion;
    current_v = readout_diff_v * current_conversion;
    current_w = readout_diff_w * current_conversion;
}



void app_init() {
    data_init();

    // Setup PWM settings.
    LL_TIM_SetAutoReload(TIM1, PWM_AUTORELOAD); // 72MHz / 1536 / 2 = 23.4KHz

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
    initialize_position_tracking();
}

static inline void motor_start_test(PWMSchedule const& schedule){
    // Stop emptying the readouts queue; we want to keep the test data.
    usb_wait_full_history = true;

    // Stop adding readouts to the queue until the test starts in the interrupt handler.
    usb_readouts_to_send = HISTORY_SIZE;
    
    // Clear the readouts buffer of old data.
    xQueueReset(readouts_queue);

    // Start the test schedule.
    motor_start_schedule(schedule);
}


bool handle_command(CommandBuffer const & buffer) {
    const CommandHeader command = parse_command_header(buffer);

    switch (command.code) {
        case NULL_COMMAND:
            // No command received; ignore it.
            return true;

        // Send the whole history buffer over USB.
        case GET_READOUTS:
            // Clear the queue of older data.
            xQueueReset(readouts_queue);
            usb_readouts_to_send = command.timeout;
            return true;

        case GET_READOUTS_SNAPSHOT:
            xQueueReset(readouts_queue);
            // Dissalow sending until we fill the queue, so it doesn't interrupt commutation.
            usb_wait_full_history = true;
            usb_readouts_to_send = HISTORY_SIZE;
            return true;

        // Turn off the motor driver.
        case SET_STATE_OFF:
            motor_break();
            return true;
            
        // Measure the motor phase currents.
        
        case SET_STATE_TEST_ALL_PERMUTATIONS:
            motor_start_test(test_all_permutations);
            return true;
        case SET_STATE_TEST_GROUND_SHORT:
            motor_start_test(test_ground_short);
            return true;
        case SET_STATE_TEST_POSITIVE_SHORT:
            motor_start_test(test_positive_short);
            return true;
        case SET_STATE_TEST_U_DIRECTIONS:
            motor_start_test(test_u_directions);
            return true;

        case SET_STATE_TEST_U_INCREASING:
            motor_start_test(test_u_increasing);
            return true;
        case SET_STATE_TEST_U_DECREASING:
            motor_start_test(test_u_decreasing);
            return true;
        case SET_STATE_TEST_V_INCREASING:
            motor_start_test(test_v_increasing);
            return true;
        case SET_STATE_TEST_V_DECREASING:
            motor_start_test(test_v_decreasing);
            return true;
        case SET_STATE_TEST_W_INCREASING:
            motor_start_test(test_w_increasing);
            return true;
        case SET_STATE_TEST_W_DECREASING:
            motor_start_test(test_w_decreasing);
            return true;

        // Drive the motor.
        case SET_STATE_DRIVE_POS:
            motor_drive_pos(command.pwm, command.timeout);
            return true;
        case SET_STATE_DRIVE_NEG:
            motor_drive_neg(command.pwm, command.timeout);
            return true;
        case SET_STATE_DRIVE_SMOOTH_POS:
            motor_drive_smooth_pos(command.pwm, command.timeout, command.leading_angle);
            return true;
        case SET_STATE_DRIVE_SMOOTH_NEG:
            motor_drive_smooth_neg(command.pwm, command.timeout, command.leading_angle);
            return true;

        // Freewheel the motor.
        case SET_STATE_FREEWHEEL:
            motor_freewheel();
            return true;

        case SET_STATE_HOLD_U_POSITIVE:
            motor_hold(command.pwm, 0, 0, command.timeout);
            return true;

        case SET_STATE_HOLD_V_POSITIVE:
            motor_hold(0, command.pwm, 0, command.timeout);
            return true;

        case SET_STATE_HOLD_W_POSITIVE:
            motor_hold(0, 0, command.pwm, command.timeout);
            return true;

        case SET_STATE_HOLD_U_NEGATIVE:
            motor_hold(0, command.pwm, command.pwm, command.timeout);
            return true;

        case SET_STATE_HOLD_V_NEGATIVE:
            motor_hold(command.pwm, 0, command.pwm, command.timeout);
            return true;

        case SET_STATE_HOLD_W_NEGATIVE:
            motor_hold(command.pwm, command.pwm, 0, command.timeout);
            return true;

        case SET_CURRENT_FACTORS:            
            /* CurrentFactors factors = */ parse_current_factors(usb_command_buffer);
            // TODO: update current factors
            return true;

        case SET_TRIGGER_ANGLES:
            /* TriggerAngles angles = */ parse_trigger_angles(usb_command_buffer);
            // TODO: update trigger angles
            return true;

        // Don't use a default case so we can catch unhandled commands.
    }

    // If we get here, we have an unknown command.
    return false;
}

void usb_queue_readouts(){
    // Check if we have to wait for the queue to fill before sending readouts.
    // This is used to prevent sending readouts while the motor is commutating.
    if (usb_wait_full_history) {
        if(xQueueIsQueueFullFromISR(readouts_queue) == pdTRUE){
            // The queue is full; we can start sending readouts.
            usb_wait_full_history = false;
        } else {
            // We have not filled the queue yet; don't send readouts.
            return;
        }
    }

    // Send readouts if requested; up to an arbitrary number of readouts so we don't block for long.
    for (size_t i = 0; usb_readouts_to_send > 0 and i < HISTORY_SIZE; i++){
        // Check if we can enqueue the readout to the USB buffer.
        if(not usb_com_queue_check(state_readout_size)) {
            // The USB buffer is full; stop sending until there's space.
            if (HAL_GetTick() > usb_last_send + USB_TIMEOUT) {
                // The USB controller is not reading data; stop sending and clear the USB buffer.
                usb_readouts_to_send = 0;
                usb_com_reset();
            }
            // Stop sending until the USB buffer is free again.
            break;
        }
        
        StateReadout readout;

        // Skip if we have no data left to read.
        if (xQueueReceive(readouts_queue, &readout, 0) != pdTRUE) break;

        // Send the readout to the host.
        uint8_t readout_data[state_readout_size] = {0};
        write_state_readout(readout_data, readout);

        // We checked whether we can send, we should always succeed.
        if(not usb_com_queue_send(readout_data, state_readout_size)) error();

        // Readout added to the USB buffer.
        usb_readouts_to_send -= 1;
        usb_last_send = HAL_GetTick();
    }
}


void app_tick() {
    main_loop_update_number += 1;

    // Show the current hall sensor state on the LEDs.
    const uint8_t hall_state = (get_latest_readout().position >> 13 & 0b111);
    set_LED_RGB_colours(hall_state & 0b001 ? 0x80 : 0, hall_state & 0b010 ? 0x40 : 0, hall_state & 0b100 ? 0x80 : 0);


    // Timing
    // ------

    // Update timing information.
    uint32_t milliseconds = HAL_GetTick();
    float seconds = milliseconds / 1000.f;
    
    main_loop_update_rate = main_loop_update_number / seconds;
    adc_update_rate = get_adc_update_number() / seconds;
    hall_unobserved_rate = get_hall_unobserved_number() / seconds;
    hall_observed_rate = get_hall_observed_number() / seconds;


    // USB comms
    // ---------

    // Send USB data from the buffer.
    usb_com_send();

    // Buffer the command from USB and handle it when it's complete.
    if (buffer_command(usb_command_buffer, usb_com_recv)){
        // Handle the complete command.
        if (not handle_command(usb_command_buffer)){
            // Invalid command; reset the USB buffers.
            usb_com_reset();
        }

        // Reset the command buffer for the next command.
        reset_command_buffer(usb_command_buffer);
    }
    
    // Queue the state readouts on the USB buffer.
    usb_queue_readouts();
    
    // Send USB data from the buffer, twice per loop, hopefully the 
    // USB module sends 64 byte packets while we computed stuff.
    usb_com_send();
}


