#include "app_main.hpp"

#include "constants.hpp"
#include "io.hpp"

#include "interrupts.hpp"
#include "interface.hpp"

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
    const StateReadout readout = latest_readout;
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




void app_tick() {

    // Show the current hall sensor state on the LEDs.
    const uint8_t hall_state = (latest_readout.position >> 13 & 0b111);
    set_LED_RGB_colours(hall_state & 0b001 ? 0x80 : 0, hall_state & 0b010 ? 0x40 : 0, hall_state & 0b100 ? 0x80 : 0);


    // Timing
    // ------

    // Update timing information.
    uint32_t milliseconds = HAL_GetTick();
    float seconds = milliseconds / 1000.f;
    
    main_loop_update_number += 1;
    main_loop_update_rate = main_loop_update_number / seconds;

    adc_update_rate = adc_update_number / seconds;
    hall_unobserved_rate = hall_unobserved_number / seconds;
    hall_observed_rate = hall_observed_number / seconds;


    // USB comms
    // ---------

    // Send USB data from the buffer.
    usb_com_send();

    // Check if we have a new command.
    usb_receive_command();
    
    // Queue the state readouts on the USB buffer.
    usb_queue_readouts();
    
    // Send USB data from the buffer, twice per loop, hopefully the 
    // USB module sends 64 byte packets while we computed stuff.
    usb_com_send();
}


