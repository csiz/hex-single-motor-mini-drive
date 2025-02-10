#include "interrupts.hpp"
#include "data.hpp"
#include "motor_control.hpp"
#include "main.h"


#include <stm32f1xx_ll_adc.h>
#include <stm32f1xx_ll_tim.h>


// Try really hard to keep interrupts fast. Use short inline functions that only rely 
// on chip primitives; don't use division, multiplication, or floating point operations.


// Interrupt handlers
// ------------------



void adc_interrupt_handler(){
    const bool injected_conversions_complete = LL_ADC_IsActiveFlag_JEOS(ADC1);
    if (injected_conversions_complete) {
        state_readout.readout_number = adc_update_number;

        // U and W phases are measured at the same time, followed by V and the reference voltage.
        // Each sampling time is 20cycles, and the conversion time is 12.5 cycles. At 12MHz this is
        // 2.08us. The injected sequence is triggered by TIM1 channel 4, which is set to trigger
        // 16ticks after the update event (PWM counter resets). This is a delay of 16/72MHz = 222ns.
        
        state_readout.u_readout = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
        // Note: in the v0 board the V phase shunt is connected in reverse to the current sense amplifier.
        state_readout.v_readout = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2);

        state_readout.w_readout = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);
        state_readout.ref_readout = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2);

        state_readout.pwm_commands = motor_u_pwm_duty * PWM_BASE * PWM_BASE + motor_v_pwm_duty * PWM_BASE + motor_w_pwm_duty;

        // Only write to the history buffer if we're not sending updates. Sending data over USB
        // uses too much time and we'll miss ADC updates so recording more data is unreliable.
        if (state_updates_to_send == 0) {
            state_readouts[state_readouts_index] = state_readout;
            state_readouts_index = (state_readouts_index + 1) % HISTORY_SIZE;
        }

        adc_update_number += 1;
        LL_ADC_ClearFlag_JEOS(ADC1);
    } else {
        Error_Handler();
    }
}



void dma_interrupt_handler() {

}

// Timer 1 is update every motor PWM cycle; at ~ 70KHz.
void tim1_update_interrupt_handler(){
    if(LL_TIM_IsActiveFlag_UPDATE(TIM1)){
        // Note, this updates on both up and down counting, get direction with: LL_TIM_GetDirection(TIM1) == LL_TIM_COUNTERDIRECTION_UP;
        
        // Update motor state once per up and down cycle.
        if (motor_register_update_needed and LL_TIM_GetDirection(TIM1) == LL_TIM_COUNTERDIRECTION_DOWN) {
            update_motor_control_registers();
        }
        tim1_update_number += 1;

        LL_TIM_ClearFlag_UPDATE(TIM1);
    } else {
        Error_Handler();
    }
}

void tim1_trigger_and_commutation_interrupt_handler() {
    // We shouldn't trigger this, but including for documentation.
    Error_Handler();
}


void tim2_global_handler(){
    // The TIM2 updates at a frequency of about 1KHz. Our motor might rotate slower than this
    // so we have to count updates between hall sensor triggers.
    if (LL_TIM_IsActiveFlag_UPDATE(TIM2)) {
        tim2_update_number += 1;
        LL_TIM_ClearFlag_UPDATE(TIM2);

    // The TIM2 channel 1 is triggered by the hall sensor toggles. Use it to measure motor rotation.
    } else if (LL_TIM_IsActiveFlag_CC1(TIM2)) {
        read_motor_hall_sensors();

        tim2_cc1_number += 1;
        LL_TIM_ClearFlag_CC1(TIM2);
    } else {
        Error_Handler();
    }
}