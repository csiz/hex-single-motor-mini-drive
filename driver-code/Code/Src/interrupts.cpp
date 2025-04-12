#include "interrupts.hpp"
#include "data.hpp"

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
        pwm_cycle_and_adc_update();

        LL_ADC_ClearFlag_JEOS(ADC1);
    } else {
        Error_Handler();
    }
}



void dma_interrupt_handler() {

}

// Timer 1 is updated every motor PWM cycle; at ~ 70KHz.
void tim1_update_interrupt_handler(){
    if(LL_TIM_IsActiveFlag_UPDATE(TIM1)){
        // Note, this updates on both up and down counting, get direction with: LL_TIM_GetDirection(TIM1) == LL_TIM_COUNTERDIRECTION_UP;
        
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
    // so we have to count updates (overflows) between hall sensor triggers.
    if (LL_TIM_IsActiveFlag_UPDATE(TIM2)) {
        update_position_unobserved();
        
        tim2_update_number += 1;
        LL_TIM_ClearFlag_UPDATE(TIM2);

    // The TIM2 channel 1 is triggered by the hall sensor toggles. Use it to measure motor rotation.
    } else if (LL_TIM_IsActiveFlag_CC1(TIM2)) {
        update_position_observation(); 

        tim2_cc1_number += 1;
        LL_TIM_ClearFlag_CC1(TIM2);
    } else {
        Error_Handler();
    }
}