#include <stm32f1xx_ll_adc.h>
#include <stm32f1xx_ll_tim.h>

#include "interrupts.hpp"
#include "io.hpp"
#include "main.h"


uint32_t adc_update_number = 0;

void adc_interrupt_handler() {
    const bool injected_conversions_complete = LL_ADC_IsActiveFlag_JEOS(ADC1);
    if (injected_conversions_complete) {
        adc_current_readouts[0] = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
        adc_current_readouts[1] = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);
        adc_current_readouts[2] = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2);
        adc_current_readouts[3] = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2);
        adc_current_updated = true;

        adc_update_number += 1;
        LL_ADC_ClearFlag_JEOS(ADC1);
    } else {
        Error_Handler();
    }
}

void dma_interrupt_handler() {

}

uint32_t tim1_update_number = 0;

// Timer 1 is update every motor PWM cycle; at ~ 70KHz.
void tim1_update_interrupt_handler(){
    if(LL_TIM_IsActiveFlag_UPDATE(TIM1)){
        LL_TIM_ClearFlag_UPDATE(TIM1);
        tim1_update_number += 1;
    } else {
        Error_Handler();
    }
}

void tim1_trigger_and_commutation_interrupt_handler() {
    // We shouldn't trigger this, but including for documentation.
    Error_Handler();
}

uint32_t tim2_update_number = 0;
uint32_t tim2_cc1_number = 0;

void tim2_global_handler() {
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

