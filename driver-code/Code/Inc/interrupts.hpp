#pragma once

#include <stdbool.h>
#include <stm32f1xx_ll_adc.h>
#include <stm32f1xx_ll_tim.h>

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

// Raw Sensor Values
// -----------------

// Hall sensor states.
extern volatile bool hall_1, hall_2, hall_3;

#define ADC_CURRENT_READ_COUNT 8
extern volatile size_t adc_current_readout_index;
// Raw ADC readouts for motor phase currents and reference voltage.
extern volatile uint16_t adc_current_readouts[ADC_CURRENT_READ_COUNT][4];


// Interrupt counters
// ------------------

extern volatile uint32_t adc_update_number;
extern volatile uint32_t tim1_update_number;
extern volatile uint32_t tim2_update_number;
extern volatile uint32_t tim2_cc1_number;


// Interrupt handlers
// ------------------


static inline void adc_interrupt_handler(){
    const bool injected_conversions_complete = LL_ADC_IsActiveFlag_JEOS(ADC1);
    adc_current_readout_index = (adc_current_readout_index + 1) % ADC_CURRENT_READ_COUNT;
    // Alias the the readouts for the current index.
    volatile uint16_t * adc_reads_at_index = adc_current_readouts[adc_current_readout_index];

    if (injected_conversions_complete) {
        adc_reads_at_index[0] = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
        adc_reads_at_index[1] = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);
        adc_reads_at_index[2] = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2);
        adc_reads_at_index[3] = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2);

        adc_update_number += 1;
        LL_ADC_ClearFlag_JEOS(ADC1);
    } else {
        Error_Handler();
    }
}

static inline void dma_interrupt_handler() {

}

// Timer 1 is update every motor PWM cycle; at ~ 70KHz.
static inline void tim1_update_interrupt_handler(){
    if(LL_TIM_IsActiveFlag_UPDATE(TIM1)){
        LL_TIM_ClearFlag_UPDATE(TIM1);
        tim1_update_number += 1;
    } else {
        Error_Handler();
    }
}

static inline void tim1_trigger_and_commutation_interrupt_handler() {
    // We shouldn't trigger this, but including for documentation.
    Error_Handler();
}

static inline void read_motor_hall_sensors(){
	uint16_t gpio_A_inputs = LL_GPIO_ReadInputPort(GPIOA);
	uint16_t gpio_B_inputs = LL_GPIO_ReadInputPort(GPIOB);

	// Hall sensors are active low.
	hall_1 = !(gpio_A_inputs & (1<<0));
	hall_2 = !(gpio_A_inputs & (1<<1));
	hall_3 = !(gpio_B_inputs & (1<<10));
}

static inline void tim2_global_handler(){
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

#ifdef __cplusplus
}
#endif