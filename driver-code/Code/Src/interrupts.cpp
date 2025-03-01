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
        // Reserve the first 3 bits for the hall sensors.
        latest_readout.readout_number = adc_update_number & 0x1FFFFFFF;
        latest_readout.readout_number |= (hall_1 << 29) | (hall_2 << 30) | (hall_3 << 31);


        // U and W phases are measured at the same time, followed by V and the reference voltage.
        // Each sampling time is 20cycles, and the conversion time is 12.5 cycles. At 12MHz this is
        // 2.08us. The injected sequence is triggered by TIM1 channel 4, which is set to trigger
        // 16ticks after the update event (PWM counter resets). This is a delay of 16/72MHz = 222ns.

        // For reference a PWM period is 1536 ticks, so the PWM frequency is 72MHz / 1536 / 2 = 23.4KHz.
        // The PWM period lasts 1/23.4KHz = 42.7us.
        
        latest_readout.u_readout = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
        // Note: in the v0 board the V phase shunt is connected in reverse to the current sense amplifier.
        latest_readout.v_readout = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2);

        latest_readout.w_readout = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);
        latest_readout.ref_readout = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2);

        latest_readout.pwm_commands = get_combined_motor_pwm_duty();

        // Always write the readout to the history buffer.
        if(xQueueSendToBackFromISR(readouts_queue, &latest_readout, NULL) != pdPASS){
            if (readouts_allow_missing) {
                // We didn't have space to add the latest readout. Discard the oldest readout and try again; this time 
                // it must work or we have a bigger error.
                readouts_missed += 1;
                if (readouts_to_send > static_cast<int>(HISTORY_SIZE)) readouts_to_send -= 1;

            } else {
                // If we filled the queue without overwriting, we now need to send the data over USB.
                // When the data is sent, overwriting will be re-enabled.
                readouts_allow_sending = true;
            }
        }

        adc_update_number += 1;
        
        update_motor_control_registers();

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
        read_hall_sensors();

        tim2_cc1_number += 1;
        LL_TIM_ClearFlag_CC1(TIM2);
    } else {
        Error_Handler();
    }
}