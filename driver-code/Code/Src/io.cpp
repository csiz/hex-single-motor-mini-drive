#include <stm32f1xx_ll_tim.h>

#include "io.hpp"
#include "interrupts.hpp"


float current_u = 0.0, current_v = 0.0, current_w = 0.0;



void init_motor_position(){
    read_motor_hall_sensors();
}

void update_motor_phase_currents(){
    // Read the latest current values from the ADC circular buffer.
    const uint16_t* adc_readout = adc_current_readouts[adc_current_readouts_head];

    const int32_t readout_diff_u = adc_readout[0] - adc_readout[3];
    const int32_t readout_diff_v = adc_readout[1] - adc_readout[3];
    const int32_t readout_diff_w = adc_readout[2] - adc_readout[3];

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



void enable_LED_channels(){
	// Green LED.
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH4);
	// Blue LED.
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
	// RED LED.
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
}

void set_RED_LED(uint8_t r){
    LL_TIM_OC_SetCompareCH2(TIM3, LL_TIM_GetAutoReload(TIM3)  * r / 0xFF);
}

void set_GREEN_LED(uint8_t g){
    LL_TIM_OC_SetCompareCH4(TIM2, LL_TIM_GetAutoReload(TIM2) * g / 0xFF);
}

void set_BLUE_LED(uint8_t b){
    LL_TIM_OC_SetCompareCH1(TIM3, LL_TIM_GetAutoReload(TIM3) * b / 0xFF);
}

void set_LED_RGB_colours(uint8_t r, uint8_t g, uint8_t b){
    set_RED_LED(r);
    set_GREEN_LED(g);
    set_BLUE_LED(b);
}
