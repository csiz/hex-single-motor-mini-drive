#pragma once

#include "stm32g431xx.h"
#include <stm32g4xx.h>
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_tim.h>


// LED functions
// -------------

// Enable the LED channels: TIM2_CH4, TIM3_CH1, TIM3_CH2.
static inline void enable_LED_channels(){
	// RED LED.
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
	// Green LED.
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH2);
	// Blue LED.
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH3);
}

// Set the intensity of the RED light.
static inline void set_RED_LED(uint8_t r){
    LL_TIM_OC_SetCompareCH1(TIM4, LL_TIM_GetAutoReload(TIM4)  * r / 0xFF);
}

// Set the intensity of the GREEN light.
static inline void set_GREEN_LED(uint8_t g){
    LL_TIM_OC_SetCompareCH2(TIM4, LL_TIM_GetAutoReload(TIM4) * g / 0xFF);
}

// Set the intensity of the BLUE light.
static inline void set_BLUE_LED(uint8_t b){
    LL_TIM_OC_SetCompareCH3(TIM2, LL_TIM_GetAutoReload(TIM2) * b / 0xFF);
}

// Set RGB colour for the indicator light.
static inline void set_LED_RGB_colours(uint8_t r, uint8_t g, uint8_t b){
    set_RED_LED(r);
    set_GREEN_LED(g);
    set_BLUE_LED(b);
}
