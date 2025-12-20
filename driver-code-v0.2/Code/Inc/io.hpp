#pragma once

#include <stm32g4xx.h>
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_tim.h>

#include "type_definitions.hpp"
#include "constants.hpp"


// Motor PWM control
// -----------------

// We should always enable the trigger on channel 5 to trigger the ADC conversion.
const uint32_t adc_trigger_enable_bits = LL_TIM_CHANNEL_CH5;
// Bit flags to enable the U phase outputs.
const uint32_t pwm_u_enable_bits = LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N;
// Bit flags to enable the V phase outputs.
const uint32_t pwm_v_enable_bits = LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N;
// Bit flags to enable the W phase outputs.
const uint32_t pwm_w_enable_bits = LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N;
// Bits to enable all motor outputs.
const uint32_t pwm_enable_bits = pwm_u_enable_bits | pwm_v_enable_bits | pwm_w_enable_bits;


// Disable (tri-state / floating connection) the motor phase U.
static inline void disable_motor_u_output(){
    LL_TIM_CC_DisableChannel(TIM1, pwm_u_enable_bits);
}

// Enable the motor phase U output.
static inline void enable_motor_u_output(){
    LL_TIM_CC_EnableChannel(TIM1, pwm_u_enable_bits);
}

// Disable (tri-state / floating connection) the motor phase V.
static inline void disable_motor_v_output(){
    LL_TIM_CC_DisableChannel(TIM1, pwm_v_enable_bits);
}

// Enable the motor phase V output.
static inline void enable_motor_v_output(){
    LL_TIM_CC_EnableChannel(TIM1, pwm_v_enable_bits);
}

// Disable (tri-state / floating connection) the motor phase W.
static inline void disable_motor_w_output(){
    LL_TIM_CC_DisableChannel(TIM1, pwm_w_enable_bits);
}

// Enable the motor phase W output.
static inline void enable_motor_w_output(){
    LL_TIM_CC_EnableChannel(TIM1, pwm_w_enable_bits);
}

// Enable all motor outputs.
static inline void enable_motor_outputs(){
    LL_TIM_CC_EnableChannel(TIM1, pwm_enable_bits);
}
// Disable all motor outputs (tri-state / floating connections).
static inline void disable_motor_outputs(){
    LL_TIM_CC_DisableChannel(TIM1, pwm_enable_bits);
}

// Set all motor outputs according to the MotorOutputs struct.
static inline void set_motor_outputs(MotorOutputs const & outputs){
    LL_TIM_OC_SetCompareCH1(TIM1, outputs.u_duty + pwm_min);
    LL_TIM_OC_SetCompareCH2(TIM1, outputs.v_duty + pwm_min);
    LL_TIM_OC_SetCompareCH3(TIM1, outputs.w_duty + pwm_min);
    switch(outputs.enable_flags) {
        case 0b000:
            disable_motor_outputs();
            return;
        case 0b111:
            enable_motor_outputs();
            return;
        case 0b001:
            enable_motor_u_output();
            disable_motor_v_output();
            disable_motor_w_output();
            return;
        case 0b010:
            disable_motor_u_output();
            enable_motor_v_output();
            disable_motor_w_output();
            return;
        case 0b011:
            enable_motor_u_output();
            enable_motor_v_output();
            disable_motor_w_output();
            return;
        case 0b100:
            disable_motor_u_output();
            disable_motor_v_output();
            enable_motor_w_output();
            return;
        case 0b101:
            enable_motor_u_output();
            disable_motor_v_output();
            enable_motor_w_output();
            return;
        case 0b110:
            disable_motor_u_output();
            enable_motor_v_output();
            enable_motor_w_output();
            return;
    }
}

// LED functions
// -------------

// Enable the LED channels: TIM4_CH1 (RED), TIM4_CH2 (GREEN), TIM2_CH3 (BLUE).
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

