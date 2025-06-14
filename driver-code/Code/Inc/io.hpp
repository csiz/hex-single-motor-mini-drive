#pragma once

#include <stm32f103xb.h>
#include <stm32f1xx_ll_gpio.h>
#include <stm32f1xx_ll_tim.h>

#include "type_definitions.hpp"

// Motor PWM control
// -----------------

const uint32_t pwm_u_enable_bits = LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N;
const uint32_t pwm_v_enable_bits = LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N;
const uint32_t pwm_w_enable_bits = LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N;
const uint32_t pwm_enable_bits = pwm_u_enable_bits | pwm_v_enable_bits | pwm_w_enable_bits;


static inline void enable_motor_outputs(){
    LL_TIM_CC_EnableChannel(TIM1, pwm_enable_bits);
}

static inline void disable_motor_outputs(){
    LL_TIM_CC_DisableChannel(TIM1, pwm_enable_bits);
}


static inline MotorOutputs get_motor_outputs(){
    return MotorOutputs{
        .u_duty_cycle = static_cast<uint16_t>(LL_TIM_OC_GetCompareCH1(TIM1)),
        .v_duty_cycle = static_cast<uint16_t>(LL_TIM_OC_GetCompareCH2(TIM1)),
        .w_duty_cycle = static_cast<uint16_t>(LL_TIM_OC_GetCompareCH3(TIM1))
    };
}

static inline void set_motor_outputs(MotorOutputs const & outputs){
    LL_TIM_OC_SetCompareCH1(TIM1, outputs.u_duty_cycle);
    LL_TIM_OC_SetCompareCH2(TIM1, outputs.v_duty_cycle);
    LL_TIM_OC_SetCompareCH3(TIM1, outputs.w_duty_cycle);
}


static inline void set_motor_u_pwm_duty(uint32_t duty_cycle){
    LL_TIM_OC_SetCompareCH1(TIM1, duty_cycle);
}

static inline uint32_t get_motor_u_pwm_duty(){
    return LL_TIM_OC_GetCompareCH1(TIM1);
}

static inline void disable_motor_u_output(){
    LL_TIM_CC_DisableChannel(TIM1, pwm_u_enable_bits);
}

static inline void enable_motor_u_output(){
    LL_TIM_CC_EnableChannel(TIM1, pwm_u_enable_bits);
}

static inline void set_motor_v_pwm_duty(uint32_t duty_cycle){
    LL_TIM_OC_SetCompareCH2(TIM1, duty_cycle);
}

static inline uint32_t get_motor_v_pwm_duty(){
    return LL_TIM_OC_GetCompareCH2(TIM1);
}

static inline void disable_motor_v_output(){
    LL_TIM_CC_DisableChannel(TIM1, pwm_v_enable_bits);
}

static inline void enable_motor_v_output(){
    LL_TIM_CC_EnableChannel(TIM1, pwm_v_enable_bits);
}


static inline void set_motor_w_pwm_duty(uint32_t duty_cycle){
    LL_TIM_OC_SetCompareCH3(TIM1, duty_cycle);
}

static inline uint32_t get_motor_w_pwm_duty(){
    return LL_TIM_OC_GetCompareCH3(TIM1);
}

static inline void disable_motor_w_output(){
    LL_TIM_CC_DisableChannel(TIM1, pwm_w_enable_bits);
}

static inline void enable_motor_w_output(){
    LL_TIM_CC_EnableChannel(TIM1, pwm_w_enable_bits);
}


// LED functions
// -------------

static inline void enable_LED_channels(){
	// Green LED.
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH4);
	// Blue LED.
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
	// RED LED.
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
}

static inline void set_RED_LED(uint8_t r){
    LL_TIM_OC_SetCompareCH2(TIM3, LL_TIM_GetAutoReload(TIM3)  * r / 0xFF);
}

static inline void set_GREEN_LED(uint8_t g){
    LL_TIM_OC_SetCompareCH4(TIM2, LL_TIM_GetAutoReload(TIM2) * g / 0xFF);
}

static inline void set_BLUE_LED(uint8_t b){
    LL_TIM_OC_SetCompareCH1(TIM3, LL_TIM_GetAutoReload(TIM3) * b / 0xFF);
}

static inline void set_LED_RGB_colours(uint8_t r, uint8_t g, uint8_t b){
    set_RED_LED(r);
    set_GREEN_LED(g);
    set_BLUE_LED(b);
}
