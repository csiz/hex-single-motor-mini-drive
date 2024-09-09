#pragma once
/*
 * outputs.h
 *
 *  Created on: Sep 2, 2024
 *      Author: Calin
 */

#include <stm32f103xb.h>
#include <stm32f1xx_ll_gpio.h>



// Computer Sensor Values
// -----------------------

// Computed motor phase currents.
extern float current_u, current_v, current_w;


// Motor Currents
// --------------

// Voltage reference for the ADC; it's a filtered 3.3V that power the board.
const float adc_voltage_reference = 3.3;
// Shunt resistance for the motor phase current sensing are 10mOhm, 500mW resistors.
const float motor_shunt_resistance = 0.010;
// The voltage on the shunt resistor is amplified by INA4181 Bidirectional, Low and 
// High Side Voltage Output, Current-Sense Amplifier.
const float amplifier_gain = 20.0;
// The ADC has a 12-bit resolution.
const uint16_t adc_max_value = 0xFFF;
// The formula that determines the current from the ADC readout: 
//   Vout = (Iload * Rsense * GAIN) + Vref
//   Vout = adc_current_readout / adc_max_value * adc_voltage_reference.
// The minus sign is because of the way the INA4181 is wired up...
const float readout_diff_to_current = -adc_voltage_reference / (adc_max_value * motor_shunt_resistance * amplifier_gain);

// Compute motor phase currents using latest ADC readouts; clearing the adc_current_updated flag.
void calculate_motor_phase_currents();

static inline void set_motor_u_pwm_duty_cycle(uint16_t duty_cycle){
    LL_TIM_OC_SetCompareCH1(TIM1, duty_cycle);
}

static inline void disable_motor_u_output(){
    LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
}

static inline void enable_motor_u_output(){
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
}

static inline void set_motor_v_pwm_duty_cycle(uint16_t duty_cycle){
    LL_TIM_OC_SetCompareCH2(TIM1, duty_cycle);
}

static inline void disable_motor_v_output(){
    LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
}

static inline void enable_motor_v_output(){
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
}


static inline void set_motor_w_pwm_duty_cycle(uint16_t duty_cycle){
    LL_TIM_OC_SetCompareCH3(TIM1, duty_cycle);
}

static inline void disable_motor_w_output(){
    LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
}

static inline void enable_motor_w_output(){
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
}

// Motor position
// --------------

void init_motor_position();


// LED functions
// -------------

void enable_LED_channels();
void set_RED_LED(uint8_t r);
void set_GREEN_LED(uint8_t g);
void set_BLUE_LED(uint8_t b);
void set_LED_RGB_colours(uint8_t r, uint8_t g, uint8_t b);
