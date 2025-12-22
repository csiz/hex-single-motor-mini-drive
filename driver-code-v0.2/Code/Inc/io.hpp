#pragma once

#include <cstdint>
#include <stm32g4xx.h>
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_tim.h>

#include "stm32g4xx_ll_adc.h"
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



// Mapping from hall state to sector number.
static inline uint8_t get_hall_sector(const uint8_t hall_state){
    // Get the hall sector from the state.
    switch (hall_state & 0b111) {
        // No hall sensors; either it's not powered or no magnet
        case 0b000: return hall_sector_base; // Out of range, indicates invalid.
        // Hall U active; 0 degrees
        case 0b001: return 0;
        // Hall U and hall V active; angle is 60 degrees.
        case 0b011: return 1;
        // Hall V active; angle is 120 degrees.
        case 0b010: return 2;
        // Hall V and hall W active; angle is 180 degrees.
        case 0b110: return 3;
        // Hall W active; angle is 240 degrees.
        case 0b100: return 4;
        // Hall U and hall W active; angle is 300 degrees.
        case 0b101: return 5;
        // All hall sensors active; this would be quite unusual; but carry on.
        case 0b111: return hall_sector_base; // Out of range, indicates invalid.
    }
    // We shouldn't reach here.
    return hall_sector_base;
}

// Read the hall sensors and update the motor rotation angle. Sensor chips might be: SS360NT (can't read the inprint clearly).
static inline uint8_t read_hall_sensors_state(){
    // Grab the registers for the GPIO ports with the hall sensors.

    // uint16_t gpio_A_inputs = LL_GPIO_ReadInputPort(GPIOA);
    uint16_t gpio_B_inputs = LL_GPIO_ReadInputPort(GPIOB);
    uint16_t gpio_C_inputs = LL_GPIO_ReadInputPort(GPIOC);

    // Note: Hall sensors are active low!
    const bool hall_1 = !(gpio_C_inputs & (1<<6)); // Hall sensor 1, corresponding to phase V
    const bool hall_2 = !(gpio_C_inputs & (1<<7)); // Hall sensor 2, corresponding to phase W
    const bool hall_3 = !(gpio_B_inputs & (1<<0)); // Hall sensor 3, corresponding to phase U

    // Combine the hall sensor states into a single byte.
    // Note: Reorder the sensors according to the phase order; U on bit 0, V on bit 1, W on bit 2.
    const uint8_t hall_state = hall_3 | (hall_1 << 1) | (hall_2 << 2);

    return hall_state;
}


// ADC Mappings
// ------------

// ADC1 IN7 = POT 1
// ADC1 IN8 = POT 2
// ADC2 IN17 = POT 3

// ADC1 IN3 = Half 3v3 Reference

// ADC1 IN1 = M1 current
// ADC2 IN3 = M2 current
// ADC1 IN12 = M3 current
// ADC2 IN14 = M4 current

// ADC2 IN15 = Voltage

// OPAMP1 VINP = M1 voltage
// OPAMP2 VINP = M2 voltage
// OPAMP3 VINP = M3 voltage
// ADC1 IN11 = M4 voltage

static inline ADCReadings read_adc_values(){    

    // U and W phases are measured at the same time, followed by V and the reference voltage.
    // Each sampling time is 20cycles, and the conversion time is 12.5 cycles. At 12MHz this is
    // 2.08us. The injected sequence is triggered by TIM1 channel 4, which is set to trigger
    // 16ticks after the update event (PWM counter resets). This is a delay of 16/72MHz = 222ns.
    
    // For reference a PWM period is 1536 ticks, so the PWM frequency is 72MHz / 1536 / 2 = 23.4KHz.
    // The PWM period lasts 1/23.4KHz = 42.7us.

    // Read the reference voltage first; this is the voltage of the reference line for the current sense amplifier.
    const uint16_t ref_readout = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_4);

    // I wired the shunt resistors in the wrong way, so we need to flip the sign of the current readings.
    // Flip the sign of V because we accidentally wired it the other way (the correct way...). Oopsie doopsie.
    const int u_readout = -(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2) - ref_readout);
    const int v_readout = -(LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2) - ref_readout);
    const int w_readout = -(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_3) - ref_readout);

    // Note that the reference voltage is only connected to the current sense amplifier, not the
    // microcontroller. The ADC reference voltage is 3.3V.

    // Also read the controller chip temperature.
    const uint16_t temp_readout = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);

    // And motor supply voltage.
    const uint16_t vcc_readout = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);

    return ADCReadings{
        ThreePhase{
            u_readout,
            v_readout,
            w_readout,
        },
        ref_readout,
        temp_readout,
        vcc_readout,
    };
}

// Setup ADC for reading motor phase currents.
static void adc_init(){

    // Enable the ADC interrupt for the end of the injected sequence.
    LL_ADC_EnableIT_JEOS(ADC1);
    // Disable the ADC interrupt for the end of the regular sequence; we are not using it.
    LL_ADC_DisableIT_EOS(ADC1);
    


    // Calibrate the ADCs.

    // Take a little break before.
    HAL_Delay(100);
    LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC1)) {}
    LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC2)) {}

    // Enable the ADCs and wait for them to settle.
    LL_ADC_Enable(ADC1);
    LL_ADC_Enable(ADC2);
    HAL_Delay(100);
    
    // Start ADC conversions from the external trigger in TIM1.
    // TODO: recheck timing of conversions; it was set on the rising trigger
    LL_ADC_REG_StartConversion(ADC1);
    LL_ADC_REG_StartConversion(ADC2);

    LL_ADC_INJ_StartConversion(ADC1);
    LL_ADC_INJ_StartConversion(ADC2);

    // Gotta enable ext triggers for the second channel to work in dual mode.
    LL_ADC_REG_StartConversion(ADC2);
}

// Select which interrupts to handle.
static void interrupts_init(){

    // TIM1 is used to generate the PWM signals for the motor phases, and to trigger the ADC.
    // 
    // Note that we don't need to set dead-time at this point; the gate driver FD6288T&Q has a 
    // built-in dead-time of 100-300ns. The MOSFETs AO4266E have turn on rise time of 8ns and 
    // turn off fall time of 22ns so we should be covered.

    // Disable TIM1 update interrupt; occuring every PWM cycle. We will use the ADC interrupt to update the motor control.
    LL_TIM_DisableIT_UPDATE(TIM1);
    // Disable TIM1 trigger interrupt; it's only triggered during resets.
    LL_TIM_DisableIT_TRIG(TIM1);
    // Disable commutation interrupts; 
    LL_TIM_DisableIT_COM(TIM1);


    // Disable TIM2 update interrupt. Count updates of TIM2 to measure slow motor 
    // speed in case the timer overflows before a toggle in the hall sensor.
    LL_TIM_DisableIT_UPDATE(TIM2);
    // Disable TIM2 trigger interrupt. It is triggered every reset and hall sensor toggle.
    LL_TIM_DisableIT_TRIG(TIM2);
    // Enable TIM2 channel 1 interrupt. It is triggered every hall sensor toggle; we can 
    // read the time from the last time the input changed.
    LL_TIM_DisableIT_CC1(TIM2);
    // Disable TIM2 channel 2 interrupt. We are ignoring the delayed trigger output from 
    // TIM2 as we will commutate faster than the hall sensor toggles. To use this effectively
    // we need to use the prescaler to slow the timer down; but then we lose resolution.
    LL_TIM_DisableIT_CC2(TIM2);

    // Disable TIM1 capture/compare update selection preload. This is used to update the
    // output modes of all channels at the same time when a commutation event is triggered.
    // We don't need this as we always enable PWM mode and update the compare register to 
    // modulate the PWM duty cycle.
    // 
    // This setting is mostly here to document that in order to trigger commutations on TRGI 
    // we also need to enable preload for the motor ouput channel mode registers.
    LL_TIM_CC_DisablePreload(TIM1);

    // Don't allow TIM1 commutations to be triggered by TIM2 hall sensor toggles.
    LL_TIM_CC_SetUpdate(TIM1, LL_TIM_CCUPDATESOURCE_COMG_ONLY);
    
}

// Enable the timers to start the ADC and motor control loops.
static void enable_timers(){
    // We use the ADC1 and ADC2 in simulatenous mode to read the motor phase currents
    // around the PWM cycle up to down transition (when the ground mosfet is on). When 
    // the ADC is triggered in injected mode both ADC modules read the current for the 
    // U and V phases first then W and reference voltage second.
    // 
    // The ADC sample time is 20cycles, so the total sampling period is 20/12MHz = 1.67us.
    // 
    // Use the TIM1 channel 5 to generate an event a short time before the counter reaches
    // the auto reload value. This event triggers the ADC to read the motor phase currents.
    const uint16_t injected_conversion_start = pwm_base - sample_lead_time;
    LL_TIM_OC_SetCompareCH5(TIM1, injected_conversion_start);

    // Invert the PWM mode of channel 5 instead so we emit a rising edge during up counting.
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH5, LL_TIM_OCMODE_PWM2);

    // Reinitialize the timers; reset the counters and update registers. Because the timers
    // are setup with preload registers the values we write to them are stored in shadow registers
    // and only applied when the update event occurs (to any individual PWM cycle consistent). We
    // need to generate an update event to force the timers to load the new values.
    LL_TIM_GenerateEvent_UPDATE(TIM1);
    LL_TIM_GenerateEvent_UPDATE(TIM2);
    LL_TIM_GenerateEvent_UPDATE(TIM4);

    // Start the timers.
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_EnableCounter(TIM2);
    LL_TIM_EnableCounter(TIM4);
    
    // Enable TIM1 channel 5 used to trigger the ADC.
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH5);
    
    // Enable the TIM1 outputs following a break or clock issue event.
    LL_TIM_EnableAllOutputs(TIM1);
    
    // Enable TIM2 channels 1 as the hall sensor timer between commutations.
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
}

static inline void io_init(){
    // Setup PWM settings.
    LL_TIM_SetAutoReload(TIM1, pwm_autoreload);

    // Set which interrupts to handle.
    interrupts_init();
    
    // Setup the ADC for reading motor phase currents, temperature and voltage.
    // This function will run an initial calibration of the chip's ADCs (takes a few ms).
    adc_init();
    
    // Enable the timers: TIM1, TIM2, TIM3;
    // and outputs: TIM1_CH1, TIM1_CH1N, TIM1_CH2, TIM1_CH2N, TIM1_CH3, TIM1_CH3N, TIM1_CH4, TIM2_CH1.
    enable_timers();

    // Enable LED outputs: TIM2_CH4, TIM3_CH1, TIM3_CH2.
    enable_LED_channels();
}