#include <stm32f1xx_ll_tim.h>

#include "io.hpp"

volatile bool hall_1 = false, hall_2 = false, hall_3 = false;
volatile bool adc_current_updated = false;
volatile uint16_t adc_current_readouts[4] = {0, 0, 0, 0};
float current_u = 0.0, current_v = 0.0, current_w = 0.0;

void read_motor_hall_sensors(){
	uint16_t gpio_A_inputs = LL_GPIO_ReadInputPort(GPIOA);
	uint16_t gpio_B_inputs = LL_GPIO_ReadInputPort(GPIOB);

	// Hall sensors are active low.
	hall_1 = not(gpio_A_inputs & (1<<0));
	hall_2 = not(gpio_A_inputs & (1<<1));
	hall_3 = not(gpio_B_inputs & (1<<10));
}

void init_motor_position(){
    read_motor_hall_sensors();
}

void calculate_motor_phase_currents(){
    // Copy the current readouts to local memory; restart if we are interrupted.
    uint16_t adc_current_readouts_local[4];
    do {
        adc_current_updated = false;
        for (uint8_t i = 0; i < 4; i++) {
            adc_current_readouts_local[i] = adc_current_readouts[i];
        }
    } while (adc_current_updated);

    // The amplifier voltage output is specified by the formula:
    //     Vout = (Iload * Rsense * GAIN) + Vref
    // Therefore:
    //     Iload = (Vout - Vref) / (Rsense * GAIN)
    // Where:
    //     Vout = adc_current_readout / adc_max_value * adc_voltage_reference;

    const int16_t readout_u = adc_current_readouts_local[0] - adc_current_readouts_local[3];
    const int16_t readout_v = adc_current_readouts_local[1] - adc_current_readouts_local[3];
    const int16_t readout_w = adc_current_readouts_local[2] - adc_current_readouts_local[3];

    current_u = readout_u * readout_to_current;
    current_v = readout_v * readout_to_current;
    current_w = readout_w * readout_to_current;
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
