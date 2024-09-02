#include <stm32f1xx_ll_tim.h>

#include "io.hpp"

volatile bool hall_1 = false, hall_2 = false, hall_3 = false;


void read_hall_sensors(){
	uint16_t gpio_A_inputs = LL_GPIO_ReadInputPort(GPIOA);
	uint16_t gpio_B_inputs = LL_GPIO_ReadInputPort(GPIOB);

	// Hall sensors are active low.
	hall_1 = not(gpio_A_inputs & (1<<0));
	hall_2 = not(gpio_A_inputs & (1<<1));
	hall_3 = not(gpio_B_inputs & (1<<10));
}

void initialise_LED_channels(){
	LL_TIM_EnableCounter(TIM2);
	// Green LED.
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH4);

	LL_TIM_EnableCounter(TIM3);

	// Blue LED.
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
	// RED LED.
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
}

void set_LED_RGB_colours(uint8_t r, uint8_t g, uint8_t b){
	// RED LED.
	TIM3->CCR2 = TIM3->ARR  * r / 0xFF;
	// Green LED.
	TIM2->CCR4 = TIM2->ARR * g / 0xFF;
	// Blue LED.
	TIM3->CCR1 = TIM3->ARR * b / 0xFF;
}
