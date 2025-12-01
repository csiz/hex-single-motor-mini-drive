#include "app_main.hpp"
#include "io.hpp"
#include "stm32g431xx.h"


void app_init(){
    LL_TIM_GenerateEvent_UPDATE(TIM2);
    LL_TIM_GenerateEvent_UPDATE(TIM4);

    // Start the timers.
    LL_TIM_EnableCounter(TIM2);
    LL_TIM_EnableCounter(TIM4);


    enable_LED_channels();
    set_BLUE_LED(0x00);
    set_GREEN_LED(0x10);
    set_RED_LED(0x00);
}

int i = 0;
void app_tick(){
    // i = (i + 1) % 256;
    // set_BLUE_LED(i);
}