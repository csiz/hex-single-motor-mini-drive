#pragma once

#include <stdbool.h>
#include <stm32f1xx_ll_adc.h>
#include <stm32f1xx_ll_tim.h>

#include "main.h"


extern uint32_t adc_update_number;
extern uint32_t tim1_update_number;
extern uint32_t tim2_update_number;
extern uint32_t tim2_cc1_number;

#define HISTORY_SIZE 512
extern uint16_t adc_current_readouts[HISTORY_SIZE][4];
extern size_t adc_current_readouts_head;

extern bool hall_1, hall_2, hall_3;

// Use 8 bit electric angle (1.4 degrees per LSB) to track motor position; we'll use this
// value as a lookup into the sine table to generate the PWM duty cycles.
extern uint8_t motor_electric_angle;
// Same as the angle, but split into the 6 commutation steps.
extern uint8_t motor_electric_phase;
extern bool hall_sensor_valid;
extern uint8_t hall_state;


#ifdef __cplusplus
extern "C" {
#endif


void adc_interrupt_handler();
void dma_interrupt_handler();
void tim1_update_interrupt_handler();
void tim1_trigger_and_commutation_interrupt_handler();
void read_motor_hall_sensors();
void tim2_global_handler();

#ifdef __cplusplus
}
#endif