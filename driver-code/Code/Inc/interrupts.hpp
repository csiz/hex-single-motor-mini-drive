#pragma once

#include <stdbool.h>
#include <stm32f1xx_ll_adc.h>
#include <stm32f1xx_ll_tim.h>

#include <FreeRTOS.h>
#include <queue.h>

#include "main.h"


extern uint32_t adc_update_number;
extern uint32_t tim1_update_number;
extern uint32_t tim2_update_number;
extern uint32_t tim2_cc1_number;

struct ADC_Readout{
    uint32_t readout_number;
    uint16_t u_readout;
    uint16_t v_readout;
    uint16_t w_readout;
    uint16_t ref_readout;
};
#define ADC_HISTORY_SIZE 512
#define ADC_QUEUE_SIZE 32
#define ADC_SKIP_SIZE 4
#define ADC_ITEMSIZE sizeof(struct ADC_Readout)


extern struct ADC_Readout latest_readout;

extern QueueHandle_t adc_queue;
extern StaticQueue_t adc_queue_storage;
extern uint8_t adc_queue_buffer[ADC_QUEUE_SIZE * ADC_ITEMSIZE];

extern struct ADC_Readout adc_readouts[ADC_HISTORY_SIZE];
extern size_t adc_readouts_index;
extern uint32_t adc_processed_number;

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

void interrupt_init();
void adc_interrupt_handler();
void dma_interrupt_handler();
void tim1_update_interrupt_handler();
void tim1_trigger_and_commutation_interrupt_handler();
void read_motor_hall_sensors();
void tim2_global_handler();
size_t move_adc_readouts(bool overwrite);

#ifdef __cplusplus
}
#endif