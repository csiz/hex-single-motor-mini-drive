#pragma once


#ifdef __cplusplus
extern "C" {
#endif

void adc_interrupt_handler();
void dma_interrupt_handler();
void tim1_update_interrupt_handler();
void tim1_trigger_and_commutation_interrupt_handler();
void tim2_global_handler();

void initialize_position_tracking();

#ifdef __cplusplus
}

// The autogenerated code defaults to C which calls our functions above; otherwise *we* prefer using C++.

#include "constants.hpp"

#include "FreeRTOS.h"
#include "queue.h"

#include <cstdint>
#include <cstddef>


extern uint32_t adc_update_number;
extern uint32_t tim1_update_number;
extern uint32_t tim2_update_number;
extern uint32_t tim2_cc1_number;

// Cycle state
// -----------

struct StateReadout{
    uint16_t readout_number;
    uint16_t u_readout;
    uint16_t v_readout;
    uint16_t w_readout;
    uint16_t ref_readout;
    uint16_t position;
    uint32_t pwm_commands;
};

extern StateReadout latest_readout;


// Data queue
// ----------

const size_t READOUT_ITEMSIZE = sizeof(StateReadout);


extern QueueHandle_t readouts_queue;
extern StaticQueue_t readouts_queue_storage;
extern uint8_t readouts_queue_buffer[HISTORY_SIZE * READOUT_ITEMSIZE];

extern uint32_t readouts_missed;

// Initialize the queue for data passing from interrupt.
void data_init();

// End ifdef __cplusplus
#endif