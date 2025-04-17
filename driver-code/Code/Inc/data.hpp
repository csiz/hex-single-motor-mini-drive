#pragma once

#include "FreeRTOS.h"
#include "queue.h"

#include "constants.hpp"


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
const size_t READOUT_ITEMSIZE = sizeof(StateReadout);

extern QueueHandle_t readouts_queue;
extern StaticQueue_t readouts_queue_storage;
extern uint8_t readouts_queue_buffer[HISTORY_SIZE * READOUT_ITEMSIZE];

extern bool readouts_allow_sending;
extern uint32_t readouts_to_send;


// Initialize the data structures.
void data_init();

