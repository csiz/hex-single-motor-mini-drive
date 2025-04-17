#pragma once

#include "FreeRTOS.h"
#include "queue.h"

#include "constants.hpp"


extern uint32_t adc_update_number;
extern uint32_t tim1_update_number;
extern uint32_t tim2_update_number;
extern uint32_t tim2_cc1_number;

// Electrical state
// ----------------

struct UpdateReadout{
    uint16_t readout_number;
    uint16_t u_readout;
    uint16_t v_readout;
    uint16_t w_readout;
    uint16_t ref_readout;
    uint16_t position;
    uint32_t pwm_commands;
};

extern UpdateReadout latest_readout;
const size_t READOUT_ITEMSIZE = sizeof(UpdateReadout);

extern QueueHandle_t readouts_queue;
extern StaticQueue_t readouts_queue_storage;
extern uint8_t readouts_queue_buffer[HISTORY_SIZE * READOUT_ITEMSIZE];
extern uint32_t readouts_missed;

extern bool readouts_allow_missing;
extern bool readouts_allow_sending;

extern int32_t readouts_to_send;


// Initialize the data structures.
void data_init();
