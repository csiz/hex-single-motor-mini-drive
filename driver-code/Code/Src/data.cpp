#include "data.hpp"

#include "main.h"


// Timing data
uint32_t adc_update_number = 0;
uint32_t tim1_update_number = 0;
uint32_t tim2_update_number = 0;
uint32_t tim2_cc1_number = 0;


// Electrical state
UpdateReadout latest_readout = {};
QueueHandle_t readouts_queue = nullptr;
StaticQueue_t readouts_queue_storage = {};
uint8_t readouts_queue_buffer[HISTORY_SIZE * READOUT_ITEMSIZE] = {};
uint32_t readouts_missed = 0;
int32_t readouts_to_send = 0;
bool readouts_allow_sending = true;
bool readouts_allow_missing = true;


// Functions
// ---------

void data_init(){
    // Create the queue for the readouts.
    readouts_queue = xQueueCreateStatic(HISTORY_SIZE, READOUT_ITEMSIZE, readouts_queue_buffer, &readouts_queue_storage);
    if (readouts_queue == nullptr) Error_Handler();
}


