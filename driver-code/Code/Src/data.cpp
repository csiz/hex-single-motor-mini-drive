#include "data.hpp"




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

// Hall sensors
// ------------

// Hall states as bits, 0b001 = hall 1, 0b010 = hall 2, 0b100 = hall 3.
uint8_t hall_state = 0b000; 
uint8_t hall_sector = 0;
bool hall_sector_valid = false;


// Position tracking
// -----------------

bool angle_valid = false;
uint8_t previous_hall_sector = 0;
bool previous_hall_sector_valid = false;

bool new_observation = false;
int time_since_observation = 0;
int angle_at_observation = 0;
int angle_variance_at_observation = 0;
int angular_speed_at_observation = 0;
int angular_speed_variance_at_observation = 0;

// Phase currents
float current_u = 0.0, current_v = 0.0, current_w = 0.0;


// Functions
// ---------

void data_init(){
    // Create the queue for the readouts.
    readouts_queue = xQueueCreateStatic(HISTORY_SIZE, READOUT_ITEMSIZE, readouts_queue_buffer, &readouts_queue_storage);
    if (readouts_queue == nullptr) Error_Handler();
}


void calculate_motor_phase_currents_gated(){
    // Get the latest readout; we have to gate the ADC interrupt so we copy a consistent readout.
    NVIC_DisableIRQ(ADC1_2_IRQn);
    const UpdateReadout readout = latest_readout;
    NVIC_EnableIRQ(ADC1_2_IRQn);

    const int32_t readout_diff_u = readout.u_readout - readout.ref_readout;
    const int32_t readout_diff_v = readout.v_readout - readout.ref_readout;
    const int32_t readout_diff_w = readout.w_readout - readout.ref_readout;

    // The amplifier voltage output is specified by the formula:
    //     Vout = (Iload * Rsense * GAIN) + Vref
    // Therefore:
    //     Iload = (Vout - Vref) / (Rsense * GAIN)
    // Where:
    //     Vout = adc_current_readout / adc_max_value * adc_voltage_reference;

    current_u = readout_diff_u * current_conversion;
    current_v = readout_diff_v * current_conversion;
    current_w = readout_diff_w * current_conversion;
}
