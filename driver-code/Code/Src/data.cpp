#include "data.hpp"
#include "main.h"

#include <stm32f1xx_ll_gpio.h>

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
bool hall_sensor_valid = false;


// Phase currents
float current_u = 0.0, current_v = 0.0, current_w = 0.0;


// Functions
// ---------

void data_init(){
    // Create the queue for the readouts.
    readouts_queue = xQueueCreateStatic(HISTORY_SIZE, READOUT_ITEMSIZE, readouts_queue_buffer, &readouts_queue_storage);
    if (readouts_queue == nullptr) Error_Handler();
}

/* Get Hall sensor data from: SS360NT */
void read_hall_sensors(){
    uint16_t gpio_A_inputs = LL_GPIO_ReadInputPort(GPIOA);
    uint16_t gpio_B_inputs = LL_GPIO_ReadInputPort(GPIOB);

    // Note: Hall sensors are active low!
    const bool hall_1 = !(gpio_A_inputs & (1<<0)); // Hall sensor 1, corresponding to phase V
    const bool hall_2 = !(gpio_A_inputs & (1<<1)); // Hall sensor 2, corresponding to phase W
    const bool hall_3 = !(gpio_B_inputs & (1<<10)); // Hall sensor 3, corresponding to phase U

    // Combine the hall sensor states into a single byte.
    // Note: Reorder the sensors according to the phase order; U on bit 0, V on bit 1, W on bit 2.
    hall_state = hall_3 | (hall_1 << 1) | (hall_2 << 2);

    switch (hall_state) {
        case 0b000: // no hall sensors; either it's not ready or no magnet
            hall_sensor_valid = false;
            break;
        case 0b001: // hall U active; 0 degrees
            hall_sector = 0;
            hall_sensor_valid = true;
            break;
        case 0b011: // hall U and hall V active; 60 degrees
            hall_sector = 1;
            hall_sensor_valid = true;
            break;
        case 0b010: // hall V active; 120 degrees
            hall_sector = 2;
            hall_sensor_valid = true;
            break;
        case 0b110: // hall V and hall W active; 180 degrees
            hall_sector = 3;
            hall_sensor_valid = true;
            break;
        case 0b100: // hall W active; 240 degrees
            hall_sector = 4;
            hall_sensor_valid = true;
            break;
        case 0b101: // hall U and hall W active; 300 degrees
            hall_sector = 5;
            hall_sensor_valid = true;
            break;
        case 0b111: // all hall sensors active; this would be quite unusual
            hall_sensor_valid = false;
            Error_Handler();
            break;
    }
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
