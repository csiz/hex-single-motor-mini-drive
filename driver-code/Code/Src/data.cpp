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
bool hall_1 = false, hall_2 = false, hall_3 = false;

uint8_t motor_electric_angle = 0;
uint8_t motor_electric_phase = 0;
bool hall_sensor_valid = false;
uint8_t hall_state = 0;


// Phase currents
float current_u = 0.0, current_v = 0.0, current_w = 0.0;


// Functions
// ---------

void data_init(){
    // Create the queue for the readouts.
    readouts_queue = xQueueCreateStatic(HISTORY_SIZE, READOUT_ITEMSIZE, readouts_queue_buffer, &readouts_queue_storage);
    if (readouts_queue == nullptr) Error_Handler();
}

void read_hall_sensors(){
	uint16_t gpio_A_inputs = LL_GPIO_ReadInputPort(GPIOA);
	uint16_t gpio_B_inputs = LL_GPIO_ReadInputPort(GPIOB);

	// Hall sensors are active low.
	hall_1 = !(gpio_A_inputs & (1<<0));
	hall_2 = !(gpio_A_inputs & (1<<1));
	hall_3 = !(gpio_B_inputs & (1<<10));

    hall_state = (hall_1 << 2) | (hall_2 << 1) | hall_3;

    switch (hall_state) {
        case 0b000: // no hall sensors; either it's not ready or no magnet
            hall_sensor_valid = false;
            break;
        case 0b100: // hall 1 active; 0 degrees
            motor_electric_angle = 0;
            motor_electric_phase = 0;
            hall_sensor_valid = true;
            break;
        case 0b110: // hall 1 and hall 2 active; 60 degrees
            motor_electric_angle = 43;
            motor_electric_phase = 1;
            hall_sensor_valid = true;
            break;
        case 0b010: // hall 2 active; 120 degrees
            motor_electric_angle = 85;
            motor_electric_phase = 2;
            hall_sensor_valid = true;
            break;
        case 0b011: // hall 2 and hall 3 active; 180 degrees
            motor_electric_angle = 128;
            motor_electric_phase = 3;
            hall_sensor_valid = true;
            break;
        case 0b001: // hall 3 active; 240 degrees
            motor_electric_angle = 171;
            motor_electric_phase = 4;
            hall_sensor_valid = true;
            break;
        case 0b101: // hall 1 and hall 3 active; 300 degrees
            motor_electric_angle = 213;
            motor_electric_phase = 5;
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
