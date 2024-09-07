#include "interrupts.hpp"

// Try really hard to keep interrupts fast. Use short inline functions that only rely 
// on chip primitives; don't use division, multiplication, or floating point operations.

volatile uint32_t adc_update_number = 0;
volatile uint32_t tim1_update_number = 0;
volatile uint32_t tim2_update_number = 0;
volatile uint32_t tim2_cc1_number = 0;

volatile size_t adc_current_readout_index = 0;
volatile uint16_t adc_current_readouts[ADC_CURRENT_READ_COUNT][4] = {0};

volatile bool hall_1 = false, hall_2 = false, hall_3 = false;
