#include "interrupts.hpp"

// Try really hard to keep interrupts fast. Use short inline functions that only rely 
// on chip primitives; don't use division, multiplication, or floating point operations.

uint32_t adc_update_number = 0;
uint32_t tim1_update_number = 0;
uint32_t tim2_update_number = 0;
uint32_t tim2_cc1_number = 0;

QueueHandle_t adc_queue = {};
StaticQueue_t adc_queue_storage = {};
uint8_t adc_queue_buffer[ADC_QUEUE_SIZE * ADC_ITEMSIZE] = {0};
ADC_Readout adc_readouts[ADC_HISTORY_SIZE] = {0};
size_t adc_readouts_index = 0;
uint32_t adc_processed_number = 0;
ADC_Readout latest_readout = {0};


bool hall_1 = false, hall_2 = false, hall_3 = false;

uint8_t motor_electric_angle = 0;
uint8_t motor_electric_phase = 0;
bool hall_sensor_valid = false;
uint8_t hall_state = 0;


// Interrupt handlers
// ------------------

void interrupt_init(){
    adc_queue = xQueueCreateStatic(ADC_QUEUE_SIZE, ADC_ITEMSIZE, adc_queue_buffer, &adc_queue_storage);
}

void adc_interrupt_handler(){
    const bool injected_conversions_complete = LL_ADC_IsActiveFlag_JEOS(ADC1);
    if (injected_conversions_complete) {
        ADC_Readout readout;
        readout.readout_number = adc_update_number;

        readout.u_readout = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
        readout.v_readout = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);
        readout.w_readout = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2);
        readout.ref_readout = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2);

        latest_readout.readout_number = readout.readout_number;
        latest_readout.u_readout = (latest_readout.u_readout * 12 + readout.u_readout * 4)/16;
        latest_readout.v_readout = (latest_readout.v_readout * 12 + readout.v_readout * 4)/16;
        latest_readout.w_readout = (latest_readout.w_readout * 12 + readout.w_readout * 4)/16;
        latest_readout.ref_readout = (latest_readout.ref_readout * 12 + readout.ref_readout * 4)/16;

        if ((adc_update_number % ADC_SKIP_SIZE) == 0) xQueueSendToBackFromISR(adc_queue, &latest_readout, NULL);

        adc_update_number += 1;
        LL_ADC_ClearFlag_JEOS(ADC1);
    } else {
        Error_Handler();
    }
}

size_t move_adc_readouts(bool overwrite){
    ADC_Readout readout;
    size_t values_read = 0;
    
    for (size_t i = 0; i < ADC_QUEUE_SIZE; i++) {
        if (xQueueReceive(adc_queue, &readout, 0) == pdFALSE) break;
        values_read += 1;

        if (overwrite) {
            adc_readouts[adc_readouts_index] = readout;
            adc_readouts_index = (adc_readouts_index + 1) % ADC_HISTORY_SIZE;
        }
    }

    adc_processed_number += values_read;
    
    return values_read;
}

void dma_interrupt_handler() {

}

// Timer 1 is update every motor PWM cycle; at ~ 70KHz.
void tim1_update_interrupt_handler(){
    if(LL_TIM_IsActiveFlag_UPDATE(TIM1)){
        LL_TIM_ClearFlag_UPDATE(TIM1);
        tim1_update_number += 1;
    } else {
        Error_Handler();
    }
}

void tim1_trigger_and_commutation_interrupt_handler() {
    // We shouldn't trigger this, but including for documentation.
    Error_Handler();
}




void read_motor_hall_sensors(){
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

void tim2_global_handler(){
    // The TIM2 updates at a frequency of about 1KHz. Our motor might rotate slower than this
    // so we have to count updates between hall sensor triggers.
    if (LL_TIM_IsActiveFlag_UPDATE(TIM2)) {
        tim2_update_number += 1;
        LL_TIM_ClearFlag_UPDATE(TIM2);

    // The TIM2 channel 1 is triggered by the hall sensor toggles. Use it to measure motor rotation.
    } else if (LL_TIM_IsActiveFlag_CC1(TIM2)) {
        read_motor_hall_sensors();

        tim2_cc1_number += 1;
        LL_TIM_ClearFlag_CC1(TIM2);
    } else {
        Error_Handler();
    }
}