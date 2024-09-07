#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void adc_interrupt_handler();
void dma_interrupt_handler();
void tim1_update_interrupt_handler();
void tim1_trigger_and_commutation_interrupt_handler();
void tim2_global_handler();

extern uint32_t adc_update_number;
extern uint32_t tim1_update_number;
extern uint32_t tim2_update_number;
extern uint32_t tim2_cc1_number;

#ifdef __cplusplus
}
#endif