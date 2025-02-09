#pragma once


#ifdef __cplusplus
extern "C" {
#endif

void adc_interrupt_handler();
void dma_interrupt_handler();
void tim1_update_interrupt_handler();
void tim1_trigger_and_commutation_interrupt_handler();
void tim2_global_handler();

#ifdef __cplusplus
}
#endif