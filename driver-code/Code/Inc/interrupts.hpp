#pragma once


#ifdef __cplusplus
extern "C" {
#endif

void adc_interrupt_handler();
void dma_interrupt_handler();
void tim1_update_interrupt_handler();
void tim1_trigger_and_commutation_interrupt_handler();
void tim2_global_handler();

// Initialize the angle tracking system on startup to read the initial hall sensor state.
void initialize_position_tracking();

// Setup ADC for reading motor phase currents.
void adc_init();

// Select which interrupts to handle.
void interrupts_init();

// Enable the timers to start the ADC and motor control loops.
void enable_timers();


#ifdef __cplusplus
}
#endif