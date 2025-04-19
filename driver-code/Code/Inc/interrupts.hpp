#pragma once


#ifdef __cplusplus
extern "C" {
#endif

// Handle new ADC readings for the motor phase currents and update the PWM registers.
void adc_interrupt_handler();

// Handle the DMA interrupt for the ADC; used for voltage and temperature readings.
void dma_interrupt_handler();

// Handle TIM1 cycles, this is triggered for both up and down counting.
void tim1_update_interrupt_handler();

// Handle TIM1 commutation. We might want to update motor registers here, however
// we're updating the motor registers in the adc handler to use fresh readings. 
void tim1_trigger_and_commutation_interrupt_handler();

// Handle TIM2 interrupts; used to time hall sensor toggles and update angle.
void tim2_global_handler();

// Initialize the angle tracking system on startup to read the initial hall sensor state.
void initialize_angle_tracking();

// Setup ADC for reading motor phase currents.
void adc_init();

// Select which interrupts to handle.
void interrupts_init();

// Enable the timers to start the ADC and motor control loops.
void enable_timers();


#ifdef __cplusplus
}
#endif