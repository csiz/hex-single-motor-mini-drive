#pragma once


#ifdef __cplusplus
extern "C" {
#endif

// Handle new ADC readings for the motor phase currents and update the PWM registers.
void adc_interrupt_handler();

// Initialize the angle tracking system on startup to read the initial hall sensor state.
void initialize_angle_tracking();

#ifdef __cplusplus
}
#endif