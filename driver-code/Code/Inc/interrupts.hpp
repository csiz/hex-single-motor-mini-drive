// The interrupts handler contains the main driving loop of the motor.
// 
// We must do a lot of math in the ADC interrupt handler; unlike a traditional
// handler that is meant to be quick, our handler is meant to be the main execution
// engine of the motor driver, with the app loop handling comunication and user 
// interface whenever it gets to execute (quite often anyway). The reason for doing
// the math in the interrupt loop is to execute it at predictable intervals with
// the latest available data, process it and set the motor outputs before the next
// cycle is loaded into the PWM counter registers. 
// 
// The other functions besides the interrupt controls the data flow 
// between the main loop and the interrupt loop.
#pragma once


// Handle new ADC readings for the motor phase currents and update the PWM registers.
extern "C" void ADC1_2_IRQHandler(void);
