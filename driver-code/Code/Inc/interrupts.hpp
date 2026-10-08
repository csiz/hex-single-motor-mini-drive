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


#include "circular_buffer.hpp"

#include "hex_mini_drive_interface.hpp"


// Control data
// ------------

// Runtime information.
extern hex_mini_drive::Info info;

// Calibration factors.
extern hex_mini_drive::Calibration calibration;

// Interrupt loop control parameters.
extern hex_mini_drive::ControlParameters control_parameters;

// Medium length data recording, taken continuously from the interrupt handler.
extern CircularQueue<hex_mini_drive::Readout, hex_mini_drive::HISTORY_SIZE> readout_history_queue;

// Command queue, the interrupt loop will switch to executing the latest set of commands from this queue.
// ! Note, we should keep it twice the size of the maximum schedule + 1. This way we can load the next
// schedule while the first is executing, but also we will have 1 more space to write an emergency break
// command! The break command will also lead to the interrupt loop freeing up space in the command queue.
extern CircularQueue<hex_mini_drive::DriveState, 25> drive_state_queue;

// Check if the motor is in a safe state (breaking or freewheeling).
bool is_motor_safed();
