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



#include "type_definitions.hpp"

#include "hex_mini_drive_interface.hpp"

// Latest Data
// -----------

// Get a copy of the latest readout data.
hex_mini_drive::FullReadout get_readout();


// Data queue
// ----------

// Reset the history buffer indexes.
void readout_history_mark_reset();

// Check if the history buffer is still marked for reset.
bool readout_history_get_reset_flag();

// Get a pointer to the readout history buffer.
hex_mini_drive::Readout const* get_readout_history();

// Get the number of readouts available in the history buffer.
size_t get_readout_history_size();


// Motor control
// -------------

// Check if the motor is in a safe state (breaking or freewheeling).
bool is_motor_safed();

// Set the motor command to be executed by the interrupt loop.
void set_motor_command(DriverState const& driver_state);

// Set the rotor angle directly, eg. from an external sensor. The angle
// is set as an offset to the current angle.
void set_angle(int32_t angle);

// Offset the rotations counter to home the motor.
void set_rotations(int32_t rotations);

// Set the live maximum PWM value for the motor driver.
void set_live_max_pwm(float live_max_pwm);

// Position tracking
// -----------------

// Current calibration factors.
extern hex_mini_drive::CurrentCalibration current_calibration;

// Interrupt loop control parameters.
extern hex_mini_drive::ControlParameters control_parameters;