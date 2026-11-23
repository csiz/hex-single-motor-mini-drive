#pragma once

#include "hex_mini_drive_interface.hpp"

#include "parameters_store.hpp"
#include "circular_buffer.hpp"


// Average out temperature reading for the main loop to calculate.
extern float temperature_reading;

// Angle adjustment from the main loop.
extern int32_t external_angle_adjustment;


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
extern CircularQueue<hex_mini_drive::DriveState, hex_mini_drive::DRIVE_SCHEDULE_SIZE + 1> drive_state_queue;

// Set an external angle adjustment for the motor control loop. Returns true if the adjustment was successfully set, false otherwise.
bool set_angle_adjustment(int32_t const& adjustment);

// Get the temperature sensor reading.
float get_temperature();
