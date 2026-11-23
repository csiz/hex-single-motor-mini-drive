#include "shared_data.hpp"


// Interrupt Data Interface
// ------------------------

// Initialize the loop control parameters and the calibration data. Either load 
// them from the flash or use the defaults. Docstrings are in the header file.

// Angle adjustment from the main loop.
int32_t external_angle_adjustment = 0;

// Average out temperature reading for the main loop to calculate.
float temperature_reading = 0.f;

hex_mini_drive::Info info = {/* all default to 0 */};

hex_mini_drive::Calibration calibration = get_calibration();

hex_mini_drive::ControlParameters control_parameters = get_control_parameters();

CircularQueue<hex_mini_drive::Readout, hex_mini_drive::HISTORY_SIZE> readout_history_queue = {};

CircularQueue<hex_mini_drive::DriveState, hex_mini_drive::DRIVE_SCHEDULE_SIZE + 1> drive_state_queue = {};

bool set_angle_adjustment(int32_t const& adjustment){
    if (external_angle_adjustment) return false;
    external_angle_adjustment = adjustment;
    return true;
}

float get_temperature(){
    return temperature_reading;
}