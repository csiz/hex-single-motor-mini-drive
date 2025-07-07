#pragma once

#include "type_definitions.hpp"

#include <cstdint>
#include <cstddef>


const size_t user_data_size = 2048;

extern uint8_t user_data[user_data_size] __attribute__((__section__(".user_data")));

// Load the current calibration from flash memory.
CurrentCalibration get_current_calibration();

// Load the PID parameters from flash memory.
PIDParameters get_pid_parameters();

// Load the observer parameters from flash memory.
ObserverParameters get_observer_parameters();

// Write all calibration data to flash memory. (Flash memory is erased by pages, so we need to re-write it all.)
void save_settings_to_flash(
  CurrentCalibration const& current_calibration, 
  PIDParameters const& pid_parameters,
  ObserverParameters const& observer_parameters
);