#pragma once

#include "type_definitions.hpp"


uint32_t get_adc_update_number();
uint32_t get_hall_unobserved_number();
uint32_t get_hall_observed_number();
StateReadout get_latest_readout();


// Data queue
// ----------

void readout_history_reset();
bool readout_history_full();
bool readout_history_available();
StateReadout readout_history_pop();
