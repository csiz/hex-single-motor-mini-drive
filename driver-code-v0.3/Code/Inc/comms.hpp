#pragma once

#include "hex_mini_drive_interface.hpp"

void comms_init();

void comms_update(hex_mini_drive::FullReadout const& readout);