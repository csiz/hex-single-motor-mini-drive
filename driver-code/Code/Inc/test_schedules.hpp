#pragma once

#include "constants.hpp"
#include "type_definitions.hpp"
#include <cstdint>



// Test procedures
// ---------------


const uint32_t short_duration = hex_mini_drive::HISTORY_SIZE / schedule_size;

const PWMSchedule test_all_permutations = {
    {short_duration, 0.0, 0.0, 0.0},
    {short_duration, 1.0, 0.0, 0.0}, // Positive U
    {short_duration, 0.0, 0.0, 0.0},
    {short_duration, 0.0, 1.0, 0.0}, // Positive V
    {short_duration, 0.0, 0.0, 0.0},
    {short_duration, 0.0, 0.0, 1.0}, // Positive W
    {short_duration, 0.0, 0.0, 0.0},
    {short_duration, 0.0, 1.0, 1.0}, // Negative U
    {short_duration, 0.0, 0.0, 0.0},
    {short_duration, 1.0, 0.0, 1.0}, // Negative V
    {short_duration, 0.0, 0.0, 0.0},
    {short_duration, 1.0, 1.0, 0.0} // Negative W
};


const PWMSchedule test_ground_short = {
    {short_duration, 0, 0, 0},
    {short_duration, 0, 0, 0},
    {short_duration, 0, 0, 0},
    {short_duration, 0, 0, 0},
    {short_duration, 0, 0, 0},
    {short_duration, 0, 0, 0},
    {short_duration, 0, 0, 0},
    {short_duration, 0, 0, 0},
    {short_duration, 0, 0, 0},
    {short_duration, 0, 0, 0},
    {short_duration, 0, 0, 0},
    {short_duration, 0, 0, 0},
};

const PWMSchedule test_positive_short = {
    {short_duration, 1.0,   1.0,   1.0},
    {short_duration, 1.0,   1.0,   1.0},
    {short_duration, 1.0,   1.0,   1.0},
    {short_duration, 1.0,   1.0,   1.0},
    {short_duration, 1.0,   1.0,   1.0},
    {short_duration, 1.0,   1.0,   1.0},
    {short_duration, 1.0,   1.0,   1.0},
    {short_duration, 1.0,   1.0,   1.0},
    {short_duration, 1.0,   1.0,   1.0},
    {short_duration, 1.0,   1.0,   1.0},
    {short_duration, 1.0,   1.0,   1.0},
    {short_duration, 1.0,   1.0,   1.0},
};
