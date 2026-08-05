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

const PWMSchedule test_u_directions = {
    {short_duration, 0.0, 0.0, 0.0},
    {short_duration, 0.0, 1.0,  1.0},
    {short_duration, 0.0, 0.0, 0.0},
    {short_duration, 1.0,  0.0, 0.0},
    {short_duration, 0.0, 0.0, 0.0},
    {short_duration, 0.0, 1.0,  1.0},
    {short_duration, 0.0, 0.0, 0.0},
    {short_duration, 1.0, 0.0, 0.0},
    {short_duration, 0.0, 0.0, 0.0},
    {short_duration, 0.0, 1.0, 1.0},
    {short_duration, 0.0, 0.0, 0.0},
    {short_duration, 0.0, 0.0, 0.0},
};

const PWMSchedule test_u_increasing = {
    {short_duration, 0.1, 0.0, 0.0},
    {short_duration, 0.2, 0.0, 0.0},
    {short_duration, 0.3, 0.0, 0.0},
    {short_duration, 0.4, 0.0, 0.0},
    {short_duration, 0.5, 0.0, 0.0},
    {short_duration, 0.6, 0.0, 0.0},
    {short_duration, 0.7, 0.0, 0.0},
    {short_duration, 0.8, 0.0, 0.0},
    {short_duration, 0.9, 0.0, 0.0},
    {short_duration, 1.0, 0.0, 0.0},
    {short_duration, 0.5, 0.0, 0.0},
    {short_duration, 0.0, 0.0, 0.0},
};

const PWMSchedule test_u_decreasing = {
    {short_duration, 0.0, 0.1, 0.1},
    {short_duration, 0.0, 0.2, 0.2},
    {short_duration, 0.0, 0.3, 0.3},
    {short_duration, 0.0, 0.4, 0.4},
    {short_duration, 0.0, 0.5, 0.5},
    {short_duration, 0.0, 0.6, 0.6},
    {short_duration, 0.0, 0.7, 0.7},
    {short_duration, 0.0, 0.8, 0.8},
    {short_duration, 0.0, 0.9, 0.9},
    {short_duration, 0.0, 1.0, 1.0},
    {short_duration, 0.0, 0.5, 0.5},
    {short_duration, 0.0, 0.0, 0.0}
};

const PWMSchedule test_v_increasing = {
    {short_duration, 0.0, 0.1, 0.0},
    {short_duration, 0.0, 0.2, 0.0},
    {short_duration, 0.0, 0.3, 0.0},
    {short_duration, 0.0, 0.4, 0.0},
    {short_duration, 0.0, 0.5, 0.0},
    {short_duration, 0.0, 0.6, 0.0},
    {short_duration, 0.0, 0.7, 0.0},
    {short_duration, 0.0, 0.8, 0.0},
    {short_duration, 0.0, 0.9, 0.0},
    {short_duration, 0.0, 1.0, 0.0},
    {short_duration, 0.0, 0.5, 0.0},
    {short_duration, 0.0, 0.0, 0.0}
};

const PWMSchedule test_v_decreasing = {
    {short_duration, 0.1, 0.0, 0.1},
    {short_duration, 0.2, 0.0, 0.2},
    {short_duration, 0.3, 0.0, 0.3},
    {short_duration, 0.4, 0.0, 0.4},
    {short_duration, 0.5, 0.0, 0.5},
    {short_duration, 0.6, 0.0, 0.6},
    {short_duration, 0.7, 0.0, 0.7},
    {short_duration, 0.8, 0.0, 0.8},
    {short_duration, 0.9, 0.0, 0.9},
    {short_duration, 1.0, 0.0, 1.0},
    {short_duration, 0.5, 0.0, 0.5},
    {short_duration, 0.0, 0.0, 0.0}
};

const PWMSchedule test_w_increasing = {
    {short_duration, 0.0,   0.0, 0.1},
    {short_duration, 0.0,   0.0, 0.2},
    {short_duration, 0.0,   0.0, 0.3},
    {short_duration, 0.0,   0.0, 0.4},
    {short_duration, 0.0,   0.0, 0.5},
    {short_duration, 0.0,   0.0, 0.6},
    {short_duration, 0.0,   0.0, 0.7},
    {short_duration, 0.0,   0.0, 0.8},
    {short_duration, 0.0,   0.0, 0.9},
    {short_duration, 0.0,   0.0, 1.0},
    {short_duration, 0.0,   0.0, 0.5},
    {short_duration, 0.0,   0.0, 0.0}
};

const PWMSchedule test_w_decreasing = {
    {short_duration, 0.1,   0.1, 0.0},
    {short_duration, 0.2,   0.2, 0.0},
    {short_duration, 0.3,   0.3, 0.0},
    {short_duration, 0.4,   0.4, 0.0},
    {short_duration, 0.5,   0.5, 0.0},
    {short_duration, 0.6,   0.6, 0.0},
    {short_duration, 0.7,   0.7, 0.0},
    {short_duration, 0.8,   0.8, 0.0},
    {short_duration, 0.9,   0.9, 0.0},
    {short_duration, 1.0,   1.0, 0.0},
    {short_duration, 0.5,   0.5, 0.0},
    {short_duration, 0.0,   0.0, 0.0}
};