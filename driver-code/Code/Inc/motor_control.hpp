#pragma once

#include "constants.hpp"
#include "type_definitions.hpp"


// Functions to set motor driver state.

void motor_drive_neg(uint16_t pwm, uint16_t timeout);
void motor_drive_pos(uint16_t pwm, uint16_t timeout);
void motor_drive_smooth_pos(uint16_t pwm, uint16_t timeout, uint16_t new_leading_angle);
void motor_drive_smooth_neg(uint16_t pwm, uint16_t timeout, uint16_t new_leading_angle);
void motor_hold(uint16_t u, uint16_t v, uint16_t w, uint16_t timeout);
void motor_break();
void motor_freewheel();
void motor_start_schedule(const PWMSchedule & schedule);


// Test procedures
// ---------------


const uint16_t PWM_TEST = PWM_BASE / 2;

const uint16_t SHORT_DURATION = HISTORY_SIZE / SCHEDULE_SIZE;

const PWMSchedule test_all_permutations = {
    {SHORT_DURATION, 0,         0,         0},
    {SHORT_DURATION, PWM_TEST,  0,         0}, // Positive U
    {SHORT_DURATION, 0,         0,         0},
    {SHORT_DURATION, 0,         PWM_TEST,  0}, // Positive V
    {SHORT_DURATION, 0,         0,         0},
    {SHORT_DURATION, 0,         0,         PWM_TEST}, // Positive W
    {SHORT_DURATION, 0,         0,         0},
    {SHORT_DURATION, 0,         PWM_TEST,  PWM_TEST}, // Negative U
    {SHORT_DURATION, 0,         0,         0},
    {SHORT_DURATION, PWM_TEST,  0,         PWM_TEST}, // Negative V
    {SHORT_DURATION, 0,         0,         0},
    {SHORT_DURATION, PWM_TEST,  PWM_TEST,  0} // Negative W
};


const PWMSchedule test_ground_short = {
    {SHORT_DURATION, 0,         0,         0},
    {SHORT_DURATION, 0,         0,         0},
    {SHORT_DURATION, 0,         0,         0},
    {SHORT_DURATION, 0,         0,         0},
    {SHORT_DURATION, 0,         0,         0},
    {SHORT_DURATION, 0,         0,         0},
    {SHORT_DURATION, 0,         0,         0},
    {SHORT_DURATION, 0,         0,         0},
    {SHORT_DURATION, 0,         0,         0},
    {SHORT_DURATION, 0,         0,         0},
    {SHORT_DURATION, 0,         0,         0},
    {SHORT_DURATION, 0,         0,         0},
};

const PWMSchedule test_positive_short = {
    {SHORT_DURATION, PWM_MAX,   PWM_MAX,   PWM_MAX},
    {SHORT_DURATION, PWM_MAX,   PWM_MAX,   PWM_MAX},
    {SHORT_DURATION, PWM_MAX,   PWM_MAX,   PWM_MAX},
    {SHORT_DURATION, PWM_MAX,   PWM_MAX,   PWM_MAX},
    {SHORT_DURATION, PWM_MAX,   PWM_MAX,   PWM_MAX},
    {SHORT_DURATION, PWM_MAX,   PWM_MAX,   PWM_MAX},
    {SHORT_DURATION, PWM_MAX,   PWM_MAX,   PWM_MAX},
    {SHORT_DURATION, PWM_MAX,   PWM_MAX,   PWM_MAX},
    {SHORT_DURATION, PWM_MAX,   PWM_MAX,   PWM_MAX},
    {SHORT_DURATION, PWM_MAX,   PWM_MAX,   PWM_MAX},
    {SHORT_DURATION, PWM_MAX,   PWM_MAX,   PWM_MAX},
    {SHORT_DURATION, PWM_MAX,   PWM_MAX,   PWM_MAX},
};

const PWMSchedule test_u_directions = {
    {SHORT_DURATION, 0,         0,         0},
    {SHORT_DURATION, 0,         PWM_TEST,  PWM_TEST},
    {SHORT_DURATION, 0,         0,         0},
    {SHORT_DURATION, PWM_TEST,  0,         0},
    {SHORT_DURATION, 0,         0,         0},
    {SHORT_DURATION, 0,         PWM_TEST,  PWM_TEST},
    {SHORT_DURATION, 0,         0,         0},
    {SHORT_DURATION, PWM_TEST,  0,         0},
    {SHORT_DURATION, 0,         0,         0},
    {SHORT_DURATION, 0,         PWM_TEST,  PWM_TEST},
    {SHORT_DURATION, 0,         0,         0},
    {SHORT_DURATION, 0,         0,         0},
};

const PWMSchedule test_u_increasing = {
    {SHORT_DURATION, 0,                   0,         0},
    {SHORT_DURATION, PWM_BASE * 1 / 10,   0,         0},
    {SHORT_DURATION, PWM_BASE * 2 / 10,   0,         0},
    {SHORT_DURATION, PWM_BASE * 3 / 10,   0,         0},
    {SHORT_DURATION, PWM_BASE * 4 / 10,   0,         0},
    {SHORT_DURATION, PWM_BASE * 5 / 10,   0,         0},
    {SHORT_DURATION, PWM_BASE * 6 / 10,   0,         0},
    {SHORT_DURATION, PWM_BASE * 7 / 10,   0,         0},
    {SHORT_DURATION, PWM_BASE * 8 / 10,   0,         0},
    {SHORT_DURATION, PWM_BASE * 9 / 10,   0,         0},
    {SHORT_DURATION, 0,                   0,         0},
    {SHORT_DURATION, 0,                   0,         0},
};

const PWMSchedule test_u_decreasing = {
    {SHORT_DURATION, 0,                   0,                 0},
    {SHORT_DURATION, 0,                   PWM_BASE * 1 / 10, PWM_BASE * 1 / 10},
    {SHORT_DURATION, 0,                   PWM_BASE * 2 / 10, PWM_BASE * 2 / 10},
    {SHORT_DURATION, 0,                   PWM_BASE * 3 / 10, PWM_BASE * 3 / 10},
    {SHORT_DURATION, 0,                   PWM_BASE * 4 / 10, PWM_BASE * 4 / 10},
    {SHORT_DURATION, 0,                   PWM_BASE * 5 / 10, PWM_BASE * 5 / 10},
    {SHORT_DURATION, 0,                   PWM_BASE * 6 / 10, PWM_BASE * 6 / 10},
    {SHORT_DURATION, 0,                   PWM_BASE * 7 / 10, PWM_BASE * 7 / 10},
    {SHORT_DURATION, 0,                   PWM_BASE * 8 / 10, PWM_BASE * 8 / 10},
    {SHORT_DURATION, 0,                   PWM_BASE * 9 / 10, PWM_BASE * 9 / 10},
    {SHORT_DURATION, 0,                   0,                 0},
    {SHORT_DURATION, 0,                   0,                 0}
};

const PWMSchedule test_v_increasing = {
    {SHORT_DURATION, 0,                   0,                 0},
    {SHORT_DURATION, 0,                   PWM_BASE * 1 / 10, 0},
    {SHORT_DURATION, 0,                   PWM_BASE * 2 / 10, 0},
    {SHORT_DURATION, 0,                   PWM_BASE * 3 / 10, 0},
    {SHORT_DURATION, 0,                   PWM_BASE * 4 / 10, 0},
    {SHORT_DURATION, 0,                   PWM_BASE * 5 / 10, 0},
    {SHORT_DURATION, 0,                   PWM_BASE * 6 / 10, 0},
    {SHORT_DURATION, 0,                   PWM_BASE * 7 / 10, 0},
    {SHORT_DURATION, 0,                   PWM_BASE * 8 / 10, 0},
    {SHORT_DURATION, 0,                   PWM_BASE * 9 / 10, 0},
    {SHORT_DURATION, 0,                   0,                 0},
    {SHORT_DURATION, 0,                   0,                 0}
};

const PWMSchedule test_v_decreasing = {
    {SHORT_DURATION, 0,                   0,         0},
    {SHORT_DURATION, PWM_BASE * 1 / 10,   0,         PWM_BASE * 1 / 10},
    {SHORT_DURATION, PWM_BASE * 2 / 10,   0,         PWM_BASE * 2 / 10},
    {SHORT_DURATION, PWM_BASE * 3 / 10,   0,         PWM_BASE * 3 / 10},
    {SHORT_DURATION, PWM_BASE * 4 / 10,   0,         PWM_BASE * 4 / 10},
    {SHORT_DURATION, PWM_BASE * 5 / 10,   0,         PWM_BASE * 5 / 10},
    {SHORT_DURATION, PWM_BASE * 6 / 10,   0,         PWM_BASE * 6 / 10},
    {SHORT_DURATION, PWM_BASE * 7 / 10,   0,         PWM_BASE * 7 / 10},
    {SHORT_DURATION, PWM_BASE * 8 / 10,   0,         PWM_BASE * 8 / 10},
    {SHORT_DURATION, PWM_BASE * 9 / 10,   0,         PWM_BASE * 9 / 10},
    {SHORT_DURATION, 0,                   0,         0},
    {SHORT_DURATION, 0,                   0,         0}
};

const PWMSchedule test_w_increasing = {
    {SHORT_DURATION, 0,                   0,         0},
    {SHORT_DURATION, 0,                   0,         PWM_BASE * 1 / 10},
    {SHORT_DURATION, 0,                   0,         PWM_BASE * 2 / 10},
    {SHORT_DURATION, 0,                   0,         PWM_BASE * 3 / 10},
    {SHORT_DURATION, 0,                   0,         PWM_BASE * 4 / 10},
    {SHORT_DURATION, 0,                   0,         PWM_BASE * 5 / 10},
    {SHORT_DURATION, 0,                   0,         PWM_BASE * 6 / 10},
    {SHORT_DURATION, 0,                   0,         PWM_BASE * 7 / 10},
    {SHORT_DURATION, 0,                   0,         PWM_BASE * 8 / 10},
    {SHORT_DURATION, 0,                   0,         PWM_BASE * 9 / 10},
    {SHORT_DURATION, 0,                   0,         0},
    {SHORT_DURATION, 0,                   0,         0}
};

const PWMSchedule test_w_decreasing = {
    {SHORT_DURATION, 0,                   0,         0},
    {SHORT_DURATION, PWM_BASE * 1 / 10,   PWM_BASE * 1 / 10, 0},
    {SHORT_DURATION, PWM_BASE * 2 / 10,   PWM_BASE * 2 / 10, 0},
    {SHORT_DURATION, PWM_BASE * 3 / 10,   PWM_BASE * 3 / 10, 0},
    {SHORT_DURATION, PWM_BASE * 4 / 10,   PWM_BASE * 4 / 10, 0},
    {SHORT_DURATION, PWM_BASE * 5 / 10,   PWM_BASE * 5 / 10, 0},
    {SHORT_DURATION, PWM_BASE * 6 / 10,   PWM_BASE * 6 / 10, 0},
    {SHORT_DURATION, PWM_BASE * 7 / 10,   PWM_BASE * 7 / 10, 0},
    {SHORT_DURATION, PWM_BASE * 8 / 10,   PWM_BASE * 8 / 10, 0},
    {SHORT_DURATION, PWM_BASE * 9 / 10,   PWM_BASE * 9 / 10, 0},
    {SHORT_DURATION, 0,                   0,         0},
    {SHORT_DURATION, 0,                   0,         0}
};