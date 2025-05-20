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

bool is_motor_stopped();

// Test procedures
// ---------------


const uint16_t pwm_test = pwm_base / 2;

const uint16_t short_duration = history_size / schedule_size;

const PWMSchedule test_all_permutations = {
    {short_duration, 0,         0,         0},
    {short_duration, pwm_test,  0,         0}, // Positive U
    {short_duration, 0,         0,         0},
    {short_duration, 0,         pwm_test,  0}, // Positive V
    {short_duration, 0,         0,         0},
    {short_duration, 0,         0,         pwm_test}, // Positive W
    {short_duration, 0,         0,         0},
    {short_duration, 0,         pwm_test,  pwm_test}, // Negative U
    {short_duration, 0,         0,         0},
    {short_duration, pwm_test,  0,         pwm_test}, // Negative V
    {short_duration, 0,         0,         0},
    {short_duration, pwm_test,  pwm_test,  0} // Negative W
};


const PWMSchedule test_ground_short = {
    {short_duration, 0,         0,         0},
    {short_duration, 0,         0,         0},
    {short_duration, 0,         0,         0},
    {short_duration, 0,         0,         0},
    {short_duration, 0,         0,         0},
    {short_duration, 0,         0,         0},
    {short_duration, 0,         0,         0},
    {short_duration, 0,         0,         0},
    {short_duration, 0,         0,         0},
    {short_duration, 0,         0,         0},
    {short_duration, 0,         0,         0},
    {short_duration, 0,         0,         0},
};

const PWMSchedule test_positive_short = {
    {short_duration, pwm_max,   pwm_max,   pwm_max},
    {short_duration, pwm_max,   pwm_max,   pwm_max},
    {short_duration, pwm_max,   pwm_max,   pwm_max},
    {short_duration, pwm_max,   pwm_max,   pwm_max},
    {short_duration, pwm_max,   pwm_max,   pwm_max},
    {short_duration, pwm_max,   pwm_max,   pwm_max},
    {short_duration, pwm_max,   pwm_max,   pwm_max},
    {short_duration, pwm_max,   pwm_max,   pwm_max},
    {short_duration, pwm_max,   pwm_max,   pwm_max},
    {short_duration, pwm_max,   pwm_max,   pwm_max},
    {short_duration, pwm_max,   pwm_max,   pwm_max},
    {short_duration, pwm_max,   pwm_max,   pwm_max},
};

const PWMSchedule test_u_directions = {
    {short_duration, 0,         0,         0},
    {short_duration, 0,         pwm_test,  pwm_test},
    {short_duration, 0,         0,         0},
    {short_duration, pwm_test,  0,         0},
    {short_duration, 0,         0,         0},
    {short_duration, 0,         pwm_test,  pwm_test},
    {short_duration, 0,         0,         0},
    {short_duration, pwm_test,  0,         0},
    {short_duration, 0,         0,         0},
    {short_duration, 0,         pwm_test,  pwm_test},
    {short_duration, 0,         0,         0},
    {short_duration, 0,         0,         0},
};

const PWMSchedule test_u_increasing = {
    {short_duration, 0,                   0,         0},
    {short_duration, pwm_base * 1 / 10,   0,         0},
    {short_duration, pwm_base * 2 / 10,   0,         0},
    {short_duration, pwm_base * 3 / 10,   0,         0},
    {short_duration, pwm_base * 4 / 10,   0,         0},
    {short_duration, pwm_base * 5 / 10,   0,         0},
    {short_duration, pwm_base * 6 / 10,   0,         0},
    {short_duration, pwm_base * 7 / 10,   0,         0},
    {short_duration, pwm_base * 8 / 10,   0,         0},
    {short_duration, pwm_base * 9 / 10,   0,         0},
    {short_duration, 0,                   0,         0},
    {short_duration, 0,                   0,         0},
};

const PWMSchedule test_u_decreasing = {
    {short_duration, 0,                   0,                 0},
    {short_duration, 0,                   pwm_base * 1 / 10, pwm_base * 1 / 10},
    {short_duration, 0,                   pwm_base * 2 / 10, pwm_base * 2 / 10},
    {short_duration, 0,                   pwm_base * 3 / 10, pwm_base * 3 / 10},
    {short_duration, 0,                   pwm_base * 4 / 10, pwm_base * 4 / 10},
    {short_duration, 0,                   pwm_base * 5 / 10, pwm_base * 5 / 10},
    {short_duration, 0,                   pwm_base * 6 / 10, pwm_base * 6 / 10},
    {short_duration, 0,                   pwm_base * 7 / 10, pwm_base * 7 / 10},
    {short_duration, 0,                   pwm_base * 8 / 10, pwm_base * 8 / 10},
    {short_duration, 0,                   pwm_base * 9 / 10, pwm_base * 9 / 10},
    {short_duration, 0,                   0,                 0},
    {short_duration, 0,                   0,                 0}
};

const PWMSchedule test_v_increasing = {
    {short_duration, 0,                   0,                 0},
    {short_duration, 0,                   pwm_base * 1 / 10, 0},
    {short_duration, 0,                   pwm_base * 2 / 10, 0},
    {short_duration, 0,                   pwm_base * 3 / 10, 0},
    {short_duration, 0,                   pwm_base * 4 / 10, 0},
    {short_duration, 0,                   pwm_base * 5 / 10, 0},
    {short_duration, 0,                   pwm_base * 6 / 10, 0},
    {short_duration, 0,                   pwm_base * 7 / 10, 0},
    {short_duration, 0,                   pwm_base * 8 / 10, 0},
    {short_duration, 0,                   pwm_base * 9 / 10, 0},
    {short_duration, 0,                   0,                 0},
    {short_duration, 0,                   0,                 0}
};

const PWMSchedule test_v_decreasing = {
    {short_duration, 0,                   0,         0},
    {short_duration, pwm_base * 1 / 10,   0,         pwm_base * 1 / 10},
    {short_duration, pwm_base * 2 / 10,   0,         pwm_base * 2 / 10},
    {short_duration, pwm_base * 3 / 10,   0,         pwm_base * 3 / 10},
    {short_duration, pwm_base * 4 / 10,   0,         pwm_base * 4 / 10},
    {short_duration, pwm_base * 5 / 10,   0,         pwm_base * 5 / 10},
    {short_duration, pwm_base * 6 / 10,   0,         pwm_base * 6 / 10},
    {short_duration, pwm_base * 7 / 10,   0,         pwm_base * 7 / 10},
    {short_duration, pwm_base * 8 / 10,   0,         pwm_base * 8 / 10},
    {short_duration, pwm_base * 9 / 10,   0,         pwm_base * 9 / 10},
    {short_duration, 0,                   0,         0},
    {short_duration, 0,                   0,         0}
};

const PWMSchedule test_w_increasing = {
    {short_duration, 0,                   0,         0},
    {short_duration, 0,                   0,         pwm_base * 1 / 10},
    {short_duration, 0,                   0,         pwm_base * 2 / 10},
    {short_duration, 0,                   0,         pwm_base * 3 / 10},
    {short_duration, 0,                   0,         pwm_base * 4 / 10},
    {short_duration, 0,                   0,         pwm_base * 5 / 10},
    {short_duration, 0,                   0,         pwm_base * 6 / 10},
    {short_duration, 0,                   0,         pwm_base * 7 / 10},
    {short_duration, 0,                   0,         pwm_base * 8 / 10},
    {short_duration, 0,                   0,         pwm_base * 9 / 10},
    {short_duration, 0,                   0,         0},
    {short_duration, 0,                   0,         0}
};

const PWMSchedule test_w_decreasing = {
    {short_duration, 0,                   0,         0},
    {short_duration, pwm_base * 1 / 10,   pwm_base * 1 / 10, 0},
    {short_duration, pwm_base * 2 / 10,   pwm_base * 2 / 10, 0},
    {short_duration, pwm_base * 3 / 10,   pwm_base * 3 / 10, 0},
    {short_duration, pwm_base * 4 / 10,   pwm_base * 4 / 10, 0},
    {short_duration, pwm_base * 5 / 10,   pwm_base * 5 / 10, 0},
    {short_duration, pwm_base * 6 / 10,   pwm_base * 6 / 10, 0},
    {short_duration, pwm_base * 7 / 10,   pwm_base * 7 / 10, 0},
    {short_duration, pwm_base * 8 / 10,   pwm_base * 8 / 10, 0},
    {short_duration, pwm_base * 9 / 10,   pwm_base * 9 / 10, 0},
    {short_duration, 0,                   0,         0},
    {short_duration, 0,                   0,         0}
};