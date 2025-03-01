#pragma once

#include "data.hpp"

#include <cstdint>
#include <cstddef>


struct PWMStage {
    uint16_t duration;
    uint16_t u;
    uint16_t v;
    uint16_t w;
};

const size_t SCHEDULE_SIZE = 12;

using PWMSchedule = PWMStage[SCHEDULE_SIZE];

// Motor control
// -------------

enum struct DriverState {
    OFF,
    DRIVE_3PHASE,
    DRIVE_2PHASE,
    HOLD,
    TEST_SCHEDULE,
};

const uint16_t PWM_AUTORELOAD = 1535;
const uint16_t PWM_BASE = PWM_AUTORELOAD + 1;

// Maximum duty cycle for the high side mosfet needs to allow some off time for 
// the bootstrap capacitor to charge so it has enough voltage to turn mosfet on.
const uint16_t MIN_BOOTSTRAP_DUTY = 16; // 16/72MHz = 222ns
const uint16_t PWM_MAX = PWM_BASE - MIN_BOOTSTRAP_DUTY; // 1536/72MHz = 21.3us
// Sentinel value to indicate that the phase output should be floating.
const uint16_t PWM_FLOAT = PWM_BASE - 1;

const uint16_t PWM_MAX_HOLD = PWM_BASE * 2 / 10;

const uint16_t MAX_TIMEOUT = 0xFFFF;

// Motor control functions
void motor_control_init();

void drive_motor_2phase(uint16_t pwm, uint16_t timeout);
void drive_motor_3phase(uint16_t pwm, uint16_t timeout);

void hold_motor(uint16_t u, uint16_t v, uint16_t w, uint16_t timeout);

uint32_t get_combined_motor_pwm_duty();

void turn_motor_off();

void update_motor_control();

void update_motor_control_registers();

void start_test(const PWMSchedule & schedule);



// Calibration procedures
// ----------------------

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
    {SHORT_DURATION, PWM_FLOAT, PWM_FLOAT, PWM_FLOAT},
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
    {SHORT_DURATION, PWM_FLOAT, PWM_FLOAT, PWM_FLOAT}
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