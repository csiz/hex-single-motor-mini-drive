#pragma once

#include "constants.hpp"

#include <cstdint>
#include <cstddef>

// Type Definitions
// ----------------

enum struct DriverState {
    OFF,
    FREEWHEEL,
    DRIVE_POS,
    DRIVE_NEG,
    DRIVE_SMOOTH_POS,
    DRIVE_SMOOTH_NEG,
    HOLD,
    TEST_SCHEDULE,
};

// Stage for a PWM test schedule.
struct PWMStage {
    uint16_t duration; // Duration in PWM cycles.
    uint16_t u; // PWM duty cycle for U phase.
    uint16_t v; // PWM duty cycle for V phase.
    uint16_t w; // PWM duty cycle for W phase.
};


// Number of steps in test schedules.
const size_t SCHEDULE_SIZE = 12;

// Motor driving PWM schedule.
using PWMSchedule = PWMStage[SCHEDULE_SIZE];



// Driver State
// ------------

// Motor driver state.
extern DriverState driver_state;

extern uint16_t duration_till_timeout;

extern uint16_t hold_u_pwm_duty;
extern uint16_t hold_v_pwm_duty;
extern uint16_t hold_w_pwm_duty;

extern uint16_t pwm_command;

extern uint8_t leading_angle;

extern PWMSchedule const* test_schedule_pointer;
extern size_t test_schedule_counter;
extern size_t test_schedule_stage;

// Functions to set motor driver state.

void motor_drive_neg(uint16_t pwm, uint16_t timeout);
void motor_drive_pos(uint16_t pwm, uint16_t timeout);
void motor_drive_smooth_pos(uint16_t pwm, uint16_t timeout, uint16_t new_leading_angle);
void motor_drive_smooth_neg(uint16_t pwm, uint16_t timeout, uint16_t new_leading_angle);
void motor_hold(uint16_t u, uint16_t v, uint16_t w, uint16_t timeout);
void motor_break();
void motor_freewheel();
void motor_start_test(const PWMSchedule & schedule);



// Motor control tables
// --------------------

// Motor voltage fraction for the 6-step commutation.
const uint16_t motor_voltage_table_pos[6][3] = {
    {0,        PWM_BASE, 0       },
    {0,        PWM_BASE, PWM_BASE},
    {0,        0,        PWM_BASE},
    {PWM_BASE, 0,        PWM_BASE},
    {PWM_BASE, 0,        0       },
    {PWM_BASE, PWM_BASE, 0       },
};

// Surpirsingly good schedule for the 6-step commutation.
const uint16_t motor_voltage_table_neg[6][3] {
    {0,        0,        PWM_BASE},
    {PWM_BASE, 0,        PWM_BASE},
    {PWM_BASE, 0,        0       },
    {PWM_BASE, PWM_BASE, 0       },
    {0,        PWM_BASE, 0       },
    {0,        PWM_BASE, PWM_BASE},
};

const uint16_t phases_waveform[256] = {
	1330, 1349, 1366, 1383, 1399, 1414, 1429, 1442, 1455, 1466, 1477, 1487, 1496, 1504, 1511, 1518,
	1523, 1527, 1531, 1534, 1535, 1536, 1536, 1535, 1533, 1530, 1526, 1521, 1516, 1509, 1501, 1493,
	1484, 1474, 1462, 1450, 1438, 1424, 1409, 1394, 1378, 1361, 1343, 1324, 1304, 1284, 1263, 1241,
	1219, 1195, 1171, 1147, 1121, 1095, 1068, 1041, 1013,  984,  955,  925,  895,  864,  832,  800,
	 768,  735,  702,  668,  634,  599,  565,  529,  494,  458,  422,  385,  349,  312,  275,  238,
	 200,  163,  126,   88,   50,   13,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
	   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
	   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
	   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
	   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
	   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,   13,   50,   88,  126,  163,
	 200,  238,  275,  312,  349,  385,  422,  458,  494,  529,  565,  599,  634,  668,  702,  735,
	 768,  800,  832,  864,  895,  925,  955,  984, 1013, 1041, 1068, 1095, 1121, 1147, 1171, 1195,
	1219, 1241, 1263, 1284, 1304, 1324, 1343, 1361, 1378, 1394, 1409, 1424, 1438, 1450, 1462, 1474,
	1484, 1493, 1501, 1509, 1516, 1521, 1526, 1530, 1533, 1535, 1536, 1536, 1535, 1534, 1531, 1527,
	1523, 1518, 1511, 1504, 1496, 1487, 1477, 1466, 1455, 1442, 1429, 1414, 1399, 1383, 1366, 1349
};



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