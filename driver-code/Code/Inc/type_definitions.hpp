#pragma once

#include <cstddef>
#include <cstdint>
#include <array>

// PWM schedule
// ------------

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



// Response data structures
// ------------------------

struct Readout{
    uint32_t pwm_commands;
    uint16_t readout_number;
    uint16_t u_readout;
    uint16_t v_readout;
    uint16_t w_readout;
    uint16_t ref_readout;
    uint16_t position;
    int16_t speed;
    int16_t vcc_voltage;
    int16_t torque;
    int16_t hold;
    int16_t total_power;
    int16_t resistive_power;
};

struct FullReadout : public Readout {
    uint16_t tick_rate;
    uint16_t adc_update_rate;
    uint16_t hall_unobserved_rate;
    uint16_t hall_observed_rate;
    uint16_t temperature;
    uint16_t vcc_voltage;
    int16_t cycle_start_tick;
    int16_t cycle_end_tick;
    int16_t current_angle;
    int16_t current_angle_stdev;
};


// Command data structures
// -----------------------

struct CommandHeader {
    uint16_t code;
    uint16_t timeout;
    uint16_t pwm;
    uint16_t leading_angle;
};

struct CurrentCalibration {
    int16_t u_factor;
    int16_t v_factor;
    int16_t w_factor;
};

using TriggerAngles = std::array<std::array<int16_t, 2>, 6>;
using TriggerAngleVariances = std::array<std::array<int16_t, 2>, 6>;
using CenterAngles = std::array<int16_t, 6>;
using CenterVariances = std::array<int16_t, 6>;

struct PositionCalibration {
    TriggerAngles trigger_angles;
    TriggerAngleVariances trigger_angle_variances;
    CenterAngles sector_center_angles;
    CenterVariances sector_center_variances;
};