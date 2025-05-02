#pragma once

#include <cstddef>
#include <cstdint>

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
    uint16_t readout_number;
    uint16_t u_readout;
    uint16_t v_readout;
    uint16_t w_readout;
    uint16_t ref_readout;
    uint16_t position;
    uint32_t pwm_commands;
};

struct FullReadout{
    Readout readout;
    uint16_t tick_rate;
    uint16_t adc_update_rate;
    uint16_t hall_unobserved_rate;
    uint16_t hall_observed_rate;
    uint16_t temperature;
    uint16_t vcc_voltage;
    int16_t cycle_start_tick;
    int16_t cycle_end_tick;
};


// Command data structures
// -----------------------

struct CommandHeader {
    uint16_t code;
    uint16_t timeout;
    uint16_t pwm;
    uint16_t leading_angle;
};

struct CurrentFactors {
    uint16_t u_pos_factor;
    uint16_t u_neg_factor;
    uint16_t v_pos_factor;
    uint16_t v_neg_factor;
    uint16_t w_pos_factor;
    uint16_t w_neg_factor;
};

struct TriggerAngles {
    uint16_t trigger_angle[6][2];
    uint16_t trigger_angle_variance[6][2];
};