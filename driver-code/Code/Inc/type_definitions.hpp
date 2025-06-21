#pragma once

#include <cstddef>
#include <cstdint>
#include <array>


// Most useful datatype for this system, the 3 phase coordinates.
struct ThreePhase {
    int u; // Value for U phase.
    int v; // Value for V phase.
    int w; // Value for W phase.
};


// Driver State
// ------------

enum struct DriverState : uint16_t {
    NO_CHANGE,
    OFF,
    FREEWHEEL,
    HOLD,
    SCHEDULE,
    DRIVE_6_SECTOR,
    DRIVE_SMOOTH,
};

// Motor duty cycle (compare register values).
struct MotorOutputs {
    uint16_t duration; // Duration in PWM cycles.
    uint16_t u_duty; // PWM duty cycle for U phase.
    uint16_t v_duty; // PWM duty cycle for V phase.
    uint16_t w_duty; // PWM duty cycle for W phase.
};

// Zeroed motor outputs, used to short circuit break the motor outputs.
const MotorOutputs null_motor_outputs = {};

// The hold command is the same as the motor outputs.
using DriveHold = MotorOutputs;

// Number of steps in test schedules.
const size_t schedule_size = 12;

// Motor driving PWM schedule.
using PWMSchedule = MotorOutputs[schedule_size];

// Pointer to a PWM schedule to run the test.
using DriveSchedule = PWMSchedule const*;

// Drive motor using the 6 sector commutation method.
struct Drive6Sector {
    uint16_t duration; // Duration for the command in pwm cycles.
    int16_t pwm_target;
};

// Drive the motor using FOC targeting an output current (torque).
struct DriveSmooth {
    uint16_t duration; // Duration for the command in pwm cycles.
    int16_t current_target;
    int16_t leading_angle;
};

// Drive parameters for each state.
union DriverParameters {
    DriveHold hold;
    DriveSchedule schedule;
    Drive6Sector sector;
    DriveSmooth smooth;
};

const DriverParameters null_driver_parameters = {};

using DriverData = std::tuple<MotorOutputs, DriverState, DriverParameters>;

const size_t driver_data_size = sizeof(MotorOutputs) + sizeof(DriverState) + sizeof(DriverParameters);

// Response data structures
// ------------------------

struct Readout{
    uint32_t pwm_commands;
    uint16_t readout_number;
    int16_t u_readout;
    int16_t v_readout;
    int16_t w_readout;
    uint16_t ref_readout;
    int16_t u_readout_diff;
    int16_t v_readout_diff;
    int16_t w_readout_diff;
    uint16_t position;
    int16_t angular_speed;
    int16_t instant_vcc_voltage;
    int16_t current_angle_offset;
};

struct FullReadout : public Readout {
    uint16_t tick_rate;
    uint16_t adc_update_rate;
    uint16_t temperature;
    uint16_t vcc_voltage;
    int16_t cycle_start_tick;
    int16_t cycle_end_tick;
    int16_t emf_voltage_angle_offset;
    uint16_t current_angle_offset_variance;
    uint16_t angle_variance;
    uint16_t angular_speed_variance;
    int16_t total_power;
    int16_t resistive_power;
    int16_t emf_power;
    int16_t inductive_power;
    int16_t current_angle_error;
    int16_t current_angle_control;
    int16_t current_angle_derivative;
    int16_t current_angle_integral;
    int16_t torque_error;
    int16_t torque_control;
    int16_t torque_derivative;
    int16_t torque_integral;
};


// Command data structures
// -----------------------

struct BasicCommand {
    uint16_t code;
    uint16_t timeout;
    int16_t pwm;
    int16_t leading_angle;
};

struct CurrentCalibration {
    int16_t u_factor;
    int16_t v_factor;
    int16_t w_factor;
    int16_t inductance_factor;
};

using TriggerAngles = std::array<std::array<uint16_t, 2>, 6>;
using TriggerAngleVariances = std::array<std::array<uint16_t, 2>, 6>;
using CenterAngles = std::array<uint16_t, 6>;
using CenterVariances = std::array<uint16_t, 6>;

struct PositionCalibration {
    TriggerAngles sector_transition_angles;
    TriggerAngleVariances sector_transition_variances;
    CenterAngles sector_center_angles;
    CenterVariances sector_center_variances;
    uint16_t initial_angular_speed_variance;
    uint16_t angular_acceleration_div_2_variance;
};


struct PIDGains {
    int16_t kp; // Proportional gain.
    int16_t ki; // Integral gain.
    int16_t kd; // Derivative gain.
    int16_t max_output;
};

struct PIDControl {
    int integral = 0; // Integral term.
    int derivative = 0; // Derivative term.
    int16_t error = 0; // Previous error for derivative calculation.
    int16_t output = 0; // Output value.
};

struct PIDParameters {
    PIDGains current_angle_gains;
    PIDGains torque_gains;
    PIDGains angular_speed_gains;
    PIDGains position_gains;
};