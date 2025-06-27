#pragma once

#include <cstddef>
#include <cstdint>
#include <array>
#include <tuple>

// Most useful datatype for this system, the 3 phase coordinates.
using ThreePhase = std::tuple<int, int, int>;

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
    DRIVE_TORQUE,
    DRIVE_BATTERY_POWER
};

// Motor duty cycle (compare register values and enable settings).
struct MotorOutputs {
    uint16_t enable_flags; // Flags to enable/disable the motor outputs.
    uint16_t u_duty; // PWM duty cycle for U phase.
    uint16_t v_duty; // PWM duty cycle for V phase.
    uint16_t w_duty; // PWM duty cycle for W phase.
};

// Enable all motor outputs (U, V, W).
const uint16_t enable_flags_all = 0b111;
// Enable U phase outputs.
const uint16_t enable_flags_u = 0b001;
// Enable V phase outputs.
const uint16_t enable_flags_v = 0b010;
// Enable W phase outputs.
const uint16_t enable_flags_w = 0b100;
// Disable all motor outputs.
const uint16_t enable_flags_none = 0b000;

// Zeroed motor outputs, used to short circuit break the motor outputs.
const MotorOutputs breaking_motor_outputs = {
    .enable_flags = enable_flags_all,
    .u_duty = 0,
    .v_duty = 0,
    .w_duty = 0
};


struct PWMStage {
    uint16_t duration; // Duration in PWM cycles.
    uint16_t u_duty; // PWM duty cycle for U phase.
    uint16_t v_duty; // PWM duty cycle for V phase.
    uint16_t w_duty; // PWM duty cycle for W phase.
};

// The hold command is the same as the motor outputs.
using DriveHold = PWMStage;

// Number of steps in test schedules.
const size_t schedule_size = 12;

// Motor driving PWM schedule.
using PWMSchedule = PWMStage[schedule_size];

// Pointer to a PWM schedule to run the test.
struct DriveSchedule {
    PWMSchedule const* pointer;
    uint16_t current_stage;
    uint16_t stage_counter;
};

// Drive motor using the 6 sector commutation method.
struct Drive6Sector {
    uint16_t duration; // Duration for the command in pwm cycles.
    int16_t pwm_target;
};

// Drive the motor using FOC targeting a PWM value.
struct DriveSmooth {
    uint16_t duration; // Duration for the command in pwm cycles.
    int16_t pwm_target;
    int16_t leading_angle;
};

// Drive the motor to a specific current target (torque target).
struct DriveTorque {
    uint16_t duration; // Duration for the command in pwm cycles.
    int16_t current_target; // Target current in fixed point format.
    int16_t leading_angle; // Leading angle for the current control.
};

// Drive the motor to a specific battery power drain.
struct DriveBatteryPower {
    uint16_t duration; // Duration for the command in pwm cycles.
    int16_t power_target; // Target power in fixed point format.
    int16_t leading_angle; // Leading angle for the power control.
};

// Drive parameters for each state.
union DriverParameters {
    DriveHold hold;
    DriveSchedule schedule;
    Drive6Sector sector;
    DriveSmooth smooth;
    DriveTorque torque;
    DriveBatteryPower battery_power;
};

const DriverParameters null_driver_parameters = {};


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
    int16_t position;
    uint16_t angle;
    int16_t angular_speed;
    int16_t instant_vcc_voltage;
};

struct FullReadout : public Readout {
    uint16_t tick_rate;
    uint16_t adc_update_rate;
    uint16_t temperature;
    uint16_t vcc_voltage;
    int16_t cycle_start_tick;
    int16_t cycle_end_tick;
    uint16_t angle_variance;
    uint16_t angular_speed_variance;
    int16_t alpha_current;
    int16_t beta_current;
    int16_t motor_constant;
    int16_t emf_voltage;
    int16_t emf_voltage_variance;
    int16_t residual_acceleration;
    int16_t total_power;
    int16_t resistive_power;
    int16_t emf_power;
    int16_t inductive_power;
    int16_t current_angle_error;
    int16_t current_angle_control;
    int16_t torque_error;
    int16_t torque_control;
    int16_t battery_power_error;
    int16_t battery_power_control;
    int16_t angular_speed_error;
    int16_t angular_speed_control;
    int16_t position_error;
    int16_t position_control;
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
    int16_t error = 0; // Previous error for derivative calculation.
    int16_t output = 0; // Output value.
};

struct PIDParameters {
    PIDGains current_angle_gains;
    PIDGains torque_gains;
    PIDGains battery_power_gains;
    PIDGains angular_speed_gains;
    PIDGains position_gains;
};

struct PIDControlState {
    PIDControl current_angle_control;
    PIDControl torque_control;
    PIDControl battery_power_control;
    PIDControl angular_speed_control;
    PIDControl position_control;
};

const PIDControlState null_pid_control_state = {
    .current_angle_control = {},
    .torque_control = {},
    .battery_power_control = {},
    .angular_speed_control = {},
    .position_control = {}
};