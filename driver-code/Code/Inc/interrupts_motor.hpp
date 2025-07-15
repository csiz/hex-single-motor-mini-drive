#pragma once

#include "interrupts_pid.hpp"
#include "interrupts_data.hpp"

#include "type_definitions.hpp"
#include "constants.hpp"
#include "byte_handling.hpp"
#include "math_utils.hpp"
#include "integer_math.hpp"


// Motor Control Functions
// =======================

// Linearly add two motor outputs together; note that we don't do bounds checking here.
static inline MotorOutputs add_motor_outputs(
    MotorOutputs const& a,
    MotorOutputs const& b
){
    return MotorOutputs{
        .enable_flags = enable_flags_all,
        .u_duty = static_cast<uint16_t>(a.u_duty + b.u_duty),
        .v_duty = static_cast<uint16_t>(a.v_duty + b.v_duty),
        .w_duty = static_cast<uint16_t>(a.w_duty + b.w_duty)
    };
}

static inline MotorOutputs update_motor_6_sector(
    Drive6Sector const& sector_parameters,
    FullReadout const& readout
){

    // Update the sector variable.
    const uint8_t hall_sector = get_hall_sector(readout.state_flags & hall_state_bit_mask);

    // Check if the magnet is present.
    const bool angle_valid = hall_sector < hall_sector_base;

    if (not angle_valid) return breaking_motor_outputs;

    auto const& motor_sector_driving_table = sector_parameters.pwm_target >= 0 ? 
        motor_sector_driving_positive : 
        motor_sector_driving_negative;

    // Get the voltage for the three phases from the table.

    const uint16_t voltage_phase_u = motor_sector_driving_table[hall_sector][0];
    const uint16_t voltage_phase_v = motor_sector_driving_table[hall_sector][1];
    const uint16_t voltage_phase_w = motor_sector_driving_table[hall_sector][2];

    const uint16_t pwm = faster_abs(sector_parameters.pwm_target);

    return MotorOutputs{
        .enable_flags = enable_flags_all,
        .u_duty = static_cast<uint16_t>(voltage_phase_u * pwm / pwm_base),
        .v_duty = static_cast<uint16_t>(voltage_phase_v * pwm / pwm_base),
        .w_duty = static_cast<uint16_t>(voltage_phase_w * pwm / pwm_base)
    };
}

static inline int16_t get_periodic_angle(FullReadout const& readout, int16_t const& zero_offset, int16_t const& angular_speed) {
    return normalize_angle(angular_speed * readout.readout_number / speed_fixed_point + zero_offset);
}

static inline int16_t get_zero_offset(FullReadout const& readout, int16_t const& target_angle, int16_t const& angular_speed) {
    return signed_angle(target_angle - angular_speed * readout.readout_number / speed_fixed_point);
}

static inline MotorOutputs pwm_at_angle(
    const uint16_t pwm_target,
    const int16_t target_angle
){
    // Get the voltage for the three phases from the table.
    const uint16_t voltage_phase_u = get_phase_pwm(target_angle);
    const uint16_t voltage_phase_v = get_phase_pwm(target_angle - third_circle);
    const uint16_t voltage_phase_w = get_phase_pwm(target_angle - two_thirds_circle);

    return MotorOutputs{
        .enable_flags = enable_flags_all,
        .u_duty = static_cast<uint16_t>(voltage_phase_u * pwm_target / pwm_base),
        .v_duty = static_cast<uint16_t>(voltage_phase_v * pwm_target / pwm_base),
        .w_duty = static_cast<uint16_t>(voltage_phase_w * pwm_target / pwm_base)
    };
}

// Drive the inductors around the circle at the specified PWM and speed. Open loop control.
static inline MotorOutputs update_motor_periodic(
    DrivePeriodic const& periodic_parameters,
    FullReadout const& readout,
    PIDControlState & pid_state
){
    const int pwm_target = periodic_parameters.pwm_target;

    const int target_angle = get_periodic_angle(
        readout,
        periodic_parameters.zero_offset,
        periodic_parameters.angular_speed
    );

    return pwm_at_angle(pwm_target, target_angle);
}

const int probing_angular_speed = 32;

// Drive the motor using FOC targeting a PWM value. The current is controlled to be as 
// close to 90 degrees ahead of the magnetic angle as possible; stray currents absorbed.
static inline MotorOutputs update_motor_smooth(
    DriveSmooth & smooth_parameters,
    PIDControlState & pid_state,
    FullReadout const& readout
){
    // Check if we have an accurate readout angle.
    const bool emf_fix = readout.state_flags & emf_fix_bit_mask;

    const int16_t pwm_target = (
        emf_fix ? 
        clip_to(-pwm_max, +pwm_max, smooth_parameters.pwm_target) :
        clip_to(-pwm_max_hold, +pwm_max_hold, smooth_parameters.pwm_target)
    );

    const int pwm_change = pwm_target - smooth_parameters.pwm_active;

    smooth_parameters.pwm_active += signed_ceil_div(pwm_change * observer_parameters.pwm_change_ki, observer_fixed_point);

    // Get the abs value of the target PWM.
    const uint16_t abs_pwm = faster_abs(smooth_parameters.pwm_active);

    // Base the direction on the sign of the target PWM.
    const int direction = sign(smooth_parameters.pwm_active);


    // Ideally the inductor current is exactly 90 degrees ahead of the magnetic angle.
    // 
    // Of course, the inductors take a while to charge and the rotor is producing an EMF
    // which all interacts with the current. However, the current that we end up measuring
    // should be as close to the 90 degrees as possible for maximum torque per current use.
    const int ideal_lead_angle = direction * quarter_circle;
    
    // The ideal angle.
    const int ideal_angle = readout.angle + ideal_lead_angle;

    if (emf_fix) {
        // If we have an accurate position, we can use it to adjust our control.

        // Get the error between the measured current and the ideal current angle.
        const int current_angle_error = signed_angle(readout.inductor_angle - ideal_angle);

        // Adjust the target angle to keep the alpha current small; reset if the motor is not moving.
        pid_state.current_angle_control = compute_pid_control(
            pid_parameters.current_angle_gains,
            pid_state.current_angle_control,
            current_angle_error * (half_circle - faster_abs(current_angle_error)) / half_circle,
            0
        );
        
        // Compensate the target angle by the control output. At high speed we need
        // to lead by more than 90 degrees to compensate for the RL time constant.
        const int16_t target_angle = normalize_angle(ideal_angle + pid_state.current_angle_control.output);

        // Update the zero offset in case we lose the emf fix.
        smooth_parameters.zero_offset = get_zero_offset(readout, target_angle, direction * probing_angular_speed);

        return pwm_at_angle(abs_pwm, target_angle);
    } else {
        // If we don't have an accurate position, we need drive the motor open loop until we get an EMF fix.
        const int probing_angle = get_periodic_angle(readout, smooth_parameters.zero_offset, direction * probing_angular_speed);

        // Reset the current angle control; it needs to start from 0 at low speed.
        pid_state.current_angle_control = {};

        // Output a capped PWM at the probing angle; moving in a circle slowly.
        return pwm_at_angle(abs_pwm, probing_angle);
    }
}

static inline MotorOutputs update_motor_torque(
    DriveTorque const& torque_parameters,
    FullReadout const& readout,
    PIDControlState & pid_state
){
    // Calculate the target current in fixed point format.
    const int16_t current_target = torque_parameters.current_target;

    // Update the PID control for the torque.
    pid_state.torque_control = compute_pid_control(
        pid_parameters.torque_gains,
        pid_state.torque_control,
        readout.beta_current,
        current_target
    );

    // Get the target PWM after torque control.
    const int16_t pwm_target = pid_state.torque_control.output;

    DriveSmooth smooth_parameters = {
        .duration = torque_parameters.duration,
        .zero_offset = torque_parameters.zero_offset,
        .pwm_target = pwm_target
    };

    return update_motor_smooth(
        smooth_parameters,
        pid_state,
        readout
    );
}

static inline MotorOutputs update_motor_battery_power(
    DriveBatteryPower const& battery_power_parameters,
    FullReadout const& readout,
    PIDControlState & pid_state
){
    const bool direction_is_negative = battery_power_parameters.power_target < 0;

    pid_state.battery_power_control = compute_pid_control(
        pid_parameters.battery_power_gains,
        pid_state.battery_power_control,
        max(0, -readout.total_power),
        faster_abs(battery_power_parameters.power_target)
    );

    // Get the target PWM after power control.
    const int16_t pwm_target = clip_to(-pwm_max, pwm_max, (direction_is_negative ? -1 : 1) * pid_state.battery_power_control.output);

    DriveSmooth smooth_parameters = {
        .duration = battery_power_parameters.duration,
        .zero_offset = battery_power_parameters.zero_offset,
        .pwm_target = pwm_target
    };

    return update_motor_smooth(
        smooth_parameters,
        pid_state,
        readout
    );
}


static inline MotorOutputs update_motor_schedule(
    DriveSchedule & schedule_parameters,
    FullReadout const& readout
){
    PWMSchedule const& schedule = *schedule_parameters.pointer;
    PWMStage const& schedule_stage = schedule[schedule_parameters.current_stage];

    schedule_parameters.stage_counter += 1;

    if (schedule_parameters.stage_counter >= schedule_stage.duration) {
        // Move to the next stage in the schedule.
        schedule_parameters.current_stage += 1;
        schedule_parameters.stage_counter = 0;
    }
    
    return MotorOutputs{
        .enable_flags = enable_flags_all,
        .u_duty = schedule_stage.u_duty,
        .v_duty = schedule_stage.v_duty,
        .w_duty = schedule_stage.w_duty
    };
}


// Motor control
// -------------

static inline bool set_breaking_control(
    MotorOutputs & active_motor_outputs,
    DriverState & driver_state,
    DriverParameters & driver_parameters
){
    // Set the active motor outputs to breaking.
    active_motor_outputs = breaking_motor_outputs;

    // Set the driver state to OFF.
    driver_state = DriverState::OFF;

    // Reset the driver parameters.
    driver_parameters = null_driver_parameters;

    return false; // No critical error.
}

static inline bool update_motor_control(
    MotorOutputs & active_motor_outputs,
    DriverState & driver_state,
    DriverParameters & driver_parameters,
    PIDControlState & pid_state,
    DriverState const pending_state,
    DriverParameters const& pending_parameters,
    FullReadout const& readout
){
    // Copy the pending command if we have one.
    switch(pending_state){
        case DriverState::NO_CHANGE:
            break;
        
        case DriverState::OFF:
            driver_state = DriverState::OFF;
            driver_parameters = null_driver_parameters;
            break;

        case DriverState::FREEWHEEL:
            driver_state = DriverState::FREEWHEEL;
            driver_parameters = null_driver_parameters;
            break;

        case DriverState::HOLD:
            driver_state = DriverState::HOLD;
            driver_parameters = DriverParameters{
                .hold = DriveHold {
                    .duration = static_cast<uint16_t>(clip_to(0, max_timeout, pending_parameters.hold.duration)),
                    .u_duty = static_cast<uint16_t>(clip_to(0, pwm_max_hold, pending_parameters.hold.u_duty)),
                    .v_duty = static_cast<uint16_t>(clip_to(0, pwm_max_hold, pending_parameters.hold.v_duty)),
                    .w_duty = static_cast<uint16_t>(clip_to(0, pwm_max_hold, pending_parameters.hold.w_duty))
                }
            };
            break;

        case DriverState::SCHEDULE:
            // We should not enter testing mode without a valid schedule.
            if (pending_parameters.schedule.pointer == nullptr) return true;
            driver_state = DriverState::SCHEDULE;
            driver_parameters = DriverParameters{
                .schedule = DriveSchedule{
                    .pointer = pending_parameters.schedule.pointer,
                    .current_stage = 0,
                    .stage_counter = 0
                }
            };
            // Clear the readouts buffer of old data.
            readout_history_reset();
            break;

        case DriverState::DRIVE_6_SECTOR:
            driver_state = DriverState::DRIVE_6_SECTOR;
            driver_parameters = DriverParameters{
                .sector = Drive6Sector{
                    .duration = static_cast<uint16_t>(clip_to(0, max_timeout, pending_parameters.sector.duration)),
                    .pwm_target = static_cast<int16_t>(clip_to(-pwm_max, pwm_max, pending_parameters.sector.pwm_target))
                }
            };
            break;

        case DriverState::DRIVE_PERIODIC:
            driver_state = DriverState::DRIVE_PERIODIC;
            driver_parameters = DriverParameters{
                .periodic = DrivePeriodic{
                    .duration = static_cast<uint16_t>(clip_to(0, max_timeout, pending_parameters.periodic.duration)),
                    .zero_offset = pending_parameters.periodic.zero_offset,
                    .pwm_target = static_cast<int16_t>(clip_to(0, pwm_max_hold, pending_parameters.periodic.pwm_target)),
                    .angular_speed = static_cast<int16_t>(clip_to(-max_angular_speed, max_angular_speed, pending_parameters.periodic.angular_speed))
                }
            };
            break;

        case DriverState::DRIVE_SMOOTH:
            driver_state = DriverState::DRIVE_SMOOTH;
            driver_parameters = DriverParameters{
                .smooth = DriveSmooth{
                    .duration = static_cast<uint16_t>(clip_to(0, max_timeout, pending_parameters.smooth.duration)),
                    .zero_offset = get_zero_offset(readout, readout.angle + sign(pending_parameters.smooth.pwm_target) * quarter_circle, probing_angular_speed),
                    .pwm_target = static_cast<int16_t>(clip_to(-pwm_max, +pwm_max, pending_parameters.smooth.pwm_target)),
                }
            };
            break;
            
        case DriverState::DRIVE_TORQUE:
            // Reset the PID for torque control unless we are switching between DRIVE_TORQUE commands.
            if (driver_state != DriverState::DRIVE_TORQUE) {
                pid_state.torque_control = PIDControl{};
            }
            driver_state = DriverState::DRIVE_TORQUE;
            driver_parameters = DriverParameters{
                .torque = DriveTorque{
                    .duration = static_cast<uint16_t>(clip_to(0, max_timeout, pending_parameters.torque.duration)),
                    .zero_offset = pending_parameters.torque.zero_offset,
                    .current_target = static_cast<int16_t>(clip_to(-max_drive_current, +max_drive_current, pending_parameters.torque.current_target)),
                }
            };
            break;

        case DriverState::DRIVE_BATTERY_POWER:
            if (driver_state != DriverState::DRIVE_BATTERY_POWER) {
                pid_state.battery_power_control = PIDControl{};
            }
            driver_state = DriverState::DRIVE_BATTERY_POWER;
            driver_parameters = DriverParameters{
                .battery_power = DriveBatteryPower{
                    .duration = static_cast<uint16_t>(clip_to(0, max_timeout, pending_parameters.battery_power.duration)),
                    .zero_offset = pending_parameters.battery_power.zero_offset,
                    .power_target = static_cast<int16_t>(clip_to(-max_drive_power, +max_drive_power, pending_parameters.battery_power.power_target)),
                }
            };
            break;

    }

    // Update the active state.
    switch (driver_state) {

        // The active driver state should not be NO_CHANGE.
        case DriverState::NO_CHANGE:
            return true;

        case DriverState::OFF:
            active_motor_outputs = breaking_motor_outputs;
            return false;

        case DriverState::FREEWHEEL:
            active_motor_outputs = MotorOutputs{
                .enable_flags = enable_flags_none,
                .u_duty = 0,
                .v_duty = 0,
                .w_duty = 0
            };
            return false;

        case DriverState::HOLD:
            if (driver_parameters.hold.duration-- <= 0) {
                return set_breaking_control(active_motor_outputs, driver_state, driver_parameters);
            }

            // Set the motor outputs to hold the current settings.
            active_motor_outputs = MotorOutputs{
                .enable_flags = enable_flags_all,
                .u_duty = driver_parameters.hold.u_duty,
                .v_duty = driver_parameters.hold.v_duty,
                .w_duty = driver_parameters.hold.w_duty
            };
            return false;

        case DriverState::SCHEDULE:
            // We're done at the end of the schedule.
            if (driver_parameters.schedule.pointer == nullptr or driver_parameters.schedule.current_stage >= schedule_size) {
                return set_breaking_control(active_motor_outputs, driver_state, driver_parameters);
            }
            
            active_motor_outputs = update_motor_schedule(driver_parameters.schedule, readout);

            return false;

        case DriverState::DRIVE_6_SECTOR:
            if (driver_parameters.sector.duration-- <= 0) {
                return set_breaking_control(active_motor_outputs, driver_state, driver_parameters);
            }

            // Update motor outputs for the 6 sector driving.
            active_motor_outputs = update_motor_6_sector(driver_parameters.sector, readout);

            return false;

        case DriverState::DRIVE_PERIODIC:
            if (driver_parameters.periodic.duration-- <= 0) {
                return set_breaking_control(active_motor_outputs, driver_state, driver_parameters);
            }

            active_motor_outputs = update_motor_periodic(
                driver_parameters.periodic,
                readout,
                pid_state
            );
            return false;
                

        case DriverState::DRIVE_SMOOTH:
            if (driver_parameters.smooth.duration-- <= 0) {
                return set_breaking_control(active_motor_outputs, driver_state, driver_parameters);
            }

            // Update the motor outputs for the smooth driving.
            active_motor_outputs = update_motor_smooth(driver_parameters.smooth, pid_state, readout);

            return false;


        case DriverState::DRIVE_TORQUE:
            if (driver_parameters.torque.duration-- <= 0) {
                return set_breaking_control(active_motor_outputs, driver_state, driver_parameters);
            }

            // Update the motor outputs for the torque driving.
            active_motor_outputs = update_motor_torque(driver_parameters.torque, readout, pid_state);

            return false;

        case DriverState::DRIVE_BATTERY_POWER:
            if (driver_parameters.battery_power.duration-- <= 0) {
                return set_breaking_control(active_motor_outputs, driver_state, driver_parameters);
            }
            
            // Break at the end of the command duration.
            active_motor_outputs = update_motor_battery_power(driver_parameters.battery_power, readout, pid_state);

            return false;
    }

    // If we get here, we have an unknown/corrupted driver state.
    return true;
}