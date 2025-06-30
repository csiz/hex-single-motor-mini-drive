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

static inline MotorOutputs update_motor_6_sector(
    Drive6Sector const& sector_parameters,
    FullReadout & readout
){
    // Read new data from the hall sensors.
    const uint8_t hall_state = read_hall_sensors_state();

    // Update the sector variable.
    const uint8_t hall_sector = get_hall_sector(hall_state);

    // Check if the magnet is present.
    const bool angle_valid = hall_sector < hall_sector_base;

    const uint16_t angle_keep_bits = readout.angle &= ~(hall_state_bit_mask | angle_valid_bit_mask);
    readout.angle = angle_keep_bits | (hall_state << hall_state_bit_offset) | (angle_valid << angle_valid_bit_offset);

    auto const& motor_sector_driving_table = sector_parameters.pwm_target >= 0 ? 
        motor_sector_driving_positive : 
        motor_sector_driving_negative;

    // Get the voltage for the three phases from the table.

    const uint16_t voltage_phase_u = motor_sector_driving_table[hall_sector][0];
    const uint16_t voltage_phase_v = motor_sector_driving_table[hall_sector][1];
    const uint16_t voltage_phase_w = motor_sector_driving_table[hall_sector][2];

    const uint16_t pwm = abs(sector_parameters.pwm_target);

    return MotorOutputs{
        .enable_flags = enable_flags_all,
        .u_duty = static_cast<uint16_t>(voltage_phase_u * pwm / pwm_base),
        .v_duty = static_cast<uint16_t>(voltage_phase_v * pwm / pwm_base),
        .w_duty = static_cast<uint16_t>(voltage_phase_w * pwm / pwm_base)
    };
}

static inline MotorOutputs update_motor_smooth(
    DriveSmooth const& smooth_parameters,
    FullReadout & readout,
    PIDControlState & pid_state
){
    const bool emf_detected = readout.angle & emf_detected_bit_mask;

    const int emf_compensation = - emf_detected * readout.emf_voltage * emf_base / readout.vcc_voltage;

    const int max_allowed_target = max(0, +pwm_max_smooth - emf_compensation);
    const int min_allowed_target = min(0, -pwm_max_smooth - emf_compensation);

    const int pwm_target = emf_compensation + clip_to(min_allowed_target, max_allowed_target, smooth_parameters.pwm_target);

    // Get the abs value of the target PWM.
    const int abs_pwm = min(pwm_max, abs(pwm_target));

    // Base the direction on the sign of the target PWM.
    const int direction = 1 - (pwm_target < 0) * 2;

    // Adjust the target angle to keep the alpha current small; reset if the motor is not moving.
    pid_state.current_angle_control = (
        not emf_detected ? PIDControl{} : 
        compute_pid_control(
            pid_parameters.current_angle_gains,
            pid_state.current_angle_control,
            -readout.alpha_current * abs(readout.emf_voltage) / dq0_to_power_fixed_point,
            0
        )
    );

    readout.current_angle_error = pid_state.current_angle_control.error;
    readout.current_angle_control = pid_state.current_angle_control.output;


    const int target_lead_angle = direction * clip_to(0, half_circle, 
        smooth_parameters.leading_angle + 
        pid_state.current_angle_control.output / 4);

    const int rotor_angle = readout.angle & angle_bit_mask;

    const int target_angle = normalize_angle(rotor_angle + target_lead_angle);

    const uint16_t voltage_phase_u = get_phase_pwm(target_angle);
    const uint16_t voltage_phase_v = get_phase_pwm(target_angle - third_circle);
    const uint16_t voltage_phase_w = get_phase_pwm(target_angle - two_thirds_circle);
    

    return MotorOutputs{
        .enable_flags = enable_flags_all,
        .u_duty = static_cast<uint16_t>(voltage_phase_u * abs_pwm / pwm_base),
        .v_duty = static_cast<uint16_t>(voltage_phase_v * abs_pwm / pwm_base),
        .w_duty = static_cast<uint16_t>(voltage_phase_w * abs_pwm / pwm_base)
    };
}

static inline MotorOutputs update_motor_torque(
    DriveTorque const& torque_parameters,
    FullReadout & readout,
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

    readout.torque_error = pid_state.torque_control.error;
    readout.torque_control = pid_state.torque_control.output;


    // Get the target PWM after torque control.
    const int16_t pwm_target = pid_state.torque_control.output;

    return update_motor_smooth(
        DriveSmooth{
            .duration = torque_parameters.duration,
            .pwm_target = pwm_target,
            .leading_angle = torque_parameters.leading_angle
        },
        readout,
        pid_state
    );
}

static inline MotorOutputs update_motor_battery_power(
    DriveBatteryPower const& battery_power_parameters,
    FullReadout & readout,
    PIDControlState & pid_state
){
    const bool direction_is_negative = battery_power_parameters.power_target < 0;

    pid_state.battery_power_control = compute_pid_control(
        pid_parameters.battery_power_gains,
        pid_state.battery_power_control,
        max(0, -readout.total_power),
        abs(battery_power_parameters.power_target)
    );

    readout.battery_power_error = pid_state.battery_power_control.error;
    readout.battery_power_control = pid_state.battery_power_control.output;

    // Get the target PWM after power control.
    const int16_t pwm_target = clip_to(-pwm_max, pwm_max, (direction_is_negative ? -1 : 1) * pid_state.battery_power_control.output);

    return update_motor_smooth(
        DriveSmooth{
            .duration = battery_power_parameters.duration,
            .pwm_target = pwm_target,
            .leading_angle = battery_power_parameters.leading_angle
        },
        readout,
        pid_state
    );
}


// Motor control
// -------------
template<typename AnyDriverParameters>
static inline bool break_at_end_of_duration(
    DriverState & driver_state,
    DriverParameters & driver_parameters,
    AnyDriverParameters & driver_parameters_with_duration
){
    // Break at the end of the command duration.
    if (driver_parameters_with_duration.duration <= 0) {
        driver_state = DriverState::OFF;
        driver_parameters = null_driver_parameters;
        return true;
    } else {
        driver_parameters_with_duration.duration -= 1;
        return false;
    }
}

static inline MotorOutputs update_motor_control(
    DriverState const pending_state,
    DriverParameters const& pending_parameters,
    FullReadout & readout,
    DriverState & driver_state,
    DriverParameters & driver_parameters,
    PIDControlState & pid_state
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
            if (pending_parameters.schedule.pointer == nullptr) {
                error();
                return breaking_motor_outputs;
            }
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

        case DriverState::DRIVE_SMOOTH:
            driver_state = DriverState::DRIVE_SMOOTH;
            driver_parameters = DriverParameters{
                .smooth = DriveSmooth{
                    .duration = static_cast<uint16_t>(clip_to(0, max_timeout, pending_parameters.smooth.duration)),
                    .pwm_target = static_cast<int16_t>(clip_to(-pwm_max, +pwm_max, pending_parameters.smooth.pwm_target)),
                    .leading_angle = static_cast<int16_t>(clip_to(0, angle_base, pending_parameters.smooth.leading_angle))
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
                    .current_target = static_cast<int16_t>(clip_to(-max_drive_current, +max_drive_current, pending_parameters.torque.current_target)),
                    .leading_angle = static_cast<int16_t>(clip_to(0, angle_base, pending_parameters.torque.leading_angle))
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
                    .power_target = static_cast<int16_t>(clip_to(-max_drive_power, +max_drive_power, pending_parameters.battery_power.power_target)),
                    .leading_angle = static_cast<int16_t>(clip_to(0, angle_base, pending_parameters.battery_power.leading_angle))
                }
            };
            break;

    }

    // Update the active state.
    switch (driver_state) {

        // The active driver state should not be NO_CHANGE.
        case DriverState::NO_CHANGE:
            error();
            return breaking_motor_outputs;

        case DriverState::OFF:
            return breaking_motor_outputs;

        case DriverState::FREEWHEEL:
            return MotorOutputs{
                .enable_flags = enable_flags_none,
                .u_duty = 0,
                .v_duty = 0,
                .w_duty = 0
            };

        case DriverState::HOLD:
            // Set the motor outputs to hold the current settings.
            return break_at_end_of_duration(driver_state, driver_parameters, driver_parameters.hold) ?
                breaking_motor_outputs :
                MotorOutputs{
                    .enable_flags = enable_flags_all,
                    .u_duty = driver_parameters.hold.u_duty,
                    .v_duty = driver_parameters.hold.v_duty,
                    .w_duty = driver_parameters.hold.w_duty
                };

        case DriverState::SCHEDULE:
            // We're done at the end of the schedule.
            if (driver_parameters.schedule.pointer == nullptr or driver_parameters.schedule.current_stage >= schedule_size) {
                driver_state = DriverState::OFF;
                driver_parameters = null_driver_parameters;
                return breaking_motor_outputs;
            } else {
                PWMSchedule const& schedule = *driver_parameters.schedule.pointer;
                PWMStage const& schedule_stage = schedule[driver_parameters.schedule.current_stage];
    
                driver_parameters.schedule.stage_counter += 1;
                
                if (driver_parameters.schedule.stage_counter >= schedule_stage.duration) {
                    // Move to the next stage in the schedule.
                    driver_parameters.schedule.current_stage += 1;
                    driver_parameters.schedule.stage_counter = 0;
                }
                
                return MotorOutputs{
                    .enable_flags = enable_flags_all,
                    .u_duty = schedule_stage.u_duty,
                    .v_duty = schedule_stage.v_duty,
                    .w_duty = schedule_stage.w_duty
                };
            }

        case DriverState::DRIVE_6_SECTOR:
            // Update motor outputs for the 6 sector driving.
            return break_at_end_of_duration(driver_state, driver_parameters, driver_parameters.sector) ?
                breaking_motor_outputs :
                update_motor_6_sector(driver_parameters.sector, readout);

        case DriverState::DRIVE_SMOOTH:
            // Update the motor outputs for the smooth driving.
            return break_at_end_of_duration(driver_state, driver_parameters, driver_parameters.smooth) ?
                breaking_motor_outputs :
                update_motor_smooth(driver_parameters.smooth, readout, pid_state);

        case DriverState::DRIVE_TORQUE:
            // Update the motor outputs for the torque driving.
            return break_at_end_of_duration(driver_state, driver_parameters, driver_parameters.torque) ?
                breaking_motor_outputs :
                update_motor_torque(driver_parameters.torque, readout, pid_state);

        case DriverState::DRIVE_BATTERY_POWER:
            // Break at the end of the command duration.
            return break_at_end_of_duration(driver_state, driver_parameters, driver_parameters.battery_power) ?
                breaking_motor_outputs :
                update_motor_battery_power(driver_parameters.battery_power, readout, pid_state);
    }

    // If we get here, we have an unknown/corrupted driver state.
    error();
    return breaking_motor_outputs;
}