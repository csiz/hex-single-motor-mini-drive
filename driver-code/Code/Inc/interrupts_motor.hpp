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
    uint8_t const& hall_sector,
    Drive6Sector const& sector_parameters
){

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
    FullReadout const& readout,
    DriveSmooth const& smooth_parameters,
    PIDControlState & pid_state
){
    const int target_direction = smooth_parameters.current_target >= 0 ? +1 : -1;

    const bool is_motor_moving = abs(readout.angular_speed) > threshold_speed;

    const int current_target = smooth_parameters.current_target + (is_motor_moving ? 0 : target_direction * min_drive_current);

    pid_state.torque_control = compute_pid_control(
        pid_parameters.torque_gains,
        pid_state.torque_control,
        readout.beta_current,
        current_target
    );

    // Get the target PWM after torque control.
    const int pwm = min(pwm_max, abs(pid_state.torque_control.output));

    // Base the direction on the torque control output.
    const int direction = pid_state.torque_control.output >= 0 ? +1 : -1;

    // Poor man's atan2, what we need to do is drive the alpha current to zero relative to 
    // the beta current. Since calculating the angle is expensive just consider the ratio.
    const int approximate_angle_error = clip_to(
        -eighth_circle, +eighth_circle, 
        not readout.beta_current ? 0 : eighth_circle * readout.alpha_current / abs(readout.beta_current)
    );

    // Adjust the target angle to keep the alpha current small; reset if the motor is not moving.
    pid_state.current_angle_control = compute_pid_control(
        pid_parameters.current_angle_gains,
        pid_state.current_angle_control,
        -approximate_angle_error,
        is_motor_moving ? 0 : -get_cos(smooth_parameters.leading_angle) * eighth_circle / angle_base
    );

    const int target_lead_angle = direction * (smooth_parameters.leading_angle + pid_state.current_angle_control.output);

    const int rotor_angle = readout.angle & 0x3FF;

    const int target_angle = normalize_angle(rotor_angle + target_lead_angle);

    const uint16_t voltage_phase_u = get_phase_pwm(target_angle);
    const uint16_t voltage_phase_v = get_phase_pwm(target_angle - third_circle);
    const uint16_t voltage_phase_w = get_phase_pwm(target_angle - two_thirds_circle);
    

    return MotorOutputs{
        .enable_flags = enable_flags_all,
        .u_duty = static_cast<uint16_t>(voltage_phase_u * pwm / pwm_base),
        .v_duty = static_cast<uint16_t>(voltage_phase_v * pwm / pwm_base),
        .w_duty = static_cast<uint16_t>(voltage_phase_w * pwm / pwm_base)
    };
}

// Motor control
// -------------

static inline MotorOutputs update_motor_control(
    FullReadout const& readout,
    DriverState const pending_state,
    DriverParameters const& pending_parameters,
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
            // Reset the PID controls when switching to smooth drive mode; but keep the
            // same control state between smooth commands.
            if (driver_state != DriverState::DRIVE_SMOOTH) pid_state = null_pid_control_state;
            driver_state = DriverState::DRIVE_SMOOTH;
            driver_parameters = DriverParameters{
                .smooth = DriveSmooth{
                    .duration = static_cast<uint16_t>(clip_to(0, max_timeout, pending_parameters.smooth.duration)),
                    .current_target = static_cast<int16_t>(clip_to(-max_drive_current, +max_drive_current, pending_parameters.smooth.current_target)),
                    .leading_angle = static_cast<int16_t>(clip_to(0, angle_base, pending_parameters.smooth.leading_angle))
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
            // Break at the end of the command duration.
            if (not driver_parameters.hold.duration) {
                driver_state = DriverState::OFF;
                driver_parameters = null_driver_parameters;
                return breaking_motor_outputs;
            } else {
                driver_parameters.hold.duration -= 1;
                // Set the motor outputs to hold the current settings.
                return MotorOutputs{
                    .enable_flags = enable_flags_all,
                    .u_duty = driver_parameters.hold.u_duty,
                    .v_duty = driver_parameters.hold.v_duty,
                    .w_duty = driver_parameters.hold.w_duty
                };
            }

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

        case DriverState::DRIVE_6_SECTOR: {
            // Re-extract the hall sector in this function.
            const uint8_t hall_sector = get_hall_sector(readout.angle >> 13);

            // Break at the end of the command duration or if the hall sensor is missing the magnet.
            if (not driver_parameters.sector.duration or hall_sector >= hall_sector_base) {
                driver_state = DriverState::OFF;
                driver_parameters = null_driver_parameters;
                return breaking_motor_outputs;
            } else {
                driver_parameters.sector.duration -= 1;
                return update_motor_6_sector(hall_sector, driver_parameters.sector);
            }
        }
        case DriverState::DRIVE_SMOOTH: {
            const bool angle_valid = (readout.angle >> 12) & 0b1;

            // Break at the end of the command duration or if the angle is invalid.
            if (not driver_parameters.smooth.duration or not angle_valid) {
                // Reset the PID state.
                pid_state = null_pid_control_state;
                driver_state = DriverState::OFF;
                driver_parameters = null_driver_parameters;
                return breaking_motor_outputs;
            } else {
                driver_parameters.smooth.duration -= 1;
                return update_motor_smooth(readout, driver_parameters.smooth, pid_state);
            }
        }
    }

    // If we get here, we have an unknown/corrupted driver state.
    error();
    return breaking_motor_outputs;
}