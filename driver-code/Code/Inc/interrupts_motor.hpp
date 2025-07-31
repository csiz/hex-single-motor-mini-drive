#pragma once

#include "interrupts_data.hpp"

#include "type_definitions.hpp"
#include "constants.hpp"
#include "byte_handling.hpp"
#include "math_utils.hpp"
#include "integer_math.hpp"

// The interrupts must not enter the error handler!
// 
// The error handle will block forever, however we must safe the motor no matter what. The interrupt loop
// will always timeout any command and return to a safe state if it doesn't receive new commands from the
// main app loop.
// 
// Do not: #include "error_handler.hpp"


// Motor Control Functions
// =======================

// Update the motor outputs using simple 6 sector driving based on the hall sensors.
static inline MotorOutputs update_motor_6_sector(
    DriverState const& driver_state,
    FullReadout const& readout
){

    // Update the sector variable.
    const uint8_t hall_sector = get_hall_sector(readout.state_flags & hall_state_bit_mask);

    // Check if the magnet is present.
    const bool angle_valid = hall_sector < hall_sector_base;

    if (not angle_valid) return breaking_motor_outputs;

    auto const& motor_sector_driving_table = driver_state.active_pwm >= 0 ? 
        motor_sector_driving_positive : 
        motor_sector_driving_negative;

    // Get the voltage for the three phases from the table.

    const uint16_t voltage_phase_u = motor_sector_driving_table[hall_sector][0];
    const uint16_t voltage_phase_v = motor_sector_driving_table[hall_sector][1];
    const uint16_t voltage_phase_w = motor_sector_driving_table[hall_sector][2];

    const uint16_t abs_pwm = min(
        readout.live_max_pwm,
        faster_abs(driver_state.active_pwm)
    );

    return MotorOutputs{
        .enable_flags = enable_flags_all,
        .u_duty = static_cast<uint16_t>(voltage_phase_u * abs_pwm / pwm_base),
        .v_duty = static_cast<uint16_t>(voltage_phase_v * abs_pwm / pwm_base),
        .w_duty = static_cast<uint16_t>(voltage_phase_w * abs_pwm / pwm_base)
    };
}

// Set the motor outputs to the specified active_pwm and active_angle.
static inline MotorOutputs update_motor_at_angle(
    DriverState & driver_state,
    FullReadout const& readout
) {
    // The PWM counter value must be positive, we use the sign to determine the direction.
    const int abs_pwm = min(readout.live_max_pwm, faster_abs(driver_state.active_pwm));

    // Use the active angle or flip it depending on the sign of the PWM.
    const int angle = driver_state.active_angle + (driver_state.active_pwm < 0 ? half_circle : 0);

    driver_state.active_pwm = driver_state.active_pwm < 0 ? -abs_pwm : abs_pwm;

    // Get the voltage for the three phases from the waveform table.

    const uint16_t voltage_phase_u = get_phase_pwm(angle);
    const uint16_t voltage_phase_v = get_phase_pwm(angle - third_circle);
    const uint16_t voltage_phase_w = get_phase_pwm(angle - two_thirds_circle);

    return MotorOutputs{
        .enable_flags = enable_flags_all,
        .u_duty = static_cast<uint16_t>(voltage_phase_u * abs_pwm / pwm_base),
        .v_duty = static_cast<uint16_t>(voltage_phase_v * abs_pwm / pwm_base),
        .w_duty = static_cast<uint16_t>(voltage_phase_w * abs_pwm / pwm_base)
    };
}

// Drive the inductors around a circle at the specified PWM and speed (open loop control).
static inline MotorOutputs update_motor_periodic(
    DriverState & driver_state,
    FullReadout const& readout
){
    const int active_angle_hires_diff = driver_state.angular_speed + driver_state.active_angle_residual;

    driver_state.active_angle = normalize_angle(driver_state.active_angle + active_angle_hires_diff / speed_fixed_point);

    driver_state.active_angle_residual = active_angle_hires_diff % speed_fixed_point;

    return update_motor_at_angle(driver_state, readout);
}

// Drive the motor using FOC targeting a PWM value. The current is controlled to be as 
// close to 90 degrees ahead of the magnetic angle as possible; stray currents absorbed.
// The PWM is varried smoothly and is bounded by the back EMF from the rotating magnet.
static inline MotorOutputs update_motor_smooth(
    DriverState & driver_state,
    FullReadout const& readout
){
    // Check if we have an accurate readout angle.
    const bool angle_fix = readout.state_flags & angle_fix_bit_mask;
    const bool current_detected = readout.state_flags & current_detected_bit_mask;
    const bool emf_detected = readout.state_flags & emf_detected_bit_mask;

    const int pwm_for_emf_compensation = -readout.beta_emf_voltage * pwm_waveform_base / readout.vcc_voltage;

    // Adjust the target PWM to be close to the current EMF voltage (which is 0 at standstill).
    const int16_t target_pwm = clip_to(
        pwm_for_emf_compensation - control_parameters.probing_max_pwm, 
        pwm_for_emf_compensation + control_parameters.probing_max_pwm, 
        driver_state.target_pwm
    );

    const int pwm_error = target_pwm - driver_state.active_pwm;

    driver_state.active_pwm += clip_to(
        -control_parameters.max_pwm_change, 
        +control_parameters.max_pwm_change, 
        pwm_error
    );

    // Base the direction on the sign of the target PWM.
    const int active_pwm_direction = sign(driver_state.active_pwm);


    // Calculate the predicted active angle (we want to maintain constant speed in angular coordinate space).
    const int active_angle_hires_diff = driver_state.angular_speed + driver_state.active_angle_residual;

    driver_state.active_angle = normalize_angle(driver_state.active_angle + active_angle_hires_diff / speed_fixed_point);

    driver_state.active_angle_residual = active_angle_hires_diff % speed_fixed_point;

    // Ideally the inductor current is exactly 90 degrees ahead of the magnetic angle.
    // 
    // Of course, the inductors take a while to charge and the rotor is producing an EMF
    // which all interacts with the current. However, the current that we end up measuring
    // should be as close to the 90 degrees as possible for maximum torque per current use.
    const int ideal_angle = normalize_angle(readout.angle + quarter_circle);

    // Get the error between the measured current and the ideal current angle.
    const int ideal_angle_diff = current_detected * signed_angle(ideal_angle - readout.inductor_angle);

    // Drive towards the ideal angle; however decay to 0 at low EMF voltage.
    const int lead_angle_error = control_parameters.lead_angle_control_ki * (
        emf_detected ? active_pwm_direction * ideal_angle_diff :
        -sign(driver_state.lead_angle_control)
    );

    // Adjust the target angle to keep the alpha current small; reset if the motor is not moving.
    driver_state.lead_angle_control = clip_to(
        -max_lead_angle_control,
        +max_lead_angle_control,
        driver_state.lead_angle_control + lead_angle_error
    );

    if (angle_fix) {
        // If we have an accurate position, we can use it to adjust our control.

        const int target_angle = normalize_angle(ideal_angle + driver_state.lead_angle_control / control_parameters_fixed_point);
        
        // Compensate the target angle by the control output. At high speed we need
        // to lead by more than 90 degrees to compensate for the RL time constant.
        const int active_angle_error = clip_to(
            -control_parameters.max_angle_change,
            +control_parameters.max_angle_change,
            signed_angle(target_angle - driver_state.active_angle)
        );

        // Update the driving angle.
        driver_state.active_angle = normalize_angle(driver_state.active_angle + active_angle_error);

        // Push our speed towards the target angle.
        driver_state.angular_speed += active_angle_error;

        return update_motor_at_angle(driver_state, readout);
    } else {
        // If we don't have an accurate position, we need drive the motor open loop until we get an EMF fix.

        // Use the probing speed.
        driver_state.angular_speed = active_pwm_direction * control_parameters.probing_angular_speed;

        return update_motor_at_angle(driver_state, readout);
    }
}

// Drive the motor with the desired output current (note the DQ0 current is 3/2 phase current).
// This mode also uses the smooth driving algorithm to drive using field oriented control, thus
// keeping the current as close to 90 degrees ahead of the magnetic angle as possible. Allowing
// control of the torque produced by the motor which will be proportional to the current target.
static inline MotorOutputs update_motor_torque(
    DriverState & driver_state,
    FullReadout const& readout
){
    // Calculate the target current in fixed point format.
    const int current_target = driver_state.torque.current_target;

    const bool angle_fix = readout.state_flags & angle_fix_bit_mask;
    const bool current_detected = readout.state_flags & current_detected_bit_mask;

    const int measured_current = (angle_fix and current_detected) * (
        // Rely on the sign of the target current because we are driving with the smooth 
        // mode which always targets the current at 90 degrees ahead of the magnetic angle.
        current_target > 0 ? +readout.current_magnitude : 
        current_target < 0 ? -readout.current_magnitude : 
        // But if set to 0, we always have to cancel the beta_current.
        readout.beta_current
    );

    const int control_error = (current_target - measured_current) * control_parameters.torque_control_ki;

    // Update the PID control for the torque.
    driver_state.pwm_control = clip_to(
        (driver_state.active_pwm - control_parameters.probing_max_pwm) * control_parameters_fixed_point,
        (driver_state.active_pwm + control_parameters.probing_max_pwm) * control_parameters_fixed_point,
        driver_state.pwm_control + control_error
    );

    // Get the target PWM after torque control.
    driver_state.target_pwm = driver_state.pwm_control / control_parameters_fixed_point;

    return update_motor_smooth(driver_state, readout);
}

// Drive motor using up to a target battery power consumption.
// 
// The sign of the target power determines the direction of driving. When motor breaking, we
// try to absorb the target power instead and use it to charge the battery.
static inline MotorOutputs update_motor_battery_power(
    DriverState & driver_state,
    FullReadout const& readout
){
    // Note that total power will be 0 when not driving; in that case we want to
    // counter the EMF voltage to minimize phase resistance heating, but we don't
    // want to absorb more power than the target setting.

    const int target_power = driver_state.battery_power.target_power;
    
    const bool total_power_dominates = faster_abs(readout.total_power) > faster_abs(readout.emf_power);

    const int measured_power = (total_power_dominates ? 
        sign(driver_state.active_pwm) * readout.total_power :
        -sign(readout.beta_emf_voltage) * readout.emf_power
    );

    const int control_error = (target_power - measured_power) * control_parameters.battery_power_control_ki;


    // Update the PID control for the torque.
    driver_state.pwm_control = clip_to(
        (driver_state.active_pwm - control_parameters.probing_max_pwm) * control_parameters_fixed_point,
        (driver_state.active_pwm + control_parameters.probing_max_pwm) * control_parameters_fixed_point,
        driver_state.pwm_control + control_error
    );

    // Get the target PWM after torque control.
    driver_state.target_pwm = driver_state.pwm_control / control_parameters_fixed_point;

    return update_motor_smooth(driver_state, readout);
}

// static inline MotorOutputs update_motor_seek_angle(
//     DriverState & driver_state,
//     FullReadout const& readout
// ){
//     // TODO: ...
    
//     // const int target_rotation = driver_state.seek_angle.target_rotation;

//     // const int control_error = target_rotation - readout.rotations;

// }

// Drive the motor using a fixed schedule for the PWM outputs.
static inline MotorOutputs update_motor_schedule(
    DriverState & driver_state,
    FullReadout const& readout
){
    PWMSchedule const& schedule = *driver_state.schedule.pointer;
    PWMStage const& schedule_stage = schedule[driver_state.schedule.current_stage];

    driver_state.schedule.stage_counter += 1;

    if (driver_state.schedule.stage_counter >= schedule_stage.duration) {
        // Move to the next stage in the schedule.
        driver_state.schedule.current_stage += 1;
        driver_state.schedule.stage_counter = 0;
    }
    
    return MotorOutputs{
        .enable_flags = enable_flags_all,
        .u_duty = static_cast<uint16_t>(schedule_stage.u_duty * driver_state.target_pwm / pwm_base),
        .v_duty = static_cast<uint16_t>(schedule_stage.v_duty * driver_state.target_pwm / pwm_base),
        .w_duty = static_cast<uint16_t>(schedule_stage.w_duty * driver_state.target_pwm / pwm_base)
    };
}


// Motor control
// -------------

// Set the driver state to OFF.
static inline void set_breaking_control(DriverState & driver_state){
    driver_state = breaking_driver_state;
}

// Copy the pending driver state with all values clamped to valid ranges.
static inline DriverState setup_driver_state(
    DriverState const& driver_state,
    DriverState const& pending_state,
    FullReadout const& readout
){
    switch(pending_state.mode){
        case DriverMode::OFF:
            return DriverState{
                .motor_outputs = breaking_motor_outputs,
                .mode = DriverMode::OFF
            };

        case DriverMode::FREEWHEEL:
            return DriverState{
                .motor_outputs = MotorOutputs{.enable_flags = enable_flags_none},
                .mode = DriverMode::FREEWHEEL
            };

        case DriverMode::HOLD:
            return DriverState{
                .motor_outputs = MotorOutputs {
                    .enable_flags = pending_state.motor_outputs.enable_flags,
                    .u_duty = static_cast<uint16_t>(clip_to(0, pwm_max_hold, pending_state.motor_outputs.u_duty)),
                    .v_duty = static_cast<uint16_t>(clip_to(0, pwm_max_hold, pending_state.motor_outputs.v_duty)),
                    .w_duty = static_cast<uint16_t>(clip_to(0, pwm_max_hold, pending_state.motor_outputs.w_duty))
                },
                .mode = DriverMode::HOLD,
                .duration = static_cast<uint16_t>(clip_to(0, max_timeout, pending_state.duration)),
            };

        case DriverMode::SCHEDULE:
            // We should not enter testing mode without a valid schedule.
            if (pending_state.schedule.pointer == nullptr) return breaking_driver_state;

            // Clear the readouts buffer of old data.
            readout_history_reset();
            
            return DriverState{
                .mode = DriverMode::SCHEDULE,
                .duration = history_size,
                .target_pwm = static_cast<int16_t>(clip_to(0, pwm_max, pending_state.target_pwm)),
                .schedule = DriveSchedule{
                    .pointer = pending_state.schedule.pointer,
                    .current_stage = 0,
                    .stage_counter = 0
                }
            };

        case DriverMode::DRIVE_6_SECTOR:
            return DriverState{
                .mode = DriverMode::DRIVE_6_SECTOR,
                .duration = static_cast<uint16_t>(clip_to(0, max_timeout, pending_state.duration)),
                .active_pwm = static_cast<int16_t>(clip_to(-pwm_max, pwm_max, pending_state.active_pwm))
            };

        case DriverMode::DRIVE_PERIODIC:
            return DriverState{
                .mode = DriverMode::DRIVE_PERIODIC,
                .duration = static_cast<uint16_t>(clip_to(0, max_timeout, pending_state.duration)),
                .active_angle = static_cast<int16_t>(normalize_angle(
                    pending_state.active_angle + (pending_state.active_pwm < 0 ? half_circle : 0)
                )),
                .active_pwm = static_cast<int16_t>(min(pwm_max_hold, faster_abs(pending_state.active_pwm))),
                .angular_speed = static_cast<int16_t>(clip_to(-max_angular_speed, max_angular_speed, pending_state.angular_speed)),
                .active_angle_residual = 0,
            };

        case DriverMode::DRIVE_SMOOTH:
            // Maintain a some of the previous state so we can smoothly transition to the new state.
            return DriverState{
                .mode = DriverMode::DRIVE_SMOOTH,
                .duration = static_cast<uint16_t>(clip_to(0, max_timeout, pending_state.duration)),
                .active_angle = driver_state.active_pwm != 0 ? driver_state.active_angle :
                    static_cast<int16_t>(normalize_angle(readout.angle + sign(pending_state.target_pwm) * quarter_circle)),
                .active_pwm = driver_state.active_pwm,
                .angular_speed = driver_state.angular_speed,
                .active_angle_residual = driver_state.active_angle_residual,
                .target_pwm = static_cast<int16_t>(clip_to(-pwm_max, +pwm_max, pending_state.target_pwm)),
                .lead_angle_control = driver_state.lead_angle_control,
            };
            
        case DriverMode::DRIVE_TORQUE:
            return DriverState{
                .mode = DriverMode::DRIVE_TORQUE,
                .duration = static_cast<uint16_t>(clip_to(0, max_timeout, pending_state.duration)),
                .active_angle = driver_state.active_pwm != 0 ? driver_state.active_angle :
                    static_cast<int16_t>(normalize_angle(readout.angle + sign(pending_state.target_pwm) * quarter_circle)),
                .active_pwm = driver_state.active_pwm,
                .angular_speed = driver_state.angular_speed,
                .active_angle_residual = driver_state.active_angle_residual,
                .lead_angle_control = driver_state.lead_angle_control,
                .pwm_control = driver_state.pwm_control,
                .torque = DriveTorque{
                    .current_target = static_cast<int16_t>(clip_to(-max_drive_current, +max_drive_current, pending_state.torque.current_target)),
                }
            };

        case DriverMode::DRIVE_BATTERY_POWER:
            return DriverState{
                .mode = DriverMode::DRIVE_BATTERY_POWER,
                .duration = static_cast<uint16_t>(clip_to(0, max_timeout, pending_state.duration)),
                .active_angle = driver_state.active_pwm != 0 ? driver_state.active_angle :
                    static_cast<int16_t>(normalize_angle(readout.angle + sign(pending_state.target_pwm) * quarter_circle)),
                .active_pwm = driver_state.active_pwm,
                .angular_speed = driver_state.angular_speed,
                .active_angle_residual = driver_state.active_angle_residual,
                .lead_angle_control = driver_state.lead_angle_control,
                .pwm_control = driver_state.pwm_control,
                .battery_power = DriveBatteryPower{
                    .target_power = static_cast<int16_t>(clip_to(-max_drive_power, +max_drive_power, pending_state.battery_power.target_power)),
                }
            };
    }

    return null_driver_state;
}

// Update the motor outputs based on the active driver state and measured phase currents and other derived values.
static inline void update_motor_control(
    DriverState & driver_state,
    FullReadout const& readout
){
    // Update based on the active mode.
    switch (driver_state.mode) {
        
        case DriverMode::OFF:
            // Continously reset the motor outputs to breaking state.
            driver_state.motor_outputs = breaking_motor_outputs;
            return;

        case DriverMode::FREEWHEEL:
            // Continuously reset the motor outputs to freewheel state.
            driver_state.motor_outputs = freewheel_motor_outputs;
            return;

        case DriverMode::HOLD:
            if (driver_state.duration-- <= 0) return set_breaking_control(driver_state);
            // The motor outputs are already set in the setup_driver_state function; do nothing else.
            return;

        case DriverMode::SCHEDULE: 
            // We're done at the end of the schedule.
            if (
                driver_state.schedule.pointer == nullptr or 
                driver_state.schedule.current_stage >= schedule_size
            ) {
                return set_breaking_control(driver_state);
            }

            driver_state.motor_outputs = update_motor_schedule(driver_state, readout);
            return;

        case DriverMode::DRIVE_6_SECTOR:
            if (driver_state.duration-- <= 0) return set_breaking_control(driver_state);

            // Update motor outputs for the 6 sector driving.
            driver_state.motor_outputs = update_motor_6_sector(driver_state, readout);
            return;

        case DriverMode::DRIVE_PERIODIC:
            if (driver_state.duration-- <= 0) return set_breaking_control(driver_state);

            driver_state.motor_outputs = update_motor_periodic(driver_state, readout);
            return;
                

        case DriverMode::DRIVE_SMOOTH:
            if (driver_state.duration-- <= 0) return set_breaking_control(driver_state);

            // Update the motor outputs for the smooth driving.
            driver_state.motor_outputs = update_motor_smooth(driver_state, readout);
            return;


        case DriverMode::DRIVE_TORQUE:
            if (driver_state.duration-- <= 0) return set_breaking_control(driver_state);

            // Update the motor outputs for the torque driving.
            driver_state.motor_outputs = update_motor_torque(driver_state, readout);
            return;

        case DriverMode::DRIVE_BATTERY_POWER:
            if (driver_state.duration-- <= 0) return set_breaking_control(driver_state);
            
            driver_state.motor_outputs = update_motor_battery_power(driver_state, readout);
            return;
    }

    // If we get here, we have an unknown/corrupted driver state.
    return set_breaking_control(driver_state);
}