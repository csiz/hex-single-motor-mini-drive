#pragma once

#include "interrupts_data.hpp"

#include "type_definitions.hpp"
#include "constants.hpp"
#include "byte_handling.hpp"
#include "math_utils.hpp"
#include "integer_math.hpp"
#include "error_handler.hpp"

#include <cstdlib>

// Motor Control Functions
// =======================

// Linearly add two motor outputs together; note that we don't do bounds checking here.
static inline MotorOutputs add_motor_outputs(
    MotorOutputs const& a,
    MotorOutputs const& b
){
    // Assume that all phases are enabled; we don't do bounds checking here.

    return MotorOutputs{
        .enable_flags = enable_flags_all,
        .u_duty = static_cast<uint16_t>(a.u_duty + b.u_duty),
        .v_duty = static_cast<uint16_t>(a.v_duty + b.v_duty),
        .w_duty = static_cast<uint16_t>(a.w_duty + b.w_duty)
    };
}

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

    const uint16_t abs_pwm = faster_abs(driver_state.active_pwm);

    return MotorOutputs{
        .enable_flags = enable_flags_all,
        .u_duty = static_cast<uint16_t>(voltage_phase_u * abs_pwm / pwm_base),
        .v_duty = static_cast<uint16_t>(voltage_phase_v * abs_pwm / pwm_base),
        .w_duty = static_cast<uint16_t>(voltage_phase_w * abs_pwm / pwm_base)
    };
}

// Get the motor outputs at the specified active_pwm and active_angle.
static inline MotorOutputs update_motor_at_angle(DriverState const& driver_state) {
    // The PWM counter value must be positive, we use the sign to determine the direction.
    const int abs_pwm = faster_abs(driver_state.active_pwm);
    
    // Use the active angle or flip it depending on the sign of the PWM.
    const int angle = driver_state.active_angle + (driver_state.active_pwm < 0 ? half_circle : 0);

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

// Drive the inductors around the circle at the specified PWM and speed. Open loop control.
static inline MotorOutputs update_motor_periodic(
    DriverState & driver_state,
    FullReadout const& readout
){
    const int active_angle_hires_diff = driver_state.angular_speed + driver_state.angle_residual;

    driver_state.active_angle = normalize_angle(driver_state.active_angle + active_angle_hires_diff / speed_fixed_point);

    driver_state.angle_residual = active_angle_hires_diff % speed_fixed_point;

    return update_motor_at_angle(driver_state);
}

// Drive the motor using FOC targeting a PWM value. The current is controlled to be as 
// close to 90 degrees ahead of the magnetic angle as possible; stray currents absorbed.
static inline MotorOutputs update_motor_smooth(
    DriverState & driver_state,
    FullReadout const& readout
){
    // Check if we have an accurate readout angle.
    const bool angle_fix = readout.state_flags & angle_fix_bit_mask;
    const bool current_detected = readout.state_flags & current_detected_bit_offset;

    const int16_t target_pwm = (
        angle_fix ? 
        clip_to(-pwm_max, +pwm_max, driver_state.target_pwm) :
        clip_to(-control_parameters.probing_max_pwm, +control_parameters.probing_max_pwm, driver_state.target_pwm)
    );

    const int pwm_error = target_pwm - driver_state.active_pwm;

    driver_state.active_pwm += clip_to(-control_parameters.max_pwm_change, +control_parameters.max_pwm_change, pwm_error);

    // Base the direction on the sign of the target PWM.
    const int active_pwm_direction = sign(driver_state.active_pwm);


    // Calculate the predicted active angle (we want to maintain constant speed in angular coordinate space).
    const int active_angle_hires_diff = driver_state.angular_speed + driver_state.angle_residual;

    driver_state.active_angle = normalize_angle(driver_state.active_angle + active_angle_hires_diff / speed_fixed_point);

    driver_state.angle_residual = active_angle_hires_diff % speed_fixed_point;


    if (angle_fix) {
        // If we have an accurate position, we can use it to adjust our control.
        
        // Ideally the inductor current is exactly 90 degrees ahead of the magnetic angle.
        // 
        // Of course, the inductors take a while to charge and the rotor is producing an EMF
        // which all interacts with the current. However, the current that we end up measuring
        // should be as close to the 90 degrees as possible for maximum torque per current use.
        const int ideal_angle = normalize_angle(readout.angle + quarter_circle);

        // Get the error between the measured current and the ideal current angle.
        const int lead_angle_error = active_pwm_direction * current_detected * signed_angle(ideal_angle - readout.inductor_angle);

        // Adjust the target angle to keep the alpha current small; reset if the motor is not moving.
        driver_state.lead_angle_control = clip_to(
            - half_circle,
            + half_circle,
            driver_state.lead_angle_control + signed_ceil_div(
                lead_angle_error * control_parameters.lead_angle_control_ki,
                control_parameters_fixed_point
            )
        );

        const int target_angle = normalize_angle(ideal_angle + driver_state.lead_angle_control);
        
        // Compensate the target angle by the control output. At high speed we need
        // to lead by more than 90 degrees to compensate for the RL time constant.
        const int active_angle_error = clip_to(
            -control_parameters.max_angle_change,
            +control_parameters.max_angle_change,
            signed_angle(target_angle - driver_state.active_angle)
        );

        // Update the driving angle.
        driver_state.active_angle = normalize_angle(driver_state.active_angle + active_angle_error);

        // Match the angular speed to the rotor speed.
        driver_state.angular_speed = readout.angular_speed;

        return update_motor_at_angle(driver_state);
    } else {
        // If we don't have an accurate position, we need drive the motor open loop until we get an EMF fix.

        // Decay the current angle control; it needs to start from 0 at low speed.
        driver_state.lead_angle_control += signed_ceil_div(
            -driver_state.lead_angle_control * control_parameters.lead_angle_control_ki,
            control_parameters_fixed_point
        );

        // Speed should be the max between the probing speed and current rotor speed. We always want
        // to probe at some minimum speed, but also want to keep up with the rotor in case it gets going.
        driver_state.angular_speed = active_pwm_direction * max(
            control_parameters.probing_angular_speed,
            // Use the speed value in the direction of movement; negative values will be handled by the max.
            active_pwm_direction * readout.angular_speed
        );

        return update_motor_at_angle(driver_state);
    }
}

static inline MotorOutputs update_motor_torque(
    DriverState & driver_state,
    FullReadout const& readout
){
    // Calculate the target current in fixed point format.
    const int16_t current_target = driver_state.torque.current_target;

    // Update the PID control for the torque.
    driver_state.torque.torque_control = clip_to(
        -pwm_max,
        +pwm_max,
        signed_ceil_div(
            (current_target - readout.beta_current) * control_parameters.torque_control_ki, 
            control_parameters_fixed_point
        )
    );

    // Get the target PWM after torque control.
    driver_state.target_pwm = driver_state.torque.torque_control;

    return update_motor_smooth(driver_state, readout);
}

static inline MotorOutputs update_motor_battery_power(
    DriverState & driver_state,
    FullReadout const& readout
){
    const bool direction_is_negative = driver_state.battery_power.power_target < 0;

    driver_state.battery_power.battery_power_control = clip_to(
        -pwm_max,
        +pwm_max,
        signed_ceil_div(
            (max(0, -readout.total_power) - faster_abs(driver_state.battery_power.power_target)) * control_parameters.battery_power_control_ki,
            control_parameters_fixed_point
        )
    );

    // Get the target PWM after power control.
    driver_state.target_pwm = clip_to(-pwm_max, pwm_max, (direction_is_negative ? -1 : 1) * driver_state.battery_power.battery_power_control);

    return update_motor_smooth(driver_state, readout);
}



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
        .u_duty = schedule_stage.u_duty,
        .v_duty = schedule_stage.v_duty,
        .w_duty = schedule_stage.w_duty
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
            if (pending_state.schedule.pointer == nullptr) error();

            // Clear the readouts buffer of old data.
            readout_history_reset();
            
            return DriverState{
                .mode = DriverMode::SCHEDULE,
                .duration = history_size,
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
                .angle_residual = 0,
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
                .angle_residual = driver_state.angle_residual,
                .target_pwm = static_cast<int16_t>(clip_to(-pwm_max, +pwm_max, pending_state.target_pwm)),
                .lead_angle_control = driver_state.lead_angle_control,
            };
            
        case DriverMode::DRIVE_TORQUE:
            return DriverState{
                .mode = DriverMode::DRIVE_TORQUE,
                .duration = static_cast<uint16_t>(clip_to(0, max_timeout, pending_state.duration)),
                .torque = DriveTorque{
                    .current_target = static_cast<int16_t>(clip_to(-max_drive_current, +max_drive_current, pending_state.torque.current_target)),
                }
            };

        case DriverMode::DRIVE_BATTERY_POWER:
            return DriverState{
                .mode = DriverMode::DRIVE_BATTERY_POWER,
                .duration = static_cast<uint16_t>(clip_to(0, max_timeout, pending_state.duration)),
                .battery_power = DriveBatteryPower{
                    .power_target = static_cast<int16_t>(clip_to(-max_drive_power, +max_drive_power, pending_state.battery_power.power_target)),
                }
            };
    }

    return null_driver_state;
}

static inline void update_motor_control(
    DriverState & driver_state,
    FullReadout const& readout
){
    // Update the active state.
    switch (driver_state.mode) {

        case DriverMode::OFF:
            driver_state.motor_outputs = breaking_motor_outputs;
            return;

        case DriverMode::FREEWHEEL:
            driver_state.motor_outputs = freewheel_motor_outputs;
            return;

        case DriverMode::HOLD:
            if (driver_state.duration-- <= 0) return set_breaking_control(driver_state);
            // The motor outputs are already set in the setup_driver_state function; do nothing else.
            return;

        case DriverMode::SCHEDULE:
            // We're done at the end of the schedule.
            if (driver_state.schedule.pointer == nullptr or driver_state.schedule.current_stage >= schedule_size) {
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
    return;
}