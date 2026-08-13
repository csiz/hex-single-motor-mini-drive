#pragma once

#include "constants.hpp"
#include "math_utils.hpp"
#include "type_definitions.hpp"

#include "hex_mini_drive_interface.hpp"

// Calculate the PID control for seeking a target position.
static inline float compute_seek_pid_control(
    SeekAngle & seek_angle,
    hex_mini_drive::FullReadout const& readout,
    const float k_prediction,
    const float ki,
    const float kp,
    const float kd
) {
    // The derivative of the error is negative the angular speed.
    const float position_error_derivative = -readout.angular_speed;
    
    // Get the error between the target angle and the current angle; predicted to
    // a future position determined by prediction parameter.
    const float uncapped_position_error = (
        (seek_angle.target_rotation - readout.rotations) * angle_base +
        k_prediction * position_error_derivative
    );

    // Cap to the maximum seek error which is set to 2x the maximum control output so
    // that the position term can overcome the derivative term. The cap is applied
    // here so we can use the position_error for the integral term as well.
    const float position_error = clip_to(-max_seek_position_control, +max_seek_position_control, uncapped_position_error);

    // Decay the integral term over time.
    seek_angle.error_integral -= sign(seek_angle.error_integral);

    // Proportional term with respect to the maximum position error (control maxes out at greater errors).
    const float proportional = kp * position_error;
        
    // Derivative term with respect to the reference speed.
    const float derivative = clip_to(-max_seek_derivative_control, +max_seek_derivative_control,
        kd * position_error_derivative
    );

    // Calculate the new integral term using the predicted position error (to minimize oscillations).
    // 
    // Note that we don't update the integral term just yet, we will update it if the output isn't saturated.
    const float integral = clip_to(-max_seek_integral_control, +max_seek_integral_control,
        seek_angle.error_integral + ki * position_error
    );

    // Update the output of the Proportional Integral Derivative (PID) control.
    const float output = (proportional + integral + derivative);
    
    // Only accumulate the integral if the output isn't saturated.

    if (output > +1.0) {
        // Decay the integral if we are saturating the output.
        seek_angle.error_integral -= seek_angle.error_integral * seek_integral_decay_fraction;
        // Return the maximum output.
        return +1.0;
    } else if (output < -1.0) {
        // Decay the integral if we are saturating the output negatively.
        seek_angle.error_integral -= seek_angle.error_integral * seek_integral_decay_fraction;
        // Return the minimum output.
        return -1.0;
    } else {
        // Output is not saturated, we can update the integral either direction.
        seek_angle.error_integral = integral;
        // Return the output as it is within the valid range.
        return output;
    }
}