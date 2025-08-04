#pragma once

#include "constants.hpp"
#include "integer_math.hpp"
#include "math_utils.hpp"
#include "type_definitions.hpp"


static inline int16_t compute_seek_pid_control(
    SeekAngle & seek_angle,
    FullReadout const& readout,
    const int16_t integral_speed_prediction,
    const int16_t ki,
    const int16_t kp,
    const int16_t kd
) {
    // The derivative of the error is negative the angular speed.
    const int position_error_speed = -readout.angular_speed;
    
    // Get the error between the target angle and the current angle; predicted to
    // a future position determined by the kd term.
    const int position_error = clip_to(-max_seek_error, +max_seek_error, 
        (seek_angle.target_rotation - readout.rotations) * seek_rotation_multiplier +
        signed_angle(seek_angle.target_angle - readout.angle) / seek_angle_divisor +
        integral_speed_prediction * position_error_speed / seek_pid_fixed_point
    );

    // Decay the integral term over time.
    seek_angle.error_integral -= sign(seek_angle.error_integral);

    // Proportional term.
    const int proportional = kp * position_error / seek_position_error_reference;
        
    // Derivative term.
    const int derivative = clip_to(
        -seek_pid_fixed_point,
        +seek_pid_fixed_point,
        kd * position_error_speed / seek_speed_error_reference
    );

    // Calculate the new integral term using the predicted position error (to minimize oscillations).
    // 
    // Note that we don't update the integral term just yet, we will update it if the output isn't saturated.
    const int error_integral = clip_to(-max_seek_integral, +max_seek_integral, 
        seek_angle.error_integral + ki * position_error
    );

    // Compute the integral term. The integral is higher resolution than the error because we accumulate it over
    // a few cycles. However at 23KHz we would need 17bits to count the cycles every second; we can spare 12bits.
    const int integral = error_integral / seek_integral_divisor;


    // Update the output.
    const int output = (proportional + integral + derivative);
    
    // Only accumulate the integral if the output isn't saturated.

    if (output > +seek_pid_fixed_point) {
        // Decay the integral if we are saturating the output.
        seek_angle.error_integral -= seek_angle.error_integral / seek_integral_divisor;
        return +seek_pid_fixed_point;
    } else if (output < -seek_pid_fixed_point) {
        // Decay the integral if we are saturating the output.
        seek_angle.error_integral -= seek_angle.error_integral / seek_integral_divisor;
        return -seek_pid_fixed_point;
    } else {
        // Output is not saturated, we can update the integral either direction.
        seek_angle.error_integral = error_integral;
        return output;
    }
}