#pragma once

#include "constants.hpp"
#include "integer_math.hpp"
#include "math_utils.hpp"
#include "type_definitions.hpp"


static inline PIDControl compute_pid_control(
    PIDGains const & gains,
    PIDControl const & control,
    const int16_t measurement,
    const int16_t target = 0
) {
    const int scaled_max_output = gains.max_output * gains_fixed_point;

    const int16_t error = target - measurement;

    // Proportional term.
    const int proportional = gains.kp * error;
    
    // Derivative term; assuming dt = 1.
    const int derivative = gains.kd * (error - control.error);

    // Only accumulate the integral if the output isn't saturated.
    const int integral = abs(proportional) > scaled_max_output ? 
        control.integral : 
        clip_to(-scaled_max_output, scaled_max_output, control.integral + gains.ki * error);

    // Update the output.
    const int16_t output = clip_to(-gains.max_output, gains.max_output, round_div(proportional + integral + derivative, gains_fixed_point));


    return PIDControl{
        .integral = integral,
        .derivative = derivative,
        .error = error,
        .output = output
    };
}