#pragma once

#include "constants.hpp"
#include "integer_math.hpp"
#include "math_utils.hpp"
#include "type_definitions.hpp"


static inline int16_t compute_seek_pid_control(
    int32_t & error_integral,
    const int16_t position_error,
    const int16_t speed_of_error,
    const int16_t ki,
    const int16_t kp,
    const int16_t kd
) {
    // Decay the integral term over time.
    error_integral -= sign(error_integral);

    
    // Proportional term.
    const int proportional = kp * position_error / seek_error_reference;
    
    // Derivative term; assuming dt = 1.
    const int derivative = kd * speed_of_error / seek_speed_reference;
    
    // Calculate the new integral term; but don't update it yet.
    const int new_error_integral = clip_to(-max_seek_integral, +max_seek_integral, error_integral + position_error);

    // Compute an uncapped integral term.
    const int integral = new_error_integral / seek_error_reference * ki / seek_integral_reference;

    // Update the output.
    const int output = (proportional + integral + derivative);
    
    // Only accumulate the integral if the output isn't saturated.
    if (output > +control_parameters_fixed_point) {
        error_integral = min(new_error_integral, error_integral);
        return control_parameters_fixed_point;
    }

    if (output < -control_parameters_fixed_point) {
        error_integral = max(new_error_integral, error_integral);
        return -control_parameters_fixed_point;
    }

    error_integral = new_error_integral;
    return output;
}