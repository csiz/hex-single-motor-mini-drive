#pragma once

#include "constants.hpp"
#include "integer_math.hpp"
#include "math_utils.hpp"
#include "type_definitions.hpp"


static inline PIDControl compute_pid_control(
    PIDGains const& gains,
    PIDControl const& control,
    const int16_t measurement,
    const int16_t target
) {
    const int16_t error = target - measurement;

    // Proportional term.
    const int proportional = gains.kp * error;
    
    // Derivative term; assuming dt = 1.
    const int derivative = gains.kd * (error - control.error);

    // Compute an uncapped integral term.
    const int integral = control.integral + gains.ki * error;
    
    // Update the output.
    const int16_t output = (proportional + integral + derivative) / gains_fixed_point;
    
    // Only accumulate the integral if the output isn't saturated in the same direction.
    const int new_integral = (
        output > gains.max_output ? min(integral, control.integral) :
        output < -gains.max_output ? max(integral, control.integral) :
        integral
    );

    return PIDControl{
        .integral = new_integral,
        .error = error,
        .output = output
    };
}

// Update the observer state based on the error.
static inline void update_observer(ObserverState & observer, const int16_t error, const int16_t observer_gain) {
    observer.error = error;

    observer.error_variance = min(max_16bit, 1 + (square(error) + observer.error_variance * 15) / 16);

    const int value_adjustment = sign(observer.error) + observer.error * observer_gain / observer_fixed_point;

    observer.value += value_adjustment;

    observer.value_variance = min(max_16bit, 1 + (square(value_adjustment) + observer.value_variance * 15) / 16);
}

