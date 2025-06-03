#pragma once

#include "constants.hpp"
#include "io.hpp"
#include "error_handler.hpp"

#include <cstdint>

// Hall sensors
// ------------

// Hall states as bits, 0b001 = hall 1, 0b010 = hall 2, 0b100 = hall 3.
extern uint8_t hall_state;
// The 6 valid configurations of hall sensors in trigonometric order.
// 
// The order is u, uv, v, vw, w, wu. Treat values outside the range as
// invalid, the magnet is not present or the sensors are not ready.
extern uint8_t hall_sector;
// We need to keep track of the previous hall sector to determine the direction of rotation.
extern uint8_t previous_hall_sector;

// Position tracking
// -----------------

// Whether the position is valid.
extern bool angle_valid;
// Time, in our units, since the last observation of the hall sensor. Incremented by PWM cycle loop.
extern int time_since_observation;
// Flag to indicate a new observation; the hall sensor interrupt sets the flag for the PWM cycle loop.
extern bool new_observation;

// Estimated angle at the last observation; this is our best guess of the angle using a kalman filter.
extern int angle_at_observation;
// Variance of the angle at the last observation.
extern int angle_variance_at_observation;
// Estimated angular speed at the last observation; this is our best guess of the speed using a kalman filter.
extern int angular_speed_at_observation;
// Variance of the angular speed at the last observation.
extern int angular_speed_variance_at_observation;

// Position calibration data. These are the trigger angles for each hall sensor output.
extern PositionCalibration position_calibration;


static inline void increment_time_since_observation(){
    // Check if we had a new hall sensor observation.
    if(new_observation) {
        new_observation = false;
        time_since_observation = 0;
    }
    // Increment the time since the last observation.
    time_since_observation = min(time_since_observation + time_units_per_cycle, max_time_between_observations);
        
}

// Read the hall sensors and update the motor rotation angle. Sensor chips might be: SS360NT (can't read the inprint clearly).
static inline void read_hall_sensors(){
    // Grab the registers for the GPIO ports with the hall sensors.

    uint16_t gpio_A_inputs = LL_GPIO_ReadInputPort(GPIOA);
    uint16_t gpio_B_inputs = LL_GPIO_ReadInputPort(GPIOB);

    // Note: Hall sensors are active low!
    const bool hall_1 = !(gpio_A_inputs & (1<<0)); // Hall sensor 1, corresponding to phase V
    const bool hall_2 = !(gpio_A_inputs & (1<<1)); // Hall sensor 2, corresponding to phase W
    const bool hall_3 = !(gpio_B_inputs & (1<<10)); // Hall sensor 3, corresponding to phase U

    // Combine the hall sensor states into a single byte.
    // Note: Reorder the sensors according to the phase order; U on bit 0, V on bit 1, W on bit 2.
    hall_state = hall_3 | (hall_1 << 1) | (hall_2 << 2);

    // Get the hall sector from the state.
    switch (hall_state) {
        case 0b000: // no hall sensors; either it's not ready or no magnet
            hall_sector = hall_sector_base; // Out of range, indicates invalid.
            break;
        case 0b001: // hall U active; 0 degrees
            hall_sector = 0;
            break;
        case 0b011: // hall U and hall V active; 60 degrees
            hall_sector = 1;
            break;
        case 0b010: // hall V active; 120 degrees
            hall_sector = 2;
            break;
        case 0b110: // hall V and hall W active; 180 degrees
            hall_sector = 3;
            break;
        case 0b100: // hall W active; 240 degrees
            hall_sector = 4;
            break;
        case 0b101: // hall U and hall W active; 300 degrees
            hall_sector = 5;
            break;
        case 0b111: // all hall sensors active; this would be quite unusual
            hall_sector = hall_sector_base; // Out of range, indicates invalid.
            error();
            break;
    }
}

// Update position estimate after not observing any trigger for a while.
static inline void update_position_unobserved(){
    // Check if the current angle and hall sector are valid.
    if (not angle_valid or hall_sector >= hall_sector_base) return;

    const int time = max(1, time_since_observation);

    const int direction = angular_speed_at_observation >= 0 ? 1 : -1;
    const size_t direction_index = angular_speed_at_observation >= 0 ? 0 : 1;

    const int next_sector = (hall_sector_base + hall_sector + direction) % hall_sector_base;

    const int trigger_angle = position_calibration.sector_transition_angles[next_sector][direction_index];

    const int distance_to_trigger = signed_angle(trigger_angle - angle_at_observation);

    const int max_abs_speed = direction * (distance_to_trigger + direction * sector_transition_confidence) * speed_scale / time;

    angular_speed_at_observation = direction * clip_to(0, max_abs_speed, direction * angular_speed_at_observation);

    if (time_since_observation >= max_time_between_observations) {
        // We haven't observed the hall sensors for a while; we need to reset the position estimate.
        angle_at_observation = position_calibration.sector_center_angles[hall_sector];
        angle_variance_at_observation = position_calibration.sector_center_variances[hall_sector];
        angular_speed_at_observation = initial_angular_speed;
        angular_speed_variance_at_observation = position_calibration.initial_angular_speed_variance;
    }
}

// Update the position from the hall sensors. Use a kalman filter to estimate the position and speed.
static inline void update_position_observation(){
    // Get the time since the last observation.
    const int time = max(1, time_since_observation);

    read_hall_sensors();

    // Check if the magnet is present.
    if (hall_sector >= hall_sector_base) {
        previous_hall_sector = hall_sector_base; // Out of range, indicates invalid.
        angle_valid = false;
        new_observation = true;
        return;
    }

    // The new hall sector is valid, let's calculate the angle.

    // Check if the previous sector was valid, or if we have a repeat sector (it shouldn't happen).
    if (previous_hall_sector >= hall_sector_base || hall_sector == previous_hall_sector) {
        // This is the first time we have a valid hall sector; we need to set the angle and the angular speed.
        previous_hall_sector = hall_sector;

        // Reset position tracking to the sector center; our best guess.
        angle_at_observation = position_calibration.sector_center_angles[hall_sector];
        angle_variance_at_observation = position_calibration.sector_center_variances[hall_sector];
        angular_speed_at_observation = initial_angular_speed;
        angular_speed_variance_at_observation = position_calibration.initial_angular_speed_variance;

        new_observation = true;
        angle_valid = true;
        return;
    }

    
    // We have a valid sector transition; perform a Kalman filter update.

    // Get the sector integer distance to determine the direction of rotation.
    const int sector_distance = (9 + hall_sector - previous_hall_sector) % 6 - 3;

    // Establish direction of rotation.
    const int direction = sector_distance >= 0 ? 1 : -1;
    // Positive rotations index the first element in the calibration table, negative the second.
    const size_t direction_index = sector_distance >= 0 ? 0 : 1;

    // Get data about this transition from the calibration table.

    const int trigger_angle = position_calibration.sector_transition_angles[hall_sector][direction_index];
    const int trigger_variance = position_calibration.sector_transition_variances[hall_sector][direction_index];
    const int sector_variance = position_calibration.sector_center_variances[hall_sector];

    // Change coordinates with angle as the center. The next trigger is always close to the current 
    // angle (max distance between sectors is 120 degrees). On the other hand the distance since
    // the last measurement can grow arbitrarily large, we need to be careful to not wrap the estiamte
    // around the circle, but we do need to wrap and normalize the distance to the trigger.
    // 
    // Note: Don't add up variances for the change of coordinates! It's just a math trick.

    // Calculate the distance travelled since the last observation to the current trigger angle.
    const int distance_to_trigger = signed_angle(trigger_angle - angle_at_observation);
    
    // Make sure we don't overshoot the upper bound of the trigger angle. This is likely because
    // our speed doesn't update between observations and a long time can pass until we get a new one.
    // A low residual speed will push us quite a bit over a long enough time. Cap the speed based
    // on the observation time such that we don't overshoot the trigger angle by more than the sector confidence.

    // Calculate the maximum allowed speed while handling the direction sign.
    const int max_abs_speed = direction * (distance_to_trigger + direction * sector_transition_confidence) * speed_scale / time;

    // Clip the angular speed to the maximum allowed speed; we also clip it so it does't go backwards. A positive
    // sector transition necesarily implies a positive speed of the rotor.
    const int clipped_angular_speed = direction * clip_to(0, max_abs_speed, direction * angular_speed_at_observation);

    // The distance traveled is the speed * time divided by our extra scaling factor (for integer math precision).
    const int estimated_distance = clipped_angular_speed * time / speed_scale;

    // Get the error in our estimate.
    const int estimated_distance_error = distance_to_trigger - estimated_distance;

    // Get the speed error.
    const int estimated_speed_error = speed_scale * estimated_distance_error / time;

    // Precompute the square of time and downscale by our magic scale to keep it within integer arithmetic bounds.
    const int square_time_div_square_speed_scale = clip_to(1, max_16bit, square(time / speed_scale));
    const int square_time_div_square_accel_scale = clip_to(1, max_16bit, square(time / accel_scale));
    // Calculate the variances.

    // The angular speed variance increases with acceleration over time.
    const int angular_speed_variance = min(max_16bit,
        angular_speed_variance_at_observation + position_calibration.angular_acceleration_div_2_variance * square_time_div_square_accel_scale);
    
    // The distance variance does not depend on the trigger variance as we're treating it as a coordinate change.
    // This is the variance of the estimated angle! It depends on the last angle and speed variances.
    const int estimated_distance_variance = min(max_16bit,
        angle_variance_at_observation + square_time_div_square_speed_scale * angular_speed_variance);

    // Variance of the speed error. This is the variance of our predicted speed. It depends on the estimate of the
    // last angle, the trigger angle, and the previous speed variance.
    const int estimated_speed_error_variance = min(max_16bit, 
        angular_speed_variance + (angle_variance_at_observation + trigger_variance) / square_time_div_square_speed_scale);

    // Adjust the distance using the kalman gain. This is actually the new mean of a product of gaussians.
    const int distance_adjustment = (estimated_distance_error * estimated_distance_variance) / (estimated_distance_variance + trigger_variance);
    // Similarly adjust the speed based on our new guess and previous variance.
    const int speed_adjustment = (estimated_speed_error * angular_speed_variance) / (angular_speed_variance + estimated_speed_error_variance);
    
    // Return to the original coordinates and use the distance adjustment computed with the kalman gain.
    const int kalman_angle = normalize_angle(angle_at_observation + estimated_distance + distance_adjustment);
    
    // Update the position parameters.
    
    // Store the previous hall sector and mark it as valid.
    previous_hall_sector = hall_sector;
    
    // Ensure the angle at observation is within the confidence band of the current sector. We only need to check the lower bound now, we
    // did the upper bound by clipping the angular speed.
    angle_at_observation = kalman_angle + direction * max(0, 
        direction * signed_angle(trigger_angle - direction * sector_transition_confidence - kalman_angle));

    // Calculate the new angle variance by combining the trigger variance and the estimated angle variance.
    angle_variance_at_observation = clip_to(2, sector_variance,
        trigger_variance * estimated_distance_variance / (trigger_variance + estimated_distance_variance));
    
    // Update the angular speed using the clipped speed and the adjustment calculated with the kalman gain.
    angular_speed_at_observation = clipped_angular_speed + speed_adjustment;

    // Calculate the new angular speed variance by the old variance and the estimated speed error variance.
    angular_speed_variance_at_observation = clip_to(2, position_calibration.initial_angular_speed_variance,
        angular_speed_variance * estimated_speed_error_variance / (angular_speed_variance + estimated_speed_error_variance));

    // Flag to the PWM cycle loop that we have a new observation.
    new_observation = true;
    angle_valid = true;
}