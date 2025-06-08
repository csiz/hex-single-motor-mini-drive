#pragma once

#include "constants.hpp"
#include "io.hpp"
#include "error_handler.hpp"
#include "math_utils.hpp"
#include "integer_math.hpp"

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

struct PositionStatistics {
    int angle; // Estimated angle.
    int angle_variance; // Variance of the angle estimate.
    int angular_speed; // Estimated angular speed.
    int angular_speed_variance; // Variance of the angular speed estimate.
};

// Whether the position is valid.
extern bool angle_valid;

// Best estimate of the **electric** angle and speed (that is the angle between the coil 
// U and the magnetic North). The actual position of the output depends on the number of
// pole pairs, and the slot triplets, and the mechanical gear ratio.
extern PositionStatistics electric_position;

// Number of updates since we last seen a hall transition.
extern int updates_since_last_hall_transition;

// Last angle prediction error.
extern PositionStatistics position_error;

// Position calibration data. These are the trigger angles for each hall sensor output.
extern PositionCalibration position_calibration;


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

// Combine gaussian with mean a and variance a with gaussian with mean 0 and variance b.
static inline int combined_gaussian_mean(int a, int variance_a, int variance_b){
    const int variance_sum = variance_a + variance_b;
    return (a * variance_b) / variance_sum;
}

static inline int combined_gaussian_variance(int variance_a, int variance_b){
    const int variance_sum = variance_a + variance_b;
    // Round to nearest integer rather than floor.
    return (variance_a * variance_b + variance_sum / 2) / variance_sum;
}

// Update the position from the hall sensors. Use a kalman filter to estimate the position and speed.
static inline void update_position(){
    read_hall_sensors();

    // Check if the magnet is present.
    if (hall_sector >= hall_sector_base) {
        previous_hall_sector = hall_sector_base; // Out of range, indicates invalid.
        angle_valid = false;
        return;
    }

    // The new hall sector is valid, let's calculate the angle.

    // Check if the previous sector was valid.
    if (previous_hall_sector >= hall_sector_base) {

        // This is the first time we have a valid hall sector; we need to set the angle and the angular speed.
        previous_hall_sector = hall_sector;

        // Reset position tracking to the sector center; our best guess.
        electric_position.angle = position_calibration.sector_center_angles[hall_sector];
        electric_position.angle_variance = position_calibration.sector_center_variances[hall_sector];
        electric_position.angular_speed = 0;
        electric_position.angular_speed_variance = position_calibration.initial_angular_speed_variance;

        angle_valid = true;
        return;
    }

    // We have a valid state; perform a Kalman filter prediction.
    
    // Apply the minimum amount of friction to bias speed towards 0.
    const int predicted_angular_speed = electric_position.angular_speed; // - sign(electric_position.angular_speed);

    const int predicted_angular_speed_variance = min(
        max_16bit,
        electric_position.angular_speed_variance + 
        position_calibration.angular_acceleration_div_2_variance / acceleration_variance_fixed_point + 1
    );

    const int predicted_angle = normalize_angle(
        electric_position.angle + 
        (electric_position.angular_speed + speed_fixed_point / 2) / speed_fixed_point
    );

    const int predicted_angle_variance = min(
        max_16bit,
        electric_position.angle_variance +
        predicted_angular_speed_variance / speed_variance_fixed_point + 1
    );


    // Check if we've switched sectors.
    const bool is_hall_transition = (hall_sector != previous_hall_sector);

    if (is_hall_transition) {
        // We have a hall transition, we know our position crossed the trigger angle in the last cycle.

        // Get the sector integer distance to determine the direction of rotation.
        const int sector_distance = (9 + hall_sector - previous_hall_sector) % 6 - 3;
        
        // Positive rotations index the first element in the calibration table, negative the second.
        const size_t direction_index = sector_distance >= 0 ? 0 : 1;
        
        
        // Get data about this transition from the calibration table.
        
        const int hall_angle = position_calibration.sector_transition_angles[hall_sector][direction_index];
        const int hall_variance = position_calibration.sector_transition_variances[hall_sector][direction_index];
        
        
        // Change coordinates with predicted angle as the center. The predicted angle becomes the 0
        // point and we'll calculate the trigger angle relative to it.
        // 
        // Note: Don't add up variances for the change of coordinates! It's just a math trick.
        
        // Calculate the prediction error for the angle.
        position_error.angle = signed_angle(hall_angle - predicted_angle);

        const int timing_variance = square(predicted_angular_speed) / angle_variance_to_square_speed + 1;
        
        // The variance increases with angular speed as the transition could have occured at any point in the cycle.
        position_error.angle_variance = min(max_16bit, hall_variance + timing_variance);

        const int previous_hall_angle = position_calibration.sector_transition_angles[previous_hall_sector][direction_index];

        const int sector_hall_distance = signed_angle(hall_angle - previous_hall_angle);
        const int measured_speed = (
            (sector_hall_distance * speed_fixed_point + updates_since_last_hall_transition / 2) / 
            updates_since_last_hall_transition
        );

        const int speed_timing_error = measured_speed - predicted_angular_speed;
        const int speed_timing_variance = min(
            max_16bit,
            position_error.angle_variance / updates_since_last_hall_transition +
            updates_since_last_hall_transition * position_calibration.angular_acceleration_div_2_variance / acceleration_variance_fixed_point +
            1
        );

        const int angle_change_error = position_error.angle * speed_fixed_point;
        const int angle_change_variance = min(
            max_16bit,
            position_error.angle_variance * speed_variance_fixed_point
        );

        position_error.angular_speed = speed_timing_variance > angle_change_variance ? 
            angle_change_error : 
            speed_timing_error;
        position_error.angular_speed_variance = speed_timing_variance > angle_change_variance ? 
            angle_change_variance : 
            speed_timing_variance;

        updates_since_last_hall_transition = 1;
    } else {
        // No transition seen yet; anticipate the next hall transition and compare to our predicted
        // angle. If we should have crossed it by prediction then we should slow down our speed estimate.

        updates_since_last_hall_transition = min(max_updates_between_transitions, updates_since_last_hall_transition + 1);
    
        const uint8_t next_hall_sector = predicted_angular_speed >= 0 ? 
            ((hall_sector + 1) % hall_sector_base) : 
            ((hall_sector + hall_sector_base - 1) % hall_sector_base);

        const size_t direction_index = predicted_angular_speed >= 0 ? 0 : 1;

        // Get the upcoming hall transition angle.
        const int hall_angle = position_calibration.sector_transition_angles[next_hall_sector][direction_index];
    
        // The variance of the upcoming transition doesn't depend on speed, because we haven't seen it yet.
        const int hall_variance = position_calibration.sector_transition_variances[next_hall_sector][direction_index];
    
        // Calculate the distance to the trigger angle.
        const int distance_to_hall_angle = signed_angle(hall_angle - predicted_angle);
    
        // The distance to the trigger encompases our uncertainty in the predicted angle.
        const int distance_to_hall_angle_variance = min(max_16bit, predicted_angle_variance + hall_variance);
        
        // Grab data for the current hall sector; assume we are correcting towards the center.
    
        const int distance_to_hall_center = signed_angle(position_calibration.sector_center_angles[hall_sector] - predicted_angle);
        const int distance_to_hall_center_variance = position_calibration.sector_center_variances[hall_sector];
    
    
        // Check if we are more than 2 standard deviations away from our prediction.
        const bool tail_end = square(distance_to_hall_angle) > (4 * distance_to_hall_angle_variance * variance_divider);
    
        // Check if the distance to the next hall angle is in the same direction as the predicted speed;
        // in that case, we will eventually cross the angle if we keep going at the same speed.
        const bool same_direction = (predicted_angular_speed * distance_to_hall_angle) >= 0;
        
        if (same_direction) {
            // No transition expected and no information gained; return magic variance 0 indicating infinity.
            position_error.angle = 0;
            position_error.angle_variance = 0;
            position_error.angular_speed = 0;
            position_error.angular_speed_variance = 0;
        } else {
            // If we're moving past the next hall transition without seeing the transition; bring back towards the center.
            position_error.angle = distance_to_hall_center;
            position_error.angle_variance = (tail_end ? 1 : 2) * distance_to_hall_center_variance;
            // Distance to hall angle is opposite the speed; correct by this amount.
            position_error.angular_speed = distance_to_hall_angle * speed_fixed_point;
            position_error.angular_speed_variance = min(
                max_16bit,
                (tail_end ? 2 : 4) * (predicted_angular_speed_variance + distance_to_hall_angle_variance * speed_variance_fixed_point)
            );
        }
    }
    
    // Store the current hall sector.
    previous_hall_sector = hall_sector;

    const int max_position_variance = position_calibration.sector_center_variances[hall_sector];

    // Perform the Kalman filter update. We are finding the posterior distribution given our
    // prior distribution (the predicted angle and speed) and the observation (the hall sensor trigger).
    // We consider both inputs to be gaussian distributions, the result is also a gaussian distribution.
    // 
    // Note that we have changed coordinates so the predicted angle is 0 and so is the predicted speed.
    // In this case we only need to adjust based on the distance error.
    
    if (not position_error.angle_variance) {
        // Infinite variance, keep the prediction.
        electric_position.angle = predicted_angle;
        electric_position.angle_variance = clip_to(
            1, max_position_variance,
            predicted_angle_variance
        );
    } else {
        // Adjust the distance using the kalman gain. This is actually the new mean of a product of gaussians.
        const int distance_adjustment = combined_gaussian_mean(
            position_error.angle,
            position_error.angle_variance,
            predicted_angle_variance
        );


        // Update the position parameters.
    
        // Adjust angle by at least 1 LSB in the direction of the error in case we rounded down to 0.
        electric_position.angle = normalize_angle(predicted_angle + (distance_adjustment ? distance_adjustment : sign(position_error.angle)));

        electric_position.angle_variance = clip_to(
            1, max_position_variance,
            combined_gaussian_variance(
                position_error.angle_variance,
                predicted_angle_variance
            )
        );
    }


    
    if (not position_error.angular_speed_variance) {
        // Infinite variance for the angular speed, keep the prediction.
        electric_position.angular_speed = predicted_angular_speed;
        electric_position.angular_speed_variance = clip_to(
            1, position_calibration.initial_angular_speed_variance,
            predicted_angular_speed_variance
        );
    } else {
        // Similarly adjust the speed based on our new guess and previous variance.
        const int speed_adjustment = combined_gaussian_mean(
            position_error.angular_speed,
            position_error.angular_speed_variance,
            predicted_angular_speed_variance
        );

        electric_position.angular_speed = predicted_angular_speed + (speed_adjustment ? speed_adjustment : sign(position_error.angular_speed));

        electric_position.angular_speed_variance = clip_to(
            1, position_calibration.initial_angular_speed_variance,
            combined_gaussian_variance(
                position_error.angular_speed_variance,
                predicted_angular_speed_variance
            )
        );
    }
}