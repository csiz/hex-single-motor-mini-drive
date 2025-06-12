#pragma once

#include "constants.hpp"
#include "io.hpp"
#include "error_handler.hpp"
#include "math_utils.hpp"
#include "integer_math.hpp"

#include <cstdint>


// Position tracking
// -----------------

// Position calibration data. These are the trigger angles for each hall sensor output.
extern PositionCalibration position_calibration;


// Track angle, angular speed and their uncertainties as gaussian distributions.
struct PositionStatistics {
    int angle; // Estimated angle.
    int angle_variance; // Variance of the angle estimate.
    int angular_speed; // Estimated angular speed.
    int angular_speed_variance; // Variance of the angular speed estimate.
};

// Zeroes position statistics; also means infinite variance.
const PositionStatistics null_position_statistics = {0};


// Combine gaussian with mean a and variance a with gaussian with mean 0 and variance b.
static inline int combined_gaussian_adjustment(int a, int variance_a, int variance_b){
    // Magic value 0 for infinite variance return 0 error.
    if (variance_a == 0) return 0;

    const int variance_sum = variance_a + variance_b;
    return signed_round_div(a * variance_b, variance_sum);
}

// Combine gaussians; this is the product of two normal PDFs with means a and b 
// and variances variance_a and variance_b. The result is a normal PDF with 
// inbetween mean and better variance. The meaning of the product is the a PDF
// representing the posterior distribution of the variable of interest assuming
// both input distribtutions represent the same underlying variable.
static inline int combined_gaussian_mean(int a, int variance_a, int b, int variance_b){
    const int variance_sum = variance_a + variance_b;
    return signed_round_div(a * variance_b + b * variance_a, variance_sum);
}

// Get the variance of the product of two normal PDFs.
static inline int combined_gaussian_variance(int variance_a, int variance_b){
    // Magic value 0 for infinite variance; return prior variance unchanged.
    if (variance_a == 0) return variance_b;

    const int variance_sum = variance_a + variance_b;
    // Round to nearest integer rather than floor.
    return round_div(variance_a * variance_b, variance_sum);
}

// Perform the Kalman filter update. We are finding the posterior distribution given our
// prior distribution (the predicted angle and speed) and the observation (the hall sensor trigger).
// We consider both inputs to be gaussian distributions, the result is also a gaussian distribution.
static inline PositionStatistics bayesian_update(
    PositionStatistics const & prior, 
    PositionStatistics const & measurement_error,
    int max_angle_variance
){
    // Note that we have changed coordinates so the predicted angle is 0 and so is the predicted speed.
    // In this case we only need to adjust based on the distance error.
    

    // Adjust the distance using the kalman gain. This is actually the new mean of a product of gaussians.
    const int distance_adjustment = combined_gaussian_adjustment(
        measurement_error.angle,
        measurement_error.angle_variance,
        prior.angle_variance
    );


    // Update the position parameters.

    // Adjust angle by at least 1 LSB in the direction of the error in case we rounded down to 0.
    const int angle = normalize_angle(prior.angle + (distance_adjustment ? distance_adjustment : sign(measurement_error.angle)));

    const int angle_variance = clip_to(
        1, max_angle_variance,
        combined_gaussian_variance(
            measurement_error.angle_variance,
            prior.angle_variance
        )
    );


    // Similarly adjust the speed based on our new guess and previous variance.
    const int speed_adjustment = combined_gaussian_adjustment(
        measurement_error.angular_speed,
        measurement_error.angular_speed_variance,
        prior.angular_speed_variance
    );

    const int angular_speed = prior.angular_speed + (speed_adjustment ? speed_adjustment : sign(measurement_error.angular_speed));

    const int angular_speed_variance = clip_to(
        1, position_calibration.initial_angular_speed_variance,
        combined_gaussian_variance(
            measurement_error.angular_speed_variance,
            prior.angular_speed_variance
        )
    );

    return PositionStatistics{
        .angle = angle,
        .angle_variance = angle_variance,
        .angular_speed = angular_speed,
        .angular_speed_variance = angular_speed_variance
    };
}


// Simple Euler iteration of the position based on the previous state.
static inline PositionStatistics predict_position(PositionStatistics const & previous){
    // We predict the position exactly once every cycle, thus we've chosen the time step dt = 1.
    // Also note that we can only use integer arithmetic on the STM32F01x so we use fixed point math.

    const int predicted_angular_speed = previous.angular_speed;

    const int predicted_angular_speed_variance = min(
        max_16bit,
        previous.angular_speed_variance + 
        position_calibration.angular_acceleration_div_2_variance / acceleration_variance_fixed_point + 1
    );

    const int predicted_angle = normalize_angle(
        previous.angle + 
        signed_round_div(previous.angular_speed, speed_fixed_point)
    );

    const int predicted_angle_variance = min(
        max_16bit,
        previous.angle_variance +
        predicted_angular_speed_variance / speed_variance_fixed_point + 1
    );

    return PositionStatistics{
        .angle = predicted_angle,
        .angle_variance = predicted_angle_variance,
        .angular_speed = predicted_angular_speed,
        .angular_speed_variance = predicted_angular_speed_variance
    };
}

// Compute the error in the position when we see a hall sensor toggle.
static inline PositionStatistics compute_transition_error(
    PositionStatistics const & predicted_position, 
    const uint8_t hall_sector, 
    const uint8_t previous_hall_sector
) {
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
    const int angle_error = signed_angle(hall_angle - predicted_position.angle);

    // We could track the timing of the hall sensor more accurately with the second interrupt, but then
    // we'd need to perform a more complicated update. Since we omit it, we don't know when the transition
    // occurred during the previous cycle; add the covariance between time and angle (it depends on speed).
    const int timing_variance = (
        predicted_position.angular_speed / speed_variance_to_square_speed * 
        predicted_position.angular_speed / speed_variance_fixed_point
    );

    // The variance increases with angular speed as the transition could have occured at any point in the cycle.
    const int angle_variance_error = min(max_16bit, hall_variance + timing_variance);


    return PositionStatistics{
        .angle = angle_error,
        .angle_variance = angle_variance_error,
        .angular_speed = angle_error * speed_fixed_point,
        .angular_speed_variance = min(max_16bit, angle_variance_error * speed_variance_fixed_point)
    };
}

// Calculate any adjustments when we don't have new hall data.
static inline PositionStatistics compute_non_transition_error(
    PositionStatistics const & predicted_position, 
    const uint8_t hall_sector
) {
    // No transition seen yet; anticipate the next hall transition and compare to our predicted
    // angle. If we should have crossed it by prediction then we should slow down our speed estimate.

    const uint8_t next_hall_sector = predicted_position.angular_speed >= 0 ? 
        ((hall_sector + 1) % hall_sector_base) : 
        ((hall_sector + hall_sector_base - 1) % hall_sector_base);

    // Positive rotations index the first element in the calibration table, negative the second.
    const size_t direction_index = predicted_position.angular_speed >= 0 ? 0 : 1;

    // Get the upcoming hall transition angle.
    const int hall_angle = position_calibration.sector_transition_angles[next_hall_sector][direction_index];

    // The variance of the upcoming transition doesn't depend on speed, because we haven't seen it yet.
    const int hall_variance = position_calibration.sector_transition_variances[next_hall_sector][direction_index];

    // Calculate the distance to the trigger angle.
    const int distance_to_hall_angle = signed_angle(hall_angle - predicted_position.angle);

    // The distance to the trigger encompases our uncertainty in the predicted angle.
    const int distance_to_hall_angle_variance = min(max_16bit, predicted_position.angle_variance + hall_variance);
    
    // Grab data for the current hall sector; assume we are correcting towards the center.

    const int distance_to_hall_center = signed_angle(position_calibration.sector_center_angles[hall_sector] - predicted_position.angle);
    const int distance_to_hall_center_variance = position_calibration.sector_center_variances[hall_sector];


    // Check if we are more than 2 standard deviations away from our prediction.
    const bool tail_end = square(distance_to_hall_angle) > (16 * distance_to_hall_angle_variance * variance_divider);

    // Check if the distance to the next hall angle is in the same direction as the predicted speed;
    // in that case, we will eventually cross the angle if we keep going at the same speed.
    const bool same_direction = (predicted_position.angular_speed * distance_to_hall_angle) >= 0;

    if (same_direction) {
        // No transition expected and no information gained; return magic variance 0 indicating infinity.
        return null_position_statistics;

    } else {
        const int uncertainty = tail_end ? 1 : 2;

        // If we're moving past the next hall transition without seeing the transition; bring back towards the center.
        return PositionStatistics{
            .angle = distance_to_hall_center,
            .angle_variance = uncertainty * distance_to_hall_center_variance,
            .angular_speed = distance_to_hall_angle * speed_fixed_point,
            .angular_speed_variance = min(
                // We max out the variance every time at the moment.
                max_16bit,
                uncertainty * distance_to_hall_center_variance * speed_variance_fixed_point
            )
        };
    }
}

// Get the sector center position.
static inline PositionStatistics get_default_sector_position(const uint8_t hall_sector) {
    return PositionStatistics{
        .angle = position_calibration.sector_center_angles[hall_sector],
        .angle_variance = position_calibration.sector_center_variances[hall_sector],
        .angular_speed = 0,
        .angular_speed_variance = position_calibration.initial_angular_speed_variance
    };
}

// Update the predicted position based on the hall sensors (whether there's a reading or not).
static inline PositionStatistics infer_position_from_hall_sensors(
    PositionStatistics const & predicted_position,
    const uint8_t hall_sector, 
    const uint8_t previous_hall_sector
) {
    // Check if we've switched sectors.
    const bool is_hall_transition = (hall_sector != previous_hall_sector);

    // Calculate the error depending on whether we have a transition or not.
    const auto position_error = (is_hall_transition ? 
        compute_transition_error(
            predicted_position,
            hall_sector,
            previous_hall_sector) : 
        compute_non_transition_error(
            predicted_position,
            hall_sector
        )
    );

    // Reference the maximum angle variance for this sector.
    const int max_angle_variance = position_calibration.sector_center_variances[hall_sector];

    // Update the position statistics by combining gaussian distributions (this is the Kalman update).
    return bayesian_update(predicted_position, position_error, max_angle_variance);
}

// Read the hall sensors and update the motor rotation angle. Sensor chips might be: SS360NT (can't read the inprint clearly).
static inline uint8_t read_hall_sensors_state(){
    // Grab the registers for the GPIO ports with the hall sensors.

    uint16_t gpio_A_inputs = LL_GPIO_ReadInputPort(GPIOA);
    uint16_t gpio_B_inputs = LL_GPIO_ReadInputPort(GPIOB);

    // Note: Hall sensors are active low!
    const bool hall_1 = !(gpio_A_inputs & (1<<0)); // Hall sensor 1, corresponding to phase V
    const bool hall_2 = !(gpio_A_inputs & (1<<1)); // Hall sensor 2, corresponding to phase W
    const bool hall_3 = !(gpio_B_inputs & (1<<10)); // Hall sensor 3, corresponding to phase U

    // Combine the hall sensor states into a single byte.
    // Note: Reorder the sensors according to the phase order; U on bit 0, V on bit 1, W on bit 2.
    const uint8_t hall_state = hall_3 | (hall_1 << 1) | (hall_2 << 2);

    return hall_state;
}

// Mapping from hall state to sector number.
static inline uint8_t get_hall_sector(uint8_t hall_state){
    // Get the hall sector from the state.
    switch (hall_state) {
        case 0b000: // no hall sensors; either it's not ready or no magnet
            return hall_sector_base; // Out of range, indicates invalid.
        case 0b001: // hall U active; 0 degrees
            return 0;
        case 0b011: // hall U and hall V active; 60 degrees
            return 1;
        case 0b010: // hall V active; 120 degrees
            return 2;
        case 0b110: // hall V and hall W active; 180 degrees
            return 3;
        case 0b100: // hall W active; 240 degrees
            return 4;
        case 0b101: // hall U and hall W active; 300 degrees
            return 5;
        case 0b111: // all hall sensors active; this would be quite unusual; but carry on
            return hall_sector_base; // Out of range, indicates invalid.
        default:
            error(); // Invalid hall state.
            return hall_sector_base; // Out of range, indicates invalid.
    }
}
