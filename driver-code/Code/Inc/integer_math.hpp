#pragma once

#include <cstdint> // For int16_t, int32_t
#include <array>
#include <utility> // For std::pair

#include "constants.hpp"
#include "math_utils.hpp"




// --- Constants and Look-Up Tables ---
const int ATAN2_CORDIC_ITERATIONS = 10; // Number of CORDIC iterations. Affects precision and gain.

// CORDIC gain K = product_{i=0 to N-1} sqrt(1 + 2^(-2i)).
// We need 1/K for magnitude scaling.
// (1/K) is pre-scaled by (1 << ATAN2_CORDIC_GAIN_SHIFT) for fixed-point multiplication.
const int ATAN2_CORDIC_GAIN_SHIFT = 15;

// Initializes the CORDIC look-up tables and precomputed gain.
// 
// This function MUST be called once globally before using atan2_cordic.
// It uses floating-point math for table generation. 
inline constexpr std::array<int16_t, ATAN2_CORDIC_ITERATIONS> make_atan2_cordic_lookup() {
    std::array<int16_t, ATAN2_CORDIC_ITERATIONS> atan2_cordic_lookup;

    for (int i = 0; i < ATAN2_CORDIC_ITERATIONS; ++i) {
        // The angle output is in units where 1024 represents a full circle (2*PI radians).
        // So, 1 angle unit = (2*PI)/1024 = PI/512 radians.
        // The lookup table stores atan(2^-i) in these angle units.
        // lookup_value = atan(2^-i) [radians] * (512 / PI) [units/radian]
        atan2_cordic_lookup[i] = static_cast<int16_t>(round(atan(pow(2.0, -i)) * static_cast<float>(half_circle) / M_PI));
    }

    return atan2_cordic_lookup;
}

inline constexpr int32_t make_atan2_cordic_inverse_gain_scaled() {
    double K_gain = 1.0;
    for (int i = 0; i < ATAN2_CORDIC_ITERATIONS; ++i) {
        K_gain *= sqrt(1.0 + pow(2.0, -2.0 * i));
    }
    return static_cast<int32_t>(round((1.0 / K_gain) * (1 << ATAN2_CORDIC_GAIN_SHIFT)));
}

const int32_t atan2_cordic_inverse_gain_scaled = make_atan2_cordic_inverse_gain_scaled();

const std::array<int16_t, ATAN2_CORDIC_ITERATIONS> atan2_cordic_lookup = make_atan2_cordic_lookup();


/**
 * @brief Computes the 2-argument arctangent (atan2) using an integer CORDIC algorithm.
 *
 * @param y_coord The y-coordinate (int16_t).
 * @param x_coord The x-coordinate (int16_t).
 * @return std::pair<int16_t, int16_t> The angle in the range [0, 1023], representing [0, ~360) degrees.
 * and the computed magnitude sqrt(x^2+y^2).
 * Uses only integer math operations.
 */
static inline std::pair<int16_t, int16_t> int_atan2(int16_t y_coord, int16_t x_coord) {
    
    int32_t x = x_coord; // Use 32-bit integers for intermediate calculations
    int32_t y = y_coord;

    // Handle (0,0) case
    if (x == 0 && y == 0) {
        return {0, 0};
    }

    // Handle cases on the axes directly for precision and to simplify CORDIC logic.
    // Angle scale: 0-1023 for 0 to ~360 degrees.
    // 0 degrees = 0
    // 90 degrees = 1024 / 4 = 256
    // 180 degrees = 1024 / 2 = 512
    // 270 degrees = 1024 * 3 / 4 = 768
    if (x == 0) {
        if (y > 0) { // Angle is 90 degrees
            return {quarter_circle, y};
        } else { // Angle is 270 degrees (y < 0)
            return {three_quarters_circle, -y};
        }
    }
    if (y == 0) {
        if (x > 0) { // Angle is 0 degrees

            return {0, x};
        } else { // Angle is 180 degrees (x < 0)
            return {half_circle, -x};
        }
    }

    // Store original signs to determine the correct quadrant for the final angle
    bool x_is_negative = (x < 0);
    bool y_is_negative = (y < 0);

    // Use absolute values for the CORDIC vectoring stage.
    // This effectively processes the vector as if it's in the first quadrant.
    int32_t current_x = abs(x);
    int32_t current_y = abs(y);
    
    // Accumulated angle from CORDIC iterations (atan(abs_y / abs_x))
    // This will be in the range [0, 256] (representing 0 to 90 degrees).
    int32_t first_quadrant_angle_acc = 0; 

    // CORDIC vectoring mode: rotate (current_x, current_y) to align with the positive X-axis.
    // The sum of applied rotations gives the angle.
    for (int i = 0; i < ATAN2_CORDIC_ITERATIONS; ++i) {
        int32_t x_previous_iter = current_x;
        int32_t y_previous_iter = current_y;

        // Pre-shifted values for the current iteration
        int32_t x_shifted = x_previous_iter >> i; // Effectively x_previous_iter * 2^(-i)
        int32_t y_shifted = y_previous_iter >> i; // Effectively y_previous_iter * 2^(-i)

        // Determine direction of rotation to drive y towards zero.
        // Since current_y starts positive (due to abs_int32),
        // if y_previous_iter > 0, rotate clockwise.
        // if y_previous_iter < 0, rotate counter-clockwise (can happen if y overshoots zero).
        // if y_previous_iter == 0, no rotation for this step.
        if (y_previous_iter > 0) { // Rotate clockwise: (x', y') = (x + y*2^-i, y - x*2^-i)
            current_x = x_previous_iter + y_shifted;
            current_y = y_previous_iter - x_shifted;
            first_quadrant_angle_acc += atan2_cordic_lookup[i];
        } else if (y_previous_iter < 0) { // Rotate counter-clockwise: (x', y') = (x - y*2^-i, y + x*2^-i)
            current_x = x_previous_iter - y_shifted;
            current_y = y_previous_iter + x_shifted;
            first_quadrant_angle_acc -= atan2_cordic_lookup[i];
        }
        // If y_previous_iter == 0, current_x, current_y, and angle_acc remain unchanged for this iteration.
    }
    // At this point, first_quadrant_angle_acc contains atan(abs(y_coord)/abs(x_coord)).
    // current_x contains K * sqrt(x_coord^2 + y_coord^2), where K is the CORDIC gain.

    // Adjust angle based on the original quadrant
    int16_t final_angle_units;
    if (!x_is_negative && !y_is_negative) {         // Quadrant 1 (x > 0, y >= 0)
        final_angle_units = static_cast<int16_t>(first_quadrant_angle_acc);
    } else if (x_is_negative && !y_is_negative) {  // Quadrant 2 (x < 0, y >= 0)
        final_angle_units = static_cast<int16_t>(half_circle - first_quadrant_angle_acc); // PI - alpha
    } else if (x_is_negative && y_is_negative) {   // Quadrant 3 (x < 0, y < 0)
        final_angle_units = static_cast<int16_t>(half_circle + first_quadrant_angle_acc); // PI + alpha
    } else { // (!x_is_negative && y_is_negative)   // Quadrant 4 (x > 0, y < 0)
        final_angle_units = static_cast<int16_t>(angle_base - first_quadrant_angle_acc); // 2*PI - alpha
    }

    // Normalize the angle to be strictly within [0, 1023]
    // (e.g., if 1024 - 0 results in 1024, it should be 0)
    final_angle_units %= angle_base;


    // Calculate magnitude: original_magnitude = current_x / K_gain
    // This is done using the pre-scaled inverse gain: (current_x * (1/K_gain * 2^SHIFT)) >> SHIFT
    const int16_t magnitude = (current_x * atan2_cordic_inverse_gain_scaled) >> ATAN2_CORDIC_GAIN_SHIFT;

    return {final_angle_units, magnitude};
}


void unit_test_atan(char * buffer, size_t max_size);


// Square root of integer (straight from https://en.wikipedia.org/wiki/Integer_square_root).
static inline int int_sqrt(int s)
{
	// Zero yields zero
    // One yields one
	if (s <= 1) 
		return s;

    // Initial estimate (must be too high)
	int x0 = s / 2;

	// Update
	int x1 = (x0 + s / x0) / 2;

	while (x1 < x0)	// Bound check
	{
		x0 = x1;
		x1 = (x0 + s / x0) / 2;
	}		
	return x0;
}

// Minimum magnitude for funky atan2 to be considered valid
const int funky_atan2_constant = 4;

// Square of the minimum magnitude for atan2 to be considered useful.
const int sq_ok_atan2_magnitude = 16 * 16;

static inline int funky_atan2(int y, int x) {    
    if (x == 0 and y == 0) return 0;

    int result = 0;
    if (x < 0) {
        // Rotate 180 degrees and compensate in the result.
        result += half_circle;
        x = -x;
        y = -y;
    }

    if (y < 0) {
        // Rotate 270 degrees and compensate in the result.
        result += three_quarters_circle;
        int temp = x;
        x = -y;
        y = temp;
    }
    
    // Now x >= 0 and y >= 0.

    if (x >= y) {
        result += (funky_atan2_constant * y * eighth_circle) / (funky_atan2_constant * x + y);
    } else {
        result += quarter_circle - (funky_atan2_constant * x * eighth_circle) / (funky_atan2_constant * y + x);
    }

    // Return the result sign normalized to [-half_circle to +half_circle].
    return (result + half_circle) % angle_base - half_circle;
}


void unit_test_funky_atan(char * buffer, size_t max_size);
void unit_test_funky_atan_part_2(char * buffer, size_t max_size);
void unit_test_funky_atan_part_3(char * buffer, size_t max_size);