#pragma once

#include <cstdint> // For int16_t, int32_t
#include <array>
#include <utility> // For std::pair

#include "constants.hpp"

// --- Constants and Look-Up Tables (LUTs) ---
const int ATAN2_CORDIC_ITERATIONS = 10; // Number of CORDIC iterations. Affects precision and gain.

extern const std::array<int16_t, ATAN2_CORDIC_ITERATIONS> atan2_cordic_lut;

// CORDIC gain K = product_{i=0 to N-1} sqrt(1 + 2^(-2i)).
// We need 1/K for magnitude scaling.
// (1/K) is pre-scaled by (1 << ATAN2_CORDIC_GAIN_SHIFT) for fixed-point multiplication.
const int ATAN2_CORDIC_GAIN_SHIFT = 15;
extern const int32_t atan2_cordic_inverse_gain_scaled;



// Helper for integer absolute value, as std::abs might not be constexpr or pull <cstdlib>
inline constexpr int32_t internal_abs_int32(int32_t val) {
    return (val < 0) ? -val : val;
}

/**
 * @brief Computes the 2-argument arctangent (atan2) using an integer CORDIC algorithm.
 *
 * @param y_coord The y-coordinate (int16_t).
 * @param x_coord The x-coordinate (int16_t).
 * @return std::pair<int16_t, int16_t> The angle in the range [0, 1023], representing [0, ~360) degrees.
 * and the computed magnitude sqrt(x^2+y^2).
 * Uses only integer math operations.
 */
static inline std::pair<int16_t, int16_t> atan2_integer(int16_t y_coord, int16_t x_coord) {
    // Ensure atan2_init_tables() has been called once globally.

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
    int32_t current_x = internal_abs_int32(x);
    int32_t current_y = internal_abs_int32(y);
    
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
            first_quadrant_angle_acc += atan2_cordic_lut[i];
        } else if (y_previous_iter < 0) { // Rotate counter-clockwise: (x', y') = (x - y*2^-i, y + x*2^-i)
            current_x = x_previous_iter - y_shifted;
            current_y = y_previous_iter + x_shifted;
            first_quadrant_angle_acc -= atan2_cordic_lut[i];
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
        final_angle_units = static_cast<int16_t>(angle_units_per_circle - first_quadrant_angle_acc); // 2*PI - alpha
    }

    // Normalize the angle to be strictly within [0, 1023]
    // (e.g., if 1024 - 0 results in 1024, it should be 0)
    final_angle_units %= angle_units_per_circle;


    // Calculate magnitude: original_magnitude = current_x / K_gain
    // This is done using the pre-scaled inverse gain: (current_x * (1/K_gain * 2^SHIFT)) >> SHIFT
    const int16_t magnitude = (current_x * atan2_cordic_inverse_gain_scaled) >> ATAN2_CORDIC_GAIN_SHIFT;

    return {final_angle_units, magnitude};
}


void unit_test_atan(char * buffer, size_t max_size);