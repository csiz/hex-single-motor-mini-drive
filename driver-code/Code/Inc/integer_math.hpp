#pragma once

#include <cstdint> // For int16_t, int32_t
#include <array>
#include <utility> // For std::pair
#include <bit>

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


struct IntyFloat {
    uint16_t mantissa;
    uint8_t sign; // 0 for positive, 1 for negative
    int8_t exponent;
};

static inline IntyFloat make_inty_float(uint32_t mantissa, uint8_t sign = 0, int8_t exponent = 0) {
    // Return in case of zero; default exponent to 0 as well.
    if (mantissa == 0) return IntyFloat{0, 0, 0};

    // Get the position of the most significant bit in the mantissa.
    const int shift = 32 - 15 - std::countl_zero(mantissa);
    
    return IntyFloat{
        // Shift the result to keep the significant bit at position 15; 
        // we will return int16_t on request so we need 15 bits + 1 for the sign.
        static_cast<uint16_t>(shift < 0 ? mantissa << -shift : mantissa >> shift),
        // XOR to determine the sign of the result.
        static_cast<uint8_t>(sign),
        // Add the exponents and adjust for the shift.
        static_cast<int8_t>(exponent + shift)
    };
}

static inline IntyFloat make_inty_float(int const & value) {
    return make_inty_float(static_cast<uint32_t>(value < 0 ? -value : value), value < 0, 0);
}

static inline IntyFloat operator * (IntyFloat const & a, IntyFloat const & b) {
    const uint32_t mantissa = static_cast<uint32_t>(a.mantissa) * static_cast<uint32_t>(b.mantissa);

    return make_inty_float(mantissa, a.sign ^ b.sign, a.exponent + b.exponent);
}

static inline IntyFloat operator * (IntyFloat const & a, int const & b){
    return a * make_inty_float(b);
}

static inline IntyFloat operator * (int const & a, IntyFloat const & b){
    return make_inty_float(a) * b;
}


static inline IntyFloat operator / (IntyFloat const & a, IntyFloat const & b) {
    // Shift a by 15 bits to keep all the precision of a.
    const uint32_t mantissa = (static_cast<uint32_t>(a.mantissa) << 15) / b.mantissa;
    
    // We need to compensate for the initial multiplication of a by 15 bits.
    return make_inty_float(mantissa, a.sign ^ b.sign, a.exponent - b.exponent - 15);
}

static inline IntyFloat operator / (IntyFloat const & a, int const & b){
    return a / make_inty_float(b);
}

static inline IntyFloat operator / (int const & a, IntyFloat const & b){
    return make_inty_float(a) / b;
}

static inline IntyFloat operator + (IntyFloat const & a, IntyFloat const & b) {
    // Assume IntyFloat is normalized with the most significant bit at position 15.
    // Make sure we do math with the correct sign.

    const int exponent_diff = a.exponent - b.exponent;

    const int a_value = ((exponent_diff >= 0) ? a.mantissa : (a.mantissa >> -exponent_diff)) * (a.sign ? -1 : 1);
    const int b_value = ((exponent_diff <= 0) ? b.mantissa : (b.mantissa >> exponent_diff)) * (b.sign ? -1 : 1);

    const int result = a_value + b_value;

    const uint8_t sign = (result < 0) ? 1 : 0;
    const uint32_t mantissa = (result < 0) ? -result : result;

    return make_inty_float(mantissa, sign, (exponent_diff >= 0) ? a.exponent : b.exponent);
}

static inline IntyFloat operator - (IntyFloat const & a, IntyFloat const & b) {
    // Subtraction is just addition with the sign flipped.
    return a + IntyFloat{b.mantissa, static_cast<uint8_t>(b.sign ^ 1), b.exponent};
}

static inline IntyFloat make_inty_float(float value) {
    // Convert a float to IntyFloat.
    if (value == 0.0f) return IntyFloat{0, 0, 0};

    // Determine the sign.
    const uint8_t sign = (value < 0.0f) ? 1 : 0;
    if (sign) value = -value;

    const float max_value = (1 << 15);
    const float min_value = (1 << 14);

    // Normalize the value to get the mantissa and exponent.
    int exponent = 0;
    while (value >= max_value) {
        value /= 2.0f;
        exponent += 1;
    }
    while (value < min_value) {
        value *= 2.0f;
        exponent -= 1;
    }

    return IntyFloat{static_cast<uint16_t>(value), sign, static_cast<int8_t>(exponent)};
}

static inline int16_t value_of_inty_float(IntyFloat const & value) {
    // Adjust for the exponent.
    const int abs_result = (value.exponent >= 0 ? value.mantissa << value.exponent : value.mantissa >> -value.exponent);

    // Adjust for the sign.
    return (value.sign ? -1 : 1) * abs_result;
}

void unit_test_integer_arithmetic(char * buffer, size_t max_size);