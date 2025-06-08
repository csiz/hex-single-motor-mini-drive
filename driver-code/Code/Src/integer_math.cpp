#include "integer_math.hpp"

#include <cmath>   // For M_PI, atan, round, sqrt (used only in init_tables)
#include <cstdio>

// Inlcude printf that prints to uint8_t buffer.


// Define M_PI if not available (e.g., on some compilers without _USE_MATH_DEFINES)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif



/**
 * @brief Initializes the CORDIC look-up tables and precomputed gain.
 * This function MUST be called once globally before using atan2_cordic.
 * It uses floating-point math for table generation.
 */
std::array<int16_t, ATAN2_CORDIC_ITERATIONS> init_atan2_cordic_lut() {
    std::array<int16_t, ATAN2_CORDIC_ITERATIONS> atan2_cordic_lut;

    for (int i = 0; i < ATAN2_CORDIC_ITERATIONS; ++i) {
        // The angle output is in units where 1024 represents a full circle (2*PI radians).
        // So, 1 angle unit = (2*PI)/1024 = PI/512 radians.
        // The LUT stores atan(2^-i) in these angle units.
        // lut_value = atan(2^-i) [radians] * (512 / PI) [units/radian]
        atan2_cordic_lut[i] = static_cast<int16_t>(round(atan(pow(2.0, -i)) * static_cast<float>(half_circle) / M_PI));
    }

    return atan2_cordic_lut;
}

int32_t init_atan2_cordic_inverse_gain_scaled() {
    double K_gain = 1.0;
    for (int i = 0; i < ATAN2_CORDIC_ITERATIONS; ++i) {
        K_gain *= sqrt(1.0 + pow(2.0, -2.0 * i));
    }
    return static_cast<int32_t>(round((1.0 / K_gain) * (1 << ATAN2_CORDIC_GAIN_SHIFT)));
}

const int32_t atan2_cordic_inverse_gain_scaled = init_atan2_cordic_inverse_gain_scaled();

const std::array<int16_t, ATAN2_CORDIC_ITERATIONS> atan2_cordic_lut = init_atan2_cordic_lut();


void unit_test_atan(char * buffer, size_t max_size) {
    const auto [angle_1, mag_1] = int_atan2(100, 0); // 90 degrees
    const auto [angle_2, mag_2] = int_atan2(0, 100); // 0 degrees
    const auto [angle_3, mag_3] = int_atan2(-100, 0); // 270 degrees
    const auto [angle_4, mag_4] = int_atan2(0, -100); // 180 degrees
    const auto [angle_5, mag_5] = int_atan2(100, 100); // 45 degrees
    const auto [angle_6, mag_6] = int_atan2(-100, -100); // 225 degrees
    const auto [angle_7, mag_7] = int_atan2(100, -100); // 315 degrees
    const auto [angle_8, mag_8] = int_atan2(-100, 100); // 135 degrees

    sniprintf(buffer, max_size,
            "atan2(100, 0) = (%d, %d)\n"
            "atan2(0, 100) = (%d, %d)\n"
            "atan2(-100, 0) = (%d, %d)\n"
            "atan2(0, -100) = (%d, %d)\n"
            "atan2(100, 100) = (%d, %d)\n"
            "atan2(-100, -100) = (%d, %d)\n"
            "atan2(100, -100) = (%d, %d)\n"
            "atan2(-100, 100) = (%d, %d)\n",
            angle_1, mag_1,
            angle_2, mag_2,
            angle_3, mag_3,
            angle_4, mag_4,
            angle_5, mag_5,
            angle_6, mag_6,
            angle_7, mag_7,
            angle_8, mag_8);
}