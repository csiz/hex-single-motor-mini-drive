#include "integer_math.hpp"

#include <cstdio>


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