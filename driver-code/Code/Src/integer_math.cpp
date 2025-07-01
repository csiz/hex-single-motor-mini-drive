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

void unit_test_funky_atan(char * buffer, size_t max_size) {
    const int angle_1 = funky_atan2(100, 0); // 90 degrees
    const int angle_2 = funky_atan2(0, 100); // 0 degrees
    const int angle_3 = funky_atan2(-100, 0); // 270 degrees
    const int angle_4 = funky_atan2(0, -100); // 180 degrees
    const int angle_5 = funky_atan2(100, 100); // 45 degrees
    const int angle_6 = funky_atan2(-100, -100); // 225 degrees
    const int angle_7 = funky_atan2(100, -100); // 315 degrees
    const int angle_8 = funky_atan2(-100, 100); // 135 degrees

    snprintf(buffer, max_size,
            "funky_atan2(100, 0) = %d\n"
            "funky_atan2(0, 100) = %d\n"
            "funky_atan2(-100, 0) = %d\n"
            "funky_atan2(0, -100) = %d\n"
            "funky_atan2(100, 100) = %d\n"
            "funky_atan2(-100, -100) = %d\n"
            "funky_atan2(100, -100) = %d\n"
            "funky_atan2(-100, 100) = %d\n",
            angle_1,
            angle_2,
            angle_3,
            angle_4,
            angle_5,
            angle_6,
            angle_7,
            angle_8);
}

void unit_test_funky_atan_part_2(char * buffer, size_t max_size) {
    
    const int angle_1 = funky_atan2(10, 100);
    const int angle_2 = funky_atan2(20, 100);
    const int angle_3 = funky_atan2(30, 100);
    const int angle_4 = funky_atan2(40, 100);
    const int angle_5 = funky_atan2(50, 100);
    const int angle_6 = funky_atan2(60, 100);
    const int angle_7 = funky_atan2(70, 100);
    const int angle_8 = funky_atan2(80, 100);
    const int angle_9 = funky_atan2(90, 100);

    snprintf(buffer, max_size,
            "funky_atan2(10, 100) = %d\n"
            "funky_atan2(20, 100) = %d\n"
            "funky_atan2(30, 100) = %d\n"
            "funky_atan2(40, 100) = %d\n"
            "funky_atan2(50, 100) = %d\n"
            "funky_atan2(60, 100) = %d\n"
            "funky_atan2(70, 100) = %d\n"
            "funky_atan2(80, 100) = %d\n"
            "funky_atan2(90, 100) = %d\n",
            angle_1,
            angle_2,
            angle_3,
            angle_4,
            angle_5,
            angle_6,
            angle_7,
            angle_8,
            angle_9);
}

void unit_test_funky_atan_part_3(char * buffer, size_t max_size) {
    const int angle_1 = funky_atan2(-100, 1000);
    const int angle_2 = funky_atan2(-200, 1000);
    const int angle_3 = funky_atan2(-300, 1000);
    const int angle_4 = funky_atan2(-400, 1000);
    const int angle_5 = funky_atan2(-500, 1000);
    const int angle_6 = funky_atan2(-600, 1000);
    const int angle_7 = funky_atan2(-700, 1000);
    const int angle_8 = funky_atan2(-800, 1000);
    const int angle_9 = funky_atan2(-900, 1000);

    snprintf(buffer, max_size,
            "f_atan2(-100, 1000) = %d\n"
            "f_atan2(-200, 1000) = %d\n"
            "f_atan2(-300, 1000) = %d\n"
            "f_atan2(-400, 1000) = %d\n"
            "f_atan2(-500, 1000) = %d\n"
            "f_atan2(-600, 1000) = %d\n"
            "f_atan2(-700, 1000) = %d\n"
            "f_atan2(-800, 1000) = %d\n"
            "f_atan2(-900, 1000) = %d\n",
            angle_1,
            angle_2,
            angle_3,
            angle_4,
            angle_5,
            angle_6,
            angle_7,
            angle_8,
            angle_9);
}