#include "interrupts_angle.hpp"

#include "user_data.hpp"


uint8_t hall_state = 0b000;
uint8_t hall_sector = hall_sector_base;
uint8_t previous_hall_sector = hall_sector_base;

bool angle_valid = false;

PositionStatistics electric_position = {
    .angle = 0,
    .angle_variance = 0,
    .angular_speed = 0,
    .angular_speed_variance = 0
};

int updates_since_last_hall_transition = 1;

PositionCalibration position_calibration = get_position_calibration();