#include "interrupts_angle.hpp"

#include "user_data.hpp"


uint8_t hall_state = 0b000;
uint8_t hall_sector = hall_sector_base;

bool angle_valid = false;

PositionStatistics electric_position = null_position_statistics;

PositionCalibration position_calibration = get_position_calibration();