#include "interrupts_angle.hpp"

uint8_t hall_state = 0b000;
uint8_t hall_sector = 0;
uint8_t previous_hall_sector = 0;

bool angle_valid = false;
int time_since_observation = 0;
bool new_observation = false;

int angle_at_observation = 0;
int angle_variance_at_observation = 0;
int angular_speed_at_observation = 0;
int angular_speed_variance_at_observation = 0;

PositionCalibration position_calibration = default_position_calibration;