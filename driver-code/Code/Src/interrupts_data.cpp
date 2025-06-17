#include "interrupts_data.hpp"

#include "user_data.hpp"
#include "constants.hpp"


PositionCalibration position_calibration = get_position_calibration();
CurrentCalibration current_calibration = get_current_calibration();
PIDParameters pid_parameters = get_pid_parameters();