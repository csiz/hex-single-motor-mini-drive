#include "interrupts_data.hpp"

#include "user_data.hpp"
#include "constants.hpp"


PositionCalibration position_calibration = get_position_calibration();

CurrentCalibration current_calibration = get_current_calibration();

ControlParameters control_parameters = get_control_parameters();