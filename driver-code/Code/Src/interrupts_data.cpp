#include "interrupts_data.hpp"

#include "parameters_store.hpp"
#include "constants.hpp"


PositionCalibration position_calibration = get_position_calibration();

CurrentCalibration current_calibration = get_current_calibration();

ControlParameters control_parameters = get_control_parameters();