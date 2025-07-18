#include "interrupts_data.hpp"

#include "user_data.hpp"
#include "constants.hpp"


CurrentCalibration current_calibration = get_current_calibration();
PIDParameters pid_parameters = get_pid_parameters();
ControlParameters control_parameters = get_control_parameters();