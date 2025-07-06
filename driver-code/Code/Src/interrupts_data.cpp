#include "interrupts_data.hpp"

#include "user_data.hpp"
#include "constants.hpp"


CurrentCalibration current_calibration = get_current_calibration();
PIDParameters pid_parameters = get_pid_parameters();
ObserverParameters observer_parameters = get_observer_parameters();