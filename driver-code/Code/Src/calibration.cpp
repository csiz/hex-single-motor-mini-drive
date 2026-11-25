#include "calibration.hpp"

#include "parameters_store.hpp"

hex_mini_drive::CalibrationParameters calibration_parameters = get_calibration_parameters();

hex_mini_drive::CalibrationInstance calibration_instance = make_calibration_instance(calibration);
