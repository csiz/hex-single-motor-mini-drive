#pragma once

#include "shared_data.hpp"
#include "constants.hpp"
#include "sine_tables.hpp"
#include <cmath>

// Calibration state
// -----------------

constexpr uint32_t calibration_stability_check_period = 64;
constexpr float multiplier_minimum = 0.000'1f;
constexpr float multiplier_maximum = 10.f;

// Current calibration result.
extern hex_mini_drive::CalibrationInstance calibration_instance;

static inline hex_mini_drive::CalibrationInstance init_calibration_instance(
  float target_pwm,
  float target_speed
) {
  return hex_mini_drive::CalibrationInstance {
    .count = 0,
    .target_pwm = target_pwm,
    .target_speed = target_speed,
    .stable = false,
    .resistance_excited = false,
    .inductance_excited = false,
    .hysteresis_excited = false,
    .avg_square_voltage = 0.f,
    .avg_square_current = 0.f,
    .avg_square_error = 0.f,
    .gradients_dot = 0.f,
    .learning_rate_multiplier = 1.f,
    .learning_rate_cap = multiplier_maximum,
    .resistance_excitation_fraction = 0.f,
    .inductance_excitation_fraction = 0.f,
    .hysteresis_excitation_fraction = 0.f,
  };
}

// Update calibration from readout and return whether the series should end.
static inline bool calibrate_from_readout(hex_mini_drive::Readout const& readout) {
  // Calibration checks
  // ------------------
  // 
  // For the remaining parameters we can only optimize them when there's no EMF voltage because
  // of the way we've set our equations. We detect the EMF voltage as basically the error term
  // of our electrical equations so if we optimize resistance and inductance while the motor spins
  // then we're going to slowly attribute all of the resistance and inductance voltage to the EMF.
  // 
  // It appears that it's really difficult to gauge when we can calibrate and when we cannot because
  // of EMF. Instead we will only calibrate when commanded and the driver is realatively certain the
  // motor is in a stopped state. We then run the calibration at various frequencies and check the
  // response assuming the motor is stationary and not generating any EMF voltage.

  // Stop calibration updates when we are undervoltage or overcurrent, we won't get accurate readings.
  const bool undervoltage = readout.vcc_voltage < control_parameters.vcc_undervoltage;
  const float square_current = square(readout.x_current) + square(readout.y_current);
  const bool overcurrent = square_current > max_drive_current_square;

  // Abort the calibration instance if we triggered the stop conditions.
  if (undervoltage or overcurrent) {
    if (calibration_instance.count) {
      // We already have stats, abort calibration instance.
      return true;
    } else {
      // We don't have stats, wait until the condition settles, hopefully.
      return false;
    }
  }

  const bool zero_speed = calibration_instance.target_speed == 0.0f;

  // From now on we are definitely calibrating.
  calibration_instance.count += 1;

  learning_rate_multiplier = (
    calibration_instance.learning_rate_multiplier * 
    (zero_speed ? 1.0f : (faster_abs(calibration_instance.target_speed) * calibration_parameters.nominal_speed_inverse))
  );

  // TODO: what to do about the checks, we can't edit the interrupt readout.
  // if(not std::isfinite(calibration_instance.resistance) or calibration_instance.resistance < minimum_resistance) {
  //   calibration_instance.resistance = minimum_resistance;
  // }
  // if(not std::isfinite(calibration_instance.inductance) or calibration_instance.inductance < minimum_inductance) {
  //   calibration_instance.inductance = minimum_inductance;
  // }
  // if(not std::isfinite(calibration_instance.inductance_saliency) or calibration_instance.inductance_saliency < minimum_inductance_saliency) {
  //   calibration_instance.inductance_saliency = minimum_inductance_saliency;
  // }
  // if(not std::isfinite(calibration_instance.hysteresis_loss) or calibration_instance.hysteresis_loss < minimum_hysteresis_loss) {
  //   calibration_instance.hysteresis_loss = minimum_hysteresis_loss;
  // }

  const float drive_voltage_square = square(readout.x_drive_voltage) + square(readout.y_drive_voltage);
  const float square_voltage_error = square(readout.x_voltage_error) + square(readout.y_voltage_error);

  calibration_instance.avg_square_voltage += (drive_voltage_square - calibration_instance.avg_square_voltage) * calibration_parameters.stats_ki;
  calibration_instance.avg_square_current += (square_current - calibration_instance.avg_square_current) * calibration_parameters.stats_ki;
  // The error is evolving during the optimization, we want to store the end-ish value after we've optimized our parameters.
  calibration_instance.avg_square_error += (square_voltage_error - calibration_instance.avg_square_error) * calibration_parameters.stats_ki;


  // Count how often we see enough current to meaningfully calibrate resistance (we still have to do it, but we mark it as uncertain).
  calibration_instance.resistance_excited = square_current > calibration_parameters.current_variance;
  calibration_instance.resistance_excitation_fraction += (calibration_instance.resistance_excited - calibration_instance.resistance_excitation_fraction) * calibration_parameters.stats_ki;

  const float square_inductance = (square(readout.x_current_diff) + square(readout.y_current_diff)) * square(current_diff_to_voltage_units);

  // Also track inductance excitation.
  calibration_instance.inductance_excited = square_inductance > calibration_parameters.inductance_variance;
  calibration_instance.inductance_excitation_fraction += (calibration_instance.inductance_excited - calibration_instance.inductance_excitation_fraction) * calibration_parameters.stats_ki;
  
  const float square_hysteresis = square_current * square_inductance * hysteresis_conversion_square;

  calibration_instance.hysteresis_excited = square_hysteresis > calibration_parameters.hysteresis_variance;
  calibration_instance.hysteresis_excitation_fraction += (calibration_instance.hysteresis_excited - calibration_instance.hysteresis_excitation_fraction) * calibration_parameters.stats_ki;

  // Keep track of total unexplained variance.
  const float gradients_dot = (
    square_current * square(readout.resistance_gradient) +
    square_inductance * (
      square(readout.inductance_gradient) +
      square(readout.inductance_saliency_gradient)
    ) +
    square_hysteresis * square(readout.hysteresis_loss_gradient)
  );

  calibration_instance.gradients_dot = ewma(calibration_instance.gradients_dot, gradients_dot, calibration_parameters.stats_ki);

  calibration_instance.resistance = readout.resistance;
  calibration_instance.inductance = readout.inductance;
  calibration_instance.inductance_saliency = readout.inductance_saliency;
  calibration_instance.hysteresis_loss = readout.hysteresis_loss;
  calibration_instance.saliency_angle = readout.saliency_angle;
  calibration_instance.hysteresis_angle = readout.hysteresis_angle;
  


  // Check for stability
  // -------------------


  // If our gradients are aligned over many steps than we should increase the learning rate;
  // otherwise if the gradients are negative or below the expected variance we should lower
  // the learning rate and see if we can optimize more.
  if(calibration_instance.gradients_dot > calibration_parameters.voltage_variance) {
    calibration_instance.learning_rate_multiplier *= 1.f + calibration_parameters.multiplier_change;
    if (calibration_instance.learning_rate_multiplier > calibration_instance.learning_rate_cap) {
      calibration_instance.learning_rate_multiplier = calibration_instance.learning_rate_cap;
      // Slowly decrease the cap to 1 so we don't get stuck oscillating at high error.
      calibration_instance.learning_rate_cap = ewma(calibration_instance.learning_rate_cap, 1.0f, calibration_parameters.multiplier_cap_decay);
    } else {
      calibration_instance.learning_rate_cap = ewma(calibration_instance.learning_rate_cap, multiplier_maximum, calibration_parameters.multiplier_cap_decay);
    }
  } else {
    calibration_instance.learning_rate_multiplier = max(calibration_instance.learning_rate_multiplier * (1.f - calibration_parameters.multiplier_change), multiplier_minimum);
    calibration_instance.learning_rate_cap = ewma(calibration_instance.learning_rate_cap, multiplier_maximum, calibration_parameters.multiplier_cap_decay);
  }

  
  // Periodically run checks then move on.
  if (calibration_instance.count % calibration_stability_check_period == 0){
    calibration_instance.stable = calibration_instance.learning_rate_multiplier < calibration_parameters.multiplier_stable_threshold;
    if (calibration_parameters.advance_when_stable and calibration_instance.stable) {
      return true;
    }
  }
  return false;
}
