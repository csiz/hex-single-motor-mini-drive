#pragma once

#include "shared_data.hpp"
#include "constants.hpp"
#include "sine_tables.hpp"
#include <cmath>


// Calibration state
// -----------------


constexpr uint32_t calibration_stability_check_period = 128;
constexpr float multiplier_minimum = 0.000'1f;
constexpr float multiplier_maximum = 10.f;




static inline hex_mini_drive::CalibrationInstance init_calibration_instance(
  hex_mini_drive::CalibrationInstance const& active_calibration,
  float target_pwm,
  float target_speed,
  int32_t holding_angle
) {
  return hex_mini_drive::CalibrationInstance {
    .count = 0,
    .target_pwm = target_pwm,
    .target_speed = target_speed,
    .holding_angle = holding_angle,
    .stable = false,
    .resistance_excited = false,
    .inductance_excited = false,
    .hysteresis_excited = false,
    .avg_drive_voltage = 0.f,
    .avg_current_magnitude = 0.f,
    .avg_current_speed = 0.f,
    .avg_square_error = 0.f,
    .gradients_dot = 0.f,
    .learning_rate_multiplier = 1.f,
    .learning_rate_cap = multiplier_maximum,
    .resistance_excitation_fraction = 0.f,
    .inductance_excitation_fraction = 0.f,
    .hysteresis_excitation_fraction = 0.f,
    .resistance_sensitivity = calibration_parameters.current_variance,
    .inductance_sensitivity = calibration_parameters.inductance_variance,
    .hysteresis_loss_sensitivity = calibration_parameters.hysteresis_variance,
    .resistance = active_calibration.resistance,
    .resistance_gradient = 0.f,
    .inductance = active_calibration.inductance,
    .inductance_gradient = 0.f,
    .inductance_saliency = active_calibration.inductance_saliency,
    .inductance_saliency_gradient = 0.f,
    .saliency_angle = active_calibration.saliency_angle,
    .hysteresis_loss = active_calibration.hysteresis_loss,
    .hysteresis_loss_gradient = 0.f,
    .hysteresis_angle = active_calibration.hysteresis_angle,
  };
}


static inline void start_calibration_series() {
  // Clear the schedule of any pending states.
  if (drive_state_queue.read_available()) drive_state_queue.mark_all_read();

  drive_state_queue.write(hex_mini_drive::DriveState{
    .motor_flags = ENABLE_ALL_OUTPUTS,
    .closed_loop = false,
    .duration = calibration_parameters.instance_duration,
    .target_value = calibration_instance.target_pwm,
    .angle_value = calibration_instance.holding_angle,
    .target_speed = calibration_instance.target_speed,
  });
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

  // Check when we naturally finish the current calibration series by seeing when the drive state has finished.
  if (not drive_state_queue.read_available()) {
    // Check if we've actually started.
    if (calibration_instance.count == 0) {
      // Hmm, we haven't started this series yet; restart it.
      start_calibration_series();
      // Skip this readout since we just queued a new state.
    } else {
      // We started so now that drive_state is done, we are naturally ending, without reaching stability.
      return true;
    }
    // In either case we skip this readout.
    return false;
  }

  // Stop calibration updates when we are undervoltage or overcurrent, we won't get accurate readings.
  const bool undervoltage = readout.vcc_voltage < control_parameters.vcc_undervoltage;
  const bool overcurrent = readout.current_magnitude > max_drive_current;

  // Abort the calibration instance if we triggered the stop conditions.
  if (undervoltage or overcurrent) {
    // We already have stats, abort calibration instance.
    if (calibration_instance.count) {
      drive_state_queue.mark_all_read();
      return true;
    }
    // In either case we skip this readout from calibration.
    return false;
  }

  const bool zero_speed = calibration_instance.target_speed == 0.0f;

  // From now on we are definitely calibrating.
  calibration_instance.count += 1;

  learning_rate_multiplier = (
    calibration_instance.learning_rate_multiplier * 
    (zero_speed ? 1.0f : (faster_abs(calibration_instance.target_speed) * calibration_parameters.nominal_speed_inverse))
  );

  if(not std::isfinite(calibration_instance.resistance) or calibration_instance.resistance < minimum_resistance) {
    calibration_instance.resistance = minimum_resistance;
  }
  if(not std::isfinite(calibration_instance.inductance) or calibration_instance.inductance < minimum_inductance) {
    calibration_instance.inductance = minimum_inductance;
  }
  if(not std::isfinite(calibration_instance.inductance_saliency) or calibration_instance.inductance_saliency < minimum_inductance_saliency) {
    calibration_instance.inductance_saliency = minimum_inductance_saliency;
  }
  if(not std::isfinite(calibration_instance.hysteresis_loss) or calibration_instance.hysteresis_loss < minimum_hysteresis_loss) {
    calibration_instance.hysteresis_loss = minimum_hysteresis_loss;
  }

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