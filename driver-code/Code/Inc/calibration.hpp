#pragma once

#include "shared_data.hpp"
#include "constants.hpp"
#include "sine_tables.hpp"


// Calibration state
// -----------------

// Calibration parameters in use.
extern hex_mini_drive::CalibrationParameters calibration_parameters;

// Current calibration result.
extern hex_mini_drive::CalibrationInstance calibration_instance;

constexpr uint32_t calibration_stability_check_period = 128;
constexpr float multiplier_minimum = 0.000'1f;
constexpr float multiplier_maximum = 10.f;


static inline hex_mini_drive::CalibrationInstance make_calibration_instance(hex_mini_drive::Calibration const& calibration){
  return hex_mini_drive::CalibrationInstance{
    .resistance = calibration.resistance,
    .inductance = calibration.inductance,
    .inductance_saliency = calibration.inductance_saliency,
    .hysteresis_loss = calibration.hysteresis_loss
  };
}


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
    .probing_factor = 0,
    .probing_factor_modulation = 0,
    .probing_speed_modulation = 0,
    .probing_offset = 0,
  });
}


// Keep zeroes to 0, check for nan and inifities and catch those with 0 too. Otherwise cap to minimum value.
// Return whether the parameter was valid (true if unchanged, false if changed).
static inline bool validate_parameter_update(float const& minimum_value, float & value, float const& value_change) {
  // Keep zeroes 0 and also zero out infinite/nan values.
  if (value == 0 or not std::isfinite(value)) {
    value = 0;
    return false;
  }
  
  const float new_value = value + value_change;

  if (new_value < minimum_value) {
    value = minimum_value;
    return false;
  } else {
    value = new_value;
    return true;
  }
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

  // Because our best source of voltage information is the drive_voltage we also need to make sure
  // we are driving with enough power to get any information.

  const float drive_voltage_square = square(readout.x_drive_voltage) + square(readout.y_drive_voltage);

  const bool low_drive = drive_voltage_square < calibration_parameters.voltage_variance;

  // The rest of the calibration can only be done if we're actively driving the motor phases as that
  // is our only source of reference scale for the phase voltages. In any case, idle driving isn't an
  // error so we continue the calibration, we just ignore this readout.
  if (low_drive) return false;

  const bool zero_speed = calibration_instance.target_speed == 0.0f;

  // From now on we are definitely calibrating.
  calibration_instance.count += 1;

  const float learning_rate_multiplier = (
    calibration_instance.learning_rate_multiplier * 
    (zero_speed ? 1.0f : (faster_abs(calibration_instance.target_speed) * calibration_parameters.nominal_speed_inverse))
  );

  // Recalculate
  // -----------
  // We need to recalculate these values in the main loop as they are not the primary tracking variables.

  // Calculate the resistive voltage drop across the coil and MOSFET resistance.
  const float x_resistive_voltage = readout.x_current * calibration_instance.resistance;
  const float y_resistive_voltage = readout.y_current * calibration_instance.resistance;

  // And the current diffs for calculating inductance.
  const float x_inductance = readout.x_current_diff * current_diff_to_voltage_units;
  const float y_inductance = readout.y_current_diff * current_diff_to_voltage_units;

  // Note: The saliency effect is due to the iron between magnets so it occurs twice per 
  // electrical revolution, once for each pole. Thus we must use the doubled rotor angle.
  const int32_t saliency_angle_thing = calibration_instance.saliency_angle - readout.current_diff_angle;
  const float sin_saliency = get_sin(saliency_angle_thing);
  const float cos_saliency = get_cos(saliency_angle_thing);

  const float inductance_magnitude = readout.current_diff_magnitude * current_diff_to_voltage_units;
  
  const float x_inductance_saliency = -inductance_magnitude * cos_saliency;
  const float y_inductance_saliency = -inductance_magnitude * sin_saliency;

  const float x_inductance_saliency_voltage = x_inductance_saliency * calibration_instance.inductance_saliency;
  const float y_inductance_saliency_voltage = y_inductance_saliency * calibration_instance.inductance_saliency;

  const float x_inductance_base_voltage = x_inductance * calibration_instance.inductance;
  const float y_inductance_base_voltage = y_inductance * calibration_instance.inductance;

  const float x_inductance_voltage = x_inductance_base_voltage + x_inductance_saliency_voltage;
  const float y_inductance_voltage = y_inductance_base_voltage + y_inductance_saliency_voltage;
  
  // Hysteresis is the cross product between current and inductance vectors, which simplifies to the sum of angles.
  const int32_t hysteresis_angle_thing = -calibration_instance.hysteresis_angle + readout.current_angle + readout.current_diff_angle;
  const float sin_hysteresis_loss = get_sin(hysteresis_angle_thing);
  const float cos_hysteresis_loss = get_cos(hysteresis_angle_thing);

  const float hysteresis_scaling = (
    readout.current_magnitude * 
    readout.current_diff_magnitude * 
    hysteresis_conversion
  );
  
  const float x_hysteresis_loss = cos_hysteresis_loss * hysteresis_scaling;
  const float y_hysteresis_loss = sin_hysteresis_loss * hysteresis_scaling;

  const float x_hysteresis_loss_voltage = calibration_instance.hysteresis_loss * x_hysteresis_loss;
  
  const float y_hysteresis_loss_voltage = calibration_instance.hysteresis_loss * y_hysteresis_loss;

  // Calibrate without EMF!!!
  // ------------------------
  // 
  // It's very important that we don't calibrate while the motor is rotating and generating EMF.
  // During sensorless running we infer the EMF as the error of the electrical equation, but
  // that means we can't use the inferred EMF to calibrate the electrical parameters accurately.
  // Especially because the resistance voltage and EMF voltage are covarying quantities.
  // Inductance and the saliency bias are also highly sensitive to any lag in the EMF estimation.

  const float x_voltage_error = (
    x_inductance_voltage + 
    x_resistive_voltage + 
    x_hysteresis_loss_voltage + 
    -readout.x_drive_voltage
  );
  const float y_voltage_error = (
    y_inductance_voltage + 
    y_resistive_voltage + 
    y_hysteresis_loss_voltage + 
    -readout.y_drive_voltage
  );

  // Defining the loss as the square(x_voltage_error) + square(y_voltage_error) we get the derivative gradients with 
  // respect to each of the parameters we're tracking. We only care about the sign so we should be careful to skip updates when 
  // there isn't enough energy in the error to warrant an update.
  const float square_voltage_error = square(x_voltage_error) + square(y_voltage_error);

  const float square_current = square(readout.current_magnitude);

  const float square_inductance = square(x_inductance) + square(y_inductance);

  const float square_hysteresis = square(hysteresis_scaling);

  // Store stats
  // -----------
  // 
  // The error is evolving during the optimization, we want to store the end-ish value after we've optimized our parameters.

  calibration_instance.avg_drive_voltage += (readout.drive_voltage_magnitude - calibration_instance.avg_drive_voltage) * calibration_parameters.stats_ki;
  calibration_instance.avg_current_magnitude += (readout.current_magnitude - calibration_instance.avg_current_magnitude) * calibration_parameters.stats_ki;
  calibration_instance.avg_current_speed += (readout.current_angular_speed - calibration_instance.avg_current_speed) * calibration_parameters.stats_ki;
  calibration_instance.avg_square_error += (square_voltage_error - calibration_instance.avg_square_error) * calibration_parameters.stats_ki;
  
  // Gradients & Parameter Updates
  // -----------------------------
  // 
  // Compute the gradients of the loss with respect to each of the tracked parameters.
  // 
  // We must not condition the updates when they are under a threshold, apparently it helps to update
  // all the time for a known driving condition.
  // 
  // However we can condition to update only resistance when driving statically.

  // Count how often we see enough current to meaningfully calibrate resistance (we still have to do it, but we mark it as uncertain).
  const bool resistance_excited = square_current >  calibration_parameters.current_variance;
  calibration_instance.resistance_excitation_fraction += (resistance_excited - calibration_instance.resistance_excitation_fraction) * calibration_parameters.stats_ki;

  // Also track inductance excitation.
  const bool inductance_excited = square_inductance >  calibration_parameters.inductance_variance;
  calibration_instance.inductance_excitation_fraction += (inductance_excited - calibration_instance.inductance_excitation_fraction) * calibration_parameters.stats_ki;
  
  const bool hysteresis_excited = square_hysteresis > calibration_parameters.hysteresis_variance;
  calibration_instance.hysteresis_excitation_fraction += (hysteresis_excited - calibration_instance.hysteresis_excitation_fraction) * calibration_parameters.stats_ki;

  // Keep track of total unexplained variance.
  float gradients_dot = 0.0f;

  calibration_instance.resistance_sensitivity = square_current + calibration_parameters.current_variance;
  const float resistance_scaling = 1.f / calibration_instance.resistance_sensitivity;

  if (resistance_excited) {
    const float resistance_gradient = ewma(
      calibration_instance.resistance_gradient,
      resistance_scaling * (
        x_voltage_error * readout.x_current +
        y_voltage_error * readout.y_current
      ),
      calibration_parameters.stats_ki
    );

    validate_parameter_update(
      minimum_resistance, 
      calibration_instance.resistance,
      -resistance_gradient * calibration_parameters.resistance_ki * learning_rate_multiplier
    );
    gradients_dot += (
      resistance_gradient * calibration_instance.resistance_gradient
    ) * square_current;

    // Can now update the stored resistance gradient.
    calibration_instance.resistance_gradient = resistance_gradient;
  }

  if (inductance_excited) {

    calibration_instance.inductance_sensitivity = square_inductance + calibration_parameters.inductance_variance;
    const float inductance_scaling = 1.f / calibration_instance.inductance_sensitivity;
    
    const float inductance_gradient = ewma(
      calibration_instance.inductance_gradient,
      inductance_scaling * (
        x_voltage_error * x_inductance +
        y_voltage_error * y_inductance
      ),
      calibration_parameters.stats_ki
    );
    
    // Keep 0 inductance exactly 0 (meaning disabled), but otherwise make sure it's minimum positive value.
    validate_parameter_update(
      minimum_inductance, 
      calibration_instance.inductance, 
      -inductance_gradient * calibration_parameters.inductance_ki * learning_rate_multiplier
    );

    const float inductance_saliency_gradient = ewma(
      calibration_instance.inductance_saliency_gradient,
      inductance_scaling * (
        x_voltage_error * x_inductance_saliency +
        y_voltage_error * y_inductance_saliency
      ),
      calibration_parameters.stats_ki
    );


    // It was valid if above minimum, otherwise it got capped before going negative. This value is mediated by a directional
    // angle that we are also optimizing, it will go negative if we are stuck at 180 degree flip. In that case we can escape
    // the false equilibrium quickly by flipping the angle (and resetting the resilience multiplier).
    validate_parameter_update(
      minimum_inductance_saliency,
      calibration_instance.inductance_saliency, 
      -inductance_saliency_gradient * calibration_parameters.inductance_saliency_ki * learning_rate_multiplier
    );

    // If we're normal condition, then just gradient descent the angle too.
    const float saliency_angle_gradient = -(
      (x_voltage_error - x_inductance_saliency_voltage) * -sin_saliency +
      (y_voltage_error - y_inductance_saliency_voltage) * cos_saliency
    ) * voltage_error_to_angle_units_scaling;

    calibration_instance.saliency_angle -= static_cast<int32_t>(saliency_angle_gradient * calibration_parameters.saliency_angle_ki);
    
    // Note that if inductance_saliency is greater than inductance, total inductance can go negative, whatever that means.
    // This usually happens when the motor is being super noisy and has to be held down, so it's clearly an error from emf,
    // but I do not know how to combine the calibration terms to extract a meaningful value from them in that case.

    // Compute the dot product between the new gradients and the previous gradients for inductance contributions.
    gradients_dot += (
      inductance_gradient * calibration_instance.inductance_gradient +
      inductance_saliency_gradient * calibration_instance.inductance_saliency_gradient * (calibration_instance.inductance_saliency != 0.0f)
    ) * square_inductance;

    // Update inductance related gradients.
    calibration_instance.inductance_gradient = inductance_gradient;
    calibration_instance.inductance_saliency_gradient = inductance_saliency_gradient;
  }

  if (hysteresis_excited) {

    calibration_instance.hysteresis_loss_sensitivity = square_hysteresis +  calibration_parameters.hysteresis_variance;
    
    const float hysteresis_loss_scaling = 1.f / calibration_instance.hysteresis_loss_sensitivity;

    // And gradients for micro rotor vibrations.
    const float hysteresis_loss_gradient = ewma(
      calibration_instance.hysteresis_loss_gradient,
      hysteresis_loss_scaling * (
        x_voltage_error * x_hysteresis_loss + 
        y_voltage_error * y_hysteresis_loss
      ),
      calibration_parameters.stats_ki
    );

    validate_parameter_update(
      minimum_hysteresis_loss,
      calibration_instance.hysteresis_loss,
      -hysteresis_loss_gradient * calibration_parameters.hysteresis_loss_ki * learning_rate_multiplier
    );

     // An extra negative sign because it enters our equation as a negative term.
    const float hysteresis_angle_gradient = - (
      (x_voltage_error - x_hysteresis_loss_voltage) * -sin_hysteresis_loss + 
      (y_voltage_error - y_hysteresis_loss_voltage) * cos_hysteresis_loss
    ) * voltage_error_to_angle_units_scaling;

    calibration_instance.hysteresis_angle -= static_cast<int32_t>(hysteresis_angle_gradient * calibration_parameters.hysteresis_angle_ki);

    // Aggregate checks
    // ----------------

    // Compute the dot product for hysteresis loss.
    gradients_dot += (
      hysteresis_loss_gradient * calibration_instance.hysteresis_loss_gradient * (calibration_instance.hysteresis_loss != 0.0f)
    ) * square_hysteresis;

    calibration_instance.hysteresis_loss_gradient = hysteresis_loss_gradient;
  }

  calibration_instance.gradients_dot = ewma(calibration_instance.gradients_dot, gradients_dot, calibration_parameters.stats_ki);

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
    calibration_instance.resistance_excited = calibration_instance.resistance_excitation_fraction > calibration_parameters.excitation_threshold;
    calibration_instance.inductance_excited = calibration_instance.inductance_excitation_fraction > calibration_parameters.excitation_threshold;
    calibration_instance.hysteresis_excited = calibration_instance.hysteresis_excitation_fraction > calibration_parameters.excitation_threshold;
    if (calibration_parameters.advance_when_stable and calibration_instance.stable) {
      return true;
    }
  }
  return false;
}