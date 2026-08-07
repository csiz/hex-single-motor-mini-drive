import {pwm_cycles_per_second, millis_per_cycle} from "./constants.js";
import {normalize_degrees, cos_degrees, sin_degrees, radians_to_degrees} from "./angular_math.js";
import {HISTORY_SIZE} from "hex-mini-drive-interface";
import {MessageCode} from "./motor_controller.js";
import {wait} from "./async_utils.js";
import {square, invalid_to_zero, dq0_transform} from "./math_utils.js";
import {zip_records} from "./data_utils.js";
import {product_of_normals} from "./stats_utils.js";

import * as d3 from "d3";

const min_inductance = 0.000_001;
const min_inductance_noise = 0.000_000_1;
const min_resistance_noise = 0.000_1;
const min_degree_noise = 1.0;

export async function run_current_calibration(motor_controller, message_code, max_pwm_value){
  if (!motor_controller.current_calibration) {
    console.error("We didn't load the active calibration data from the driver, we can't proceed without it.");
    throw new Error("Missing current calibration data");
  }

  const current_calibration = {...motor_controller.current_calibration};

  console.info("Current calibration starting");

  // Run a calibration instance.
  const sample = await motor_controller.send_command_and_await_reply({
    message: {
      message_code,
      pwm_value: max_pwm_value, 
      take_snapshot: 1,
    },
    expected_messages: HISTORY_SIZE,
    expected_code: MessageCode.READOUT,
  });
  
  // Check all calibration data is complete.
  if (sample.length !== HISTORY_SIZE) {
    console.error("U positive calibration data incomplete", sample);
    return;
  }

  let {
    u_resistance, 
    v_resistance, 
    w_resistance, 
    inductance,
    inductance_power_angle,
    inductance_power_factor,
  } = current_calibration;

  let is_stable = false;

  const max_iterations = 300;
  const stability_threshold = 0.000_001;
  let iterations = [];

  let u_resistance_rate = 0.01;
  let v_resistance_rate = 0.01;
  let w_resistance_rate = 0.01;
  let inductance_rate = 0.000_01;
  let inductance_power_factor_rate = 0.000_001;
  let inductance_power_angle_rate = 1.0;

  let u_resistance_sign = 0.0;
  let v_resistance_sign = 0.0;
  let w_resistance_sign = 0.0;
  let inductance_sign = 0.0;
  let inductance_power_factor_sign = 0.0;
  let inductance_power_angle_sign = 0.0;

  const rate_increase = 1.2;
  const rate_decrease = 0.5;

  for (let i = 0; !is_stable && (i < max_iterations); i++) {

    const gradients = sample.map((readout) => {
      const {
        u_current, v_current, w_current, 
        u_current_diff, v_current_diff, w_current_diff, 
        u_drive_voltage, v_drive_voltage, w_drive_voltage,
      } = readout;

      const [current_direct, current_quadrature] = dq0_transform(u_current, v_current, w_current, 0);
      const current_angle = radians_to_degrees(Math.atan2(current_quadrature, current_direct));
      const current_magnitude = Math.sqrt(square(current_direct) + square(current_quadrature));

      const u_scaled_current_diff = u_current_diff * pwm_cycles_per_second;
      const v_scaled_current_diff = v_current_diff * pwm_cycles_per_second;
      const w_scaled_current_diff = w_current_diff * pwm_cycles_per_second;

      const u_resistive_voltage = u_current * u_resistance;
      const v_resistive_voltage = v_current * v_resistance;
      const w_resistive_voltage = w_current * w_resistance;

      const u_inductance = inductance * 1.0;
      const v_inductance = inductance * 1.0;
      const w_inductance = inductance * 1.0;

      const u_inductance_voltage = u_scaled_current_diff * u_inductance;
      const v_inductance_voltage = v_scaled_current_diff * v_inductance;
      const w_inductance_voltage = w_scaled_current_diff * w_inductance;

      const u_inductance_power = u_inductance_voltage * u_current;
      const v_inductance_power = v_inductance_voltage * v_current;
      const w_inductance_power = w_inductance_voltage * w_current;

      const [inductance_voltage_direct, inductance_voltage_quadrature] = dq0_transform(u_inductance_voltage, v_inductance_voltage, w_inductance_voltage, 0);
      const inductance_voltage_angle = radians_to_degrees(Math.atan2(inductance_voltage_quadrature, inductance_voltage_direct));
      const inductance_voltage_magnitude = Math.sqrt(square(inductance_voltage_direct) + square(inductance_voltage_quadrature));

      const [inductance_power_direct, inductance_power_quadrature] = dq0_transform(u_inductance_power, v_inductance_power, w_inductance_power, 0);
      const inductance_power_magnitude = Math.sqrt(square(inductance_power_direct) + square(inductance_power_quadrature));

      const u_wtf = inductance_power_factor * inductance_power_magnitude * cos_degrees(2*current_angle - inductance_power_angle);
      const v_wtf = inductance_power_factor * inductance_power_magnitude * cos_degrees(2*current_angle - 120 - inductance_power_angle);
      const w_wtf = inductance_power_factor * inductance_power_magnitude * cos_degrees(2*current_angle + 120 - inductance_power_angle);

      const u_residual = u_resistive_voltage + u_inductance_voltage - u_drive_voltage + u_wtf;
      const v_residual = v_resistive_voltage + v_inductance_voltage - v_drive_voltage + v_wtf;
      const w_residual = w_resistive_voltage + w_inductance_voltage - w_drive_voltage + w_wtf;

      // We define the loss as the sum of the squares of the unexplained residual voltages. We assume
      // during the calibration that EMF is negligible, and therefore we calibrate the resistance against
      const loss = square(u_residual) + square(v_residual) + square(w_residual);

      const u_resistance_gradient = u_residual * u_current;
      const v_resistance_gradient = v_residual * v_current;
      const w_resistance_gradient = w_residual * w_current;

      const inductance_gradient = (
        u_residual * u_scaled_current_diff +
        v_residual * v_scaled_current_diff +
        w_residual * w_scaled_current_diff
      );


      const inductance_power_factor_gradient = (
        u_residual * inductance_power_magnitude * cos_degrees(2*current_angle - inductance_power_angle) +
        v_residual * inductance_power_magnitude * cos_degrees(2*current_angle - 120 - inductance_power_angle) +
        w_residual * inductance_power_magnitude * cos_degrees(2*current_angle + 120 - inductance_power_angle)
      );

      const inductance_power_angle_gradient = (
        u_residual * inductance_power_factor * inductance_power_magnitude * sin_degrees(2*current_angle - inductance_power_angle) +
        v_residual * inductance_power_factor * inductance_power_magnitude * sin_degrees(2*current_angle - 120 - inductance_power_angle) +
        w_residual * inductance_power_factor * inductance_power_magnitude * sin_degrees(2*current_angle + 120 - inductance_power_angle)
      );

      const [residual_direct, residual_quadrature] = dq0_transform(u_residual, v_residual, w_residual, 0);
      const residual_angle = radians_to_degrees(Math.atan2(residual_quadrature, residual_direct));
      const residual_magnitude = Math.sqrt(square(residual_direct) + square(residual_quadrature));
      
      return {
        ...readout,
        loss,

        u_resistive_voltage,
        v_resistive_voltage,
        w_resistive_voltage,

        u_inductance_voltage,
        v_inductance_voltage,
        w_inductance_voltage,
        
        u_residual,
        v_residual,
        w_residual,

        residual_direct,
        residual_quadrature,
        residual_angle,
        residual_magnitude,

        u_resistance_gradient,
        v_resistance_gradient,
        w_resistance_gradient,
        inductance_gradient,
        inductance_power_factor_gradient,
        inductance_power_angle_gradient,
      };
    });

    const sqrt_loss = Math.sqrt(d3.mean(gradients, (d) => d.loss));

    function compute_rate(records, accessor, rate, previous_sign) {
      const sign = Math.sign(d3.mean(records, accessor));
      if (sign === previous_sign) {
        return [rate * rate_increase, sign];
      } else {
        return [rate * rate_decrease, sign];
      }
    }

    // Resilient Backpropagation
    // -------------------------
    // 
    // Update steps and learning rates using the sign of the gradient to the unexplained residual loss.

    [u_resistance_rate, u_resistance_sign] = compute_rate(gradients, (d) => d.u_resistance_gradient, u_resistance_rate, u_resistance_sign);
    [v_resistance_rate, v_resistance_sign] = compute_rate(gradients, (d) => d.v_resistance_gradient, v_resistance_rate, v_resistance_sign);
    [w_resistance_rate, w_resistance_sign] = compute_rate(gradients, (d) => d.w_resistance_gradient, w_resistance_rate, w_resistance_sign);
    [inductance_rate, inductance_sign] = compute_rate(gradients, (d) => d.inductance_gradient, inductance_rate, inductance_sign);
    [inductance_power_factor_rate, inductance_power_factor_sign] = compute_rate(gradients, (d) => d.inductance_power_factor_gradient, inductance_power_factor_rate, inductance_power_factor_sign);
    [inductance_power_angle_rate, inductance_power_angle_sign] = compute_rate(gradients, (d) => d.inductance_power_angle_gradient, inductance_power_angle_rate, inductance_power_angle_sign);

    const u_resistance_step = u_resistance_rate * u_resistance_sign;
    const v_resistance_step = v_resistance_rate * v_resistance_sign;
    const w_resistance_step = w_resistance_rate * w_resistance_sign;

    const inductance_step = inductance_rate * inductance_sign;
    const inductance_power_factor_step = inductance_power_factor_rate * inductance_power_factor_sign;
    const inductance_power_angle_step = inductance_power_angle_rate * inductance_power_angle_sign;

    
    iterations.push({
      iteration: i,
      current_calibration: {
        u_resistance, 
        v_resistance, 
        w_resistance, 
        inductance,
        inductance_power_angle,
        inductance_power_factor,
        sqrt_loss
      },
      gradients,
    });
    
    // Update calibration values after pushing the iteration data! The iteration should then
    // contain the calibration values that were used to calculate the gradients and other values.
    u_resistance -= u_resistance_step;
    v_resistance -= v_resistance_step;
    w_resistance -= w_resistance_step;

    inductance = Math.max(min_inductance, inductance - inductance_step);
    inductance_power_factor = Math.max(0.000_000_001, inductance_power_factor - inductance_power_factor_step);

    inductance_power_angle = normalize_degrees(inductance_power_angle - inductance_power_angle_step);


    // Stop iterating if all changes are under the threshold.
    is_stable = (
      (Math.abs(u_resistance_step) < stability_threshold) &&
      (Math.abs(v_resistance_step) < stability_threshold) &&
      (Math.abs(w_resistance_step) < stability_threshold) &&
      (Math.abs(inductance_step) < stability_threshold) &&
      (Math.abs(inductance_power_factor_step) < stability_threshold) &&
      (Math.abs(inductance_power_angle_step) < stability_threshold)
    );
  }

  const current_calibration_data = {
    sample,
    is_stable,
    iterations,
    current_calibration: {
      u_resistance,
      v_resistance,
      w_resistance,
      inductance,
      inductance_power_angle,
      inductance_power_factor,
    }
  };

  return current_calibration_data;
}