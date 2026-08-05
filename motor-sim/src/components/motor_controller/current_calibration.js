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
const max_inductance_bias_fraction = 0.2;
const min_inductance_noise = 0.000_000_1;
const min_resistance_noise = 0.000_1;
const min_degree_noise = 1.0;

const saturation_current_guess = 1.0;
const saturation_width = 0.5;
const saturation_halfwidth = 0.5 * saturation_width;

function compute_gradients(readout, current_calibration){
  const {
    u_current, v_current, w_current, 
    u_current_diff, v_current_diff, w_current_diff, 
    u_drive_voltage, v_drive_voltage, w_drive_voltage,
  } = readout;

  const {
    phase_u_resistance, 
    phase_v_resistance, 
    phase_w_resistance, 
    phase_inductance_base,
    phase_inductance_angle,
    phase_inductance_bias,
    saturation_current,
  } = current_calibration;

  const u_scaled_current_diff = u_current_diff * pwm_cycles_per_second;
  const v_scaled_current_diff = v_current_diff * pwm_cycles_per_second;
  const w_scaled_current_diff = w_current_diff * pwm_cycles_per_second;

  const u_resistive_voltage = u_current * phase_u_resistance;
  const v_resistive_voltage = v_current * phase_v_resistance;
  const w_resistive_voltage = w_current * phase_w_resistance;

  function saturation_factor(current) {
    const abs_current = Math.abs(current);
    if (abs_current < (1.0 - saturation_halfwidth) * saturation_current) {
      return 1.0;
    } else if (abs_current > (1.0 + saturation_halfwidth) * saturation_current) {
      return 0.0;
    } else {
      return 1.0 - (abs_current - (1.0 - saturation_halfwidth) * saturation_current) / (saturation_width * saturation_current);
    }
  }

  const u_inductance = phase_inductance_base * saturation_factor(u_current);
  const v_inductance = phase_inductance_base * saturation_factor(v_current);
  const w_inductance = phase_inductance_base * saturation_factor(w_current);

  const u_inductance_voltage = u_scaled_current_diff * u_inductance;
  const v_inductance_voltage = v_scaled_current_diff * v_inductance;
  const w_inductance_voltage = w_scaled_current_diff * w_inductance;

  const u_residual = u_resistive_voltage + u_inductance_voltage - u_drive_voltage;
  const v_residual = v_resistive_voltage + v_inductance_voltage - v_drive_voltage;
  const w_residual = w_resistive_voltage + w_inductance_voltage - w_drive_voltage;

  // We define the loss as the sum of the squares of the unexplained residual voltages. We assume
  // during the calibration that EMF is negligible, and therefore we calibrate the resistance against
  const loss = square(u_residual) + square(v_residual) + square(w_residual);

  const u_resistance_gradient = u_residual * u_current;
  const v_resistance_gradient = v_residual * v_current;
  const w_resistance_gradient = w_residual * w_current;

  const inductance_base_gradient = (
    u_residual * u_scaled_current_diff +
    v_residual * v_scaled_current_diff +
    w_residual * w_scaled_current_diff
  );

  // Gradient with respect to the saturation_factor
  function saturation_factor_gradient(current){
    const abs_current = Math.abs(current);
    if (abs_current < (1.0 - saturation_halfwidth) * saturation_current) {
      return -1.0;
    } else if (abs_current > (1.0 + saturation_halfwidth) * saturation_current) {
      return +1.0;
    } else {
      return (abs_current - (1.0 - saturation_halfwidth) * saturation_current) / (saturation_halfwidth * saturation_current);
    }
  }

  const saturation_current_gradient = (
    u_residual * u_scaled_current_diff * phase_inductance_base * saturation_factor_gradient(u_current) +
    v_residual * v_scaled_current_diff * phase_inductance_base * saturation_factor_gradient(v_current) +
    w_residual * w_scaled_current_diff * phase_inductance_base * saturation_factor_gradient(w_current)
  );

  const inductance_bias_gradient = 0.0 * -1.0 * (
    u_residual * u_scaled_current_diff * cos_degrees(phase_inductance_angle) +
    v_residual * v_scaled_current_diff * cos_degrees(phase_inductance_angle - 120) +
    w_residual * w_scaled_current_diff * cos_degrees(phase_inductance_angle + 120)
  );

  const inductance_angle_gradient = 0.0 * (
    u_residual * u_scaled_current_diff * phase_inductance_bias * sin_degrees(phase_inductance_angle) +
    v_residual * v_scaled_current_diff * phase_inductance_bias * sin_degrees(phase_inductance_angle - 120) +
    w_residual * w_scaled_current_diff * phase_inductance_bias * sin_degrees(phase_inductance_angle + 120)
  );
  
  const [residual_direct, residual_quadrature] = dq0_transform(u_residual, v_residual, w_residual, 0);
  const residual_angle = radians_to_degrees(Math.atan2(residual_quadrature, residual_direct));
  const residual_magnitude = Math.sqrt(residual_direct * residual_direct + residual_quadrature * residual_quadrature);
  
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
    inductance_base_gradient,
    inductance_bias_gradient,
    inductance_angle_gradient,
    saturation_current_gradient,
  };
}

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
    phase_u_resistance, 
    phase_v_resistance, 
    phase_w_resistance, 
    phase_inductance_base,
    phase_inductance_angle,
    phase_inductance_bias,
    saturation_current,
  } = current_calibration;

  saturation_current = saturation_current_guess;

  let is_stable = false;

  const max_iterations = 300;
  const stability_threshold = 0.000_001;
  let iterations = [];

  let u_resistance_rate = 0.01;
  let v_resistance_rate = 0.01;
  let w_resistance_rate = 0.01;
  let inductance_base_rate = 0.000_01;
  let inductance_bias_rate = 0.000_001;
  let inductance_angle_rate = 1.0;
  let saturation_current_rate = 0.01;

  let u_resistance_sign = 0.0;
  let v_resistance_sign = 0.0;
  let w_resistance_sign = 0.0;
  let inductance_base_sign = 0.0;
  let inductance_bias_sign = 0.0;
  let inductance_angle_sign = 0.0;
  let saturation_current_sign = 0.0;

  const rate_increase = 1.2;
  const rate_decrease = 0.5;

  for (let i = 0; !is_stable && (i < max_iterations); i++) {
    const gradients = sample.map((readout) => compute_gradients(readout, {
      phase_u_resistance, 
      phase_v_resistance, 
      phase_w_resistance, 
      phase_inductance_base,
      phase_inductance_angle,
      phase_inductance_bias,
      saturation_current,
    }));

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
    [inductance_base_rate, inductance_base_sign] = compute_rate(gradients, (d) => d.inductance_base_gradient, inductance_base_rate, inductance_base_sign);
    [inductance_bias_rate, inductance_bias_sign] = compute_rate(gradients, (d) => d.inductance_bias_gradient, inductance_bias_rate, inductance_bias_sign);
    [inductance_angle_rate, inductance_angle_sign] = compute_rate(gradients, (d) => d.inductance_angle_gradient, inductance_angle_rate, inductance_angle_sign);
    [saturation_current_rate, saturation_current_sign] = compute_rate(gradients, (d) => d.saturation_current_gradient, saturation_current_rate, saturation_current_sign);

    const u_resistance_step = u_resistance_rate * u_resistance_sign;
    const v_resistance_step = v_resistance_rate * v_resistance_sign;
    const w_resistance_step = w_resistance_rate * w_resistance_sign;

    const inductance_base_step = inductance_base_rate * inductance_base_sign;
    const inductance_bias_step = inductance_bias_rate * inductance_bias_sign;
    const inductance_angle_step = inductance_angle_rate * inductance_angle_sign;

    const saturation_current_step = saturation_current_rate * saturation_current_sign;

    phase_u_resistance -= u_resistance_step;
    phase_v_resistance -= v_resistance_step;
    phase_w_resistance -= w_resistance_step;

    phase_inductance_base = Math.max(min_inductance, phase_inductance_base - inductance_base_step);
    phase_inductance_bias = Math.min(
      max_inductance_bias_fraction * phase_inductance_base,
      Math.max(min_inductance, phase_inductance_bias - inductance_bias_step)
    );

    phase_inductance_angle = normalize_degrees(phase_inductance_angle - inductance_angle_step);

    saturation_current = Math.max(0.1, saturation_current - saturation_current_step);

    iterations.push({
      iteration: i,
      current_calibration: {
        phase_u_resistance, 
        phase_v_resistance, 
        phase_w_resistance, 
        phase_inductance_base,
        phase_inductance_angle,
        phase_inductance_bias,
        saturation_current,
      },
      gradients,
    });

    // Stop iterating if all changes are under the threshold.
    is_stable = (
      (Math.abs(u_resistance_step) < stability_threshold) &&
      (Math.abs(v_resistance_step) < stability_threshold) &&
      (Math.abs(w_resistance_step) < stability_threshold) &&
      (Math.abs(inductance_base_step) < stability_threshold) &&
      (Math.abs(inductance_bias_step) < stability_threshold) &&
      (Math.abs(inductance_angle_step) < stability_threshold) &&
      (Math.abs(saturation_current_step) < stability_threshold)
    );
  }

  const current_calibration_data = {
    sample,
    is_stable,
    iterations,
    current_calibration: {
      ...current_calibration,
      phase_u_resistance,
      phase_v_resistance,
      phase_w_resistance,
      phase_inductance_base,
      phase_inductance_angle,
      phase_inductance_bias,
      saturation_current,
    }
  };

  return current_calibration_data;
}