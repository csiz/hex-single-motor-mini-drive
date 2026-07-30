import {pwm_cycles_per_second, millis_per_cycle} from "./constants.js";
import {normalize_degrees, cos_degrees, sin_degrees, radians_to_degrees} from "./angular_math.js";
import {HISTORY_SIZE} from "hex-mini-drive-interface";
import {MessageCode} from "./motor_controller.js";
import {wait} from "./async_utils.js";
import {square, invalid_to_zero} from "./math_utils.js";
import {zip_records} from "./data_utils.js";
import {product_of_normals} from "./stats_utils.js";

import * as d3 from "d3";
import _ from "lodash";

const current_noise = square(0.100);
const current_diff_noise = square(0.100 * pwm_cycles_per_second);
const min_inductance = 0.000_001;
const current_angle_noise = square(0.100 * pwm_cycles_per_second * min_inductance);


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
    phase_inductance_baseline,
    phase_inductance_angle,
    phase_inductance_offset,
  } = current_calibration;

  const u_scaled_current_diff = u_current_diff * pwm_cycles_per_second;
  const v_scaled_current_diff = v_current_diff * pwm_cycles_per_second;
  const w_scaled_current_diff = w_current_diff * pwm_cycles_per_second;

  const u_resistive_voltage = u_current * phase_u_resistance;
  const v_resistive_voltage = v_current * phase_v_resistance;
  const w_resistive_voltage = w_current * phase_w_resistance;

  const u_inductance = phase_inductance_baseline - phase_inductance_offset * cos_degrees(phase_inductance_angle);
  const v_inductance = phase_inductance_baseline - phase_inductance_offset * cos_degrees(phase_inductance_angle - 120);
  const w_inductance = phase_inductance_baseline - phase_inductance_offset * cos_degrees(phase_inductance_angle + 120);

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

  const inductance_baseline_gradient = (
    u_residual * u_scaled_current_diff +
    v_residual * v_scaled_current_diff +
    w_residual * w_scaled_current_diff
  );

  const inductance_offset_gradient = -1.0 * (
    u_residual * u_scaled_current_diff * cos_degrees(phase_inductance_angle) +
    v_residual * v_scaled_current_diff * cos_degrees(phase_inductance_angle - 120) +
    w_residual * w_scaled_current_diff * cos_degrees(phase_inductance_angle + 120)
  );

  const inductance_angle_gradient = (
    u_residual * u_scaled_current_diff * phase_inductance_offset * sin_degrees(phase_inductance_angle) +
    v_residual * v_scaled_current_diff * phase_inductance_offset * sin_degrees(phase_inductance_angle - 120) +
    w_residual * w_scaled_current_diff * phase_inductance_offset * sin_degrees(phase_inductance_angle + 120)
  );

  const u_resistance_gradient_2nd_order = square(u_current);
  const v_resistance_gradient_2nd_order = square(v_current);
  const w_resistance_gradient_2nd_order = square(w_current);

  const inductance_baseline_gradient_2nd_order = square(
    u_scaled_current_diff + 
    v_scaled_current_diff + 
    w_scaled_current_diff
  );

  const inductance_offset_gradient_2nd_order = square(
    u_scaled_current_diff * cos_degrees(phase_inductance_angle) +
    v_scaled_current_diff * cos_degrees(phase_inductance_angle - 120) +
    w_scaled_current_diff * cos_degrees(phase_inductance_angle + 120)
  );

  const inductance_angle_gradient_2nd_order = square(
    u_scaled_current_diff * phase_inductance_offset * sin_degrees(phase_inductance_angle) +
    v_scaled_current_diff * phase_inductance_offset * sin_degrees(phase_inductance_angle - 120) +
    w_scaled_current_diff * phase_inductance_offset * sin_degrees(phase_inductance_angle + 120)
  );

  
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

    u_resistance_gradient,
    v_resistance_gradient,
    w_resistance_gradient,
    inductance_baseline_gradient,
    inductance_offset_gradient,
    inductance_angle_gradient,
    
    u_resistance_gradient_2nd_order,
    v_resistance_gradient_2nd_order,
    w_resistance_gradient_2nd_order,
    inductance_baseline_gradient_2nd_order,
    inductance_offset_gradient_2nd_order,
    inductance_angle_gradient_2nd_order,
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
    phase_inductance_baseline,
    phase_inductance_angle,
    phase_inductance_offset,
  } = current_calibration;

  let is_stable = false;

  const learning_rate = 0.2;
  const max_iterations = 1000;
  const stability_threshold = 0.000_01;
  let iterations = [];
  for (let i = 0; !is_stable && (i < max_iterations); i++) {
    const gradients = sample.map((readout) => compute_gradients(readout, {
      phase_u_resistance, 
      phase_v_resistance, 
      phase_w_resistance, 
      phase_inductance_baseline,
      phase_inductance_angle,
      phase_inductance_offset,
    }));

    const u_resistance_step = (
      d3.mean(gradients, (d) => d.u_resistance_gradient) / 
      (d3.mean(gradients, (d) => d.u_resistance_gradient_2nd_order) + current_noise)
    );
    const v_resistance_step = (
      d3.mean(gradients, (d) => d.v_resistance_gradient) / 
      (d3.mean(gradients, (d) => d.v_resistance_gradient_2nd_order) + current_noise)
    );
    const w_resistance_step = (
      d3.mean(gradients, (d) => d.w_resistance_gradient) / 
      (d3.mean(gradients, (d) => d.w_resistance_gradient_2nd_order) + current_noise)
    );

    const inductance_baseline_step = (
      d3.mean(gradients, (d) => d.inductance_baseline_gradient) / 
      (d3.mean(gradients, (d) => d.inductance_baseline_gradient_2nd_order) + current_diff_noise)
    );
    const inductance_offset_step = (
      d3.mean(gradients, (d) => d.inductance_offset_gradient) / 
      (d3.mean(gradients, (d) => d.inductance_offset_gradient_2nd_order) + current_diff_noise)
    );
    const inductance_angle_step = (
      d3.mean(gradients, (d) => d.inductance_angle_gradient) / 
      (d3.mean(gradients, (d) => d.inductance_angle_gradient_2nd_order) + current_angle_noise)
    );

    const u_resistance_change = learning_rate * u_resistance_step;
    const v_resistance_change = learning_rate * v_resistance_step;
    const w_resistance_change = learning_rate * w_resistance_step;
    const inductance_baseline_change = learning_rate * inductance_baseline_step;
    const inductance_offset_change = learning_rate * inductance_offset_step;
    const inductance_angle_change = learning_rate * inductance_angle_step;

    phase_u_resistance -= u_resistance_change;
    phase_v_resistance -= v_resistance_change;
    phase_w_resistance -= w_resistance_change;
    phase_inductance_baseline = Math.max(min_inductance, phase_inductance_baseline - inductance_baseline_change);
    phase_inductance_offset = Math.max(min_inductance, phase_inductance_offset - inductance_offset_change);
    phase_inductance_angle = normalize_degrees(phase_inductance_angle - inductance_angle_change);

    iterations.push({
      iteration: i,
      current_calibration: {
        ...current_calibration,
        phase_u_resistance, 
        phase_v_resistance, 
        phase_w_resistance, 
        phase_inductance_baseline,
        phase_inductance_angle,
        phase_inductance_offset,
      },
      u_resistance_step, 
      v_resistance_step, 
      w_resistance_step, 
      inductance_baseline_step,
      inductance_offset_step,
      inductance_angle_step,

      gradients,
    });

    // Stop iterating if all changes are under the threshold.
    is_stable = (
      (Math.abs(u_resistance_change) < stability_threshold) &&
      (Math.abs(v_resistance_change) < stability_threshold) &&
      (Math.abs(w_resistance_change) < stability_threshold) &&
      (Math.abs(inductance_baseline_change) < stability_threshold)
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
      phase_inductance_baseline,
      phase_inductance_angle,
      phase_inductance_offset,
    }
  };

  return current_calibration_data;
}