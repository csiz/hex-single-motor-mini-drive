import {pwm_cycles_per_second, millis_per_cycle} from "./constants.js";
import {normalize_radians} from "./angular_math.js";
import {HISTORY_SIZE} from "hex-mini-drive-interface";
import {MessageCode} from "./motor_controller.js";
import {wait} from "./async_utils.js";
import {square, invalid_to_zero, dq0_transform} from "./math_utils.js";
import {zip_records} from "./data_utils.js";
import {product_of_normals} from "./stats_utils.js";

import * as d3 from "d3";


const rate_increase = 1.2;
const rate_decrease = 0.5;
const stability_threshold = 0.000_1;
const max_iterations = 100;



function optimizer_init(init) {
  return Object.fromEntries(Object.entries(init).map(([key, {value, max_learning_rate}]) => {
    return [key, {
      value,
      learning_rate: max_learning_rate,
      max_learning_rate,
      gradient_sign: 0.0,
    }];
  }));
}

function optimizer_update(params, gradients) {
  let is_stable = true;

  for (const [key, gradient] of Object.entries(gradients)) {
    let parameter = params[key];

    if (parameter === undefined) {
      throw new Error(`Parameter ${key} not found in optimizer parameters`);
    }

    const sign = Math.sign(gradient);
    const prev_sign = parameter.gradient_sign;
    if (sign === prev_sign) {
      parameter.learning_rate = Math.min(parameter.learning_rate * rate_increase, parameter.max_learning_rate);
    } else {
      parameter.learning_rate = parameter.learning_rate * rate_decrease;
    }

    parameter.gradient_sign = sign;

    parameter.value -= sign * parameter.learning_rate;

    if (parameter.learning_rate >= (parameter.max_learning_rate * stability_threshold)) {
      is_stable = false;
    }
  }

  return is_stable;
}

export async function run_current_calibration(motor_controller, message_options) {
  if (!motor_controller.current_calibration) {
    console.error("We didn't load the active calibration data from the driver, we can't proceed without it.");
    throw new Error("Missing current calibration data");
  }

  const current_calibration = {...motor_controller.current_calibration};
  const control_parameters = motor_controller.control_parameters;

  console.info("Current calibration starting");

  // Run a calibration instance.
  const sample = await motor_controller.send_command_and_await_reply({
    message: {
      ...message_options,
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

  // Test if we've had nominal VCC voltage throughout the calibration run.
  const all_nominal_vcc_voltage = sample.every(({nominal_vcc_voltage}) => nominal_vcc_voltage);

  if (!all_nominal_vcc_voltage) {
    console.warn("Nominal VCC voltage was not maintained during the calibration run, we can't trust the drive voltage.");
    return {
      sample,
      is_stable: false,
    }
  }

  // Test if the voltage was always 0.
  const all_zero_drive_voltage = sample.every(({u_drive_voltage, v_drive_voltage, w_drive_voltage}) => 
    u_drive_voltage === 0 && v_drive_voltage === 0 && w_drive_voltage === 0
  );

  // For all 0 voltages we calibrate the baseline offset of the currents.
  if (all_zero_drive_voltage) {
    const u_current_zero = sample.reduce((sum, {u_current}) => sum + u_current, 0) / sample.length + current_calibration?.u_current_zero;
    const v_current_zero = sample.reduce((sum, {v_current}) => sum + v_current, 0) / sample.length + current_calibration?.v_current_zero;
    const w_current_zero = sample.reduce((sum, {w_current}) => sum + w_current, 0) / sample.length + current_calibration?.w_current_zero; 
  
    return {
      sample,
      is_stable: true,
      current_calibration: {
        u_current_zero,
        v_current_zero,
        w_current_zero,
      }
    }
  }

  let parameters = optimizer_init({
    resistance: {
      value: 0.0, 
      max_learning_rate: control_parameters.phase_resistance_ki,
    },
    inductance: {
      value: 0.0, 
      max_learning_rate: control_parameters.phase_inductance_ki,
    },
    inductance_bias: {
      value: 0.0, 
      max_learning_rate: control_parameters.phase_inductance_ki,
    },
    inductance_bias_angle: {
      value: 0.0, 
      max_learning_rate: control_parameters.magnetization_angle_ki,
    },
    magnetization_angle: {
      value: 0.0, 
      max_learning_rate: control_parameters.magnetization_angle_ki,
    },
    magnetization_factor: {
      value: 0.0,
      max_learning_rate: control_parameters.magnetization_factor_ki,
    },
    predicted_angle: {
      value: 0.0, 
      max_learning_rate: control_parameters.magnetization_angle_ki,
    },
  });


  let is_stable = false;

  let iterations = [];

  for (let i = 0; !is_stable && (i < max_iterations); i++) {

    const gradients = sample.map((readout) => {
      const {
        u_current, v_current, w_current, 
        current_angle, current_magnitude, current_angular_speed,
        u_current_diff, v_current_diff, w_current_diff, 
        u_drive_voltage, v_drive_voltage, w_drive_voltage,
        drive_voltage_angle, drive_voltage_magnitude,
      } = readout;

      // Recalculate the electrical equations based on the updated calibration parameters.
      // Compute the gradients of the parameters to minimize the residual error.

      const u_scaled_current_diff = u_current_diff * pwm_cycles_per_second;
      const v_scaled_current_diff = v_current_diff * pwm_cycles_per_second;
      const w_scaled_current_diff = w_current_diff * pwm_cycles_per_second;

      const u_resistance = parameters.resistance.value;
      const v_resistance = parameters.resistance.value;
      const w_resistance = parameters.resistance.value;

      const u_resistive_voltage = u_current * u_resistance;
      const v_resistive_voltage = v_current * v_resistance;
      const w_resistive_voltage = w_current * w_resistance;

      const u_inductance = parameters.inductance.value + parameters.inductance_bias.value * Math.cos(parameters.inductance_bias_angle.value);
      const v_inductance = parameters.inductance.value + parameters.inductance_bias.value * Math.cos(parameters.inductance_bias_angle.value - 2 * Math.PI / 3);
      const w_inductance = parameters.inductance.value + parameters.inductance_bias.value * Math.cos(parameters.inductance_bias_angle.value + 2 * Math.PI / 3);

      const u_inductance_voltage = u_scaled_current_diff * u_inductance;
      const v_inductance_voltage = v_scaled_current_diff * v_inductance;
      const w_inductance_voltage = w_scaled_current_diff * w_inductance;

      const inductance_power_ish = square(current_magnitude) * Math.abs(current_angular_speed);
      const inductance_power_emf = parameters.magnetization_factor.value * inductance_power_ish;

      
      const u_residual1 = u_resistive_voltage + u_inductance_voltage - u_drive_voltage;
      const v_residual1 = v_resistive_voltage + v_inductance_voltage - v_drive_voltage;
      const w_residual1 = w_resistive_voltage + w_inductance_voltage - w_drive_voltage;
      
      const loss1 = square(u_residual1) + square(v_residual1) + square(w_residual1);
      
      const resistance_gradient = (
        u_residual1 * u_current +
        v_residual1 * v_current +
        w_residual1 * w_current
      );

      const inductance_gradient = (
        u_residual1 * u_scaled_current_diff +
        v_residual1 * v_scaled_current_diff +
        w_residual1 * w_scaled_current_diff
      );

      const inductance_bias_gradient = (
        u_residual1 * u_scaled_current_diff * Math.cos(parameters.inductance_bias_angle.value) +
        v_residual1 * v_scaled_current_diff * Math.cos(parameters.inductance_bias_angle.value - 2 * Math.PI / 3) +
        w_residual1 * w_scaled_current_diff * Math.cos(parameters.inductance_bias_angle.value + 2 * Math.PI / 3)
      );

      const inductance_bias_angle_gradient = -(
        u_residual1 * u_scaled_current_diff * parameters.inductance_bias.value * Math.sin(parameters.inductance_bias_angle.value) +
        v_residual1 * v_scaled_current_diff * parameters.inductance_bias.value * Math.sin(parameters.inductance_bias_angle.value - 2 * Math.PI / 3) +
        w_residual1 * w_scaled_current_diff * parameters.inductance_bias.value * Math.sin(parameters.inductance_bias_angle.value + 2 * Math.PI / 3)
      );

      const magnetization_offset = 2*current_angle - parameters.magnetization_angle.value;

      const u_magnetization_voltage = inductance_power_emf * Math.cos(magnetization_offset);
      const v_magnetization_voltage = inductance_power_emf * Math.cos(magnetization_offset - 2 * Math.PI / 3);
      const w_magnetization_voltage = inductance_power_emf * Math.cos(magnetization_offset + 2 * Math.PI / 3);

      const u_residual2 = u_residual1 + u_magnetization_voltage;
      const v_residual2 = v_residual1 + v_magnetization_voltage;
      const w_residual2 = w_residual1 + w_magnetization_voltage;

      // We define the loss as the sum of the squares of the unexplained residual voltages. We assume
      // during the calibration that EMF is negligible, and therefore we calibrate the resistance against
      const loss2 = square(u_residual2) + square(v_residual2) + square(w_residual2);


      const magnetization_factor_gradient = (
        u_residual2 * inductance_power_ish * Math.cos(magnetization_offset) +
        v_residual2 * inductance_power_ish * Math.cos(magnetization_offset - 2 * Math.PI / 3) +
        w_residual2 * inductance_power_ish * Math.cos(magnetization_offset + 2 * Math.PI / 3)
      );

      const magnetization_angle_gradient = (
        u_residual2 * inductance_power_emf * Math.sin(magnetization_offset) +
        v_residual2 * inductance_power_emf * Math.sin(magnetization_offset - 2 * Math.PI / 3) +
        w_residual2 * inductance_power_emf * Math.sin(magnetization_offset + 2 * Math.PI / 3)
      );

      const residual2_square = square(u_residual2) + square(v_residual2) + square(w_residual2);
      
      const residual2_square_prediction = square(inductance_power_emf * (0.5 + 0.5 * Math.cos(current_angle - parameters.predicted_angle.value)));

      const residual3 = residual2_square_prediction - residual2_square;

      const loss3 = Math.abs(residual3);

      const predicted_angle_gradient = residual3 * square(inductance_power_emf) * Math.sin(current_angle - parameters.predicted_angle.value);

      const magnet_distortion = 20.0 * Math.PI / 180.0;
      const magnet_distortion_factor = 0.5;

      const u_magnetization_voltage3 = inductance_power_emf * Math.cos(magnetization_offset + magnet_distortion * Math.sin(current_angle - parameters.predicted_angle.value)) * (1.0 + magnet_distortion_factor + magnet_distortion_factor * Math.cos(current_angle - parameters.predicted_angle.value));
      const v_magnetization_voltage3 = inductance_power_emf * Math.cos(magnetization_offset - 2 * Math.PI / 3 + magnet_distortion * Math.sin(current_angle - parameters.predicted_angle.value)) * (1.0 + magnet_distortion_factor + magnet_distortion_factor * Math.cos(current_angle - parameters.predicted_angle.value));
      const w_magnetization_voltage3 = inductance_power_emf * Math.cos(magnetization_offset + 2 * Math.PI / 3 + magnet_distortion * Math.sin(current_angle - parameters.predicted_angle.value)) * (1.0 + magnet_distortion_factor + magnet_distortion_factor * Math.cos(current_angle - parameters.predicted_angle.value));

      return {
        ...readout,
        
        u_resistive_voltage,
        v_resistive_voltage,
        w_resistive_voltage,
        
        u_inductance_voltage,
        v_inductance_voltage,
        w_inductance_voltage,
        
        loss1,
        u_residual1,
        v_residual1,
        w_residual1,

        loss2,
        u_residual2,
        v_residual2,
        w_residual2,

        u_magnetization_voltage,
        v_magnetization_voltage,
        w_magnetization_voltage,
        
        residual2_square,
        residual2_square_prediction,
        
        loss3,

        u_magnetization_voltage3,
        v_magnetization_voltage3,
        w_magnetization_voltage3,

        resistance_gradient,
        inductance_gradient,
        inductance_bias_gradient,
        inductance_bias_angle_gradient,
        magnetization_factor_gradient,
        magnetization_angle_gradient,
        predicted_angle_gradient,

        current_angle,
        drive_voltage_angle,
      };
    });

    const sqrt_loss1 = Math.sqrt(d3.mean(gradients, (d) => d.loss1));

    const sqrt_loss2 = Math.sqrt(d3.mean(gradients, (d) => d.loss2));
    
    const sqrt_loss3 = Math.sqrt(d3.mean(gradients, (d) => d.loss3));
    
    const angle_diff = normalize_radians(parameters.predicted_angle.value - parameters.magnetization_angle.value);


    iterations.push({
      iteration: i,
      current_calibration: {
        resistance: parameters.resistance.value,
        inductance: parameters.inductance.value,
        inductance_bias: parameters.inductance_bias.value,
        inductance_bias_angle: parameters.inductance_bias_angle.value,
        magnetization_angle: parameters.magnetization_angle.value,
        magnetization_factor: parameters.magnetization_factor.value,
        predicted_angle: parameters.predicted_angle.value,
        angle_diff,
        sqrt_loss1,
        sqrt_loss2,
        sqrt_loss3,
      },
      gradients,
    });

    // Resilient Backpropagation
    // -------------------------
    // 
    // Update steps and learning rates using the sign of the gradient to the unexplained residual loss.

    is_stable = optimizer_update(parameters, {
      resistance: d3.mean(gradients, (d) => d.resistance_gradient),
      inductance: d3.mean(gradients, (d) => d.inductance_gradient),
      inductance_bias: d3.mean(gradients, (d) => d.inductance_bias_gradient),
      inductance_bias_angle: d3.mean(gradients, (d) => d.inductance_bias_angle_gradient),
      magnetization_factor: d3.mean(gradients, (d) => d.magnetization_factor_gradient),
      magnetization_angle: d3.mean(gradients, (d) => d.magnetization_angle_gradient),
      predicted_angle: d3.mean(gradients, (d) => d.predicted_angle_gradient),
    });

    
    // Update calibration values after pushing the iteration data! The iteration should then
    // contain the calibration values that were used to calculate the gradients and other values.

    if (parameters.magnetization_factor.value < 0.0) {
      parameters.magnetization_factor.value = -parameters.magnetization_factor.value;
      parameters.magnetization_factor.learning_rate *= rate_decrease;
      parameters.magnetization_angle.value = normalize_radians(parameters.magnetization_angle.value + Math.PI);
    } else {
      parameters.magnetization_angle.value = normalize_radians(parameters.magnetization_angle.value);
    }

    parameters.predicted_angle.value = normalize_radians(parameters.predicted_angle.value);

    parameters.inductance.value = Math.max(0.0, parameters.inductance.value);
    
    if (parameters.inductance_bias.value < 0.0) {
      parameters.inductance_bias.value = -parameters.inductance_bias.value;
      parameters.inductance_bias.learning_rate *= rate_decrease;
      parameters.inductance_bias_angle.value = normalize_radians(parameters.inductance_bias_angle.value + Math.PI);
    } else {
      parameters.inductance_bias_angle.value = normalize_radians(parameters.inductance_bias_angle.value);
    }
  }

  const current_calibration_data = {
    sample,
    is_stable,
    iterations,
    current_calibration: {
      resistance: parameters.resistance.value,
      inductance: parameters.inductance.value,
      inductance_bias: parameters.inductance_bias.value,
      inductance_bias_angle: parameters.inductance_bias_angle.value,
      magnetization_angle: parameters.magnetization_angle.value,
      magnetization_factor: parameters.magnetization_factor.value,
      predicted_angle: parameters.predicted_angle.value,
    }
  };

  return current_calibration_data;
}