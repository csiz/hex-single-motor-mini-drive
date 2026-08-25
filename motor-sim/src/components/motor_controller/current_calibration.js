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

  let parameters = optimizer_init({
    u_resistance: {
      value: current_calibration.u_resistance, 
      max_learning_rate: control_parameters.phase_resistance_ki,
    },
    v_resistance: {
      value: current_calibration.v_resistance, 
      max_learning_rate: control_parameters.phase_resistance_ki,
    },
    w_resistance: {
      value: current_calibration.w_resistance, 
      max_learning_rate: control_parameters.phase_resistance_ki,
    },
    inductance: {
      value: current_calibration.inductance, 
      max_learning_rate: control_parameters.phase_inductance_ki,
    },
    magnetization_angle: {
      value: current_calibration.magnetization_angle, 
      max_learning_rate: control_parameters.magnetization_angle_ki,
    },
    magnetization_factor: {
      value: current_calibration.magnetization_factor, 
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
        drive_voltage_angle,
      } = readout;

      const u_scaled_current_diff = u_current_diff * pwm_cycles_per_second;
      const v_scaled_current_diff = v_current_diff * pwm_cycles_per_second;
      const w_scaled_current_diff = w_current_diff * pwm_cycles_per_second;

      const u_resistive_voltage = u_current * parameters.u_resistance.value;
      const v_resistive_voltage = v_current * parameters.v_resistance.value;
      const w_resistive_voltage = w_current * parameters.w_resistance.value;

      const u_inductance_voltage = u_scaled_current_diff * parameters.inductance.value;
      const v_inductance_voltage = v_scaled_current_diff * parameters.inductance.value;
      const w_inductance_voltage = w_scaled_current_diff * parameters.inductance.value;

      const inductance_power_ish = square(current_magnitude) * Math.abs(current_angular_speed);
      const inductance_power_emf = parameters.magnetization_factor.value * inductance_power_ish;

      const wtf_angle = 2*current_angle - parameters.magnetization_angle.value;

      const u_wtf = inductance_power_emf * Math.cos(wtf_angle);
      const v_wtf = inductance_power_emf * Math.cos(wtf_angle - 2 * Math.PI / 3);
      const w_wtf = inductance_power_emf * Math.cos(wtf_angle + 2 * Math.PI / 3);

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


      const magnetization_factor_gradient = (
        u_residual * inductance_power_ish * Math.cos(wtf_angle) +
        v_residual * inductance_power_ish * Math.cos(wtf_angle - 2 * Math.PI / 3) +
        w_residual * inductance_power_ish * Math.cos(wtf_angle + 2 * Math.PI / 3)
      );

      const magnetization_angle_gradient = (
        u_residual * inductance_power_emf * Math.sin(wtf_angle) +
        v_residual * inductance_power_emf * Math.sin(wtf_angle - 2 * Math.PI / 3) +
        w_residual * inductance_power_emf * Math.sin(wtf_angle + 2 * Math.PI / 3)
      );

      const residual_square = square(u_residual) + square(v_residual) + square(w_residual);
      
      const residual_square_prediction = square(inductance_power_emf * (0.5 + 0.5 * Math.cos(current_angle - parameters.predicted_angle.value)));

      const residual2 = residual_square_prediction - residual_square;

      const loss2 = Math.abs(residual2);

      const predicted_angle_gradient = residual2 * square(inductance_power_emf) * Math.sin(current_angle - parameters.predicted_angle.value);

      const magnet_distortion = 20.0 * Math.PI / 180.0;
      const magnet_distortion_factor = 0.5;

      const u_wtf2 = inductance_power_emf * Math.cos(wtf_angle + magnet_distortion * Math.sin(current_angle - parameters.predicted_angle.value)) * (1.0 + magnet_distortion_factor + magnet_distortion_factor * Math.cos(current_angle - parameters.predicted_angle.value));
      const v_wtf2 = inductance_power_emf * Math.cos(wtf_angle - 2 * Math.PI / 3 + magnet_distortion * Math.sin(current_angle - parameters.predicted_angle.value)) * (1.0 + magnet_distortion_factor + magnet_distortion_factor * Math.cos(current_angle - parameters.predicted_angle.value));
      const w_wtf2 = inductance_power_emf * Math.cos(wtf_angle + 2 * Math.PI / 3 + magnet_distortion * Math.sin(current_angle - parameters.predicted_angle.value)) * (1.0 + magnet_distortion_factor + magnet_distortion_factor * Math.cos(current_angle - parameters.predicted_angle.value));

      return {
        ...readout,
        loss,

        u_resistive_voltage,
        v_resistive_voltage,
        w_resistive_voltage,

        u_inductance_voltage,
        v_inductance_voltage,
        w_inductance_voltage,

        u_wtf,
        v_wtf,
        w_wtf,

        u_wtf2,
        v_wtf2,
        w_wtf2,
        
        u_residual,
        v_residual,
        w_residual,

        residual_square,
        residual_square_prediction,
        loss2,

        u_resistance_gradient,
        v_resistance_gradient,
        w_resistance_gradient,
        inductance_gradient,
        magnetization_factor_gradient,
        magnetization_angle_gradient,
        predicted_angle_gradient,

        current_angle,
        drive_voltage_angle,
      };
    });

    const sqrt_loss = Math.sqrt(d3.mean(gradients, (d) => d.loss));


    // Resilient Backpropagation
    // -------------------------
    // 
    // Update steps and learning rates using the sign of the gradient to the unexplained residual loss.

    is_stable = optimizer_update(parameters, {
      u_resistance: d3.mean(gradients, (d) => d.u_resistance_gradient),
      v_resistance: d3.mean(gradients, (d) => d.v_resistance_gradient),
      w_resistance: d3.mean(gradients, (d) => d.w_resistance_gradient),
      inductance: d3.mean(gradients, (d) => d.inductance_gradient),
      magnetization_factor: d3.mean(gradients, (d) => d.magnetization_factor_gradient),
      magnetization_angle: d3.mean(gradients, (d) => d.magnetization_angle_gradient),
      predicted_angle: d3.mean(gradients, (d) => d.predicted_angle_gradient),
    });


    const sqrt_loss2 = Math.sqrt(d3.mean(gradients, (d) => d.loss2)); 
    
    const angle_diff = normalize_radians(parameters.predicted_angle.value - parameters.magnetization_angle.value);


    iterations.push({
      iteration: i,
      current_calibration: {
        ...current_calibration,
        u_resistance: parameters.u_resistance.value, 
        v_resistance: parameters.v_resistance.value,
        w_resistance: parameters.w_resistance.value,
        inductance: parameters.inductance.value,
        magnetization_angle: parameters.magnetization_angle.value,
        magnetization_factor: parameters.magnetization_factor.value,
        predicted_angle: parameters.predicted_angle.value,
        angle_diff,
        sqrt_loss,
        sqrt_loss2,
      },
      gradients,
    });
    
    // Update calibration values after pushing the iteration data! The iteration should then
    // contain the calibration values that were used to calculate the gradients and other values.

    parameters.inductance.value = Math.max(0.0, parameters.inductance.value);
    
    if (parameters.magnetization_factor.value < 0.0) {
      parameters.magnetization_factor.value = -parameters.magnetization_factor.value;
      parameters.magnetization_angle.value = normalize_radians(parameters.magnetization_angle.value + Math.PI);
    }
  }

  const current_calibration_data = {
    sample,
    is_stable,
    iterations,
    current_calibration: {
      ...current_calibration,
      u_resistance: parameters.u_resistance.value,
      v_resistance: parameters.v_resistance.value,
      w_resistance: parameters.w_resistance.value,
      inductance: parameters.inductance.value,
      magnetization_angle: parameters.magnetization_angle.value,
      magnetization_factor: parameters.magnetization_factor.value,
      predicted_angle: parameters.predicted_angle.value,
    }
  };

  return current_calibration_data;
}