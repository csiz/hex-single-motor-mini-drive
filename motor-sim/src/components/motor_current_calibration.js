import {pwm_base, cycles_per_millisecond, millis_per_cycle, history_size, phase_resistance, phase_inductance} from "./motor_constants.js";
import {command_codes} from "./motor_interface.js";
import {wait} from "./async_utils.js";
import {square, invalid_to_zero} from "./math_utils.js";
import {zip_records} from "./data_utils.js";
import {product_of_normals} from "./stats_utils.js";

import * as d3 from "d3";
import _ from "lodash";


const short_duration = history_size / 12 * millis_per_cycle;
const settling_time = short_duration * 0.4;
const settling_end = short_duration * 0.95;

const calibration_start = short_duration * 0.5;
const calibration_end = short_duration * 10.5;

export const current_calibration_zones = [
  {pwm: 0.1, start: short_duration * 1.0},
  {pwm: 0.2, start: short_duration * 2.0},
  {pwm: 0.3, start: short_duration * 3.0},
  {pwm: 0.4, start: short_duration * 4.0},
  {pwm: 0.5, start: short_duration * 5.0},
  {pwm: 0.6, start: short_duration * 6.0},
  {pwm: 0.7, start: short_duration * 7.0},
  {pwm: 0.8, start: short_duration * 8.0},
  {pwm: 0.9, start: short_duration * 9.0},
].map((zone) => ({
  ...zone,
  settle_start: zone.start + settling_time,
  settle_end: zone.start + settling_end,
}));




export async function run_current_calibration(motor_controller, max_pwm_value){

  const settle_time = 100;
  const settle_timeout = Math.floor((settle_time + 300) * cycles_per_millisecond);
  const settle_strength = Math.floor(max_pwm_value * 2 / 10);

  const drive_options = {command_timeout: settle_timeout, command_value: settle_strength};
  const test_options = {command_timeout: 1, command_value: max_pwm_value};
  const read_options = {expected_messages: history_size, expected_code: command_codes.READOUT};

  console.info("Current calibration starting");

  // Note: hold pwm is clamped by the motor driver

  await motor_controller.send_command({command: command_codes.SET_STATE_HOLD_U_POSITIVE, ...drive_options});
  await wait(settle_time);
  const u_positive_readout = await motor_controller.command_and_read(
    {command: command_codes.SET_STATE_TEST_U_INCREASING, ...test_options},
    read_options);

  console.info("U positive done");

  await motor_controller.send_command({command: command_codes.SET_STATE_HOLD_W_NEGATIVE, ...drive_options});
  await wait(settle_time);
  const w_negative_readout = await motor_controller.command_and_read(
    {command: command_codes.SET_STATE_TEST_W_DECREASING, ...test_options},
    read_options);

  console.info("W negative done");

  await motor_controller.send_command({command: command_codes.SET_STATE_HOLD_V_POSITIVE, ...drive_options});
  await wait(settle_time);
  const v_positive_readout = await motor_controller.command_and_read(
    {command: command_codes.SET_STATE_TEST_V_INCREASING, ...test_options},
    read_options);

  console.info("V positive done");

  await motor_controller.send_command({command: command_codes.SET_STATE_HOLD_U_NEGATIVE, ...drive_options});
  await wait(settle_time);
  const u_negative_readout = await motor_controller.command_and_read(
    {command: command_codes.SET_STATE_TEST_U_DECREASING, ...test_options},
    read_options);

  console.info("U negative done");

  await motor_controller.send_command({command: command_codes.SET_STATE_HOLD_W_POSITIVE, ...drive_options});
  await wait(settle_time);
  const w_positive_readout = await motor_controller.command_and_read(
    {command: command_codes.SET_STATE_TEST_W_INCREASING, ...test_options},
    read_options);

  console.info("W positive done");

  await motor_controller.send_command({command: command_codes.SET_STATE_HOLD_V_NEGATIVE, ...drive_options});
  await wait(settle_time);
  const v_negative_readout = await motor_controller.command_and_read(
    {command: command_codes.SET_STATE_TEST_V_DECREASING, ...test_options},
    read_options);

  console.info("V negative done");

  const times = d3.range(history_size).map((i) => i * millis_per_cycle);

  // Check all calibration data is complete.
  if (u_positive_readout.length !== history_size) {
    console.error("U positive calibration data incomplete", u_positive_readout);
    return;
  }
  if (u_negative_readout.length !== history_size) {
    console.error("U negative calibration data incomplete", u_negative_readout);
    return;
  }
  if (v_positive_readout.length !== history_size) {
    console.error("V positive calibration data incomplete", v_positive_readout);
    return;
  }
  if (v_negative_readout.length !== history_size) {
    console.error("V negative calibration data incomplete", v_negative_readout);
    return;
  }
  if (w_positive_readout.length !== history_size) {
    console.error("W positive calibration data incomplete", w_positive_readout);
    return;
  }
  if (w_negative_readout.length !== history_size) {
    console.error("W negative calibration data incomplete", w_negative_readout);
    return;
  }

  const u_positive = u_positive_readout.map((d) => ({
    time: d.time,
    drive_voltage: d.u_drive_voltage,
    uncalibrated_current: d.u_readout,
    uncalibrated_current_diff: d.u_readout_diff,
  }));
  const u_negative = u_negative_readout.map((d) => ({
    time: d.time,
    drive_voltage: -d.u_drive_voltage,
    uncalibrated_current: -d.u_readout,
    uncalibrated_current_diff: -d.u_readout_diff,
  }));
  const v_positive = v_positive_readout.map((d) => ({
    time: d.time,
    drive_voltage: d.v_drive_voltage,
    uncalibrated_current: d.v_readout,
    uncalibrated_current_diff: d.v_readout_diff,
  }));
  const v_negative = v_negative_readout.map((d) => ({
    time: d.time,
    drive_voltage: -d.v_drive_voltage,
    uncalibrated_current: -d.v_readout,
    uncalibrated_current_diff: -d.v_readout_diff,
  }));
  const w_positive = w_positive_readout.map((d) => ({
    time: d.time,
    drive_voltage: d.w_drive_voltage,
    uncalibrated_current: d.w_readout,
    uncalibrated_current_diff: d.w_readout_diff,
  }));
  const w_negative = w_negative_readout.map((d) => ({
    time: d.time,
    drive_voltage: -d.w_drive_voltage,
    uncalibrated_current: -d.w_readout,
    uncalibrated_current_diff: -d.w_readout_diff,
  }));

  const expected_current = times.map((t, i) => {
    return d3.mean([
      u_positive[i].drive_voltage,
      u_negative[i].drive_voltage,
      v_positive[i].drive_voltage,
      v_negative[i].drive_voltage,
      w_positive[i].drive_voltage,
      w_negative[i].drive_voltage,
    ]) / phase_resistance;
  });


  // Make a new table with each calibration's measured phase current and
  // expected phase current inferred from the driver voltage and known
  // phase resistance (in v0 the shunt resistors vary +-50% after soldering
  // we should buy better current shunts, or maybe just bigger footprint).
  const sample = zip_records({
    "time": times,
    "expected": expected_current,
    u_positive, u_negative,
    v_positive, v_negative,
    w_positive, w_negative,
  });


  const current_calibration_data = {
    sample,
    u_positive,
    u_negative,
    v_positive,
    v_negative,
    w_positive,
    w_negative,
    ...compute_calibration_instance({
      u_positive, u_negative,
      v_positive, v_negative,
      w_positive, w_negative,
    })
  };


  console.info("Current calibration done", current_calibration_data);

  return current_calibration_data;
}


function compute_gradients({current_factor, inductance_factor}, data){

  return data.filter((d) => d.time >= calibration_start && d.time <= calibration_end).map((d) => {
    const {drive_voltage, uncalibrated_current, uncalibrated_current_diff} = d;


    const resistance_drop = current_factor * uncalibrated_current * phase_resistance;
    const inductance_drop = inductance_factor * current_factor * uncalibrated_current_diff * 1000 * phase_inductance;

    const residual = resistance_drop + inductance_drop - drive_voltage;

    const loss = square(residual);

    const resistance_weight = (Math.abs(resistance_drop) < 0.1 || Math.abs(inductance_drop / resistance_drop) > 0.2) ? 0.0 : 1.0;

    const inductance_weight = (Math.abs(resistance_drop) < 0.1) ? 0.0 : Math.min(1.0, square(inductance_drop / resistance_drop));

    const current_factor_gradient = invalid_to_zero(loss * current_factor / (residual * 2 * (resistance_drop + inductance_drop)));
    const inductance_factor_gradient = invalid_to_zero(loss * inductance_factor / (residual * 2 * inductance_drop));

    const current_factor_variance = square(current_factor_gradient);
    const inductance_factor_variance = square(inductance_factor_gradient);

    return {
      ...d,
      resistance_drop,
      inductance_drop,
      residual,

      loss,
      current_factor_gradient: current_factor_gradient * resistance_weight, 
      inductance_factor_gradient: inductance_factor_gradient * inductance_weight,
      current_factor_variance: current_factor_variance * resistance_weight, 
      inductance_factor_variance: inductance_factor_variance * inductance_weight,
      resistance_weight, inductance_weight,
    };
  });
}

function multi_mean(array_of_arrays, value_fn) {
  return d3.mean(array_of_arrays, (array) => {
    return d3.mean(array, value_fn);
  });
}

function compute_calibration_instance({u_positive, u_negative, v_positive, v_negative, w_positive, w_negative}){

  let u_factor = 1.0;
  let v_factor = 1.0;
  let w_factor = 1.0;
  let inductance_factor = 1.0;

  const learning_rate = 0.20;
  const max_iterations = 500;
  const stability_threshold = 0.000_01;

  let iterations = [];
  for (let i = 0; i < max_iterations; i++) {
    const u_positive_gradients = compute_gradients({current_factor: u_factor, inductance_factor}, u_positive);
    const u_negative_gradients = compute_gradients({current_factor: u_factor, inductance_factor}, u_negative);
    const v_positive_gradients = compute_gradients({current_factor: v_factor, inductance_factor}, v_positive);
    const v_negative_gradients = compute_gradients({current_factor: v_factor, inductance_factor}, v_negative);
    const w_positive_gradients = compute_gradients({current_factor: w_factor, inductance_factor}, w_positive);
    const w_negative_gradients = compute_gradients({current_factor: w_factor, inductance_factor}, w_negative);

    const u_gradients = [u_positive_gradients, u_negative_gradients];
    const v_gradients = [v_positive_gradients, v_negative_gradients];
    const w_gradients = [w_positive_gradients, w_negative_gradients];
    const all_gradients = [...u_gradients, ...v_gradients, ...w_gradients];

    const u_resistance_weight = multi_mean(u_gradients, (d) => d.resistance_weight);
    const v_resistance_weight = multi_mean(v_gradients, (d) => d.resistance_weight);
    const w_resistance_weight = multi_mean(w_gradients, (d) => d.resistance_weight);

    const u_factor_gradient = multi_mean(u_gradients, (d) => d.current_factor_gradient) / u_resistance_weight;
    const v_factor_gradient = multi_mean(v_gradients, (d) => d.current_factor_gradient) / v_resistance_weight;
    const w_factor_gradient = multi_mean(w_gradients, (d) => d.current_factor_gradient) / w_resistance_weight;

    const inductance_weight = multi_mean(all_gradients, (d) => d.inductance_weight);
    const inductance_factor_gradient = multi_mean(all_gradients, (d) => d.inductance_factor_gradient) / inductance_weight;

    const u_factor_stdev = Math.sqrt(multi_mean(u_gradients, (d) => d.current_factor_variance) / u_resistance_weight);
    const v_factor_stdev = Math.sqrt(multi_mean(v_gradients, (d) => d.current_factor_variance) / v_resistance_weight);
    const w_factor_stdev = Math.sqrt(multi_mean(w_gradients, (d) => d.current_factor_variance) / w_resistance_weight);

    const inductance_factor_stdev = Math.sqrt(multi_mean(all_gradients, (d) => d.inductance_factor_variance) / inductance_weight);

    const u_factor_change = learning_rate * u_factor_gradient;
    const v_factor_change = learning_rate * v_factor_gradient;
    const w_factor_change = learning_rate * w_factor_gradient;
    const inductance_factor_change = learning_rate * inductance_factor_gradient;

    u_factor -= u_factor_change;
    v_factor -= v_factor_change;
    w_factor -= w_factor_change;
    inductance_factor -= inductance_factor_change;

    iterations.push({
      iteration: i,
      current_calibration: {
        u_factor, v_factor, w_factor, inductance_factor, 
        u_factor_stdev, v_factor_stdev, w_factor_stdev, inductance_factor_stdev,
        u_resistance_weight, v_resistance_weight, w_resistance_weight, inductance_weight,
      },
      u_factor_gradient, v_factor_gradient, w_factor_gradient, inductance_factor_gradient,
      u_positive_gradients,
      u_negative_gradients,
      v_positive_gradients,
      v_negative_gradients,
      w_positive_gradients,
      w_negative_gradients,
    });

    // Stop iterating if all changes are under the threshold.
    const is_stable = (
      Math.abs(u_factor_change) < stability_threshold &&
      Math.abs(v_factor_change) < stability_threshold &&
      Math.abs(w_factor_change) < stability_threshold &&
      Math.abs(inductance_factor_change) < stability_threshold
    );

    const is_invalid = (
      u_resistance_weight < 0.1 ||
      v_resistance_weight < 0.1 ||
      w_resistance_weight < 0.1
    );

    if (is_invalid) {
      console.warn("Current calibration was not valid, is there enough voltage applied to the motor driver?");
      return {
        iterations,
        current_calibration: null,
      }
    }

    if (is_stable) {
      return {
        iterations,
        current_calibration: {
          u_factor,
          u_factor_stdev, 
          v_factor,
          v_factor_stdev, 
          w_factor,
          w_factor_stdev, 
          inductance_factor,
          inductance_factor_stdev,
        }
      };
    }
  }

  // If we reach here, the calibration did not converge.
  console.warn("Current calibration did not converge in the maximum number of iterations.");

  return {
    iterations,
    current_calibration: null,
  };
}


export function compute_current_calibration(calibration_results){

  const valid_calibration_results = calibration_results.filter(({current_calibration}) => current_calibration);

  if (valid_calibration_results.length == 0) return {
    stats: [],
    current_calibration: null,
  };
  
  const stats = d3.range(history_size).map((i) => {
    return {
      time: i * millis_per_cycle,
      expected: d3.mean(valid_calibration_results, ({sample}) => sample[i].expected),
      expected_stdev: d3.deviation(valid_calibration_results, ({sample}) => sample[i].expected),
      u_positive: d3.mean(valid_calibration_results, ({sample}) => sample[i].u_positive_uncalibrated_current),
      u_positive_stdev: d3.deviation(valid_calibration_results, ({sample}) => sample[i].u_positive_uncalibrated_current), 
      u_negative: d3.mean(valid_calibration_results, ({sample}) => sample[i].u_negative_uncalibrated_current),
      u_negative_stdev: d3.deviation(valid_calibration_results, ({sample}) => sample[i].u_negative_uncalibrated_current),
      v_positive: d3.mean(valid_calibration_results, ({sample}) => sample[i].v_positive_uncalibrated_current),
      v_positive_stdev: d3.deviation(valid_calibration_results, ({sample}) => sample[i].v_positive_uncalibrated_current),
      v_negative: d3.mean(valid_calibration_results, ({sample}) => sample[i].v_negative_uncalibrated_current),
      v_negative_stdev: d3.deviation(valid_calibration_results, ({sample}) => sample[i].v_negative_uncalibrated_current),
      w_positive: d3.mean(valid_calibration_results, ({sample}) => sample[i].w_positive_uncalibrated_current),
      w_positive_stdev: d3.deviation(valid_calibration_results, ({sample}) => sample[i].w_positive_uncalibrated_current),
      w_negative: d3.mean(valid_calibration_results, ({sample}) => sample[i].w_negative_uncalibrated_current),
      w_negative_stdev: d3.deviation(valid_calibration_results, ({sample}) => sample[i].w_negative_uncalibrated_current),
    };
  });

  const current_calibration = valid_calibration_results.map((d) => d.current_calibration).reduce((acc, current_calibration) => {

    const {mean: u_factor, stdev: u_factor_stdev} = product_of_normals({
      mean_a: acc.u_factor, 
      stdev_a: acc.u_factor_stdev, 
      mean_b: current_calibration.u_factor, 
      stdev_b: current_calibration.u_factor_stdev
    });

    const {mean: v_factor, stdev: v_factor_stdev} = product_of_normals({
      mean_a: acc.v_factor, 
      stdev_a: acc.v_factor_stdev, 
      mean_b: current_calibration.v_factor, 
      stdev_b: current_calibration.v_factor_stdev
    });

    const {mean: w_factor, stdev: w_factor_stdev} = product_of_normals({
      mean_a: acc.w_factor, 
      stdev_a: acc.w_factor_stdev, 
      mean_b: current_calibration.w_factor, 
      stdev_b: current_calibration.w_factor_stdev
    });

    const {mean: inductance_factor, stdev: inductance_factor_stdev} = product_of_normals({
      mean_a: acc.inductance_factor, 
      stdev_a: acc.inductance_factor_stdev, 
      mean_b: current_calibration.inductance_factor, 
      stdev_b: current_calibration.inductance_factor_stdev
    });

    return {
      u_factor,
      u_factor_stdev,
      v_factor,
      v_factor_stdev,
      w_factor,
      w_factor_stdev,
      inductance_factor,
      inductance_factor_stdev,
    };
  });


  return {
    stats,
    current_calibration,
  };
}

