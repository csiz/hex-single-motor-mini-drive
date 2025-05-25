import {pwm_base, cycles_per_millisecond, millis_per_cycle, history_size, phase_resistance, max_calibration_current} from "./motor_constants.js";
import {command_codes} from "./motor_interface.js";
import {wait} from "./async_utils.js";
import {even_spacing, piecewise_linear, even_piecewise_linear} from "./math_utils.js";

import * as d3 from "d3";
import _ from "lodash";


const short_duration = history_size / 12 * millis_per_cycle;
const settling_time = short_duration * 0.4;
const settling_end = short_duration * 0.95;
const transition_times = d3.range(0.0, settling_time - millis_per_cycle, millis_per_cycle);

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



const current_calibration_points = 32;
const current_calibration_reading_points = even_spacing(max_calibration_current, current_calibration_points / 2 + 1);

export async function run_current_calibration(motor_controller){

  const settle_time = 100;
  const settle_timeout = Math.floor((settle_time + 300) * cycles_per_millisecond);
  const settle_strength = Math.floor(pwm_base * 2 / 10);

  const drive_options = {command_timeout: settle_timeout, command_pwm: settle_strength};
  const test_options = {command_timeout: 0, command_pwm: 0};

  console.info("Current calibration starting");

  // Note: hold pwm is clamped by the motor driver

  await motor_controller.send_command({command: command_codes.SET_STATE_HOLD_U_POSITIVE, ...drive_options});
  await wait(settle_time);
  const u_positive = await motor_controller.command_and_read(
    {command: command_codes.SET_STATE_TEST_U_INCREASING, ...test_options},
    {expected_messages: history_size});

  console.info("U positive done");

  await motor_controller.send_command({command: command_codes.SET_STATE_HOLD_W_NEGATIVE, ...drive_options});
  await wait(settle_time);
  const w_negative = await motor_controller.command_and_read(
    {command: command_codes.SET_STATE_TEST_W_DECREASING, ...test_options},
    {expected_messages: history_size});

  console.info("W negative done");

  await motor_controller.send_command({command: command_codes.SET_STATE_HOLD_V_POSITIVE, ...drive_options});
  await wait(settle_time);
  const v_positive = await motor_controller.command_and_read(
    {command: command_codes.SET_STATE_TEST_V_INCREASING, ...test_options},
    {expected_messages: history_size});

  console.info("V positive done");

  await motor_controller.send_command({command: command_codes.SET_STATE_HOLD_U_NEGATIVE, ...drive_options});
  await wait(settle_time);
  const u_negative = await motor_controller.command_and_read(
    {command: command_codes.SET_STATE_TEST_U_DECREASING, ...test_options},
    {expected_messages: history_size});

  console.info("U negative done");

  await motor_controller.send_command({command: command_codes.SET_STATE_HOLD_W_POSITIVE, ...drive_options});
  await wait(settle_time);
  const w_positive = await motor_controller.command_and_read(
    {command: command_codes.SET_STATE_TEST_W_INCREASING, ...test_options},
    {expected_messages: history_size});

  console.info("W positive done");

  await motor_controller.send_command({command: command_codes.SET_STATE_HOLD_V_NEGATIVE, ...drive_options});
  await wait(settle_time);
  const v_negative = await motor_controller.command_and_read(
    {command: command_codes.SET_STATE_TEST_V_DECREASING, ...test_options},
    {expected_messages: history_size});

  console.info("V negative done");

  // Check all calibration data is complete.
  if (u_positive.length !== history_size) {
    console.error("U positive calibration data incomplete", u_positive);
    return;
  }
  if (u_negative.length !== history_size) {
    console.error("U negative calibration data incomplete", u_negative);
    return;
  }
  if (v_positive.length !== history_size) {
    console.error("V positive calibration data incomplete", v_positive);
    return;
  }
  if (v_negative.length !== history_size) {
    console.error("V negative calibration data incomplete", v_negative);
    return;
  }
  if (w_positive.length !== history_size) {
    console.error("W positive calibration data incomplete", w_positive);
    return;
  }
  if (w_negative.length !== history_size) {
    console.error("W negative calibration data incomplete", w_negative);
    return;
  }


  // Make a new table with each calibration's measured phase current and 
  // expected phase current inferred from the driver voltage and known
  // phase resistance (in v0 the shunt resistors vary +-50% after soldering
  // we should buy better current shunts, or maybe just bigger footprint).
  const sample = d3.range(history_size).map((i) => {
    const time = i * millis_per_cycle;

    const expected = d3.mean([
      +u_positive[i].u_drive_voltage,
      -u_negative[i].u_drive_voltage,
      +v_positive[i].v_drive_voltage,
      -v_negative[i].v_drive_voltage,
      +w_positive[i].w_drive_voltage,
      -w_negative[i].w_drive_voltage,
    ]) / phase_resistance;

    return {
      time,
      expected,
      u_positive: +u_positive[i].u_uncalibrated,
      u_negative: -u_negative[i].u_uncalibrated,
      v_positive: +v_positive[i].v_uncalibrated,
      v_negative: -v_negative[i].v_uncalibrated,
      w_positive: +w_positive[i].w_uncalibrated,
      w_negative: -w_negative[i].w_uncalibrated,
    };
  });

  
  const current_calibration_data = {
    sample,
    u_positive,
    u_negative,
    v_positive,
    v_negative,
    w_positive,
    w_negative,
  };


  console.info("Current calibration done", current_calibration_data);

  return current_calibration_data;
}

function compute_calibration_instance(calibration_data){

}


export function compute_current_calibration(calibration_results){


  const valid_calibration_results = calibration_results.filter((calibration_data, i) => {
    try {
      compute_calibration_instance(calibration_data);
      return true;
    } catch (e) {
      console.error(`Invalid calibration data (index ${i}); error: ${e}`);
      return false;
    }
  });

  if (valid_calibration_results.length == 0) return {
    stats: [],
    current_calibration: null,
  };
  
  const stats = d3.range(history_size).map((i) => {
    return {
      time: i * millis_per_cycle,
      expected: d3.mean(valid_calibration_results, ({sample}) => sample[i].expected),
      expected_stdev: d3.deviation(valid_calibration_results, ({sample}) => sample[i].expected),
      u_positive: d3.mean(valid_calibration_results, ({sample}) => sample[i].u_positive),
      u_positive_stdev: d3.deviation(valid_calibration_results, ({sample}) => sample[i].u_positive),
      u_negative: d3.mean(valid_calibration_results, ({sample}) => sample[i].u_negative),
      u_negative_stdev: d3.deviation(valid_calibration_results, ({sample}) => sample[i].u_negative),
      v_positive: d3.mean(valid_calibration_results, ({sample}) => sample[i].v_positive),
      v_positive_stdev: d3.deviation(valid_calibration_results, ({sample}) => sample[i].v_positive),
      v_negative: d3.mean(valid_calibration_results, ({sample}) => sample[i].v_negative),
      v_negative_stdev: d3.deviation(valid_calibration_results, ({sample}) => sample[i].v_negative),
      w_positive: d3.mean(valid_calibration_results, ({sample}) => sample[i].w_positive),
      w_positive_stdev: d3.deviation(valid_calibration_results, ({sample}) => sample[i].w_positive),
      w_negative: d3.mean(valid_calibration_results, ({sample}) => sample[i].w_negative),
      w_negative_stdev: d3.deviation(valid_calibration_results, ({sample}) => sample[i].w_negative),
    };
  });

  
  const current_calibration = null;
  // {
  //   u_factor: (calibration.u_positive.factor + calibration.u_negative.factor) / 2,
  //   v_factor: (calibration.v_positive.factor + calibration.v_negative.factor) / 2,
  //   w_factor: (calibration.w_positive.factor + calibration.w_negative.factor) / 2,
  // };

  return {
    stats,
    current_calibration,
  };
}

