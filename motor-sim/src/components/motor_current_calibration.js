import {pwm_base, cycles_per_millisecond, millis_per_cycle, history_size} from "./motor_constants.js";
import {command_codes} from "./motor_interface.js";
import {wait} from "./async_utils.js";
import {even_spacing, piecewise_linear, even_piecewise_linear} from "./math_utils.js";

import * as d3 from "d3";


const short_duration = history_size / 12 * millis_per_cycle;

const current_calibration_zones = [
  {pwm: 0.1, settle_start: short_duration * 1.4, settle_end: short_duration * 1.95},
  {pwm: 0.2, settle_start: short_duration * 2.4, settle_end: short_duration * 2.95},
  {pwm: 0.3, settle_start: short_duration * 3.4, settle_end: short_duration * 3.95},
  {pwm: 0.4, settle_start: short_duration * 4.4, settle_end: short_duration * 4.95},
  {pwm: 0.5, settle_start: short_duration * 5.4, settle_end: short_duration * 5.95},
  {pwm: 0.6, settle_start: short_duration * 6.4, settle_end: short_duration * 6.95},
  {pwm: 0.7, settle_start: short_duration * 7.4, settle_end: short_duration * 7.95},
  {pwm: 0.8, settle_start: short_duration * 8.4, settle_end: short_duration * 8.95},
  {pwm: 0.9, settle_start: short_duration * 9.4, settle_end: short_duration * 9.95},
];


const drive_resistance = 2.0; // 2.0 Ohm measured with voltmeter between 1 phase and the other 2 in parallel.
const drive_voltage = 12.0; // 12.0 V // TODO: get it from the chip


export const max_calibration_current = drive_voltage / drive_resistance;

const current_calibration_points = 32;
const current_calibration_reading_points = even_spacing(max_calibration_current, current_calibration_points / 2 + 1);
const current_calibration_targets = current_calibration_zones.map((zone) => zone.pwm * max_calibration_current);


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

  const targets = d3.range(history_size).map((i) => {
    const t = i * millis_per_cycle;
    const zone = current_calibration_zones.filter((zone) => zone.settle_start <= t && t <= zone.settle_end);

    if (zone.length == 0) return null;
    if (zone.length > 1) {
      console.error("Multiple zones found", zone);
      return null;
    }
    return zone[0].pwm * max_calibration_current;
  });

  // Make a new table with each calibration phase as a column.
  const current_calibration_data = d3.range(history_size).map((i) => {
    return {
      time: u_positive[i].time,
      target: targets[i],
      u_positive: u_positive[i].u_uncalibrated,
      u_negative: -u_negative[i].u_uncalibrated,
      v_positive: v_positive[i].v_uncalibrated,
      v_negative: -v_negative[i].v_uncalibrated,
      w_positive: w_positive[i].w_uncalibrated,
      w_negative: -w_negative[i].w_uncalibrated,
    };
  });

  console.info("Current calibration done");

  return current_calibration_data;
}



function compute_calibration_instance(calibration_data){
  
  const calibration_data_by_zone = current_calibration_zones.map((zone) => {
    return calibration_data.filter((d) => d.time > zone.settle_start && d.time < zone.settle_end);
  });

  function compute_zone_calibration({measurements, targets}){
    // Make sure the lengths are equal.
    if (measurements.length !== targets.length) throw new Error("Data length mismatch");

    const factor = d3.mean(targets, (target, i) => target / measurements[i]); 
    
    const slow_calibration = piecewise_linear({
      X: [0.0, ...measurements.map((x) => x)], 
      Y: [0.0, ...targets.map((y) => y / factor)],
    });

    // Recalibrate to evenly spaced points for fast processing.

    const Y = current_calibration_reading_points.map((x) => slow_calibration(x));

    const func = even_piecewise_linear({x_min: 0, x_max: max_calibration_current, Y});

    const sample = current_calibration_reading_points.map((x) => ({reading: x, target: func(x)}));
    
    return {
      measurements,
      targets,
      factor,
      func,
      sample,
    };
  }

  function compute_phase_calibration(phase_selector){    
    return compute_zone_calibration({
      measurements: calibration_data_by_zone.map((zone_data) => d3.mean(zone_data, (d) => d[phase_selector])),
      targets: current_calibration_targets,
    });
  }

  return {
    u_positive: compute_phase_calibration("u_positive"),
    u_negative: compute_phase_calibration("u_negative"),
    v_positive: compute_phase_calibration("v_positive"),
    v_negative: compute_phase_calibration("v_negative"),
    w_positive: compute_phase_calibration("w_positive"),
    w_negative: compute_phase_calibration("w_negative"),
  };
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
    samples: [],
    current_calibration: null,
    calibration_funcs: null,
  };

  const stats = d3.range(history_size).map((i) => {
    return {
      time: valid_calibration_results[0][i].time,
      target: d3.mean(valid_calibration_results, (data) => data[i].target),
      u_positive: d3.mean(valid_calibration_results, (data) => data[i].u_positive),
      u_positive_std: d3.deviation(valid_calibration_results, (data) => data[i].u_positive),
      u_negative: d3.mean(valid_calibration_results, (data) => data[i].u_negative),
      u_negative_std: d3.deviation(valid_calibration_results, (data) => data[i].u_negative),
      v_positive: d3.mean(valid_calibration_results, (data) => data[i].v_positive),
      v_positive_std: d3.deviation(valid_calibration_results, (data) => data[i].v_positive),
      v_negative: d3.mean(valid_calibration_results, (data) => data[i].v_negative),
      v_negative_std: d3.deviation(valid_calibration_results, (data) => data[i].v_negative),
      w_positive: d3.mean(valid_calibration_results, (data) => data[i].w_positive),
      w_positive_std: d3.deviation(valid_calibration_results, (data) => data[i].w_positive),
      w_negative: d3.mean(valid_calibration_results, (data) => data[i].w_negative),
      w_negative_std: d3.deviation(valid_calibration_results, (data) => data[i].w_negative),
    };
  });

  const calibration = compute_calibration_instance(stats);

  const samples = current_calibration_reading_points.map((x, i) => {
    return {
      reading: x,
      target: x,
      u_positive: calibration.u_positive.sample[i].target,
      u_negative: calibration.u_negative.sample[i].target,
      v_positive: calibration.v_positive.sample[i].target,
      v_negative: calibration.v_negative.sample[i].target,
      w_positive: calibration.w_positive.sample[i].target,
      w_negative: calibration.w_negative.sample[i].target,
    };
  });

  const current_calibration = {
    u_positive: calibration.u_positive.factor,
    u_negative: calibration.u_negative.factor,
    v_positive: calibration.v_positive.factor,
    v_negative: calibration.v_negative.factor,
    w_positive: calibration.w_positive.factor,
    w_negative: calibration.w_negative.factor,
  };

  const calibration_funcs = {
    u_positive: calibration.u_positive.func,
    u_negative: calibration.u_negative.func,
    v_positive: calibration.v_positive.func,
    v_negative: calibration.v_negative.func,
    w_positive: calibration.w_positive.func,
    w_negative: calibration.w_negative.func,
  };

  return {
    stats,
    samples,
    current_calibration,
    calibration_funcs,
  };
}

