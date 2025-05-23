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
  const current_calibration_data = d3.range(history_size).map((i) => {
    const time = i * millis_per_cycle;
    const zones = current_calibration_zones.filter((zone) => zone.settle_start <= time && time <= zone.settle_end);

    if (zones.length > 1) throw new Error("Multiple zones found");
    
    let result = {
      time,
      u_positive: +u_positive[i].u_uncalibrated,
      u_positive_expected: +u_positive[i].u_drive_voltage / phase_resistance,
      u_negative: -u_negative[i].u_uncalibrated,
      u_negative_expected: -u_negative[i].u_drive_voltage / phase_resistance,
      v_positive: +v_positive[i].v_uncalibrated,
      v_positive_expected: +v_positive[i].v_drive_voltage / phase_resistance,
      v_negative: -v_negative[i].v_uncalibrated,
      v_negative_expected: -v_negative[i].v_drive_voltage / phase_resistance,
      w_positive: +w_positive[i].w_uncalibrated,
      w_positive_expected: +w_positive[i].w_drive_voltage / phase_resistance,
      w_negative: -w_negative[i].w_uncalibrated,
      w_negative_expected: -w_negative[i].w_drive_voltage / phase_resistance,
    };

    const expected = zones.length == 0 ? null : d3.mean(Object.values(_.pick(
        result, 
        "u_positive_expected", "u_negative_expected", 
        "v_positive_expected", "v_negative_expected", 
        "w_positive_expected", "w_negative_expected",
      )));
    result.expected = expected;

    return result;
  });

  console.info("Current calibration done", current_calibration_data);

  return current_calibration_data;
}

function almost_equal(a, b){
  return Math.abs(a - b) < 0.000_000_1;
}

function compute_calibration_instance(calibration_data){
  
  const settled_zones = current_calibration_zones.map((zone) => {
    return calibration_data.filter((d) => d.time > zone.settle_start && d.time < zone.settle_end);
  });

  const transition_zones = current_calibration_zones.map((zone) => {
    return calibration_data.filter((d) => d.time > zone.start && d.time < zone.settle_start);
  });

  function compute_phase_calibration(phase_selector){
    const zone_stats = settled_zones.map((zone_data) => {
      const settled_current = d3.mean(zone_data, (d) => d[phase_selector]);
      const expected_current = d3.mean(zone_data, (d) => d[`${phase_selector}_expected`]);
      const adjustment_factor = expected_current / settled_current;
      return {
        settled_current,
        expected_current,
        adjustment_factor,
      };
    });

    const factor = d3.mean(zone_stats, (d) => d.adjustment_factor);

    const transition_data = transition_zones.map((zone_data, zone_i) => {
      if (zone_data.length == 0) throw new Error("No transition data found");

      const expected_change = zone_stats[zone_i].expected_current - (zone_i === 0 ? 0.0 : zone_stats[zone_i - 1].expected_current);
      const adjustment_factor = zone_stats[zone_i].adjustment_factor;

      const start_time = zone_data[0].time;

      return zone_data.map((d) => {
        const actual = d[phase_selector] * adjustment_factor;
        const expected = d[`${phase_selector}_expected`];
        const diff_to_expected = actual - expected;
        return {
          relative_diff: zone_i >= 6 ? null : diff_to_expected / expected_change,
          time: d.time - start_time,
          actual,
          expected,
        };
      });
    });

    const transition_stats = transition_times.map((time, time_i) => {
      // Ensure all times match up.
      transition_data.forEach((zone_data) => {
        if (!almost_equal(zone_data[time_i]?.time, time)) throw new Error(`Transition time mismatch: ${zone_data[time_i]?.time} != ${time}`);
      });

      return {
        time,
        relative_diff: d3.mean(transition_data, (zone_data) => zone_data[time_i]?.relative_diff),
        relative_diff_stdev: d3.deviation(transition_data, (zone_data) => zone_data[time_i]?.relative_diff),
      };
    });

    const slow_func = piecewise_linear({
      X: [0.0, ...zone_stats.map((d) => d.settled_current)],
      Y: [0.0, ...zone_stats.map((d) => d.expected_current)],
    });

    // Recalibrate to evenly spaced points for fast processing.
    const func = even_piecewise_linear({
      x_min: 0, x_max: max_calibration_current,
      Y: current_calibration_reading_points.map(slow_func),
    });

    const sample = current_calibration_reading_points.map((x) => ({
      reading: x,
      calibrated: func(x),
      secondary_factor: x == 0 ? 1 : func(x) / (factor * x),
    }));

    return {
      zone_stats,
      transition_data,
      transition_stats,
      factor,
      func,
      sample,
    };
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
      expected: d3.mean(valid_calibration_results, (data) => data[i].expected),
      u_positive: d3.mean(valid_calibration_results, (data) => data[i].u_positive),
      u_positive_expected: d3.mean(valid_calibration_results, (data) => data[i].u_positive_expected),
      u_positive_stdev: d3.deviation(valid_calibration_results, (data) => data[i].u_positive),
      u_negative: d3.mean(valid_calibration_results, (data) => data[i].u_negative),
      u_negative_expected: d3.mean(valid_calibration_results, (data) => data[i].u_negative_expected),
      u_negative_stdev: d3.deviation(valid_calibration_results, (data) => data[i].u_negative),
      v_positive: d3.mean(valid_calibration_results, (data) => data[i].v_positive),
      v_positive_expected: d3.mean(valid_calibration_results, (data) => data[i].v_positive_expected),
      v_positive_stdev: d3.deviation(valid_calibration_results, (data) => data[i].v_positive),
      v_negative: d3.mean(valid_calibration_results, (data) => data[i].v_negative),
      v_negative_expected: d3.mean(valid_calibration_results, (data) => data[i].v_negative_expected),
      v_negative_stdev: d3.deviation(valid_calibration_results, (data) => data[i].v_negative),
      w_positive: d3.mean(valid_calibration_results, (data) => data[i].w_positive),
      w_positive_expected: d3.mean(valid_calibration_results, (data) => data[i].w_positive_expected),
      w_positive_stdev: d3.deviation(valid_calibration_results, (data) => data[i].w_positive),
      w_negative: d3.mean(valid_calibration_results, (data) => data[i].w_negative),
      w_negative_expected: d3.mean(valid_calibration_results, (data) => data[i].w_negative_expected),
      w_negative_stdev: d3.deviation(valid_calibration_results, (data) => data[i].w_negative),
    };
  });

  const calibration = compute_calibration_instance(stats);

  const secondary_factors = current_calibration_reading_points.map((x, i) => {
    return {
      reading: x,
      expected: 1.0,
      u_positive: calibration.u_positive.sample[i].secondary_factor,
      u_negative: calibration.u_negative.sample[i].secondary_factor,
      v_positive: calibration.v_positive.sample[i].secondary_factor,
      v_negative: calibration.v_negative.sample[i].secondary_factor,
      w_positive: calibration.w_positive.sample[i].secondary_factor,
      w_negative: calibration.w_negative.sample[i].secondary_factor,
    };
  });
  
  const current_calibration = {
    u_factor: (calibration.u_positive.factor + calibration.u_negative.factor) / 2,
    v_factor: (calibration.v_positive.factor + calibration.v_negative.factor) / 2,
    w_factor: (calibration.w_positive.factor + calibration.w_negative.factor) / 2,
  };

  const calibration_funcs = {
    u_positive: calibration.u_positive.func,
    u_negative: calibration.u_negative.func,
    v_positive: calibration.v_positive.func,
    v_negative: calibration.v_negative.func,
    w_positive: calibration.w_positive.func,
    w_negative: calibration.w_negative.func,
  };

  const transition_stats = transition_times.map((time, i) => {
    return {
      time,
      u_positive: calibration.u_positive.transition_stats[i].relative_diff,
      u_positive_stdev: calibration.u_positive.transition_stats[i].relative_diff_stdev,
      u_positive_by_zone: calibration.u_positive.transition_data.map((zone_data) => zone_data[i].relative_diff),
      u_negative: calibration.u_negative.transition_stats[i].relative_diff,
      u_negative_stdev: calibration.u_negative.transition_stats[i].relative_diff_stdev,
      u_negative_by_zone: calibration.u_negative.transition_data.map((zone_data) => zone_data[i].relative_diff),
      v_positive: calibration.v_positive.transition_stats[i].relative_diff,
      v_positive_stdev: calibration.v_positive.transition_stats[i].relative_diff_stdev,
      v_positive_by_zone: calibration.v_positive.transition_data.map((zone_data) => zone_data[i].relative_diff),
      v_negative: calibration.v_negative.transition_stats[i].relative_diff,
      v_negative_stdev: calibration.v_negative.transition_stats[i].relative_diff_stdev,
      v_negative_by_zone: calibration.v_negative.transition_data.map((zone_data) => zone_data[i].relative_diff),
      w_positive: calibration.w_positive.transition_stats[i].relative_diff,
      w_positive_stdev: calibration.w_positive.transition_stats[i].relative_diff_stdev,
      w_positive_by_zone: calibration.w_positive.transition_data.map((zone_data) => zone_data[i].relative_diff),
      w_negative: calibration.w_negative.transition_stats[i].relative_diff,
      w_negative_stdev: calibration.w_negative.transition_stats[i].relative_diff_stdev,
      w_negative_by_zone: calibration.w_negative.transition_data.map((zone_data) => zone_data[i].relative_diff),
    };
  });

  return {
    stats,
    secondary_factors,
    current_calibration,
    calibration_funcs,
    transition_stats,
  };
}

