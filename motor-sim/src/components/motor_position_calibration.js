import {pwm_base, cycles_per_millisecond, history_size} from "./motor_constants.js";
import {command_codes} from "./motor_interface.js";
import {wait} from "./async_utils.js";
import {circular_stats_degrees, interpolate_degrees, shortest_distance_degrees, positive_distance_degrees} from "./angular_math.js";
import * as d3 from "d3";


export async function run_position_calibration(motor_controller) {

  console.info("Position calibration starting");
  
  const drive_time = 200;
  const drive_timeout = Math.floor((drive_time + 300) * cycles_per_millisecond);

  const drive_strength = Math.floor(pwm_base * 2 / 10);
  const drive_options = {command_timeout: drive_timeout, command_pwm: drive_strength};

  const test_options = {command_timeout: 0, command_pwm: 0};

  await motor_controller.send_command({command: command_codes.SET_STATE_DRIVE_POS, ...drive_options});  
  await wait(drive_time);
  const drive_positive = await motor_controller.command_and_read(
    {command: command_codes.SET_STATE_TEST_GROUND_SHORT, ...test_options},
    {expected_messages: history_size});

  console.info("Drive positive done");

  await motor_controller.send_command({command: command_codes.SET_STATE_DRIVE_NEG, ...drive_options});
  await wait(drive_time);
  const drive_negative = await motor_controller.command_and_read(
    {command: command_codes.SET_STATE_TEST_GROUND_SHORT, ...test_options},
    {expected_messages: history_size});


  console.info("Drive negative done");

  console.info("Position calibration done");

  if (drive_positive.length != history_size) {
    console.error("Drive positive data is not valid", drive_positive);
    return;
  }
  if (drive_negative.length != history_size) {
    console.error("Drive negative data is not valid", drive_negative);
    return;
  }

  const position_calibration_data = {
    drive_positive,
    drive_negative,
  };

  return position_calibration_data;
}

// Store the inferred angle for every hall sensor switching event.
function extract_hall_transitions(data){
  return data.filter(d => d.is_hall_transition && d.speed != 0).map(d => ({
    time: d.time,
    angle: d.angle_if_breaking,
    transition: `drive_${d.speed > 0 ? "positive" : "negative"}_sector_${d.hall_sector}`,
  }));
}

export function compute_position_calibration(calibration_results){
  if (!calibration_results || calibration_results.length == 0) {
    return {
      transition_stats: [],
      sensor_spans: [],
      center_angles: [],
      sensor_locations: [],
    };
  }

  const hall_transitions = calibration_results.flatMap((calibration_data) => {
    return [
      ...extract_hall_transitions(calibration_data.drive_positive),
      ...extract_hall_transitions(calibration_data.drive_negative),
    ];
  });
  

  const transition_angles = hall_transitions.reduce((result, d) => {
    let transitions = result[d.transition] ?? [];
    transitions.push(d.angle);
    result[d.transition] = transitions;
    return result;
  }, {});

  const transition_stats = Object.fromEntries(Object.entries(transition_angles).map(([transition, values]) => {
        const {mean, stdev} = circular_stats_degrees(values);

    return [transition, {transition, n: values.length, mean, stdev}];
  }));

  const center_angles = Object.fromEntries(d3.range(0, 6).map(sector => {
    const pos_transition = transition_stats[`drive_positive_sector_${sector}`]?.mean;
    const neg_transition = transition_stats[`drive_negative_sector_${sector}`]?.mean;
    const center = interpolate_degrees(pos_transition, neg_transition, 0.5);
    const stdev = positive_distance_degrees(pos_transition, neg_transition) / 2;
    const hall_sector = `sector_${sector}`;
    return [hall_sector, {hall_sector, pos_transition, neg_transition, center, stdev}];
  }));


  const sensor_spans = Object.fromEntries([
    ["drive_positive_hall_u", "drive_positive_sector_5", "drive_positive_sector_2"],
    ["drive_negative_hall_u", "drive_negative_sector_4", "drive_negative_sector_1"],
    ["drive_positive_hall_v", "drive_positive_sector_1", "drive_positive_sector_4"],
    ["drive_negative_hall_v", "drive_negative_sector_0", "drive_negative_sector_3"],
    ["drive_positive_hall_w", "drive_positive_sector_3", "drive_positive_sector_0"],
    ["drive_negative_hall_w", "drive_negative_sector_2", "drive_negative_sector_5"],
  ].map(([hall_transition, left_trigger, right_trigger]) => {
    const left_transition = transition_stats[left_trigger]?.mean;
    const right_transition = transition_stats[right_trigger]?.mean;

    const location = interpolate_degrees(left_transition, right_transition, 0.5);
    const hall_on_span = positive_distance_degrees(left_transition, right_transition);
    const sensing_overshoot = hall_on_span - 180;
    return [hall_transition, {hall_transition, left_transition, right_transition, location, hall_on_span, sensing_overshoot}];
  }));

  const sensor_locations = Object.fromEntries([
    ["hall_u", "drive_negative_hall_u", "drive_positive_hall_u"],
    ["hall_v", "drive_negative_hall_v", "drive_positive_hall_v"],
    ["hall_w", "drive_negative_hall_w", "drive_positive_hall_w"],
  ].map(([hall_sensor, neg_transition, pos_transition]) => {
    const neg_sensor_span = sensor_spans[neg_transition];
    const pos_sensor_span = sensor_spans[pos_transition];
    const hysterisis = positive_distance_degrees(neg_sensor_span.location, pos_sensor_span.location);
    const mid_location = interpolate_degrees(neg_sensor_span.location, pos_sensor_span.location, 0.5);

    return [hall_sensor, {hall_sensor, hysterisis, mid_location}];
  }));

  return {
    transition_stats,
    center_angles,
    sensor_spans,
    sensor_locations,
  };
}