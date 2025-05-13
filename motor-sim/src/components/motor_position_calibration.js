import {pwm_base, cycles_per_millisecond, history_size} from "./motor_constants.js";
import {command_codes} from "./motor_interface.js";
import {wait} from "./async_utils.js";
import {circular_stats_degrees, interpolate_degrees, shortest_distance_degrees} from "./angular_math.js";

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
function extract_hall_switching_events(data){
  return data.flatMap((d, i) => {
    if (d.angle_if_breaking == null) return [];
    if (i == 0) return [];

    const prev = data[i - 1];

    const prev_sector = prev.hall_sector;
    const next_sector = d.hall_sector;

    if (prev_sector == null || next_sector == null) return [];

    if (prev_sector == next_sector) return [];

    return [{time: d.time, angle_if_breaking: d.angle_if_breaking, switching: `hall_sector_${prev_sector}_to_${next_sector}`}];

  });
}

export function compute_position_calibration(calibration_results){

  const switching_events = calibration_results.flatMap((calibration_data) => {
    return [
      ...extract_hall_switching_events(calibration_data.drive_positive),
      ...extract_hall_switching_events(calibration_data.drive_negative),
    ];
  });

  const switching_angles = switching_events.reduce((acc, d) => {
    acc[d.switching]?.push(d.angle_if_breaking);
    return acc;
  }, {
    hall_sector_5_to_0: [],
    hall_sector_0_to_1: [],
    hall_sector_1_to_2: [],
    hall_sector_2_to_3: [],
    hall_sector_3_to_4: [],
    hall_sector_4_to_5: [],

    hall_sector_1_to_0: [], 
    hall_sector_2_to_1: [],
    hall_sector_3_to_2: [],
    hall_sector_4_to_3: [],
    hall_sector_5_to_4: [],
    hall_sector_0_to_5: [],
  });

  const stats = Object.entries(switching_angles).map(([key, values]) => {
    const {circular_mean: mean, circular_std: std} = circular_stats_degrees(values);
    return {key, n: values.length, mean, std};
  });

  const transitions = [
    ["drive_positive_hall_u", "hall_sector_4_to_5", "hall_sector_1_to_2"],
    ["drive_negative_hall_u", "hall_sector_5_to_4", "hall_sector_2_to_1"],
    ["drive_positive_hall_v", "hall_sector_0_to_1", "hall_sector_3_to_4"],
    ["drive_negative_hall_v", "hall_sector_1_to_0", "hall_sector_4_to_3"],
    ["drive_positive_hall_w", "hall_sector_2_to_3", "hall_sector_5_to_0"],
    ["drive_negative_hall_w", "hall_sector_3_to_2", "hall_sector_0_to_5"],
  ].map(([key, a_key, b_key]) => {
    const a_stats = stats.find(d => d.key === a_key);
    const b_stats = stats.find(d => d.key === b_key);

    const a = a_stats.mean;
    const b = b_stats.mean;

    const diff = (b - a + 360) % 360;
    const location = interpolate_degrees(a, b, 0.5);
    const err = diff - 180;
    return {key, a, b, diff, location, err};
  });

  const hysterisis = [
    ["hysterisis_hall_u", "drive_negative_hall_u", "drive_positive_hall_u"],
    ["hysterisis_hall_v", "drive_negative_hall_v", "drive_positive_hall_v"],
    ["hysterisis_hall_w", "drive_negative_hall_w", "drive_positive_hall_w"],
  ].map(([key, a_key, b_key]) => {
    const a_stats = transitions.find(d => d.key === a_key);
    const b_stats = transitions.find(d => d.key === b_key);
    const hysterisis = shortest_distance_degrees(a_stats.location, b_stats.location);
    const mid_location = interpolate_degrees(a_stats.location, b_stats.location, 0.5);

    return {key, hysterisis, mid_location};
  });

  return {
    stats,
    transitions,
    hysterisis,
  };
}