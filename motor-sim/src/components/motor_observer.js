import {normalize_degrees} from "./angular_math.js";
import {exponential_averager, square, exponential_stats, valid_number} from "./math_utils.js";
import {product_of_normals, add_stdev, weighted_product_of_normals, approx_cdf_normal} from "./stats_utils.js";
import {online_map, online_function_chain} from "./data_utils.js";
import {minimum_acceleration} from "./motor_constants.js";


// Calculate Data
// --------------



function hall_transition_direction(a, b){
  return Math.sign((b + 9 - a) % 6 - 3);
}

function diff_without_zero_crossing(x, dx){
  const result = x - dx;
  return result * x <= 0 ? 0 : result;
}


function accumulate_position_from_hall(readout, prev_readout){
  const {
    sector_center_degrees, sector_center_stdev, 
    sector_transition_degrees, sector_transition_stdev,
    angular_acceleration_stdev, initial_angular_speed_stdev,
  } = this.position_calibration;
  
  const {time, hall_sector, is_hall_transition} = readout;

  if (!valid_number(hall_sector)) return readout;

  if (!prev_readout || !valid_number(prev_readout.hall_sector)) return {
    ...readout,
    web_angle: sector_center_degrees[hall_sector],
    web_angle_stdev: sector_center_stdev[hall_sector],
    web_angular_speed: 0,
    web_angular_speed_stdev: initial_angular_speed_stdev,
    web_angular_acceleration: 0,
  }

  const {
    time: prev_time,
    hall_sector: prev_hall_sector,
    web_angle: prev_web_angle,
    web_angle_stdev: prev_web_angle_stdev,
    web_angular_speed: prev_web_angular_speed,
    web_angular_speed_stdev: prev_web_angular_speed_stdev,
    web_angular_acceleration_avg: prev_web_angular_acceleration_avg,
    web_angular_acceleration_stdev: prev_web_angular_acceleration_stdev,
  } = prev_readout;
  
  const dt = time - prev_time;

  const direction = is_hall_transition ? hall_transition_direction(prev_hall_sector, hall_sector) : Math.sign(prev_web_angular_speed);


  // We want to avoid arithmetic around the angle wrap around points. Change coordinates
  // so our center lies on the predicted angle. Thus the predicted angle becomes 0 degrees
  // and all other angles are (signed) distances relative to it.

  const predicted_angular_speed = diff_without_zero_crossing(prev_web_angular_speed, Math.sign(prev_web_angular_speed) * minimum_acceleration * dt);

  const predicted_distance = predicted_angular_speed * dt;

  const predicted_distance_stdev = add_stdev(
    prev_web_angle_stdev,
    prev_web_angular_speed_stdev * dt,
    angular_acceleration_stdev * dt * dt / 2.0,
  );

  const predicted_speed_stdev = add_stdev(
    prev_web_angular_speed_stdev,
    angular_acceleration_stdev * dt,
  );

  const predicted_angle = normalize_degrees(prev_web_angle + predicted_distance);

  const next_sector = direction >= 0 ? (hall_sector + 1) % 6 : (hall_sector - 1 + 6) % 6;
  
  const {hall_angle, hall_stdev} = is_hall_transition ? {
    hall_angle: sector_transition_degrees[hall_sector][direction >= 0 ? 0 : 1],
    // The transition could have occured at any point in previous timestep.
    hall_stdev: sector_transition_stdev[hall_sector][direction >= 0 ? 0 : 1] + Math.abs(predicted_distance),
  } : {
    hall_angle: sector_transition_degrees[next_sector][direction >= 0 ? 0 : 1],
    hall_stdev: sector_transition_stdev[next_sector][direction >= 0 ? 0 : 1],
  };


  const distance_to_hall_angle = normalize_degrees(hall_angle - predicted_angle);

  const distance_to_hall_angle_stdev = add_stdev(hall_stdev, predicted_distance_stdev);

  const evidence = is_hall_transition ? 1.0 : approx_cdf_normal(
    predicted_angular_speed >= 0 ? -distance_to_hall_angle : +distance_to_hall_angle,
    0,
    distance_to_hall_angle_stdev,
  );


  const distance_to_hall_center = normalize_degrees(sector_center_degrees[hall_sector] - predicted_angle);
  const distance_to_hall_center_stdev = sector_center_stdev[hall_sector];


  const predicted_distance_error = is_hall_transition ? distance_to_hall_angle : distance_to_hall_center;
  const predicted_distance_error_stdev = is_hall_transition ? hall_stdev : distance_to_hall_center_stdev;

  const predicted_speed_error = predicted_distance_error / dt;
  const predicted_speed_error_stdev = add_stdev(predicted_speed_stdev, predicted_distance_error_stdev / dt);

  const {mean: distance_adjustment, stdev: web_angle_stdev} = weighted_product_of_normals({
    mean_a: 0.0,
    stdev_a: predicted_distance_stdev,
    weight_a: 1.0,
    mean_b: predicted_distance_error,
    stdev_b: predicted_distance_error_stdev,
    weight_b: evidence,
  });

  const web_angle = normalize_degrees(predicted_angle + distance_adjustment);


  const {mean: speed_adjustment, stdev: web_angular_speed_stdev} = weighted_product_of_normals({
    mean_a: 0,
    stdev_a: predicted_speed_stdev,
    weight_a: 1.0,
    mean_b: predicted_speed_error,
    stdev_b: predicted_speed_error_stdev,
    weight_b: evidence,
  });

  const web_angular_speed = predicted_angular_speed + speed_adjustment;

  const web_angular_acceleration = speed_adjustment / dt;

  const {average: web_angular_acceleration_avg, stdev: web_angular_acceleration_stdev} = exponential_stats(dt, 2.0)(
    web_angular_acceleration, {average: prev_web_angular_acceleration_avg, stdev: prev_web_angular_acceleration_stdev},
  );

  return {
    ...readout,
    web_angle,
    web_angle_stdev: Math.min(web_angle_stdev, distance_to_hall_center_stdev),
    web_angular_speed,
    web_angular_speed_stdev: Math.min(web_angular_speed_stdev, initial_angular_speed_stdev),
    web_angular_acceleration, web_angular_acceleration_avg, web_angular_acceleration_stdev,
  }
}


function compute_derivative_info(readout, previous_readout){
  if (!previous_readout) return {
    ...readout,
    direction: 0,
  };

  const {
    time,
    hall_sector, 
    angle, 
    current_angle_offset,
    emf_voltage_angle,
    emf_voltage_magnitude, 
    web_emf_power,
    web_total_power,
    emf_detected,
    emf_direction_negative,
  } = readout;

  const {
    time: prev_time,
    emf_detected: prev_emf_detected,
    hall_sector: prev_hall_sector, 
    emf_voltage_angle: prev_emf_voltage_angle,
    angular_speed_from_emf_avg: prev_angular_speed_from_emf_avg,
    angular_speed_from_emf_stdev: prev_angular_speed_from_emf_stdev,
    emf_voltage_magnitude_avg: prev_emf_voltage_magnitude_avg,
    emf_voltage_magnitude_stdev: prev_emf_voltage_magnitude_stdev,
    angle_diff_to_emf_avg: prev_angle_diff_to_emf_avg,
    angle_diff_to_emf_stdev: prev_angle_diff_to_emf_stdev,
    current_angle_offset_avg: prev_current_angle_offset_avg,
    current_angle_offset_stdev: prev_current_angle_offset_stdev,
    web_emf_power_avg: prev_web_emf_power_avg,
    web_emf_power_stdev: prev_web_emf_power_stdev,
    web_total_power_avg: prev_web_total_power_avg,
    web_total_power_stdev: prev_web_total_power_stdev,
  } = previous_readout;

  // Time units are milliseconds.
  const dt = time - prev_time;

  const exp_stats = exponential_stats(dt, 0.350);


  const angular_speed_from_emf = prev_emf_detected ? normalize_degrees(emf_voltage_angle - prev_emf_voltage_angle) / dt : 0;

  const {average: angular_speed_from_emf_avg, stdev: angular_speed_from_emf_stdev} = exp_stats(
    angular_speed_from_emf, 
    {
      average: prev_angular_speed_from_emf_avg, 
      stdev: prev_angular_speed_from_emf_stdev,
    },
  );
  

  const {average: emf_voltage_magnitude_avg, stdev: emf_voltage_magnitude_stdev} = exp_stats(
    emf_voltage_magnitude,
    {
      average: prev_emf_voltage_magnitude_avg,
      stdev: prev_emf_voltage_magnitude_stdev,
    },
  );

  const is_hall_transition = prev_hall_sector != hall_sector;

  const direction = (
    is_hall_transition ? hall_transition_direction(prev_hall_sector, hall_sector) : 
    emf_detected ? (emf_direction_negative ? -1 : +1) :
    0
  );

  const angle_from_emf = normalize_degrees(emf_voltage_angle + (direction * 90));

  const angle_diff_to_emf = normalize_degrees(angle - angle_from_emf);

  const {average: angle_diff_to_emf_avg, stdev: angle_diff_to_emf_stdev} = exp_stats(
    angle_diff_to_emf,
    {
      average: normalize_degrees(prev_angle_diff_to_emf_avg),
      stdev: prev_angle_diff_to_emf_stdev,
    },
  );


  const {average: current_angle_offset_avg, stdev: current_angle_offset_stdev} = exp_stats(
    current_angle_offset, 
    {
      average: normalize_degrees(prev_current_angle_offset_avg),
      stdev: prev_current_angle_offset_stdev,
    },
  );


  const {average: web_emf_power_avg, stdev: web_emf_power_stdev} = exp_stats(
    web_emf_power,
    {
      average: prev_web_emf_power_avg,
      stdev: prev_web_emf_power_stdev,
    },
  );

  const {average: web_total_power_avg, stdev: web_total_power_stdev} = exp_stats(
    web_total_power,
    {
      average: prev_web_total_power_avg,
      stdev: prev_web_total_power_stdev,
    },
  );


    

  return {
    ...readout,
    current_angle_offset_avg, current_angle_offset_stdev,
    emf_voltage_magnitude_avg, emf_voltage_magnitude_stdev,
    is_hall_transition,
    direction,
    angle_from_emf,
    angular_speed_from_emf, angular_speed_from_emf_avg, angular_speed_from_emf_stdev,
    angle_diff_to_emf, angle_diff_to_emf_avg, angle_diff_to_emf_stdev,
    web_emf_power_avg, web_emf_power_stdev,
    web_total_power_avg, web_total_power_stdev,
  };
}

export const process_readout = online_function_chain(
  compute_derivative_info,
  accumulate_position_from_hall,
);
