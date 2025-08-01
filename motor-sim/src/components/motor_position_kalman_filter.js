import {normalize_degrees} from "./angular_math.js";
import {product_of_normals, add_stdev, weighted_product_of_normals, cdf_normal} from "./stats_utils.js";

// Calculate Data
// --------------

function hall_transition_direction(a, b){
  return Math.sign((b + 9 - a) % 6 - 3);
}

function diff_without_zero_crossing(x, dx){
  const result = x - dx;
  return result * x <= 0 ? 0 : result;
}

// Our expectation for the angular acceleration variance.
const angular_acceleration_stdev = 0.1;

const initial_angular_speed_stdev = 50.0;

const minimum_acceleration = 0.1;

export function accumulate_position_from_hall(readout, prev_readout, position_calibration){
  const {
    sector_center_degrees, 
    sector_center_stdev, 
    sector_transition_degrees, 
    sector_transition_stdev,
  } = position_calibration;

  const {time, hall_sector, is_hall_transition} = readout;

  if (hall_sector == null) return readout;

  if (!prev_readout || prev_readout.hall_sector == null) return {
    ...readout,
    web_angle: sector_center_degrees[hall_sector],
    web_angle_stdev: sector_center_stdev[hall_sector],
    web_angular_speed: 0,
    web_angular_speed_stdev: initial_angular_speed_stdev,
  }

  const {
    time: prev_time,
    hall_sector: prev_hall_sector,
    web_angle: prev_web_angle,
    web_angle_stdev: prev_web_angle_stdev,
    web_angular_speed: prev_web_angular_speed,
    web_angular_speed_stdev: prev_web_angular_speed_stdev,
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

  const evidence = is_hall_transition ? 1.0 : cdf_normal(
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

  return {
    ...readout,
    web_angle,
    web_angle_stdev: Math.min(web_angle_stdev, distance_to_hall_center_stdev),
    web_angular_speed,
    web_angular_speed_stdev: Math.min(web_angular_speed_stdev, initial_angular_speed_stdev),
  }
}