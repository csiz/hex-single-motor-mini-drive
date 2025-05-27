import {interpolate_degrees, shortest_distance_degrees, normalize_degrees, radians_to_degrees} from "./angular_math.js";
import {interpolate_linear, matrix_multiply, exponential_averager, square, exponential_stats} from "./math_utils.js";
import {product_of_normals, add_stdev} from "./stats_utils.js";
import {online_map, online_function_chain} from "./data_utils.js";



// Calculate Data
// --------------


const stdev_99_z_score = 2.575829; // 99% confidence interval for normal distribution


function shortest_distance_mod_6(a, b){
  const diff = (b + 12 - a) % 6;
  return diff > 3 ? diff - 6 : diff;
}


function accumulate_position_from_hall(readout, prev_readout){
  const {sector_center_degrees, sector_center_stdev, sector_transition_degrees, sector_transition_stdev, angular_acceleration_stdev, initial_angular_speed_stdev} = this.position_calibration;
  
  const {time, hall_sector} = readout;

  if (!prev_readout) return {
    ...readout,
    web_angle: sector_center_degrees[hall_sector],
    web_angle_stdev: sector_center_stdev[hall_sector],
    web_angular_speed: 0,
    web_angular_speed_stdev: initial_angular_speed_stdev,

    obs_time: time,
    obs_sector: hall_sector,
    obs_angle: sector_center_degrees[hall_sector],
    obs_angle_stdev: sector_center_stdev[hall_sector],
    obs_angular_speed: 0,
    obs_angular_speed_stdev: initial_angular_speed_stdev,
  }
  
  const dt = time - prev_readout.obs_time;

  // If we switched sectors, we should have an accurate position estimate.
  if (hall_sector != null && hall_sector != prev_readout.hall_sector){
    const direction = Math.sign(shortest_distance_mod_6(prev_readout.hall_sector, hall_sector));

    const trigger_angle = sector_transition_degrees[hall_sector][direction >= 0 ? 0 : 1];
    const trigger_angle_stdev = sector_transition_stdev[hall_sector][direction >= 0 ? 0 : 1];

    const distance_to_trigger = shortest_distance_degrees(prev_readout.obs_angle, trigger_angle);

    const estimated_distance = prev_readout.obs_angular_speed * dt;
    const estimated_distance_error = distance_to_trigger - estimated_distance;
    const estimated_speed_error = estimated_distance_error / dt;

    const estimated_distance_stdev = add_stdev(prev_readout.obs_angle_stdev, prev_readout.obs_angular_speed_stdev * dt, angular_acceleration_stdev * dt * dt / 4.0);
    const estimated_speed_error_stdev = add_stdev(estimated_distance_stdev / dt, trigger_angle_stdev / dt);
    
    const {mean: distance_adjustment, stdev: web_angle_stdev} = product_of_normals({
      mean_a: 0.0,
      stdev_a: estimated_distance_stdev,
      mean_b: estimated_distance_error,
      stdev_b: trigger_angle_stdev,
    });

    const kalman_angle = normalize_degrees(prev_readout.obs_angle + estimated_distance + distance_adjustment);

    // Ensure we dragged the angle within the 95% confidence interval of the trigger angle. If we're too slow
    // to update the angle we cross 180 degrees and our math switches sign. To avoid that we need to keep the 
    // angle near the trigger. The formula below handles the lower bound of the trigger angle; the upper bound
    // is handled by capping the speed above.
    const web_angle = kalman_angle + direction * Math.max(direction * normalize_degrees(trigger_angle - direction * stdev_99_z_score * trigger_angle_stdev - kalman_angle), 0);

    const {mean: speed_adjustment, stdev: web_angular_speed_stdev} = product_of_normals({
      mean_a: 0.0,
      stdev_a: prev_readout.obs_angular_speed_stdev + 0.5 * angular_acceleration_stdev * dt,
      mean_b: estimated_speed_error,
      stdev_b: estimated_speed_error_stdev,
    });

    const web_angular_speed = prev_readout.obs_angular_speed + speed_adjustment;

    return {
      ...readout,
      web_angle,
      web_angle_stdev,
      web_angular_speed,
      web_angular_speed_stdev,

      obs_time: readout.time,
      obs_sector: hall_sector,
      obs_angle: web_angle,
      obs_angle_stdev: web_angle_stdev,
      obs_angular_speed: web_angular_speed,
      obs_angular_speed_stdev: web_angular_speed_stdev,
    };
  } else {
    // If we didn't switch sectors, then we need to carry on with our previous estimate, but adjusted...
    const positive_direction = prev_readout.obs_angular_speed >= 0;
    const direction = positive_direction ? +1 : -1;

    const estimated_distance = prev_readout.obs_angular_speed * dt;

    const estimated_distance_stdev = add_stdev(prev_readout.obs_angle_stdev, prev_readout.obs_angular_speed_stdev * dt, angular_acceleration_stdev * dt * dt / 2.0);

    const next_sector = positive_direction ? (prev_readout.obs_sector + 1) % 6 : (prev_readout.obs_sector - 1 + 6) % 6;
    const next_transition_angle = sector_transition_degrees[next_sector][positive_direction ? 0 : 1];
    const next_transition_angle_stdev = sector_transition_stdev[next_sector][positive_direction ? 0 : 1];
    // Cap the position in the 95% confidence interval before the next transition. We cross
    // this threshold when distance_to_next_transition is negative. Also the adjustement
    // depends on the direction of travel.
    const distance_to_next_transition = direction * shortest_distance_degrees(prev_readout.obs_angle, next_transition_angle) + stdev_99_z_score * next_transition_angle_stdev;
    
    const distance_overshoot = Math.max(direction * estimated_distance - distance_to_next_transition, 0);
    const distance_adjustment = direction * Math.min(distance_to_next_transition - direction * estimated_distance, 0);

    const estimated_angle = normalize_degrees(prev_readout.obs_angle + estimated_distance + distance_adjustment);


    const p_stopped_in_current_sector = distance_overshoot < 90 ? 0.0 : Math.min(1.0, (distance_overshoot - 90) / 45);

    const current_sector_angle = sector_center_degrees[prev_readout.obs_sector];
    const current_sector_angle_stdev = sector_center_stdev[prev_readout.obs_sector];

    const web_angle = interpolate_degrees(estimated_angle, current_sector_angle, p_stopped_in_current_sector);
    const web_angle_stdev = interpolate_linear(estimated_distance_stdev, current_sector_angle_stdev, p_stopped_in_current_sector);

    const calculated_spin = shortest_distance_degrees(prev_readout.obs_angle, web_angle) / dt;
    const calculated_spin_stdev = (web_angle_stdev + prev_readout.obs_angle_stdev) / dt;

    const {mean: web_angular_speed, stdev: web_angular_speed_stdev} = product_of_normals({
      mean_a: calculated_spin,
      stdev_a: calculated_spin_stdev,
      mean_b: prev_readout.obs_angular_speed,
      stdev_b: prev_readout.obs_angular_speed_stdev + angular_acceleration_stdev * dt,
    });

    if (p_stopped_in_current_sector >= 0.99){
      return {
        ...readout,
        web_angle,
        web_angle_stdev,
        web_angular_speed: 0,
        web_angular_speed_stdev: initial_angular_speed_stdev,

        obs_time: readout.time,
        obs_sector: hall_sector,
        obs_angle: web_angle,
        obs_angle_stdev: web_angle_stdev,
        obs_angular_speed: 0,
        obs_angular_speed_stdev: initial_angular_speed_stdev,
      };
    } else {
      
      return {
        ...readout,
        web_angle,
        web_angle_stdev,
        web_angular_speed,
        web_angular_speed_stdev,

        obs_time: prev_readout.obs_time,
        obs_sector: prev_readout.obs_sector,
        obs_angle: prev_readout.obs_angle,
        obs_angle_stdev: prev_readout.obs_angle_stdev,
        obs_angular_speed: prev_readout.obs_angular_speed,
        obs_angular_speed_stdev: prev_readout.obs_angular_speed_stdev,
      };
    }
  }
}



function compute_derivative_info(readout, previous_readout){
  if (!previous_readout) return readout;
  
  const {
    time,
    hall_sector, 
    angle, 
    web_current_angle_offset,
    web_emf_power,
    emf_voltage_angle,
    emf_voltage_magnitude, 
  } = readout;

  const {
    time: prev_time,
    hall_sector: prev_hall_sector, 
    angle: prev_angle,
    emf_voltage_angle: prev_emf_voltage_angle,
    angular_speed_from_emf_avg: prev_angular_speed_from_emf_avg,
    angular_speed_from_emf_stdev: prev_angular_speed_from_emf_stdev,
    emf_voltage_magnitude_avg: prev_emf_voltage_magnitude_avg,
    emf_voltage_magnitude_stdev: prev_emf_voltage_magnitude_stdev,
    angle_diff_to_emf_avg: prev_angle_diff_to_emf_avg,
    angle_diff_to_emf_stdev: prev_angle_diff_to_emf_stdev,
    web_current_angle_offset_avg: prev_web_current_angle_offset_avg,
    web_current_angle_offset_stdev: prev_web_current_angle_offset_stdev,
    web_emf_power_avg: prev_web_emf_power_avg,
    web_emf_power_stdev: prev_web_emf_power_stdev,
  } = previous_readout;

  // Time units are milliseconds.
  const dt = time - prev_time;

  const exp_stats = exponential_stats(dt, 0.5);


  const angular_speed_from_emf = shortest_distance_degrees(prev_emf_voltage_angle, emf_voltage_angle) / dt;

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

  const transition_correction = is_hall_transition ? shortest_distance_degrees(angle, prev_angle) : 0;

  const angle_from_emf = normalize_degrees(emf_voltage_angle + (readout.angular_speed >= 0 ? +90 : -90));

  const angle_diff_to_emf = shortest_distance_degrees(angle_from_emf, angle);

  const {average: angle_diff_to_emf_avg, stdev: angle_diff_to_emf_stdev} = exp_stats(
    angle_diff_to_emf,
    {
      average: normalize_degrees(prev_angle_diff_to_emf_avg - transition_correction),
      stdev: prev_angle_diff_to_emf_stdev,
    },
  );


  const {average: web_current_angle_offset_avg, stdev: web_current_angle_offset_stdev} = exp_stats(
    web_current_angle_offset, 
    {
      average: normalize_degrees(prev_web_current_angle_offset_avg + transition_correction),
      stdev: prev_web_current_angle_offset_stdev,
    },
  );


  const {average: web_emf_power_avg, stdev: web_emf_power_stdev} = exp_stats(
    web_emf_power,
    {
      average: prev_web_emf_power_avg,
      stdev: prev_web_emf_power_stdev,
    },
  );

  return {
    ...readout,
    web_current_angle_offset_avg, web_current_angle_offset_stdev,
    emf_voltage_magnitude_avg, emf_voltage_magnitude_stdev,
    is_hall_transition,
    angle_from_emf,
    angular_speed_from_emf, angular_speed_from_emf_avg, angular_speed_from_emf_stdev,
    angle_diff_to_emf, angle_diff_to_emf_avg, angle_diff_to_emf_stdev,
    web_emf_power_avg, web_emf_power_stdev,
  };
}

export const process_readout = online_function_chain(
  compute_derivative_info,
  accumulate_position_from_hall,
);
