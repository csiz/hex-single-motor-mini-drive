import {interpolate_degrees, shortest_distance_degrees, normalize_degrees, radians_to_degrees} from "./angular_math.js";
import {interpolate_linear, matrix_multiply, exponential_averager, square, exponential_stats} from "./math_utils.js";

import {phase_inductance, phase_resistance} from "./motor_constants.js";


export function online_map(array, online_fn){
  if (array.length == 0) return [];

  const result = new Array(array.length);
  let previous = online_fn(array[0], undefined);
  result[0] = previous;
  for (let i = 1; i < array.length; i++){
    previous = online_fn(array[i], previous);
    result[i] = previous;
  }
  return result;
}

export function online_function_chain(...online_functions){
  return function(value, previous){
    for (const fn of online_functions){
      value = fn.call(this, value, previous);
    }
    return value;
  };
}

// Calculate Data
// --------------


const stdev_99_z_score = 2.575829; // 99% confidence interval for normal distribution

const power_invariant_clarke_matrix = [
  [Math.sqrt(2/3), -0.5 * Math.sqrt(2/3), -0.5 * Math.sqrt(2/3)],
  [0, Math.sqrt(2)/2, -Math.sqrt(2)/2],
  [1/Math.sqrt(3), 1/Math.sqrt(3), 1/Math.sqrt(3)],
];

const power_invariant_simplified_clarke_matrix = power_invariant_clarke_matrix.slice(0, 2);

function clarke_transform(u, v, w){
  return matrix_multiply(power_invariant_simplified_clarke_matrix, [u, v, w]);
}

function shortest_distance_mod_6(a, b){
  const diff = (b + 12 - a) % 6;
  return diff > 3 ? diff - 6 : diff;
}

function product_of_normals({mean_a, std_a, mean_b, std_b}){
  const mean = (mean_a * std_b * std_b + mean_b * std_a * std_a) / (std_a * std_a + std_b * std_b);
  const stdev = Math.sqrt((std_a * std_a * std_b * std_b) / (std_a * std_a + std_b * std_b));
  return {mean, stdev};
}

function product_of_normals_by_variance({mean_a, variance_a, mean_b, variance_b}){
  const mean = (mean_a * variance_b + mean_b * variance_a) / (variance_a + variance_b);
  const variance = (variance_a * variance_b) / (variance_a + variance_b);
  return {mean, variance};
}

function add_stdev(...std_values){
  return Math.sqrt(std_values.reduce((sum, stdev) => sum + stdev * stdev, 0));
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
      std_a: estimated_distance_stdev,
      mean_b: estimated_distance_error,
      std_b: trigger_angle_stdev,
    });

    const kalman_angle = normalize_degrees(prev_readout.obs_angle + estimated_distance + distance_adjustment);

    // Ensure we dragged the angle within the 95% confidence interval of the trigger angle. If we're too slow
    // to update the angle we cross 180 degrees and our math switches sign. To avoid that we need to keep the 
    // angle near the trigger. The formula below handles the lower bound of the trigger angle; the upper bound
    // is handled by capping the speed above.
    const web_angle = kalman_angle + direction * Math.max(direction * normalize_degrees(trigger_angle - direction * stdev_99_z_score * trigger_angle_stdev - kalman_angle), 0);

    const {mean: speed_adjustment, stdev: web_angular_speed_stdev} = product_of_normals({
      mean_a: 0.0,
      std_a: prev_readout.obs_angular_speed_stdev + 0.5 * angular_acceleration_stdev * dt,
      mean_b: estimated_speed_error,
      std_b: estimated_speed_error_stdev,
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
      std_a: calculated_spin_stdev,
      mean_b: prev_readout.obs_angular_speed,
      std_b: prev_readout.obs_angular_speed_stdev + angular_acceleration_stdev * dt,
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
  const {u, v, w, hall_sector, angle, u_drive_voltage, v_drive_voltage, w_drive_voltage} = readout;

  const [current_alpha, current_beta] = clarke_transform(u, v, w);

  const current_angle = radians_to_degrees(Math.atan2(current_beta, current_alpha));
  const current_magnitude = Math.sqrt(current_alpha * current_alpha + current_beta * current_beta);
  
  if (!previous_readout) return readout;

  const {
    u: prev_u, v: prev_v, w: prev_w, 
    hall_sector: prev_hall_sector, 
    voltage_angle: prev_voltage_angle,
  } = previous_readout;

  // Time units are milliseconds.
  const dt = readout.time - previous_readout.time;

  const exp_stats = exponential_stats(dt, 0.5);

  // V = L*dI/dt + R*I; Also factor of 1000 for millisecond to second conversion.
  const u_L_voltage = (u - prev_u) / dt * 1000 * phase_inductance;
  const v_L_voltage = (v - prev_v) / dt * 1000 * phase_inductance;
  const w_L_voltage = (w - prev_w) / dt * 1000 * phase_inductance;

  const u_voltage = u_L_voltage + phase_resistance * u - u_drive_voltage;
  const v_voltage = v_L_voltage + phase_resistance * v - v_drive_voltage;
  const w_voltage = w_L_voltage + phase_resistance * w - w_drive_voltage;

  const [voltage_alpha, voltage_beta] = clarke_transform(u_voltage, v_voltage, w_voltage);

  const voltage_angle = radians_to_degrees(Math.atan2(voltage_beta, voltage_alpha));

  const angular_speed_from_emf = normalize_degrees(voltage_angle - prev_voltage_angle) / dt;
  
  const voltage_magnitude = Math.sqrt(voltage_alpha * voltage_alpha + voltage_beta * voltage_beta);

  const is_hall_transition = prev_hall_sector != hall_sector;

  const angle_from_emf = normalize_degrees(voltage_angle + (readout.angular_speed >= 0 ? +90 : -90));

  const angle_diff_to_emf = shortest_distance_degrees(angle_from_emf, angle);

  const {average: angle_diff_to_emf_avg, stdev: angle_diff_to_emf_stdev} = exp_stats(
    angle_diff_to_emf, 
    {
      average: previous_readout.angle_diff_to_emf_avg, 
      stdev: previous_readout.angle_diff_to_emf_stdev,
    },
  );

  return {
    ...readout,
    current_alpha, current_beta,
    current_angle, current_magnitude,
    u_voltage, v_voltage, w_voltage,
    u_L_voltage, v_L_voltage, w_L_voltage,
    voltage_alpha, voltage_beta,
    voltage_angle, voltage_magnitude,
    angle_from_emf, angular_speed_from_emf,
    angle_diff_to_emf,
    angle_diff_to_emf_avg, angle_diff_to_emf_stdev,
    is_hall_transition,
  };
}

export const process_readout = online_function_chain(
  compute_derivative_info,
  accumulate_position_from_hall,
);
