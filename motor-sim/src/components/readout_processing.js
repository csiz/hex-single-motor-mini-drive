import {position_calibration_default} from "./motor_constants.js";

import {interpolate_degrees, shortest_distance_degrees, normalize_degrees, radians_to_degrees} from "./angular_math.js";
import {interpolate_linear, matrix_multiply} from "./math_utils.js";




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
      value = fn(value, previous);
    }
    return value;
  };
}

// Calculate Data
// --------------

const {sector_center_degrees, sector_center_std, sector_transition_degrees, sector_transition_std, accel_std, initial_angular_speed_std} = position_calibration_default;

const phase_inductance = 0.000_145; // 290 uH measured with LCR meter across phase pairs.
const phase_resistance = 2.0; // 2.0 Ohm


const std_99_z_score = 2.575829; // 99% confidence interval for normal distribution

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
  const std = Math.sqrt((std_a * std_a * std_b * std_b) / (std_a * std_a + std_b * std_b));
  return {mean, std};
}

function product_of_normals_by_variance({mean_a, variance_a, mean_b, variance_b}){
  const mean = (mean_a * variance_b + mean_b * variance_a) / (variance_a + variance_b);
  const variance = (variance_a * variance_b) / (variance_a + variance_b);
  return {mean, variance};
}

function add_std(...std_values){
  return Math.sqrt(std_values.reduce((sum, std) => sum + std * std, 0));
}

function accumulate_position_from_hall(curr, prev){
  const sector = curr.hall_sector;

  if (!prev) return {
    ...curr,
    sector,
    angle: sector_center_degrees[sector],
    angle_std: sector_center_std[sector],
    angular_speed: 0,
    angular_speed_std: initial_angular_speed_std,

    obs_time: curr.time,
    obs_sector: sector,
    obs_angle: sector_center_degrees[sector],
    obs_angle_std: sector_center_std[sector],
    obs_angular_speed: 0,
    obs_angular_speed_std: initial_angular_speed_std,
  }
  
  const dt = curr.time - prev.obs_time;

  // If we switched sectors, we should have an accurate position estimate.
  if (sector != null && sector != prev.sector){
    const direction = Math.sign(shortest_distance_mod_6(prev.sector, sector));

    const trigger_angle = sector_transition_degrees[sector][direction >= 0 ? 0 : 1];
    const trigger_angle_std = sector_transition_std[sector][direction >= 0 ? 0 : 1];

    const distance_to_trigger = shortest_distance_degrees(prev.obs_angle, trigger_angle);

    const estimated_distance = prev.obs_angular_speed * dt;
    const estimated_distance_error = distance_to_trigger - estimated_distance;
    const estimated_speed_error = estimated_distance_error / dt;

    const estimated_distance_std = add_std(prev.obs_angle_std, prev.obs_angular_speed_std * dt, accel_std * dt * dt / 5.0);
    const estimated_speed_error_std = add_std(estimated_distance_std / dt, trigger_angle_std / dt);
    
    const {mean: distance_adjustment, std: angle_std} = product_of_normals({
      mean_a: 0.0,
      std_a: estimated_distance_std,
      mean_b: estimated_distance_error,
      std_b: trigger_angle_std,
    });

    const kalman_angle = normalize_degrees(prev.obs_angle + estimated_distance + distance_adjustment);

    // Ensure we dragged the angle within the 95% confidence interval of the trigger angle. If we're too slow
    // to update the angle we cross 180 degrees and our math switches sign. To avoid that we need to keep the 
    // angle near the trigger. The formula below handles the lower bound of the trigger angle; the upper bound
    // is handled by capping the speed above.
    const angle = kalman_angle + direction * Math.max(direction * normalize_degrees(trigger_angle - direction * std_99_z_score * trigger_angle_std - kalman_angle), 0);

    const {mean: speed_adjustment, std: angular_speed_std} = product_of_normals({
      mean_a: 0.0,
      std_a: prev.obs_angular_speed_std + 0.5 * accel_std * dt,
      mean_b: estimated_speed_error,
      std_b: estimated_speed_error_std,
    });

    const angular_speed = prev.obs_angular_speed + speed_adjustment;

    return {
      ...curr,
      sector,
      angle,
      angle_std,
      angular_speed,
      angular_speed_std,

      obs_time: curr.time,
      obs_sector: sector,
      obs_angle: angle,
      obs_angle_std: angle_std,
      obs_angular_speed: angular_speed,
      obs_angular_speed_std: angular_speed_std,
    };
  } else {
    // If we didn't switch sectors, then we need to carry on with our previous estimate, but adjusted...
    const positive_direction = prev.obs_angular_speed >= 0;
    const direction = positive_direction ? +1 : -1;

    const estimated_distance = prev.obs_angular_speed * dt;

    const estimated_distance_std = add_std(prev.obs_angle_std, prev.obs_angular_speed_std * dt, accel_std * dt * dt / 2.0);

    const next_sector = positive_direction ? (prev.obs_sector + 1) % 6 : (prev.obs_sector - 1 + 6) % 6;
    const next_transition_angle = sector_transition_degrees[next_sector][positive_direction ? 0 : 1];
    const next_transition_angle_std = sector_transition_std[next_sector][positive_direction ? 0 : 1];
    // Cap the position in the 95% confidence interval before the next transition. We cross
    // this threshold when distance_to_next_transition is negative. Also the adjustement
    // depends on the direction of travel.
    const distance_to_next_transition = direction * shortest_distance_degrees(prev.obs_angle, next_transition_angle) + std_99_z_score * next_transition_angle_std;
    
    const distance_overshoot = Math.max(direction * estimated_distance - distance_to_next_transition, 0);
    const distance_adjustment = direction * Math.min(distance_to_next_transition - direction * estimated_distance, 0);

    const estimated_angle = normalize_degrees(prev.obs_angle + estimated_distance + distance_adjustment);


    const p_stopped_in_current_sector = distance_overshoot < 90 ? 0.0 : Math.min(1.0, (distance_overshoot - 90) / 45);

    const current_sector_angle = sector_center_degrees[prev.obs_sector];
    const current_sector_angle_std = sector_center_std[prev.obs_sector];

    const angle = interpolate_degrees(estimated_angle, current_sector_angle, p_stopped_in_current_sector);
    const angle_std = interpolate_linear(estimated_distance_std, current_sector_angle_std, p_stopped_in_current_sector);

    const calculated_spin = shortest_distance_degrees(prev.obs_angle, angle) / dt;
    const calculated_spin_std = (angle_std + prev.obs_angle_std) / dt;

    const {mean: angular_speed, std: angular_speed_std} = product_of_normals({
      mean_a: calculated_spin,
      std_a: calculated_spin_std,
      mean_b: prev.obs_angular_speed,
      std_b: prev.obs_angular_speed_std + accel_std * dt,
    });

    if (p_stopped_in_current_sector >= 0.99){
      return {
        ...curr,
        sector,
        angle,
        angle_std,
        angular_speed: 0,
        angular_speed_std: initial_angular_speed_std,

        obs_time: curr.time,
        obs_sector: sector,
        obs_angle: angle,
        obs_angle_std: angle_std,
        obs_angular_speed: 0,
        obs_angular_speed_std: initial_angular_speed_std,
      };
    } else {
      
      return {
        ...curr,
        sector,
        angle,
        angle_std,
        angular_speed,
        angular_speed_std,

        obs_time: prev.obs_time,
        obs_sector: prev.obs_sector,
        obs_angle: prev.obs_angle,
        obs_angle_std: prev.obs_angle_std,
        obs_angular_speed: prev.obs_angular_speed,
        obs_angular_speed_std: prev.obs_angular_speed_std,
      };
    }
  }
}



function compute_derivatives(readout, previous_readout){
  if (!previous_readout) return readout;

  const {u, v, w} = readout;
  const {u: prev_u, v: prev_v, w: prev_w} = previous_readout;

  const [current_alpha, current_beta] = clarke_transform(u, v, w);

  const current_angle = radians_to_degrees(Math.atan2(current_beta, current_alpha));
  const current_magnitude = Math.sqrt(current_alpha * current_alpha + current_beta * current_beta);

  
  // Time units are milliseconds.
  const dt = readout.time - previous_readout.time;

  const current_angular_speed = normalize_degrees(readout.current_angle - previous_readout.current_angle) / dt;

  // V = L*dI/dt + R*I; Also factor of 1000 for millisecond to second conversion.
  const u_L_voltage = (u - prev_u) / dt * 1000 * phase_inductance;
  const v_L_voltage = (v - prev_v) / dt * 1000 * phase_inductance;
  const w_L_voltage = (w - prev_w) / dt * 1000 * phase_inductance;

  const u_voltage = u_L_voltage + phase_resistance * u;
  const v_voltage = v_L_voltage + phase_resistance * v;
  const w_voltage = w_L_voltage + phase_resistance * w;


  const [voltage_alpha, voltage_beta] = clarke_transform(u_voltage, v_voltage, w_voltage);

  const voltage_angle = radians_to_degrees(Math.atan2(voltage_beta, voltage_alpha));
  
  const voltage_magnitude = Math.sqrt(voltage_alpha * voltage_alpha + voltage_beta * voltage_beta);

  const angle_if_breaking = normalize_degrees(voltage_angle + (readout.speed > 0 ? +90 : -90));
  

  return {
    ...readout,
    current_alpha, current_beta,
    current_angle, current_magnitude,
    current_angular_speed,
    u_voltage, v_voltage, w_voltage,
    u_L_voltage, v_L_voltage, w_L_voltage,
    voltage_alpha, voltage_beta,
    voltage_angle, voltage_magnitude,
    angle_if_breaking,
  };
}

export const process_readout = online_function_chain(
  compute_derivatives,
  accumulate_position_from_hall,
);
