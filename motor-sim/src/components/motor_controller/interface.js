import {
  PWM_BASE,
  CURRENT_UNITS_PER_AMP,
  VOLTAGE_UNITS_PER_VOLT,
  millis_per_cycle,
  pwm_cycles_per_second,
  readout_base,
  calculate_temperature, 
  angle_units_to_radians, 
  radians_to_angle_units,
  speed_units_to_rotations_per_millisecond,
  acceleration_units_to_rotations_per_millisecond_squared,
  parse_state_flags,
} from './constants.js';

import {normalize_radians} from './angular_math.js';
import {square, dq0_transform, exponential_stats} from './math_utils.js';
import {accumulate_position_from_hall} from './position_kalman_filter.js';

import { MessageCode, UNIT_TEST_OUTPUT_SIZE } from 'hex-mini-drive-interface';


// Maximum time between readouts to consider them part of the same series.
const readout_series_timeout = 500;


export function get_hall_sector({hall_u, hall_v, hall_w}){
  const hall_state = (hall_u ? 0b001 : 0) | (hall_v ? 0b010 : 0) | (hall_w ? 0b100 : 0);
  switch(hall_state){
    case 0b001: return 0;
    case 0b011: return 1;
    case 0b010: return 2;
    case 0b110: return 3;
    case 0b100: return 4;
    case 0b101: return 5;
    default: return null;
  }
}


function parse_readout(bare_readout, previous_readout, {current_calibration}) {
  const local_time = Date.now();


  // Get the readout number, a counter that increments with each readout & PWM cycle.
  const readout_number = bare_readout.readout_number;
  const state_flags = bare_readout.state_flags;

  // Get the raw readout values.
  const u_current = bare_readout.u_current / CURRENT_UNITS_PER_AMP;
  const v_current = bare_readout.v_current / CURRENT_UNITS_PER_AMP;
  const w_current = bare_readout.w_current / CURRENT_UNITS_PER_AMP;
  const ref_readout = bare_readout.ref_readout;

  const u_current_diff = bare_readout.u_current_diff / CURRENT_UNITS_PER_AMP;
  const v_current_diff = bare_readout.v_current_diff / CURRENT_UNITS_PER_AMP;
  const w_current_diff = bare_readout.w_current_diff / CURRENT_UNITS_PER_AMP;

  // Get electric angle data. Angle 0 means the rotor North is aligned when holding positive current on the U phase.
  const angle = angle_units_to_radians(bare_readout.angle);
  const angle_adjustment = angle_units_to_radians(bare_readout.angle_adjustment);
  const angular_speed = speed_units_to_rotations_per_millisecond(bare_readout.angular_speed);
  const vcc_voltage = bare_readout.vcc_voltage / VOLTAGE_UNITS_PER_VOLT;

  const emf_voltage_angle = angle_units_to_radians(bare_readout.emf_voltage_angle);
  const emf_voltage_magnitude = bare_readout.emf_voltage_magnitude / VOLTAGE_UNITS_PER_VOLT;
  const emf_voltage_angular_speed = speed_units_to_rotations_per_millisecond(bare_readout.emf_voltage_angular_speed);
  const current_angle = angle_units_to_radians(bare_readout.current_angle);
  const current_magnitude = bare_readout.current_magnitude / CURRENT_UNITS_PER_AMP;
  const current_angular_speed = speed_units_to_rotations_per_millisecond(bare_readout.current_angular_speed);

  
  const predicted_angle = normalize_radians(angle - angle_adjustment);

  const current_angle_offset = normalize_radians(current_angle - predicted_angle);

  const {
    hall_state,
    emf_detected,
    emf_fix,
    current_detected,
    angle_fix,
    incorrect_rotor_angle,
    rotor_direction_flip_imminent,
  } = parse_state_flags(state_flags);

  const hall_u = Boolean(hall_state & 0b001) * 1;
  const hall_v = Boolean(hall_state & 0b010) * 1;
  const hall_w = Boolean(hall_state & 0b100) * 1;

  const hall_sector = get_hall_sector({hall_u, hall_v, hall_w});


  // Approximate the angle for the 6 sectors of the hall sensor; for plotting.
  const ε = 2;
  const hall_u_as_angle = hall_u ? hall_v ? + 60 - ε : hall_w ? - 60 + ε :    0 : null;
  const hall_v_as_angle = hall_v ? hall_u ? + 60 + ε : hall_w ? +180 - ε : +120 : null;
  const hall_w_as_angle = hall_w ? hall_v ? -180 + ε : hall_u ? - 60 - ε : -120 : null;


  // Get the PWM voltage commands.
  const u_drive_voltage = bare_readout.u_drive_voltage / VOLTAGE_UNITS_PER_VOLT;
  const v_drive_voltage = bare_readout.v_drive_voltage / VOLTAGE_UNITS_PER_VOLT;
  const w_drive_voltage = bare_readout.w_drive_voltage / VOLTAGE_UNITS_PER_VOLT;
  
  // Derive the PWM duty cycles given the VCC voltage. (Done this way to avoid duplicating calculations
  // on the microcontroller.)
  const min_drive_voltage = Math.min(u_drive_voltage, v_drive_voltage, w_drive_voltage);
  const u_pwm = (u_drive_voltage - min_drive_voltage) / vcc_voltage * PWM_BASE;
  const v_pwm = (v_drive_voltage - min_drive_voltage) / vcc_voltage * PWM_BASE;
  const w_pwm = (w_drive_voltage - min_drive_voltage) / vcc_voltage * PWM_BASE;


  const [drive_voltage_direct, drive_voltage_quadrature] = dq0_transform(u_drive_voltage, v_drive_voltage, w_drive_voltage, 0);
  const drive_voltage_angle = Math.atan2(drive_voltage_quadrature, drive_voltage_direct);
  const drive_voltage_magnitude = Math.sqrt(drive_voltage_direct * drive_voltage_direct + drive_voltage_quadrature * drive_voltage_quadrature);
  const drive_voltage_angle_offset = normalize_radians(drive_voltage_angle - predicted_angle);

  
  const avg_current = (u_current + v_current + w_current) / 3.0;
  
  const [web_direct_current, web_quadrature_current] = dq0_transform(u_current, v_current, w_current, predicted_angle);
  
  const web_current_angle = normalize_radians(predicted_angle + Math.atan2(web_quadrature_current, web_direct_current));
  const web_current_magnitude = Math.sqrt(web_direct_current * web_direct_current + web_quadrature_current * web_quadrature_current);
  
  const inductance = (current_calibration?.inductance ?? 1);
  
  // V = L*dI/dt + R*I; Also factor of 1000 for millisecond to second conversion.
  const u_L_voltage = u_current_diff * inductance * pwm_cycles_per_second;
  const v_L_voltage = v_current_diff * inductance * pwm_cycles_per_second;
  const w_L_voltage = w_current_diff * inductance * pwm_cycles_per_second;
  
  const u_resistance = (current_calibration?.u_resistance ?? 1.0);
  const v_resistance = (current_calibration?.v_resistance ?? 1.0);
  const w_resistance = (current_calibration?.w_resistance ?? 1.0);

  const u_R_voltage = u_resistance * u_current;
  const v_R_voltage = v_resistance * v_current;
  const w_R_voltage = w_resistance * w_current;

  const u_emf_voltage =  u_L_voltage + u_R_voltage - u_drive_voltage;
  const v_emf_voltage =  v_L_voltage + v_R_voltage - v_drive_voltage;
  const w_emf_voltage =  w_L_voltage + w_R_voltage - w_drive_voltage;

  const [web_direct_emf_voltage, web_quadrature_emf_voltage] = dq0_transform(u_emf_voltage, v_emf_voltage, w_emf_voltage, predicted_angle);

  const emf_voltage_angle_offset = Math.atan2(web_quadrature_emf_voltage, web_direct_emf_voltage);
  const web_emf_voltage_angle = normalize_radians(predicted_angle + emf_voltage_angle_offset);
  const web_emf_voltage_magnitude = Math.sqrt(web_direct_emf_voltage * web_direct_emf_voltage + web_quadrature_emf_voltage * web_quadrature_emf_voltage);

  const web_total_power = -(u_current * u_drive_voltage + v_current * v_drive_voltage + w_current * w_drive_voltage);
  const web_emf_power = -(u_current * u_emf_voltage + v_current * v_emf_voltage + w_current * w_emf_voltage);
  const web_resistive_power = (square(u_current) * u_resistance + square(v_current) * v_resistance + square(w_current) * w_resistance);
  const web_inductive_power = (u_current * u_L_voltage + v_current * v_L_voltage + w_current * w_L_voltage);

  const steady_state_drive_current = drive_voltage_magnitude / ((u_resistance + v_resistance + w_resistance) / 3.0);


  // Time indexing
  // -------------

  // It's the first readout in the series if the previous readout is older than the timeout duration.
  const is_first_readout = !previous_readout || (previous_readout.local_time + readout_series_timeout < local_time);

  // Accumulate the readout index across readouts because the readout number is reset every 65536 readouts (~3 seconds).
  const readout_number_diff = is_first_readout ? undefined : (readout_base + readout_number - previous_readout.readout_number) % readout_base;

  const readout_index = is_first_readout ? 0 : previous_readout.readout_index + readout_number_diff;

  const time = is_first_readout ? 0 : readout_index * millis_per_cycle;

  // Time units are milliseconds.
  const dt = is_first_readout ? undefined : time - previous_readout.time;


  // Infer angle with the Kalman filter
  // ----------------------------------

  const is_hall_transition = (hall_sector !== null) && (hall_sector !== previous_readout?.hall_sector);

  const {
    web_angle,
    web_angle_stdev,
    web_angular_speed,
    web_angular_speed_stdev,
  } = accumulate_position_from_hall({dt, hall_sector, is_hall_transition}, previous_readout, {});


  // Running averages
  // ----------------
  
  const exp_stats = exponential_stats(dt, 2.0);

  const {average: web_emf_voltage_magnitude_avg, stdev: web_emf_voltage_magnitude_stdev} = exp_stats(
    web_emf_voltage_magnitude,
    {
      average: previous_readout?.web_emf_voltage_magnitude_avg,
      stdev: previous_readout?.web_emf_voltage_magnitude_stdev,
    },
  );

  const {average: web_current_magnitude_avg, stdev: web_current_magnitude_stdev} = exp_stats(
    web_current_magnitude,
    {
      average: previous_readout?.web_current_magnitude_avg,
      stdev: previous_readout?.web_current_magnitude_stdev,
    },
  );


  // Return the complete readout
  // ---------------------------

  return {
    local_time,
    // Index
    readout_number,
    readout_number_diff,
    readout_index,
    time,
    dt,
    // State flags
    hall_u,
    hall_v,
    hall_w,
    hall_sector,
    is_hall_transition,
    emf_detected,
    emf_fix,
    current_detected,
    angle_fix,
    incorrect_rotor_angle,
    rotor_direction_flip_imminent,
    // Readouts converted to physical dimensions
    u_current, v_current, w_current, avg_current,


    web_direct_current, web_quadrature_current, 
    web_current_magnitude,
    web_current_angle, 
    // Reference readouts in adc units
    ref_readout,

    u_current_diff, v_current_diff, w_current_diff,
    u_pwm, v_pwm, w_pwm,
    u_drive_voltage, v_drive_voltage, w_drive_voltage,
    drive_voltage_direct,
    drive_voltage_quadrature,
    drive_voltage_angle, 
    drive_voltage_angle_offset,
    drive_voltage_magnitude,
    steady_state_drive_current,
    u_emf_voltage, v_emf_voltage, w_emf_voltage,
    u_R_voltage, v_R_voltage, w_R_voltage,
    u_L_voltage, v_L_voltage, w_L_voltage,
    hall_u_as_angle,
    hall_v_as_angle,
    hall_w_as_angle,
    angle,
    predicted_angle,
    angle_adjustment,
    angular_speed,
    vcc_voltage,
    emf_voltage_angle, emf_voltage_magnitude, emf_voltage_angular_speed,
    current_angle, current_magnitude,
    current_angle_offset,
    current_angular_speed,
    web_direct_emf_voltage, web_quadrature_emf_voltage, 
    web_emf_voltage_magnitude,
    web_emf_voltage_angle, 
    emf_voltage_angle_offset,
    web_total_power,
    web_emf_power,
    web_resistive_power,
    web_inductive_power,
    web_angle,
    web_angle_stdev,
    web_angular_speed,
    web_angular_speed_stdev,
    web_emf_voltage_magnitude_avg, 
    web_emf_voltage_magnitude_stdev,
    web_current_magnitude_avg, 
    web_current_magnitude_stdev,
  };
}



function parse_full_readout(bare_full_readout, previous_readout, calibration_data){
  const readout = parse_readout(bare_full_readout, previous_readout, calibration_data);

  if (!readout) return null;

  const main_loop_rate = bare_full_readout.main_loop_rate;
  const adc_update_rate = bare_full_readout.adc_update_rate;
  const temperature = calculate_temperature(bare_full_readout.temperature);
  const live_max_pwm = bare_full_readout.live_max_pwm;
  const cycle_start_tick = bare_full_readout.cycle_start_tick;
  const cycle_end_tick = bare_full_readout.cycle_end_tick;

  const direct_current = bare_full_readout.direct_current / CURRENT_UNITS_PER_AMP;
  const quadrature_current = bare_full_readout.quadrature_current / CURRENT_UNITS_PER_AMP;
  const direct_emf_voltage = bare_full_readout.direct_emf_voltage / VOLTAGE_UNITS_PER_VOLT;
  const quadrature_emf_voltage = bare_full_readout.quadrature_emf_voltage / VOLTAGE_UNITS_PER_VOLT;

  const total_power = bare_full_readout.total_power;
  const resistive_power = bare_full_readout.resistive_power;
  const emf_power = bare_full_readout.emf_power;
  const inductive_power = bare_full_readout.inductive_power;

  const rotor_acceleration = acceleration_units_to_rotations_per_millisecond_squared(bare_full_readout.rotor_acceleration);
  const rotations = bare_full_readout.rotations;
  const emf_angle_error_stdev = angle_units_to_radians(Math.sqrt(bare_full_readout.emf_angle_error_variance));
  const lead_angle = angle_units_to_radians(bare_full_readout.lead_angle);
  const target_pwm = bare_full_readout.target_pwm;
  const secondary_target = bare_full_readout.secondary_target;
  const seek_integral = bare_full_readout.seek_integral;
  
  const u_resistance = bare_full_readout.u_resistance;
  const v_resistance = bare_full_readout.v_resistance;
  const w_resistance = bare_full_readout.w_resistance;
  const inductance = bare_full_readout.inductance;
  const magnetization_angle = angle_units_to_radians(bare_full_readout.magnetization_angle);
  const magnetization_factor = bare_full_readout.magnetization_factor;
  const motor_constant = bare_full_readout.motor_constant;
  const friction_torque = bare_full_readout.friction_torque;
  const rotor_mass = bare_full_readout.rotor_mass;

  const battery_current = total_power / readout.vcc_voltage;

  const exp_stats = exponential_stats(readout.dt, 2.0);

  const {average: emf_power_avg, stdev: emf_power_stdev} = exp_stats(
    emf_power,
    {
      average: previous_readout?.emf_power_avg,
      stdev: previous_readout?.emf_power_stdev,
    },
  );

  const {average: total_power_avg, stdev: total_power_stdev} = exp_stats(
    total_power,
    {
      average: previous_readout?.total_power_avg,
      stdev: previous_readout?.total_power_stdev,
    },
  );

  return {
    ...readout,
    main_loop_rate,
    adc_update_rate,
    temperature,
    live_max_pwm,
    
    cycle_start_tick,
    cycle_end_tick,
    
    direct_current,
    quadrature_current,
    direct_emf_voltage,
    quadrature_emf_voltage,
    
    battery_current,
    total_power,
    total_power_avg, 
    total_power_stdev,
    resistive_power,
    emf_power,
    emf_power_avg, 
    emf_power_stdev,
    inductive_power,
    
    emf_angle_error_stdev,
    
    rotor_acceleration,
    rotations,
    
    lead_angle,
    target_pwm,
    secondary_target,
    seek_integral,
    
    u_resistance,
    v_resistance,
    w_resistance,
    inductance,
    magnetization_angle,
    magnetization_factor,
    motor_constant,
    friction_torque,
    rotor_mass,
  };

}


function parse_unit_test_output(bare_unit_test) {
  // Convert number array to Uint8Array.
  const uint8_array = new Uint8Array(bare_unit_test);

  // Get dataview from test data buffer.
  const data_view = new DataView(uint8_array.buffer);

  // Get the length of the null terminated string.
  let len = 0;
  while (data_view.getUint8(len) !== 0 && len < UNIT_TEST_OUTPUT_SIZE) len++;

  // Return the output as an utf-8 string.
  return new TextDecoder().decode(data_view.buffer.slice(0, len));
}

function parse_current_calibration(bare_calibration) {
  return {
    u_resistance: bare_calibration.u_resistance,
    v_resistance: bare_calibration.v_resistance,
    w_resistance: bare_calibration.w_resistance,
    inductance: bare_calibration.inductance,
    magnetization_angle: angle_units_to_radians(bare_calibration.magnetization_angle),
    magnetization_factor: bare_calibration.magnetization_factor,
    motor_constant: bare_calibration.motor_constant,
    friction_torque: bare_calibration.friction_torque,
    rotor_mass: bare_calibration.rotor_mass,
  };
} 

export function make_current_calibration(current_calibration){
  return {
    u_resistance: current_calibration.u_resistance,
    v_resistance: current_calibration.v_resistance,
    w_resistance: current_calibration.w_resistance,
    inductance: current_calibration.inductance,
    magnetization_angle: radians_to_angle_units(current_calibration.magnetization_angle),
    magnetization_factor: current_calibration.magnetization_factor,
    motor_constant: current_calibration.motor_constant,
    friction_torque: current_calibration.friction_torque,
    rotor_mass: current_calibration.rotor_mass,
  };
}

export const parser_mapping = {
  [MessageCode.READOUT]: parse_readout,
  [MessageCode.FULL_READOUT]: parse_full_readout,
  [MessageCode.UNIT_TEST_OUTPUT]: parse_unit_test_output,
  [MessageCode.CURRENT_CALIBRATION]: parse_current_calibration,
};

