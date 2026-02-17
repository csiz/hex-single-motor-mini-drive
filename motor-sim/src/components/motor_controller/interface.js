import {
  millis_per_cycle, 
  pwm_base, 
  readout_base,
  current_conversion, 
  expected_ref_readout, 
  calculate_temperature, 
  calculate_voltage,
  current_calibration_base,
  angle_units_to_degrees, 
  degrees_to_angle_units,
  speed_units_to_degrees_per_millisecond,
  acceleration_units_to_degrees_per_millisecond_squared,
  convert_power_units_to_watts,
  phase_resistance, 
  phase_inductance,
  parse_state_flags,
} from './constants.js';

import {normalize_degrees, radians_to_degrees, degrees_to_radians} from './angular_math.js';
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


function parse_readout(bare_readout, previous_readout, {current_calibration, position_calibration}) {
  const local_time = Date.now();

  // Get the PWM commands.
  const pwm_commands = bare_readout.pwm_commands;
  // Get the readout number, a counter that increments with each readout & PWM cycle.
  const readout_number = bare_readout.readout_number;
  const state_flags = bare_readout.state_flags;

  // Get the raw readout values.
  const u_current = current_conversion * bare_readout.u_current;
  const v_current = current_conversion * bare_readout.v_current;
  const w_current = current_conversion * bare_readout.w_current;
  const ref_readout = current_conversion * (bare_readout.ref_readout - expected_ref_readout);
  const u_current_diff = current_conversion * bare_readout.u_current_diff / millis_per_cycle;
  const v_current_diff = current_conversion * bare_readout.v_current_diff / millis_per_cycle;
  const w_current_diff = current_conversion * bare_readout.w_current_diff / millis_per_cycle;

  // Get electric angle data. Angle 0 means the rotor North is aligned when holding positive current on the U phase.
  const angle = angle_units_to_degrees(bare_readout.angle);
  const angle_adjustment = angle_units_to_degrees(bare_readout.angle_adjustment);
  const angular_speed = speed_units_to_degrees_per_millisecond(bare_readout.angular_speed);
  const vcc_voltage = calculate_voltage(bare_readout.vcc_voltage);
  const emf_voltage_magnitude = calculate_voltage(bare_readout.emf_voltage_magnitude);


  const predicted_angle = normalize_degrees(angle - angle_adjustment);

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

  // We use 1536 ticks per PWM cycle, we can pack 3 values in 32 bits with the formula: (pwm_u*pwm_base + pwm_v)*pwm_base + pwm_w
  const u_pwm = Math.floor(pwm_commands / pwm_base / pwm_base) % pwm_base;
  const v_pwm = Math.floor(pwm_commands / pwm_base) % pwm_base;
  const w_pwm = pwm_commands % pwm_base;

  const avg_pwm = (u_pwm + v_pwm + w_pwm) / 3;

  const u_drive_voltage = (u_pwm - avg_pwm) * vcc_voltage / pwm_base;
  const v_drive_voltage = (v_pwm - avg_pwm) * vcc_voltage / pwm_base;
  const w_drive_voltage = (w_pwm - avg_pwm) * vcc_voltage / pwm_base;

  const [drive_voltage_direct, drive_voltage_quadrature] = dq0_transform(u_drive_voltage, v_drive_voltage, w_drive_voltage, 0);
  const drive_voltage_angle = radians_to_degrees(Math.atan2(drive_voltage_quadrature, drive_voltage_direct));
  const drive_voltage_magnitude = Math.sqrt(drive_voltage_direct * drive_voltage_direct + drive_voltage_quadrature * drive_voltage_quadrature);
  const drive_voltage_angle_offset = normalize_degrees(drive_voltage_angle - predicted_angle);

  const u_factor = (current_calibration?.u_factor ?? 1);
  const v_factor = (current_calibration?.v_factor ?? 1);
  const w_factor = (current_calibration?.w_factor ?? 1);

  const u_readout = u_current / u_factor;
  const v_readout = v_current / v_factor;
  const w_readout = w_current / w_factor;

  const avg_current = (u_current + v_current + w_current) / 3.0;

  const [web_direct_current, web_quadrature_current] = dq0_transform(u_current, v_current, w_current, degrees_to_radians(predicted_angle));

  const web_inductor_angle = normalize_degrees(predicted_angle + radians_to_degrees(Math.atan2(web_quadrature_current, web_direct_current)));
  const web_current_magnitude = Math.sqrt(web_direct_current * web_direct_current + web_quadrature_current * web_quadrature_current);

  const u_readout_diff = u_current_diff / u_factor;
  const v_readout_diff = v_current_diff / v_factor;
  const w_readout_diff = w_current_diff / w_factor;

  const inductance_factor = (current_calibration?.inductance_factor ?? 1);

  // V = L*dI/dt + R*I; Also factor of 1000 for millisecond to second conversion.
  const u_L_voltage = u_current_diff * 1000 * phase_inductance * inductance_factor;
  const v_L_voltage = v_current_diff * 1000 * phase_inductance * inductance_factor;
  const w_L_voltage = w_current_diff * 1000 * phase_inductance * inductance_factor;

  const u_R_voltage = phase_resistance * u_current;
  const v_R_voltage = phase_resistance * v_current;
  const w_R_voltage = phase_resistance * w_current;

  const u_emf_voltage = -u_drive_voltage + u_L_voltage + u_R_voltage;
  const v_emf_voltage = -v_drive_voltage + v_L_voltage + v_R_voltage;
  const w_emf_voltage = -w_drive_voltage + w_L_voltage + w_R_voltage;

  const [web_direct_emf_voltage, web_quadrature_emf_voltage] = dq0_transform(u_emf_voltage, v_emf_voltage, w_emf_voltage, degrees_to_radians(predicted_angle));

  const emf_voltage_angle_offset = radians_to_degrees(Math.atan2(web_quadrature_emf_voltage, web_direct_emf_voltage));
  const web_emf_voltage_angle = normalize_degrees(predicted_angle + emf_voltage_angle_offset);
  const web_emf_voltage_magnitude = Math.sqrt(web_direct_emf_voltage * web_direct_emf_voltage + web_quadrature_emf_voltage * web_quadrature_emf_voltage);

  const web_total_power = -(u_current * u_drive_voltage + v_current * v_drive_voltage + w_current * w_drive_voltage);
  const web_emf_power = -(u_current * u_emf_voltage + v_current * v_emf_voltage + w_current * w_emf_voltage);
  const web_resistive_power = (square(u_current) * phase_resistance + square(v_current) * phase_resistance + square(w_current) * phase_resistance);
  const web_inductive_power = (u_current * u_L_voltage + v_current * v_L_voltage + w_current * w_L_voltage);

  const steady_state_drive_current = drive_voltage_magnitude / phase_resistance;


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
  } = accumulate_position_from_hall({dt, hall_sector, is_hall_transition}, previous_readout, {position_calibration});


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
    // Raw readout values
    u_readout, v_readout, w_readout,
    u_readout_diff, v_readout_diff, w_readout_diff,
    ref_readout,
    // Readouts converted to physical dimensions
    u_current, v_current, w_current, avg_current,
    web_direct_current, web_quadrature_current, 
    web_current_magnitude,
    web_inductor_angle, 

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
    emf_voltage_magnitude,
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

  const tick_rate = bare_full_readout.tick_rate;
  const adc_update_rate = bare_full_readout.adc_update_rate;
  const temperature = calculate_temperature(bare_full_readout.temperature);
  const live_max_pwm = bare_full_readout.live_max_pwm;
  const cycle_start_tick = bare_full_readout.cycle_start_tick;
  const cycle_end_tick = bare_full_readout.cycle_end_tick;

  const direct_current = current_conversion * bare_full_readout.direct_current;
  const quadrature_current = current_conversion * bare_full_readout.quadrature_current;
  const direct_emf_voltage = calculate_voltage(bare_full_readout.direct_emf_voltage);
  const quadrature_emf_voltage = calculate_voltage(bare_full_readout.quadrature_emf_voltage);

  const total_power = convert_power_units_to_watts(bare_full_readout.total_power);
  const resistive_power = convert_power_units_to_watts(bare_full_readout.resistive_power);
  const emf_power = convert_power_units_to_watts(bare_full_readout.emf_power);
  const inductive_power = convert_power_units_to_watts(bare_full_readout.inductive_power);

  const motor_constant = bare_full_readout.motor_constant;
  const inductor_angle = angle_units_to_degrees(bare_full_readout.inductor_angle);
  const rotor_acceleration = acceleration_units_to_degrees_per_millisecond_squared(bare_full_readout.rotor_acceleration);
  const rotations = bare_full_readout.rotations;
  const current_magnitude = current_conversion * bare_full_readout.current_magnitude;
  const emf_angle_error_stdev = angle_units_to_degrees(Math.sqrt(bare_full_readout.emf_angle_error_variance));
  const lead_angle = angle_units_to_degrees(bare_full_readout.lead_angle);
  const target_pwm = bare_full_readout.target_pwm;
  const secondary_target = bare_full_readout.secondary_target;
  const seek_integral = bare_full_readout.seek_integral;

  const battery_current = total_power / readout.vcc_voltage;


  const inductor_angle_offset = normalize_degrees(inductor_angle - readout.predicted_angle);

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
    tick_rate,
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
    motor_constant,
    
    inductor_angle,
    inductor_angle_offset,
    
    rotor_acceleration,
    rotations,
    current_magnitude,

    lead_angle,
    target_pwm,
    secondary_target,
    seek_integral,
  };

}

function parse_current_calibration(bare_current_calibration) {
  const u_factor = bare_current_calibration.u_factor / current_calibration_base;
  const v_factor = bare_current_calibration.v_factor / current_calibration_base;
  const w_factor = bare_current_calibration.w_factor / current_calibration_base;
  const inductance_factor = bare_current_calibration.inductance_factor / current_calibration_base;

  return {
    u_factor,
    v_factor,
    w_factor,
    inductance_factor,
  };
}

export function make_current_calibration(current_calibration) {
  const u_factor = current_calibration.u_factor * current_calibration_base;
  const v_factor = current_calibration.v_factor * current_calibration_base;
  const w_factor = current_calibration.w_factor * current_calibration_base;
  const inductance_factor = current_calibration.inductance_factor * current_calibration_base;
}

function parse_position_calibration(bare_position_calibration) {
  const sector_transition_degrees = bare_position_calibration.sector_transition_angles.map((inner_array) => inner_array.map((value) => {
    const angle = angle_units_to_degrees(value);
    return angle;
  }));

  const sector_transition_stdev = bare_position_calibration.sector_transition_variances.map((inner_array) => inner_array.map((value) => {
    // We receive the variance, not the stdev.
    const stdev = angle_units_to_degrees(Math.sqrt(value));
    return stdev;
  }));

  const sector_center_degrees = bare_position_calibration.sector_center_angles.map((value) => {
    return angle_units_to_degrees(value);
  });

  const sector_center_stdev = bare_position_calibration.sector_center_variances.map((value) => {
    // We receive the variance, not the stdev.
    return angle_units_to_degrees(Math.sqrt(value));
  });

  return {
    sector_transition_degrees,
    sector_transition_stdev,
    sector_center_degrees,
    sector_center_stdev,
  };
}

export function make_position_calibration(position_calibration) {
  const sector_transition_angles = position_calibration.sector_transition_degrees.map((inner_array) => inner_array.map((value) => {
    const angle = degrees_to_angle_units(value);
    return angle;
  }));
  const sector_transition_variances = position_calibration.sector_transition_stdev.map((inner_array) => inner_array.map((value) => {
    // We send the stdev, not the variance, so we need to square it.
    const variance = square(degrees_to_angle_units(value));
    return variance;
  }));
  const sector_center_angles = position_calibration.sector_center_degrees.map((value) => {
    return degrees_to_angle_units(value);
  });
  const sector_center_variances = position_calibration.sector_center_stdev.map((value) => {
    // We send the stdev, not the variance, so we need to square it.
    return square(degrees_to_angle_units(value));
  });

  return {
    sector_transition_angles,
    sector_transition_variances,
    sector_center_angles,
    sector_center_variances,
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


export const parser_mapping = {
  [MessageCode.READOUT]: parse_readout,
  [MessageCode.FULL_READOUT]: parse_full_readout,
  [MessageCode.HALL_POSITIONS]: parse_position_calibration,
  [MessageCode.UNIT_TEST_OUTPUT]: parse_unit_test_output,
  [MessageCode.CURRENT_CALIBRATION]: parse_current_calibration,
};

