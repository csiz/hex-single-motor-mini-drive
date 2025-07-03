import {
  millis_per_cycle, pwm_base, readout_base,
  current_conversion, expected_ref_readout, calculate_temperature, calculate_voltage,
  angle_units_to_degrees, degrees_to_angle_units, current_calibration_base, unbounded_angle_units_to_degrees,
  speed_units_to_degrees_per_millisecond, 
  degrees_per_millisecond_to_speed_units,
  acceleration_units_to_degrees_per_millisecond2, 
  degrees_per_millisecond2_to_acceleration_units,
  convert_power_to_watts,
  variance_units_to_degrees_stdev,
  degrees_stdev_to_variance_units,
  speed_variance_to_degrees_per_millisecond_stdev,
  degrees_per_millisecond_stdev_to_speed_variance,
  acceleration_div_2_variance_to_degrees_per_millisecond2_stdev,
  degrees_per_millisecond2_stdev_to_acceleration_div_2_variance,
  phase_resistance, phase_inductance,
  hall_state_bit_offset,
  angle_valid_bit_offset,
  emf_detected_bit_offset,
  emf_direction_is_negative_bit_offset,
  angle_bit_mask,
} from './motor_constants.js';

import {normalize_degrees, radians_to_degrees, degrees_to_radians} from './angular_math.js';
import {square, dq0_transform, exponential_stats, sum_preserving_exponential_stats} from './math_utils.js';




export const header_size = 2; // 2 bytes

export const command_codes = {
  READOUT: 0x2020,
  STREAM_FULL_READOUTS: 0x2021,
  GET_READOUTS_SNAPSHOT: 0x2022,
  FULL_READOUT: 0x2023,

  SET_STATE_OFF: 0x2030,
  SET_STATE_DRIVE_6_SECTOR: 0x2031,
  SET_STATE_TEST_ALL_PERMUTATIONS: 0x2032,
  SET_STATE_FREEWHEEL: 0x2034,

  SET_STATE_TEST_GROUND_SHORT: 0x2036,
  SET_STATE_TEST_POSITIVE_SHORT: 0x2037,
  SET_STATE_TEST_U_DIRECTIONS: 0x2039,
  SET_STATE_TEST_U_INCREASING: 0x203A,
  SET_STATE_TEST_U_DECREASING: 0x203B,
  SET_STATE_TEST_V_INCREASING: 0x203C,
  SET_STATE_TEST_V_DECREASING: 0x203D,
  SET_STATE_TEST_W_INCREASING: 0x203E,
  SET_STATE_TEST_W_DECREASING: 0x203F,

  SET_STATE_HOLD_U_POSITIVE: 0x3020,
  SET_STATE_HOLD_V_POSITIVE: 0x3021,
  SET_STATE_HOLD_W_POSITIVE: 0x3022,
  SET_STATE_HOLD_U_NEGATIVE: 0x3023,
  SET_STATE_HOLD_V_NEGATIVE: 0x3024,
  SET_STATE_HOLD_W_NEGATIVE: 0x3025,
  SET_STATE_DRIVE_SMOOTH: 0x4030,
  SET_STATE_DRIVE_TORQUE: 0x4031,
  SET_STATE_DRIVE_BATTERY_POWER: 0x4032,

  SET_CURRENT_FACTORS: 0x4040,
  SET_TRIGGER_ANGLES: 0x4041,
  CURRENT_FACTORS: 0x4042,
  TRIGGER_ANGLES: 0x4043,
  GET_CURRENT_FACTORS: 0x4044,
  GET_TRIGGER_ANGLES: 0x4045,
  PID_PARAMETERS: 0x4046,
  GET_PID_PARAMETERS: 0x4047,
  SET_PID_PARAMETERS: 0x4048,
  OBSERVER_PARAMETERS: 0x4049,
  SET_OBSERVER_PARAMETERS: 0x404A,
  GET_OBSERVER_PARAMETERS: 0x404B,

  SAVE_SETTINGS_TO_FLASH: 0x4080,

  UNIT_TEST_OUTPUT: 0x5040,
  RUN_UNIT_TEST_ATAN: 0x5041,
  RUN_UNIT_TEST_FUNKY_ATAN: 0x5042,
  RUN_UNIT_TEST_FUNKY_ATAN_PART_2: 0x5043,
  RUN_UNIT_TEST_FUNKY_ATAN_PART_3: 0x5044,
}


function get_hall_sector({hall_u, hall_v, hall_w}){
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



function process_readout_diff(readout, previous_readout){
  if (!previous_readout) return readout;

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

  const direction = emf_detected ? (emf_direction_negative ? -1 : +1) : 0;

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


const readout_size = 30;

function parse_readout(data_view, previous_readout){
  let offset = header_size;
  
  // Get the PWM commands.
  const pwm_commands = data_view.getUint32(offset);
  offset += 4;
  // Get the readout number, a counter that increments with each readout & PWM cycle.
  const readout_number = data_view.getUint16(offset);
  offset += 2;
  // Get the raw readout values.
  const u_readout = current_conversion * data_view.getInt16(offset);
  offset += 2;
  const v_readout = current_conversion * data_view.getInt16(offset);
  offset += 2;
  const w_readout = current_conversion * data_view.getInt16(offset);
  offset += 2;
  const ref_readout = current_conversion * (data_view.getUint16(offset) - expected_ref_readout);
  offset += 2;
  const u_readout_diff = current_conversion * data_view.getInt16(offset) / millis_per_cycle;
  offset += 2;
  const v_readout_diff = current_conversion * data_view.getInt16(offset) / millis_per_cycle;
  offset += 2;
  const w_readout_diff = current_conversion * data_view.getInt16(offset) / millis_per_cycle;
  offset += 2;

  // Get motor angle and hall sensor data.
  const position = data_view.getInt16(offset);
  offset += 2;
  const angle_bytes = data_view.getUint16(offset);
  offset += 2;

  const angular_speed_raw = data_view.getInt16(offset);
  offset += 2;
  const instant_vcc_voltage = calculate_voltage(data_view.getUint16(offset));
  offset += 2;


  // We use 1536 ticks per PWM cycle, we can pack 3 values in 32 bits with the formula: (pwm_u*pwm_base + pwm_v)*pwm_base + pwm_w
  const u_pwm = Math.floor(pwm_commands / pwm_base / pwm_base) % pwm_base;
  const v_pwm = Math.floor(pwm_commands / pwm_base) % pwm_base;
  const w_pwm = pwm_commands % pwm_base;

  const avg_pwm = (u_pwm + v_pwm + w_pwm) / 3;

  const u_drive_voltage = (u_pwm - avg_pwm) * instant_vcc_voltage / pwm_base;
  const v_drive_voltage = (v_pwm - avg_pwm) * instant_vcc_voltage / pwm_base;
  const w_drive_voltage = (w_pwm - avg_pwm) * instant_vcc_voltage / pwm_base;

  // The first 3 bits are the hall sensor state.
  const hall_u = (angle_bytes >> hall_state_bit_offset) & 0b001;
  const hall_v = (angle_bytes >> hall_state_bit_offset) & 0b010;
  const hall_w = (angle_bytes >> hall_state_bit_offset) & 0b100;

  const hall_sector = get_hall_sector({hall_u, hall_v, hall_w});

  // The next bit is the motor angle valid flag.
  const angle_valid = (angle_bytes >> angle_valid_bit_offset) & 0b1;

  const emf_detected = (angle_bytes >> emf_detected_bit_offset) & 0b1;

  const emf_direction_negative = (angle_bytes >> emf_direction_is_negative_bit_offset) & 0b1;

  // The last 10 bits are the motor angle. Representing range from 0 to 360 degrees,
  // where 0 means the rotor is aligned by holding positive current on the U phase.
  const angle_raw = angle_bytes & angle_bit_mask;
  const angle = angle_units_to_degrees(angle_raw);


  const scaled_u_current = u_readout * this.current_calibration.u_factor;
  const scaled_v_current = v_readout * this.current_calibration.v_factor;
  const scaled_w_current = w_readout * this.current_calibration.w_factor;
  const avg_current = (scaled_u_current + scaled_v_current + scaled_w_current) / 3.0;

  const u_current = scaled_u_current - avg_current;
  const v_current = scaled_v_current - avg_current;
  const w_current = scaled_w_current - avg_current;

  const [web_alpha_current, web_beta_current] = dq0_transform(u_current, v_current, w_current, degrees_to_radians(angle));
  const current_magnitude = Math.sqrt(web_alpha_current * web_alpha_current + web_beta_current * web_beta_current);

  const current_angle_offset = radians_to_degrees(Math.atan2(web_beta_current, web_alpha_current));
  const current_angle = normalize_degrees(angle + current_angle_offset);


  // Approximate the angle for the 6 sectors of the hall sensor.
  const ε = 2;
  const hall_u_as_angle = hall_u ? hall_v ? + 60 - ε : hall_w ? - 60 + ε :    0 : null;
  const hall_v_as_angle = hall_v ? hall_u ? + 60 + ε : hall_w ? +180 - ε : +120 : null;
  const hall_w_as_angle = hall_w ? hall_v ? -180 + ε : hall_u ? - 60 - ε : -120 : null;

  // Accumulate the readout index across readouts because the readout number is reset every 65536 readouts (~3 seconds).
  const readout_diff = !previous_readout ? 0 : (readout_base + readout_number - previous_readout.readout_number) % readout_base;
  const readout_index = !previous_readout ? 0 : previous_readout.readout_index + readout_diff;
  const time = readout_index * millis_per_cycle;

  const scaled_u_current_diff = u_readout_diff * this.current_calibration.u_factor;
  const scaled_v_current_diff = v_readout_diff * this.current_calibration.v_factor;
  const scaled_w_current_diff = w_readout_diff * this.current_calibration.w_factor;
  const avg_current_diff = (scaled_u_current_diff + scaled_v_current_diff + scaled_w_current_diff) / 3.0;
  
  const u_current_diff = scaled_u_current_diff - avg_current_diff;
  const v_current_diff = scaled_v_current_diff - avg_current_diff;
  const w_current_diff = scaled_w_current_diff - avg_current_diff;

  // V = L*dI/dt + R*I; Also factor of 1000 for millisecond to second conversion.
  const u_L_voltage = u_current_diff * 1000 * phase_inductance * this.current_calibration.inductance_factor;
  const v_L_voltage = v_current_diff * 1000 * phase_inductance * this.current_calibration.inductance_factor;
  const w_L_voltage = w_current_diff * 1000 * phase_inductance * this.current_calibration.inductance_factor;

  const u_R_voltage = phase_resistance * u_current;
  const v_R_voltage = phase_resistance * v_current;
  const w_R_voltage = phase_resistance * w_current;

  const u_emf_voltage = -u_drive_voltage + u_L_voltage + u_R_voltage;
  const v_emf_voltage = -v_drive_voltage + v_L_voltage + v_R_voltage;
  const w_emf_voltage = -w_drive_voltage + w_L_voltage + w_R_voltage;

  const [web_alpha_emf_voltage, web_beta_emf_voltage] = dq0_transform(u_emf_voltage, v_emf_voltage, w_emf_voltage, degrees_to_radians(angle));

  const emf_voltage_angle_offset = radians_to_degrees(Math.atan2(web_beta_emf_voltage, web_alpha_emf_voltage));
  const emf_voltage_angle = normalize_degrees(angle + emf_voltage_angle_offset);
  const emf_voltage_magnitude = Math.sqrt(web_alpha_emf_voltage * web_alpha_emf_voltage + web_beta_emf_voltage * web_beta_emf_voltage);

  const web_total_power = -(u_current * u_drive_voltage + v_current * v_drive_voltage + w_current * w_drive_voltage);
  const web_emf_power = -(u_current * u_emf_voltage + v_current * v_emf_voltage + w_current * w_emf_voltage);
  const web_resistive_power = (square(u_current) * phase_resistance + square(v_current) * phase_resistance + square(w_current) * phase_resistance);
  const web_inductive_power = (u_current * u_L_voltage + v_current * v_L_voltage + w_current * w_L_voltage);



  const readout = {
    readout_number,
    readout_index, time,
    u_readout, v_readout, w_readout,
    u_readout_diff, v_readout_diff, w_readout_diff,
    ref_readout,
    u_current, v_current, w_current, avg_current,
    web_alpha_current, web_beta_current, current_magnitude,
    current_angle, current_angle_offset,
    u_current_diff, v_current_diff, w_current_diff,
    u_pwm, v_pwm, w_pwm,
    u_drive_voltage, v_drive_voltage, w_drive_voltage,
    u_emf_voltage, v_emf_voltage, w_emf_voltage,
    u_R_voltage, v_R_voltage, w_R_voltage,
    u_L_voltage, v_L_voltage, w_L_voltage,
    position,
    hall_u, hall_v, hall_w, hall_sector,
    hall_u_as_angle, hall_v_as_angle, hall_w_as_angle,
    angle_valid, 
    angle_raw,
    angle,
    angular_speed_raw,
    angular_speed: speed_units_to_degrees_per_millisecond(angular_speed_raw),
    emf_detected,
    emf_direction_negative,
    instant_vcc_voltage,
    current_angle_offset,
    web_alpha_emf_voltage, web_beta_emf_voltage, emf_voltage_magnitude,
    emf_voltage_angle, emf_voltage_angle_offset,
    web_total_power,
    web_emf_power,
    web_resistive_power,
    web_inductive_power,
  };

  return process_readout_diff.call(this, readout, previous_readout);
}

const full_readout_size = 86;

function parse_full_readout(data_view, previous_readout){
  const readout = parse_readout.call(this, data_view, previous_readout);

  let offset = readout_size;

  const tick_rate = data_view.getUint16(offset);
  offset += 2;
  const adc_update_rate = data_view.getUint16(offset);
  offset += 2;
  const temperature = calculate_temperature(data_view.getUint16(offset));
  offset += 2;
  const vcc_voltage = calculate_voltage(data_view.getUint16(offset));
  offset += 2;
  const cycle_start_tick = data_view.getInt16(offset);
  offset += 2;
  const cycle_end_tick = data_view.getInt16(offset);
  offset += 2;

  const angle_stdev_raw = Math.sqrt(data_view.getInt16(offset));
  offset += 2;
  const angular_speed_stdev_raw = Math.sqrt(data_view.getInt16(offset));
  offset += 2;

  const alpha_current = current_conversion * data_view.getInt16(offset);
  offset += 2;
  const beta_current = current_conversion * data_view.getInt16(offset);
  offset += 2;
  const alpha_emf_voltage = calculate_voltage(data_view.getInt16(offset));
  offset += 2;
  const beta_emf_voltage = calculate_voltage(data_view.getInt16(offset));
  offset += 2;


  const total_power = convert_power_to_watts(data_view.getInt16(offset));
  offset += 2;
  const resistive_power = convert_power_to_watts(data_view.getInt16(offset));
  offset += 2;
  const emf_power = convert_power_to_watts(data_view.getInt16(offset));
  offset += 2;
  const inductive_power = convert_power_to_watts(data_view.getInt16(offset));
  offset += 2;

  const angle_error = data_view.getInt16(offset);
  offset += 2;
  const angle_error_stdev = Math.sqrt(data_view.getInt16(offset));
  offset += 2;
  const angular_speed_error = data_view.getInt16(offset);
  offset += 2;
  const angular_speed_error_stdev = Math.sqrt(data_view.getInt16(offset));
  offset += 2;
  const inductor_angle_raw = data_view.getInt16(offset);
  offset += 2;
  const inductor_angle_stdev_raw = Math.sqrt(data_view.getInt16(offset));
  offset += 2;
  const inductor_angle_error = data_view.getInt16(offset);
  offset += 2;
  const inductor_angle_error_stdev = Math.sqrt(data_view.getInt16(offset));
  offset += 2;
  const inductor_angular_speed_raw = data_view.getInt16(offset);
  offset += 2;
  const inductor_angular_speed_stdev_raw = Math.sqrt(data_view.getInt16(offset));
  offset += 2;
  const inductor_angular_speed_error = data_view.getInt16(offset);
  offset += 2;
  const inductor_angular_speed_error_stdev = Math.sqrt(data_view.getInt16(offset));
  offset += 2;
  

  const battery_current = total_power / vcc_voltage;

  return {
    ...readout,
    tick_rate,
    adc_update_rate,
    temperature,
    vcc_voltage,

    cycle_start_tick,
    cycle_end_tick,
    
    angle_stdev_raw,
    angle_stdev: angle_units_to_degrees(angle_stdev_raw),
    angle_stdev_raw,
    angle_stdev: angle_units_to_degrees(angle_stdev_raw),
    angular_speed_stdev_raw,
    angular_speed_stdev: speed_units_to_degrees_per_millisecond(angular_speed_stdev_raw),

    alpha_current,
    beta_current,
    alpha_emf_voltage,
    beta_emf_voltage,

    battery_current,
    total_power,
    resistive_power,
    emf_power,
    inductive_power,

    angle_error,
    angle_error_stdev,
    angular_speed_error,
    angular_speed_error_stdev,

    inductor_angle_raw,
    inductor_angle: angle_units_to_degrees(inductor_angle_raw),
    inductor_angle_stdev_raw,
    inductor_angle_stdev: variance_units_to_degrees_stdev(inductor_angle_stdev_raw),
    inductor_angle_error,
    inductor_angle_error_stdev,

    inductor_angular_speed_raw,
    inductor_angular_speed: speed_units_to_degrees_per_millisecond(inductor_angular_speed_raw),
    inductor_angular_speed_stdev_raw,
    inductor_angular_speed_stdev: speed_units_to_degrees_per_millisecond(inductor_angular_speed_stdev_raw),
    inductor_angular_speed_error,
    inductor_angular_speed_error_stdev,
  };
}



const current_calibration_size = 10;

function parse_current_calibration(data_view){
  let offset = header_size;
  const u_factor = data_view.getInt16(offset) / current_calibration_base;
  offset += 2;
  const v_factor = data_view.getInt16(offset) / current_calibration_base;
  offset += 2;
  const w_factor = data_view.getInt16(offset) / current_calibration_base;
  offset += 2;

  const inductance_factor = data_view.getInt16(offset) / current_calibration_base;
  offset += 2;

  return {
    u_factor,
    v_factor,
    w_factor,
    inductance_factor,
  };
}


const position_calibration_size = 78;
function parse_position_calibration(data_view){
  let offset = header_size;
  const sector_transition_degrees = Array.from(Array(6), () => Array.from(Array(2), () => {
    const value = data_view.getUint16(offset);
    offset += 2;
    return angle_units_to_degrees(value);
  }));
  
  const sector_transition_stdev = Array.from(Array(6), () => Array.from(Array(2), () => {
    // We receive the variance, not the stdev.
    const value = data_view.getUint16(offset);
    offset += 2;
    return variance_units_to_degrees_stdev(value);
  }));

  const sector_center_degrees = Array.from(Array(6), () => {
    const value = data_view.getUint16(offset);
    offset += 2;
    return angle_units_to_degrees(value);
  });

  const sector_center_stdev = Array.from(Array(6), () => {
    // We receive the variance, not the stdev.
    const value = data_view.getUint16(offset);
    offset += 2;
    return variance_units_to_degrees_stdev(value);
  });

  // We receive the variance, not the stdev.
  const initial_angular_speed_stdev = speed_variance_to_degrees_per_millisecond_stdev(data_view.getUint16(offset));
  offset += 2;
  const angular_acceleration_stdev = acceleration_div_2_variance_to_degrees_per_millisecond2_stdev(data_view.getUint16(offset));
  offset += 2;

  return {
    sector_transition_degrees,
    sector_transition_stdev,
    sector_center_degrees,
    sector_center_stdev,
    initial_angular_speed_stdev,
    angular_acceleration_stdev,
  };
}


const pid_parameters_size = 42;
function parse_pid_parameters(data_view) {
  let offset = header_size;

  const current_angle_gains = {
    kp: data_view.getInt16(offset),
    ki: data_view.getInt16(offset + 2),
    kd: data_view.getInt16(offset + 4),
    max_output: data_view.getInt16(offset + 6),
  };
  offset += 8;

  const torque_gains = {
    kp: data_view.getInt16(offset),
    ki: data_view.getInt16(offset + 2),
    kd: data_view.getInt16(offset + 4),
    max_output: data_view.getInt16(offset + 6),
  };
  offset += 8;

  const battery_power_gains = {
    kp: data_view.getInt16(offset),
    ki: data_view.getInt16(offset + 2),
    kd: data_view.getInt16(offset + 4),
    max_output: data_view.getInt16(offset + 6),
  };
  offset += 8;

  const angular_speed_gains = {
    kp: data_view.getInt16(offset),
    ki: data_view.getInt16(offset + 2),
    kd: data_view.getInt16(offset + 4),
    max_output: data_view.getInt16(offset + 6),
  };
  offset += 8;

  const position_gains = {
    kp: data_view.getInt16(offset),
    ki: data_view.getInt16(offset + 2),
    kd: data_view.getInt16(offset + 4),
    max_output: data_view.getInt16(offset + 6),
  };
  offset += 8;

  return {
    current_angle_gains,
    torque_gains,
    battery_power_gains,
    angular_speed_gains,
    position_gains,
  };
}


const observer_parameters_size = 22;
function parse_observer_parameters(data_view) {
  let offset = header_size;

  const rotor_angle_ki = data_view.getInt16(offset);
  offset += 2;
  const rotor_angular_speed_ki = data_view.getInt16(offset);
  offset += 2;
  const inductor_angle_ki = data_view.getInt16(offset);
  offset += 2;
  const inductor_angular_speed_ki = data_view.getInt16(offset);
  offset += 2;
  const resistance_ki = data_view.getInt16(offset);
  offset += 2;
  const inductance_ki = data_view.getInt16(offset);
  offset += 2;
  const motor_constant_ki = data_view.getInt16(offset);
  offset += 2;
  const magnetic_resistance_ki = data_view.getInt16(offset);
  offset += 2;
  const rotor_mass_ki = data_view.getInt16(offset);
  offset += 2;
  const rotor_torque_ki = data_view.getInt16(offset);
  offset += 2;

  return {
    rotor_angle_ki,
    rotor_angular_speed_ki,
    inductor_angle_ki,
    inductor_angular_speed_ki,
    resistance_ki,
    inductance_ki,
    motor_constant_ki,
    magnetic_resistance_ki,
    rotor_mass_ki,
    rotor_torque_ki,
  };
}

const unit_test_size = 256;
function parse_unit_test_output(data_view) {
  // Get the length of the null terminated string.
  let len = 0;
  while (data_view.getUint8(header_size + len) !== 0 && len < unit_test_size - header_size) len++;
  // Return the output as an utf-8 string.
  return new TextDecoder().decode(data_view.buffer.slice(header_size, header_size + len));
}

function serialise_set_current_calibration(current_calibration) {
  const {u_factor, v_factor, w_factor, inductance_factor} = current_calibration;

  let buffer = new Uint8Array(current_calibration_size);

  let view = new DataView(buffer.buffer);
  let offset = 0;

  view.setUint16(0, command_codes.SET_CURRENT_FACTORS);
  offset += 2;

  view.setInt16(offset, Math.floor(u_factor * current_calibration_base));
  offset += 2;
  view.setInt16(offset, Math.floor(v_factor * current_calibration_base));
  offset += 2;
  view.setInt16(offset, Math.floor(w_factor * current_calibration_base));
  offset += 2;
  view.setInt16(offset, Math.floor(inductance_factor * current_calibration_base));
  offset += 2;

  return buffer;
}

function serialise_set_position_calibration(position_calibration) {
  const {
    sector_transition_degrees, sector_transition_stdev, sector_center_degrees, sector_center_stdev,
    initial_angular_speed_stdev, angular_acceleration_stdev,
  } = position_calibration;

  let buffer = new Uint8Array(position_calibration_size);

  let view = new DataView(buffer.buffer);

  let offset = 0;
  view.setUint16(offset, command_codes.SET_TRIGGER_ANGLES);
  offset += 2;

  for (let i = 0; i < 6; i++) {
    for (let j = 0; j < 2; j++) {
      view.setUint16(offset, degrees_to_angle_units(sector_transition_degrees[i][j]));
      offset += 2;
    }
  }

  for (let i = 0; i < 6; i++) {
    for (let j = 0; j < 2; j++) {
      // Send variance, not stdev.
      view.setUint16(offset, degrees_stdev_to_variance_units(sector_transition_stdev[i][j]));
      offset += 2;
    }
  }

  for (let i = 0; i < 6; i++) {
    view.setUint16(offset, degrees_to_angle_units(sector_center_degrees[i]));
    offset += 2;
  }

  for (let i = 0; i < 6; i++) {
    // Send variance, not stdev.
    view.setUint16(offset, degrees_stdev_to_variance_units(sector_center_stdev[i]));
    offset += 2;
  }

  view.setUint16(offset, degrees_per_millisecond_stdev_to_speed_variance(initial_angular_speed_stdev));
  offset += 2;
  view.setUint16(offset, degrees_per_millisecond2_stdev_to_acceleration_div_2_variance(angular_acceleration_stdev));
  offset += 2;

  return buffer;
}

function serialise_set_pid_parameters(pid_parameters) {
  const { current_angle_gains, torque_gains, battery_power_gains, angular_speed_gains, position_gains } = pid_parameters;

  let buffer = new Uint8Array(pid_parameters_size);

  let view = new DataView(buffer.buffer);
  let offset = 0;

  view.setUint16(offset, command_codes.SET_PID_PARAMETERS);
  offset += 2;

  // Serialise each set of gains.
  for (const gains of [current_angle_gains, torque_gains, battery_power_gains, angular_speed_gains, position_gains]) {
    view.setInt16(offset, gains.kp);
    offset += 2;
    view.setInt16(offset, gains.ki);
    offset += 2;
    view.setInt16(offset, gains.kd);
    offset += 2;
    view.setInt16(offset, gains.max_output);
    offset += 2;
  }

  return buffer;
}

function serialise_set_observer_parameters(observer_parameters) {
  let buffer = new Uint8Array(observer_parameters_size);
  let view = new DataView(buffer.buffer);
  let offset = 0;
  view.setUint16(offset, command_codes.SET_OBSERVER_PARAMETERS);
  offset += 2;

  view.setInt16(offset, observer_parameters.rotor_angle_ki);
  offset += 2;
  view.setInt16(offset, observer_parameters.rotor_angular_speed_ki);
  offset += 2;
  view.setInt16(offset, observer_parameters.inductor_angle_ki);
  offset += 2;
  view.setInt16(offset, observer_parameters.inductor_angular_speed_ki);
  offset += 2;
  view.setInt16(offset, observer_parameters.resistance_ki);
  offset += 2;
  view.setInt16(offset, observer_parameters.inductance_ki);
  offset += 2;
  view.setInt16(offset, observer_parameters.motor_constant_ki);
  offset += 2;
  view.setInt16(offset, observer_parameters.magnetic_resistance_ki);
  offset += 2;
  view.setInt16(offset, observer_parameters.rotor_mass_ki);
  offset += 2;
  view.setInt16(offset, observer_parameters.rotor_torque_ki);
  offset += 2;
  return buffer;
}


export const serialiser_mapping = {
  [command_codes.SET_CURRENT_FACTORS]: {serialise_func: serialise_set_current_calibration},
  [command_codes.SET_TRIGGER_ANGLES]: {serialise_func: serialise_set_position_calibration},
  [command_codes.SET_PID_PARAMETERS]: {serialise_func: serialise_set_pid_parameters},
  [command_codes.SET_OBSERVER_PARAMETERS]: {serialise_func: serialise_set_observer_parameters},
};

export const parser_mapping = {
  [command_codes.READOUT]: {parse_func: parse_readout, message_size: readout_size},
  [command_codes.FULL_READOUT]: {parse_func: parse_full_readout, message_size: full_readout_size},
  [command_codes.CURRENT_FACTORS]: {parse_func: parse_current_calibration, message_size: current_calibration_size},
  [command_codes.TRIGGER_ANGLES]: {parse_func: parse_position_calibration, message_size: position_calibration_size},
  [command_codes.PID_PARAMETERS]: {parse_func: parse_pid_parameters, message_size: pid_parameters_size},
  [command_codes.OBSERVER_PARAMETERS]: {parse_func: parse_observer_parameters, message_size: observer_parameters_size},
  [command_codes.UNIT_TEST_OUTPUT]: {parse_func: parse_unit_test_output, message_size: unit_test_size},
};


const basic_command_size = 8;

export function serialise_command({command, command_timeout = 0, command_pwm = 0, command_leading_angle = 0, additional_data = undefined}) {
  const {serialise_func} = serialiser_mapping[command] ?? {serialise_func: null};

  // Serialise the command with a special serialiser if it exists.
  if (serialise_func) return serialise_func.call(this, additional_data);

  // Otherwise, serialise a basic command.
  let buffer = new Uint8Array(basic_command_size);

  let view = new DataView(buffer.buffer);
  let offset = 0;
  view.setUint16(offset, command);
  offset += 2;
  view.setUint16(offset, command_timeout);
  offset += 2;
  view.setInt16(offset, command_pwm);
  offset += 2;
  view.setInt16(offset, command_leading_angle);
  offset += 2;

  return buffer;
}