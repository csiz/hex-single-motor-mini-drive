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
} from './motor_constants.js';

import {normalize_degrees, radians_to_degrees, degrees_to_radians} from './angular_math.js';
import {square, dq0_transform, exponential_stats} from './math_utils.js';
import {accumulate_position_from_hall} from './motor_position_kalman_filter.js';

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

  SET_STATE_DRIVE_PERIODIC: 0x3040,

  SET_STATE_DRIVE_SMOOTH: 0x4030,
  SET_STATE_DRIVE_TORQUE: 0x4031,
  SET_STATE_DRIVE_BATTERY_POWER: 0x4032,
  SET_STATE_SEEK_ANGLE_WITH_POWER: 0x4033,

  CURRENT_FACTORS: 0x4040,
  GET_CURRENT_FACTORS: 0x4041,
  SET_CURRENT_FACTORS: 0x4042,
  RESET_CURRENT_FACTORS: 0x4043,

  HALL_POSITIONS: 0x4044,
  GET_HALL_POSITIONS: 0x4045,
  SET_HALL_POSITIONS: 0x4046,
  RESET_HALL_POSITIONS: 0x4047,

  CONTROL_PARAMETERS: 0x4049,
  SET_CONTROL_PARAMETERS: 0x404A,
  GET_CONTROL_PARAMETERS: 0x404B,
  RESET_CONTROL_PARAMETERS: 0x404C,

  SET_ANGLE: 0x4050,

  SAVE_SETTINGS_TO_FLASH: 0x4080,

  UNIT_TEST_OUTPUT: 0x5040,
  RUN_UNIT_TEST_FUNKY_ATAN: 0x5042,
  RUN_UNIT_TEST_FUNKY_ATAN_PART_2: 0x5043,
  RUN_UNIT_TEST_FUNKY_ATAN_PART_3: 0x5044,
}

const END_OF_MESSAGE = 0b0101_0101_0101_0101;

const end_of_message_size = 2;
const crc_size = 4;
const tail_size = crc_size + end_of_message_size;

const basic_command_size = 16;
const readout_size = 40;
const full_readout_size = 84;
const current_calibration_size = 16;
const position_calibration_size = 80;
const control_parameters_size = 44;
const unit_test_size = 256;

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


function check_message_for_errors(data_view, message_size, message_code) {
  if (data_view.getUint16(0) !== message_code) {
    console.warn(`Message code mismatch: expected ${message_code}, got ${data_view.getUint16(0)}`);
    return true;
  }

  if (data_view.byteLength != message_size) {
    console.warn(`Message size mismatch: expected ${message_size}, got ${data_view.byteLength}`);
    return true;
  }

  // TODO: Check the CRC.
  
  const end_of_message = data_view.getUint16(message_size - end_of_message_size);
  if (end_of_message !== END_OF_MESSAGE) {
    console.warn(`Message end of message mismatch: expected ${END_OF_MESSAGE}, got ${end_of_message}`);
    return true;
  }

  return false;
}

function parse_readout(data_view, previous_readout, check_errors = true){
  if (check_errors && check_message_for_errors(data_view, readout_size, command_codes.READOUT)) return null;

  let offset = header_size;

  // Get the PWM commands.
  const pwm_commands = data_view.getUint32(offset);
  offset += 4;
  // Get the readout number, a counter that increments with each readout & PWM cycle.
  const readout_number = data_view.getUint16(offset);
  offset += 2;
  const state_flags = data_view.getUint16(offset);
  offset += 2;

  // Get the raw readout values.
  const u_current = current_conversion * data_view.getInt16(offset);
  offset += 2;
  const v_current = current_conversion * data_view.getInt16(offset);
  offset += 2;
  const w_current = current_conversion * data_view.getInt16(offset);
  offset += 2;
  const ref_readout = current_conversion * (data_view.getInt16(offset) - expected_ref_readout);
  offset += 2;
  const u_current_diff = current_conversion * data_view.getInt16(offset) / millis_per_cycle;
  offset += 2;
  const v_current_diff = current_conversion * data_view.getInt16(offset) / millis_per_cycle;
  offset += 2;
  const w_current_diff = current_conversion * data_view.getInt16(offset) / millis_per_cycle;
  offset += 2;

  // Get electric angle data. Angle 0 means the rotor North is aligned when holding positive current on the U phase.
  const angle = angle_units_to_degrees(data_view.getInt16(offset));
  offset += 2;
  const angle_adjustment = angle_units_to_degrees(data_view.getInt16(offset));
  offset += 2;
  const angular_speed = speed_units_to_degrees_per_millisecond(data_view.getInt16(offset));
  offset += 2;
  const vcc_voltage = calculate_voltage(data_view.getInt16(offset));
  offset += 2;
  const emf_voltage_magnitude = calculate_voltage(data_view.getInt16(offset));
  offset += 2;


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

  const is_hall_transition = hall_sector !== null && hall_sector !== previous_readout?.hall_sector;

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

  const [drive_voltage_alpha, drive_voltage_beta] = dq0_transform(u_drive_voltage, v_drive_voltage, w_drive_voltage, 0);
  const drive_voltage_angle = radians_to_degrees(Math.atan2(drive_voltage_beta, drive_voltage_alpha));
  const drive_voltage_magnitude = Math.sqrt(drive_voltage_alpha * drive_voltage_alpha + drive_voltage_beta * drive_voltage_beta);
  const drive_voltage_angle_offset = normalize_degrees(drive_voltage_angle - predicted_angle);

  const u_readout = u_current / this.current_calibration.u_factor;
  const v_readout = v_current / this.current_calibration.v_factor;
  const w_readout = w_current / this.current_calibration.w_factor;

  const avg_current = (u_current + v_current + w_current) / 3.0;

  const [web_alpha_current, web_beta_current] = dq0_transform(u_current, v_current, w_current, degrees_to_radians(predicted_angle));

  const web_inductor_angle = normalize_degrees(predicted_angle + radians_to_degrees(Math.atan2(web_beta_current, web_alpha_current)));
  const web_current_magnitude = Math.sqrt(web_alpha_current * web_alpha_current + web_beta_current * web_beta_current);


  // Accumulate the readout index across readouts because the readout number is reset every 65536 readouts (~3 seconds).
  const readout_diff = !previous_readout ? 0 : (readout_base + readout_number - previous_readout.readout_number) % readout_base;

  const readout_index = !previous_readout ? 0 : previous_readout.readout_index + readout_diff;
  const time = readout_index * millis_per_cycle;

  const u_readout_diff = u_current_diff / this.current_calibration.u_factor;
  const v_readout_diff = v_current_diff / this.current_calibration.v_factor;
  const w_readout_diff = w_current_diff / this.current_calibration.w_factor;


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

  const [web_alpha_emf_voltage, web_beta_emf_voltage] = dq0_transform(u_emf_voltage, v_emf_voltage, w_emf_voltage, degrees_to_radians(predicted_angle));

  const emf_voltage_angle_offset = radians_to_degrees(Math.atan2(web_beta_emf_voltage, web_alpha_emf_voltage));
  const web_emf_voltage_angle = normalize_degrees(predicted_angle + emf_voltage_angle_offset);
  const web_emf_voltage_magnitude = Math.sqrt(web_alpha_emf_voltage * web_alpha_emf_voltage + web_beta_emf_voltage * web_beta_emf_voltage);

  const web_total_power = -(u_current * u_drive_voltage + v_current * v_drive_voltage + w_current * w_drive_voltage);
  const web_emf_power = -(u_current * u_emf_voltage + v_current * v_emf_voltage + w_current * w_emf_voltage);
  const web_resistive_power = (square(u_current) * phase_resistance + square(v_current) * phase_resistance + square(w_current) * phase_resistance);
  const web_inductive_power = (u_current * u_L_voltage + v_current * v_L_voltage + w_current * w_L_voltage);

  const steady_state_beta_current = drive_voltage_magnitude / phase_resistance;


  const readout = accumulate_position_from_hall(
    {
      // Index
      readout_number,
      readout_index, 
      time,
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
      web_alpha_current, web_beta_current, 
      web_current_magnitude,
      web_inductor_angle, 

      u_current_diff, v_current_diff, w_current_diff,
      u_pwm, v_pwm, w_pwm,
      u_drive_voltage, v_drive_voltage, w_drive_voltage,
      drive_voltage_alpha,
      drive_voltage_beta,
      drive_voltage_angle, 
      drive_voltage_angle_offset,
      drive_voltage_magnitude,
      steady_state_beta_current,
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
      web_alpha_emf_voltage, web_beta_emf_voltage, 
      web_emf_voltage_magnitude,
      web_emf_voltage_angle, 
      emf_voltage_angle_offset,
      web_total_power,
      web_emf_power,
      web_resistive_power,
      web_inductive_power,
    }, 
    previous_readout, 
    this.position_calibration,
  );

  if (!previous_readout) return readout;

  // Time units are milliseconds.
  const dt = time - previous_readout.time;

  const exp_stats = exponential_stats(dt, 2.0);


  const {average: web_emf_voltage_magnitude_avg, stdev: web_emf_voltage_magnitude_stdev} = exp_stats(
    web_emf_voltage_magnitude,
    {
      average: previous_readout.web_emf_voltage_magnitude_avg,
      stdev: previous_readout.web_emf_voltage_magnitude_stdev,
    },
  );

  const {average: web_current_magnitude_avg, stdev: web_current_magnitude_stdev} = exp_stats(
    web_current_magnitude,
    {
      average: previous_readout.web_current_magnitude_avg,
      stdev: previous_readout.web_current_magnitude_stdev,
    },
  );

  return {
    ...readout,
    web_emf_voltage_magnitude_avg, 
    web_emf_voltage_magnitude_stdev,
    web_current_magnitude_avg, 
    web_current_magnitude_stdev,
  };
}


function parse_full_readout(data_view, previous_readout){
  if(check_message_for_errors(data_view, full_readout_size, command_codes.FULL_READOUT)) return null;

  const readout = parse_readout.call(this, data_view, previous_readout, false);

  if (!readout) return null;

  let offset = readout_size - tail_size;

  const tick_rate = data_view.getUint16(offset);
  offset += 2;
  const adc_update_rate = data_view.getUint16(offset);
  offset += 2;
  const temperature = calculate_temperature(data_view.getUint16(offset));
  offset += 2;
  const live_max_pwm = data_view.getInt16(offset);
  offset += 2;
  const cycle_start_tick = data_view.getInt16(offset);
  offset += 2;
  const cycle_end_tick = data_view.getInt16(offset);
  offset += 2;

  const alpha_current = current_conversion * data_view.getInt16(offset);
  offset += 2;
  const beta_current = current_conversion * data_view.getInt16(offset);
  offset += 2;
  const alpha_emf_voltage = calculate_voltage(data_view.getInt16(offset));
  offset += 2;
  const beta_emf_voltage = calculate_voltage(data_view.getInt16(offset));
  offset += 2;


  const total_power = convert_power_units_to_watts(data_view.getInt16(offset));
  offset += 2;
  const resistive_power = convert_power_units_to_watts(data_view.getInt16(offset));
  offset += 2;
  const emf_power = convert_power_units_to_watts(data_view.getInt16(offset));
  offset += 2;
  const inductive_power = convert_power_units_to_watts(data_view.getInt16(offset));
  offset += 2;

  const motor_constant = data_view.getInt16(offset);
  offset += 2;
  const inductor_angle = angle_units_to_degrees(data_view.getInt16(offset));
  offset += 2;
  const rotor_acceleration = acceleration_units_to_degrees_per_millisecond_squared(data_view.getInt16(offset));
  offset += 2;
  const rotations = data_view.getInt16(offset);
  offset += 2;
  const current_magnitude = current_conversion * data_view.getInt16(offset);
  offset += 2;
  const emf_angle_error_stdev = angle_units_to_degrees(Math.sqrt(data_view.getInt16(offset)));
  offset += 2;
  const lead_angle = angle_units_to_degrees(data_view.getInt16(offset));
  offset += 2;
  const target_pwm = data_view.getInt16(offset);
  offset += 2;
  

  const battery_current = total_power / readout.vcc_voltage;


  const inductor_angle_offset = normalize_degrees(inductor_angle - readout.predicted_angle);


  const full_readout = {
    ...readout,
    tick_rate,
    adc_update_rate,
    temperature,
    live_max_pwm,
    
    cycle_start_tick,
    cycle_end_tick,
    
    alpha_current,
    beta_current,
    alpha_emf_voltage,
    beta_emf_voltage,
    
    battery_current,
    total_power,
    resistive_power,
    emf_power,
    inductive_power,
    
    emf_angle_error_stdev,
    lead_angle,
    target_pwm,
    motor_constant,
    
    inductor_angle,
    inductor_angle_offset,
    
    rotor_acceleration,
    rotations,
    current_magnitude,
  };

  if (!previous_readout) return full_readout;

  const dt = readout.time - previous_readout.time;

  const exp_stats = exponential_stats(dt, 2.0);


    const {average: emf_power_avg, stdev: emf_power_stdev} = exp_stats(
    emf_power,
    {
      average: previous_readout.emf_power_avg,
      stdev: previous_readout.emf_power_stdev,
    },
  );

  const {average: total_power_avg, stdev: total_power_stdev} = exp_stats(
    total_power,
    {
      average: previous_readout.total_power_avg,
      stdev: previous_readout.total_power_stdev,
    },
  );

  return {
    ...full_readout,
    emf_power_avg, emf_power_stdev,
    total_power_avg, total_power_stdev,
  };
}


function parse_current_calibration(data_view){
  if(check_message_for_errors(data_view, current_calibration_size, command_codes.CURRENT_FACTORS)) return null;

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


function parse_position_calibration(data_view){
  if(check_message_for_errors(data_view, position_calibration_size, command_codes.HALL_POSITIONS)) return null;

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
    return angle_units_to_degrees(Math.sqrt(value));
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
    return angle_units_to_degrees(Math.sqrt(value));
  });

  return {
    sector_transition_degrees,
    sector_transition_stdev,
    sector_center_degrees,
    sector_center_stdev,
  };
}



function parse_control_parameters(data_view) {
  if(check_message_for_errors(data_view, control_parameters_size, command_codes.CONTROL_PARAMETERS)) return null;

  let offset = header_size;

  const rotor_angle_ki = data_view.getInt16(offset);
  offset += 2;
  const rotor_angular_speed_ki = data_view.getInt16(offset);
  offset += 2;
  const rotor_acceleration_ki = data_view.getInt16(offset);
  offset += 2;
  const motor_constant_ki = data_view.getInt16(offset);
  offset += 2;
  const resistance_ki = data_view.getInt16(offset);
  offset += 2;
  const inductance_ki = data_view.getInt16(offset);
  offset += 2;
  const max_pwm_change = data_view.getInt16(offset);
  offset += 2;
  const max_angle_change = data_view.getInt16(offset);
  offset += 2;
  const min_emf_voltage = data_view.getInt16(offset);
  offset += 2;
  const hall_angle_ki = data_view.getInt16(offset);
  offset += 2;
  const lead_angle_control_ki = data_view.getInt16(offset);
  offset += 2;
  const torque_control_ki = data_view.getInt16(offset);
  offset += 2;
  const battery_power_control_ki = data_view.getInt16(offset);
  offset += 2;
  const speed_control_ki = data_view.getInt16(offset);
  offset += 2;
  const probing_angular_speed = data_view.getInt16(offset);
  offset += 2;
  const probing_max_pwm = data_view.getInt16(offset);
  offset += 2;
  const emf_angle_error_variance_threshold = data_view.getInt16(offset);
  offset += 2;
  const min_emf_for_motor_constant = data_view.getInt16(offset);
  offset += 2;

  return {
    rotor_angle_ki,
    rotor_angular_speed_ki,
    rotor_acceleration_ki,
    motor_constant_ki,
    resistance_ki,
    inductance_ki,
    max_pwm_change,
    max_angle_change,
    min_emf_voltage,
    hall_angle_ki,
    lead_angle_control_ki,
    torque_control_ki,
    battery_power_control_ki,
    speed_control_ki,
    probing_angular_speed,
    probing_max_pwm,
    emf_angle_error_variance_threshold,
    min_emf_for_motor_constant,
  };
}


function parse_unit_test_output(data_view) {
  if(check_message_for_errors(data_view, unit_test_size, command_codes.UNIT_TEST_OUTPUT)) return null;

  // Get the length of the null terminated string.
  let len = 0;
  while (data_view.getUint8(header_size + len) !== 0 && len < unit_test_size - header_size - tail_size) len++;

  // Return the output as an utf-8 string.
  return new TextDecoder().decode(data_view.buffer.slice(header_size, header_size + len));
}


export const parser_mapping = {
  [command_codes.READOUT]: {parse_func: parse_readout, message_size: readout_size},
  [command_codes.FULL_READOUT]: {parse_func: parse_full_readout, message_size: full_readout_size},
  [command_codes.CURRENT_FACTORS]: {parse_func: parse_current_calibration, message_size: current_calibration_size},
  [command_codes.HALL_POSITIONS]: {parse_func: parse_position_calibration, message_size: position_calibration_size},
  [command_codes.CONTROL_PARAMETERS]: {parse_func: parse_control_parameters, message_size: control_parameters_size},
  [command_codes.UNIT_TEST_OUTPUT]: {parse_func: parse_unit_test_output, message_size: unit_test_size},
};


// Serialisation functions
// -----------------------

function serialise_message_tail(buffer, offset){
  // Check that we have exactly enough space to write the tail.
  if (buffer.length != offset + tail_size) throw new Error(
    `Message was not serialised correctly, expected size ${offset + tail_size}, got ${buffer.length}`);

  let view = new DataView(buffer.buffer);

  // TODO: Calculate the CRC and append it.
  // const crc = calculate_crc(buffer);

  const crc = 0;
  view.setUint32(offset, crc);
  offset += 4;
  view.setUint16(offset, END_OF_MESSAGE);
  offset += 2;
}

function serialise_current_calibration(current_calibration) {
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

  serialise_message_tail(buffer, offset);

  return buffer;
}

function serialise_set_position_calibration(position_calibration) {
  const {
    sector_transition_degrees, 
    sector_transition_stdev, 
    sector_center_degrees, 
    sector_center_stdev,
  } = position_calibration;

  let buffer = new Uint8Array(position_calibration_size);

  let view = new DataView(buffer.buffer);

  let offset = 0;
  view.setUint16(offset, command_codes.SET_HALL_POSITIONS);
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
      view.setUint16(offset, Math.pow(degrees_to_angle_units(sector_transition_stdev[i][j]), 2));
      offset += 2;
    }
  }

  for (let i = 0; i < 6; i++) {
    view.setUint16(offset, degrees_to_angle_units(sector_center_degrees[i]));
    offset += 2;
  }

  for (let i = 0; i < 6; i++) {
    // Send variance, not stdev.
    view.setUint16(offset, Math.pow(degrees_to_angle_units(sector_center_stdev[i]), 2));
    offset += 2;
  }

  serialise_message_tail(buffer, offset);

  return buffer;
}

function serialise_control_parameters(control_parameters) {
  let buffer = new Uint8Array(control_parameters_size);
  let view = new DataView(buffer.buffer);
  let offset = 0;
  view.setUint16(offset, command_codes.SET_CONTROL_PARAMETERS);
  offset += 2;

  view.setInt16(offset, control_parameters.rotor_angle_ki);
  offset += 2;
  view.setInt16(offset, control_parameters.rotor_angular_speed_ki);
  offset += 2;
  view.setInt16(offset, control_parameters.rotor_acceleration_ki);
  offset += 2;
  view.setInt16(offset, control_parameters.motor_constant_ki);
  offset += 2;
  view.setInt16(offset, control_parameters.resistance_ki);
  offset += 2;
  view.setInt16(offset, control_parameters.inductance_ki);
  offset += 2;
  view.setInt16(offset, control_parameters.max_pwm_change);
  offset += 2;
  view.setInt16(offset, control_parameters.max_angle_change);
  offset += 2;
  view.setInt16(offset, control_parameters.min_emf_voltage);
  offset += 2;
  view.setInt16(offset, control_parameters.hall_angle_ki);
  offset += 2;
  view.setInt16(offset, control_parameters.lead_angle_control_ki);
  offset += 2;
  view.setInt16(offset, control_parameters.torque_control_ki);
  offset += 2;
  view.setInt16(offset, control_parameters.battery_power_control_ki);
  offset += 2;
  view.setInt16(offset, control_parameters.speed_control_ki);
  offset += 2;
  view.setInt16(offset, control_parameters.probing_angular_speed);
  offset += 2;
  view.setInt16(offset, control_parameters.probing_max_pwm);
  offset += 2;
  view.setInt16(offset, control_parameters.emf_angle_error_variance_threshold);
  offset += 2;
  view.setInt16(offset, control_parameters.min_emf_for_motor_constant);
  offset += 2;

  serialise_message_tail(buffer, offset);

  return buffer;
}


const serialiser_mapping = {
  [command_codes.SET_CURRENT_FACTORS]: {serialise_func: serialise_current_calibration},
  [command_codes.SET_HALL_POSITIONS]: {serialise_func: serialise_set_position_calibration},
  [command_codes.SET_CONTROL_PARAMETERS]: {serialise_func: serialise_control_parameters},
};

export const default_command_options = {command_timeout: 0, command_value: 0, command_second: 0, command_third: 0, additional_data: undefined};

export function serialise_command({command, command_timeout, command_value, command_second, command_third, additional_data}) {
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
  view.setInt16(offset, command_value);
  offset += 2;
  view.setInt16(offset, command_second);
  offset += 2;
  view.setInt16(offset, command_third);
  offset += 2;

  serialise_message_tail(buffer, offset);

  return buffer;
}