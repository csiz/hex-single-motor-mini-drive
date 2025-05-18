import {process_readout} from './motor_observer.js';  

import {
  millis_per_cycle, pwm_base, readout_base,
  current_conversion, expected_ref_readout, calculate_temperature, calculate_voltage,
  angle_units_to_degrees, degrees_to_angle_units, current_calibration_base, unbounded_angle_units_to_degrees,
  speed_units_to_degrees_per_millisecond, degrees_per_millisecond_to_speed_units,
  acceleration_units_to_degrees_per_millisecond2, degrees_per_millisecond2_to_acceleration_units,
} from './motor_constants.js';

import {normalize_degrees} from './angular_math.js';

export const header_size = 2; // 2 bytes

export const command_codes = {
  READOUT: 0x2020,
  STREAM_FULL_READOUTS: 0x2021,
  GET_READOUTS_SNAPSHOT: 0x2022,
  FULL_READOUT: 0x2023,

  SET_STATE_OFF: 0x2030,
  SET_STATE_DRIVE_POS: 0x2031,
  SET_STATE_TEST_ALL_PERMUTATIONS: 0x2032,
  SET_STATE_DRIVE_NEG: 0x2033,
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
  SET_STATE_DRIVE_SMOOTH_POS: 0x4030,
  SET_STATE_DRIVE_SMOOTH_NEG: 0x4031,

  SET_CURRENT_FACTORS: 0x4040,
  SET_TRIGGER_ANGLES: 0x4041,
  CURRENT_FACTORS: 0x4042,
  TRIGGER_ANGLES: 0x4043,
  GET_CURRENT_FACTORS: 0x4044,
  GET_TRIGGER_ANGLES: 0x4045,
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

const readout_size = 28;

function parse_readout(data_view, previous_readout){
  let offset = 0;
  
  // Get the PWM commands.
  const pwm_commands = data_view.getUint32(offset);
  offset += 4;
  // Get the readout number, a counter that increments with each readout & PWM cycle.
  const readout_number = data_view.getUint16(offset);
  offset += 2;
  // Get the raw readout values.
  const u_readout = data_view.getUint16(offset);
  offset += 2;
  const v_readout = data_view.getUint16(offset);
  offset += 2;
  const w_readout = data_view.getUint16(offset);
  offset += 2;
  const ref_readout = data_view.getUint16(offset);
  offset += 2;
  // Get motor angle and hall sensor data.
  const position_readout = data_view.getUint16(offset);
  offset += 2;

  const angular_speed_readout = data_view.getInt16(offset);
  offset += 2;
  const vcc_readout = data_view.getUint16(offset);
  offset += 2;
  const torque = data_view.getInt16(offset);
  offset += 2;
  const hold = data_view.getInt16(offset);
  offset += 2;
  const total_power = data_view.getInt16(offset);
  offset += 2;
  const resistive_power = data_view.getInt16(offset);
  offset += 2;


  // We use 1536 ticks per PWM cycle, we can pack 3 values in 32 bits with the formula: (pwm_u*pwm_base + pwm_v)*pwm_base + pwm_w
  const u_pwm = Math.floor(pwm_commands / pwm_base / pwm_base) % pwm_base;
  const v_pwm = Math.floor(pwm_commands / pwm_base) % pwm_base;
  const w_pwm = pwm_commands % pwm_base;

  // The first 3 bits are the hall sensor state.
  const hall_u = (position_readout >> 13) & 0b1;
  const hall_v = (position_readout >> 14) & 0b1;
  const hall_w = (position_readout >> 15) & 0b1;

  const hall_sector = get_hall_sector({hall_u, hall_v, hall_w});

  // The next bit is the motor angle valid flag.
  const angle_valid = (position_readout >> 12) & 0b1;
  // The last 10 bits are the motor angle. Representing range from 0 to 360 degrees,
  // where 0 means the rotor is aligned by holding positive current on the U phase.
  const angle = angle_units_to_degrees(position_readout & 0x3FF);

  const angular_speed = speed_units_to_degrees_per_millisecond(angular_speed_readout);

  const ref_diff = current_conversion * (ref_readout - expected_ref_readout);

  const u_uncalibrated = -current_conversion * (u_readout - ref_readout);
  // Flip the sign of V because we accidentally wired it the other way. Oopsie doopsie.
  const v_uncalibrated = +current_conversion * (v_readout - ref_readout);
  const w_uncalibrated = -current_conversion * (w_readout - ref_readout);

  const u_calibrated = u_uncalibrated * this.current_calibration.u_factor;
  const v_calibrated = v_uncalibrated * this.current_calibration.v_factor;
  const w_calibrated = w_uncalibrated * this.current_calibration.w_factor;


  const sum = u_calibrated + v_calibrated + w_calibrated;

  const u = u_calibrated - sum / 3.0;
  const v = v_calibrated - sum / 3.0;
  const w = w_calibrated - sum / 3.0;

  // Approximate the angle for the 6 sectors of the hall sensor.
  const ε = 2;
  const hall_u_as_angle = hall_u ? hall_v ? + 60 - ε : hall_w ? - 60 + ε :    0 : null;
  const hall_v_as_angle = hall_v ? hall_u ? + 60 + ε : hall_w ? +180 - ε : +120 : null;
  const hall_w_as_angle = hall_w ? hall_v ? -180 + ε : hall_u ? - 60 - ε : -120 : null;

  const vcc_voltage = calculate_voltage(vcc_readout);

  // Accumulate the readout index across readouts because the readout number is reset every 65536 readouts (~3 seconds).
  const readout_diff = !previous_readout ? 0 : (readout_base + readout_number - previous_readout.readout_number) % readout_base;
  const readout_index = !previous_readout ? 0 : previous_readout.readout_index + readout_diff;
  const time = readout_index * millis_per_cycle;

  const readout = {
    readout_number,
    readout_index, time,
    u_readout, v_readout, w_readout,
    ref_readout,
    ref_diff,
    u, v, w,
    u_uncalibrated, v_uncalibrated, w_uncalibrated,
    sum,
    u_pwm, v_pwm, w_pwm,
    hall_u, hall_v, hall_w, hall_sector,
    hall_u_as_angle, hall_v_as_angle, hall_w_as_angle,
    angle_valid, angle, angular_speed,
    vcc_voltage,
    torque,
    hold,
    total_power,
    resistive_power,
  };

  return process_readout.call(this, readout, previous_readout);
}

const full_readout_size = readout_size + 24;

function parse_full_readout(data_view, previous_readout){
  const readout = parse_readout.call(this, data_view, previous_readout);

  let offset = readout_size;

  const tick_rate = data_view.getUint16(offset);
  offset += 2;
  const adc_update_rate = data_view.getUint16(offset);
  offset += 2;
  const hall_unobserved_rate = data_view.getUint16(offset);
  offset += 2;
  const hall_observed_rate = data_view.getUint16(offset);
  offset += 2;
  const temperature_readout = data_view.getUint16(offset);
  offset += 2;
  const vcc_readout = data_view.getUint16(offset);
  offset += 2;
  const cycle_start_tick = data_view.getInt16(offset);
  offset += 2;
  const cycle_end_tick = data_view.getInt16(offset);
  offset += 2;

  const motor_current_angle_readout = data_view.getInt16(offset);
  offset += 2;
  const motor_current_angle_variance = data_view.getUint16(offset);
  offset += 2;

  const angle_variance = data_view.getUint16(offset);
  offset += 2;
  const angular_speed_variance = data_view.getUint16(offset);
  offset += 2;

  const motor_current_angle = angle_units_to_degrees(motor_current_angle_readout);
  const motor_current_angle_stdev = unbounded_angle_units_to_degrees(Math.sqrt(motor_current_angle_variance));
  
  const vcc_voltage = calculate_voltage(vcc_readout);
  const temperature = calculate_temperature(temperature_readout);

  const angle_stdev = unbounded_angle_units_to_degrees(Math.sqrt(angle_variance));
  const angular_speed_stdev = speed_units_to_degrees_per_millisecond(Math.sqrt(angular_speed_variance));

  return {
    ...readout,
    tick_rate,
    adc_update_rate,
    hall_unobserved_rate,
    hall_observed_rate,
    temperature,
    vcc_voltage,
    cycle_start_tick,
    cycle_end_tick,
    motor_current_angle,
    motor_current_angle_stdev,
    angle_stdev,
    angular_speed_stdev,
  };
}



const current_calibration_size = 6;
function parse_current_calibration(data_view){
  let offset = 0;
  const u_factor = data_view.getInt16(offset) / current_calibration_base;
  offset += 2;
  const v_factor = data_view.getInt16(offset) / current_calibration_base;
  offset += 2;
  const w_factor = data_view.getInt16(offset) / current_calibration_base;
  offset += 2;

  return {
    u_factor,
    v_factor,
    w_factor,
  };
}

export function serialise_current_calibration(current_calibration) {
  const {u_factor, v_factor, w_factor} = current_calibration;
  
  let buffer = new Uint8Array(current_calibration_size);

  let view = new DataView(buffer.buffer);
  
  let offset = 0;
  view.setInt16(offset, Math.floor(u_factor * current_calibration_base));
  offset += 2;
  view.setInt16(offset, Math.floor(v_factor * current_calibration_base));
  offset += 2;
  view.setInt16(offset, Math.floor(w_factor * current_calibration_base));
  offset += 2;

  return buffer;
}

const position_calibration_size = 76;
function parse_position_calibration(data_view){
  let offset = 0;
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

  // We receive the variance, not the stdev.
  const initial_angular_speed_stdev = speed_units_to_degrees_per_millisecond(Math.sqrt(data_view.getUint16(offset)));
  offset += 2;
  const angular_acceleration_stdev = acceleration_units_to_degrees_per_millisecond2(Math.sqrt(data_view.getUint16(offset)));
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

function square(x){
  return x * x;
}

export function serialise_position_calibration(position_calibration) {
  const {
    sector_transition_degrees, sector_transition_stdev, sector_center_degrees, sector_center_stdev,
    initial_angular_speed_stdev, angular_acceleration_stdev,
  } = position_calibration;

  let buffer = new Uint8Array(position_calibration_size);

  let view = new DataView(buffer.buffer);

  let offset = 0;

  for (let i = 0; i < 6; i++) {
    for (let j = 0; j < 2; j++) {
      view.setUint16(offset, degrees_to_angle_units(sector_transition_degrees[i][j]));
      offset += 2;
    }
  }

  for (let i = 0; i < 6; i++) {
    for (let j = 0; j < 2; j++) {
      // Send variance, not stdev.
      view.setUint16(offset, square(degrees_to_angle_units(sector_transition_stdev[i][j])));
      offset += 2;
    }
  }

  for (let i = 0; i < 6; i++) {
    view.setUint16(offset, degrees_to_angle_units(sector_center_degrees[i]));
    offset += 2;
  }

  for (let i = 0; i < 6; i++) {
    // Send variance, not stdev.
    view.setUint16(offset, square(degrees_to_angle_units(sector_center_stdev[i])));
    offset += 2;
  }

  view.setUint16(offset, square(degrees_per_millisecond_to_speed_units(initial_angular_speed_stdev)));
  offset += 2;
  view.setUint16(offset, square(degrees_per_millisecond2_to_acceleration_units(angular_acceleration_stdev)));
  offset += 2;

  return buffer;
}

export const serialiser_mapping = {
  [command_codes.SET_CURRENT_FACTORS]: {serialise_func: serialise_current_calibration, extra_size: current_calibration_size},
  [command_codes.SET_TRIGGER_ANGLES]: {serialise_func: serialise_position_calibration, extra_size: position_calibration_size},
};

export const parser_mapping = {
  [command_codes.READOUT]: {parse_func: parse_readout, data_size: readout_size},
  [command_codes.FULL_READOUT]: {parse_func: parse_full_readout, data_size: full_readout_size},
  [command_codes.CURRENT_FACTORS]: {parse_func: parse_current_calibration, data_size: current_calibration_size},
  [command_codes.TRIGGER_ANGLES]: {parse_func: parse_position_calibration, data_size: position_calibration_size},
};


export const command_header_size = 8;
export function serialise_command({command, command_timeout = 0, command_pwm = 0, command_leading_angle = 0, additional_data = undefined}) {
  const {serialise_func, extra_size} = serialiser_mapping[command] ?? {serialise_func: null, extra_size: 0};

  let buffer = new Uint8Array(command_header_size + extra_size);

  let view = new DataView(buffer.buffer);
  let offset = 0;
  view.setUint16(offset, command);
  offset += 2;
  view.setUint16(offset, command_timeout);
  offset += 2;
  view.setUint16(offset, command_pwm);
  offset += 2;
  view.setUint16(offset, command_leading_angle);
  offset += 2;

  if (serialise_func) {
    const additional_buffer = serialise_func.call(this, additional_data);
    buffer.set(additional_buffer, offset);
    offset += additional_buffer.length;
  }

  return buffer;
}