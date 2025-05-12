import {pwm_base} from './motor_constants.js';

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
}

const readout_size = 28;

function parse_readout(data_view){
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
  const position_data = data_view.getUint16(offset);
  offset += 2;

  const speed = data_view.getInt16(offset);
  offset += 2;
  const vcc_voltage = data_view.getUint16(offset);
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
  const hall_u = (position_data >> 13) & 0b1;
  const hall_v = (position_data >> 14) & 0b1;
  const hall_w = (position_data >> 15) & 0b1;
  // The next bit is the motor angle valid flag.
  const motor_angle_valid = (position_data >> 12) & 0b1;
  // The last 10 bits are the motor angle. Representing range from 0 to 360 degrees,
  // where 0 means the rotor is aligned by holding positive current on the U phase.
  const motor_angle = position_data & 0x3FF;



  return {
    code: command_codes.READOUT,
    readout_number,
    u_readout,
    v_readout,
    w_readout,
    ref_readout,
    u_pwm,
    v_pwm,
    w_pwm,
    hall_u,
    hall_v,
    hall_w,
    motor_angle_valid,
    motor_angle,
    speed,
    vcc_voltage,
    torque,
    hold,
    total_power,
    resistive_power,
  };
}

const full_readout_size = readout_size + 20;

function parse_full_readout(data_view){
  const readout = parse_readout(data_view);
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

  const motor_current_angle = data_view.getInt16(offset);
  offset += 2;
  const motor_current_angle_stdev = data_view.getInt16(offset);
  offset += 2;

  return {
    ...readout,
    code: command_codes.FULL_READOUT,
    tick_rate,
    adc_update_rate,
    hall_unobserved_rate,
    hall_observed_rate,
    temperature_readout,
    vcc_readout,
    cycle_start_tick,
    cycle_end_tick,
    motor_current_angle,
    motor_current_angle_stdev,
  };
}

export const parser_mapping = {
  [command_codes.READOUT]: {parse_func: parse_readout, data_size: readout_size},
  [command_codes.FULL_READOUT]: {parse_func: parse_full_readout, data_size: full_readout_size},
};


export function serialise_command({command, command_timeout, command_pwm, command_leading_angle}) {
  let buffer = new Uint8Array(8);

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

  return buffer;
}