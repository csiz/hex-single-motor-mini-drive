import {
  PWM_BASE,
  CURRENT_UNITS_PER_AMP,
  VOLTAGE_UNITS_PER_VOLT,
  UNIT_TEST_OUTPUT_SIZE,
  VERSION_STRING_SIZE,
  millis_per_cycle,
  pwm_cycles_per_second,
  readout_base,
  calculate_temperature, 
  angle_units_to_radians, 
  radians_to_angle_units,
  speed_units_to_rotations_per_millisecond,
  parse_state_flags, get_hall_sector,
} from './constants.js';

import {normalize_radians} from './angular_math.js';
import {square, dq0_transform, abc_transform} from './math_utils.js';



// Post process a raw readout from the motor driver. We convert to useful units, add 
// timekeeping, and add calibration computations (that are too heavy to do on chip).
export function parse_readout(bare_readout, previous_readout) {

  // Get the readout number, a counter that increments with each readout & PWM cycle.
  const readout_number = bare_readout.readout_number;
  const cycle_start_tick = bare_readout.cycle_start_tick;
  const cycle_end_tick = bare_readout.cycle_end_tick;
  const ref_readout = bare_readout.ref_readout;

  // Time indexing
  // -------------
  const local_time = bare_readout.local_time;

  // It's the first readout in the series if the previous readout is older than the timeout duration.
  const is_first_readout = !previous_readout;

  // Accumulate the readout index across readouts because the readout number is reset every 65536 readouts (~3 seconds).
  const readout_number_diff = is_first_readout ? undefined : (readout_base + readout_number - previous_readout.readout_number) % readout_base;

  const readout_index = is_first_readout ? 0 : previous_readout.readout_index + readout_number_diff;

  const time = is_first_readout ? 0 : readout_index * millis_per_cycle;

  // Time units are milliseconds.
  const dt = is_first_readout ? undefined : time - previous_readout.time;


  // Unit Conversions
  // ----------------

  // Get the raw readout values.
  const x_current = bare_readout.x_current / CURRENT_UNITS_PER_AMP;
  const y_current = bare_readout.y_current / CURRENT_UNITS_PER_AMP;
  const zero_current = bare_readout.zero_current / CURRENT_UNITS_PER_AMP;
  
  const x_current_diff = bare_readout.x_current_diff / CURRENT_UNITS_PER_AMP;
  const y_current_diff = bare_readout.y_current_diff / CURRENT_UNITS_PER_AMP;

  // Get electric angle data. Angle 0 means the rotor North is aligned when holding positive current on the U phase.
  const angle = angle_units_to_radians(bare_readout.angle);
  const predicted_angle = angle_units_to_radians(bare_readout.predicted_angle);
  const angle_adjustment = normalize_radians(angle - predicted_angle);
  const angular_speed = speed_units_to_rotations_per_millisecond(bare_readout.angular_speed);
  const vcc_voltage = bare_readout.vcc_voltage / VOLTAGE_UNITS_PER_VOLT;

  const emf_voltage_angle = angle_units_to_radians(bare_readout.emf_voltage_angle);
  
  const emf_voltage_magnitude = bare_readout.emf_voltage_magnitude / VOLTAGE_UNITS_PER_VOLT;
  const emf_magnitude_stdev = Math.sqrt(bare_readout.emf_magnitude_variance) / VOLTAGE_UNITS_PER_VOLT;
  const emf_voltage_angular_speed = speed_units_to_rotations_per_millisecond(bare_readout.emf_voltage_angular_speed);
  const emf_angle_stdev = angle_units_to_radians(Math.sqrt(bare_readout.emf_angle_variance));

  const x_drive_voltage = bare_readout.x_drive_voltage / VOLTAGE_UNITS_PER_VOLT;
  const y_drive_voltage = bare_readout.y_drive_voltage / VOLTAGE_UNITS_PER_VOLT;

  const emf_speed_constant = bare_readout.emf_speed_constant;
  const emf_speed_constant_stdev = Math.sqrt(bare_readout.emf_speed_constant_variance);

  const current_angle_offset = angle_units_to_radians(bare_readout.current_angle_offset);
  const current_angle = normalize_radians(predicted_angle + current_angle_offset);
  const current_magnitude = bare_readout.current_magnitude / CURRENT_UNITS_PER_AMP;

  const live_max_pwm = bare_readout.live_max_pwm;

  const total_power_average = bare_readout.total_power_average;
  const resistive_power_average = bare_readout.resistive_power_average;

  const rotations = bare_readout.rotations;
  
  const emf_stable_magnitude = bare_readout.emf_stable_magnitude / VOLTAGE_UNITS_PER_VOLT;

  const active_pwm = bare_readout.active_pwm;

  const active_angle = angle_units_to_radians(bare_readout.active_angle);

  const probing_angle = angle_units_to_radians(bare_readout.probing_angle);


  // Additional computations
  // -----------------------

  const [u_current, v_current, w_current] = abc_transform(x_current, y_current, 0);

  const prev_x_current = x_current - x_current_diff;
  const prev_y_current = y_current - y_current_diff;
  const prev_current_magnitude = Math.sqrt(square(prev_x_current) + square(prev_y_current));
  const [prev_u_current, prev_v_current, prev_w_current] = abc_transform(prev_x_current, prev_y_current, 0);

  
  const u_current_diff = u_current - prev_u_current;
  const v_current_diff = v_current - prev_v_current;
  const w_current_diff = w_current - prev_w_current;

  const x_emf_voltage = emf_voltage_magnitude * Math.cos(emf_voltage_angle);
  const y_emf_voltage = emf_voltage_magnitude * Math.sin(emf_voltage_angle);
  

  // Get the PWM voltage commands.
  const [u_drive_voltage, v_drive_voltage, w_drive_voltage] = abc_transform(x_drive_voltage, y_drive_voltage, 0);

  // Derive the PWM duty cycles given the VCC voltage. (Done this way to avoid duplicating calculations
  // on the microcontroller.)
  const min_drive_voltage = Math.min(u_drive_voltage, v_drive_voltage, w_drive_voltage);
  const u_pwm = (u_drive_voltage - min_drive_voltage) / vcc_voltage * PWM_BASE;
  const v_pwm = (v_drive_voltage - min_drive_voltage) / vcc_voltage * PWM_BASE;
  const w_pwm = (w_drive_voltage - min_drive_voltage) / vcc_voltage * PWM_BASE;


  const drive_voltage_angle = Math.atan2(y_drive_voltage, x_drive_voltage);
  const drive_voltage_angle_offset = normalize_radians(drive_voltage_angle - predicted_angle);
  const drive_voltage_magnitude = Math.sqrt(square(x_drive_voltage) + square(y_drive_voltage));

  const emf_voltage_angle_offset = normalize_radians(emf_voltage_angle - predicted_angle);

  const battery_current = total_power_average / vcc_voltage;


  // Recalculate calibration stuff so we can vizualize it
  // ----------------------------------------------------

  // Calculate the resistive voltage drop across the coil and MOSFET resistance.
  const x_resistive_voltage = x_current * bare_readout.resistance;
  const y_resistive_voltage = y_current * bare_readout.resistance;

  // And the current diffs for calculating inductance.
  const x_diff = x_current_diff * pwm_cycles_per_second;
  const y_diff = y_current_diff * pwm_cycles_per_second;

  // Note: The saliency effect is due to the iron between magnets so it occurs twice per 
  // electrical revolution, once for each pole. Thus we must use the doubled rotor angle.

  const sin_saliency = Math.sin(bare_readout.saliency_angle);
  const cos_saliency = Math.cos(bare_readout.saliency_angle);

  const x_inductance_saliency = (
      -x_diff * cos_saliency 
      -y_diff * sin_saliency
  );

  const x_inductance_saliency_voltage = bare_readout.inductance_saliency * x_inductance_saliency;
  
  const y_inductance_saliency = (
      -x_diff * sin_saliency
      +y_diff * cos_saliency
  );
  const y_inductance_saliency_voltage = bare_readout.inductance_saliency * y_inductance_saliency;

  const x_inductance_base_voltage = x_diff * bare_readout.inductance;
  const y_inductance_base_voltage = y_diff * bare_readout.inductance;

  const x_inductor_voltage = x_inductance_base_voltage + x_inductance_saliency_voltage;
  const y_inductor_voltage = y_inductance_base_voltage + y_inductance_saliency_voltage;
  
  const sin_iron_loss = Math.sin(bare_readout.iron_loss_angle);
  const cos_iron_loss = Math.cos(bare_readout.iron_loss_angle);

  const A_iron_loss = (
    x_current * x_inductor_voltage - 
    y_current * y_inductor_voltage
  );
  const B_iron_loss = (
    x_current * y_inductor_voltage + 
    y_current * x_inductor_voltage
  );

  const x_iron_loss = A_iron_loss * cos_iron_loss - B_iron_loss * sin_iron_loss;
  const y_iron_loss = A_iron_loss * sin_iron_loss + B_iron_loss * cos_iron_loss;

  const x_iron_loss_voltage = bare_readout.iron_loss * x_iron_loss;
  const y_iron_loss_voltage = bare_readout.iron_loss * y_iron_loss;

  // Calibrate without EMF!!!
  // ------------------------
  // 
  // It's very important that we don't calibrate while the motor is rotating and generating EMF.
  // During sensorless running we infer the EMF as the error of the electrical equation, but
  // that means we can't use the inferred EMF to calibrate the electrical parameters accurately.
  // Especially because the resistance voltage and EMF voltage are covarying quantities.
  // Inductance and the saliency bias are also highly sensitive to any lag in the EMF estimation.

  const x_voltage_error = x_inductor_voltage + x_resistive_voltage + x_iron_loss_voltage - x_drive_voltage;
  const y_voltage_error = y_inductor_voltage + y_resistive_voltage + y_iron_loss_voltage - y_drive_voltage;

  // Defining the loss as the square(x_voltage_error) + square(y_voltage_error) we get the derivative gradients with 
  // respect to each of the parameters we're tracking. We only care about the sign so we should be careful to skip updates when 
  // there isn't enough energy in the error to warrant an update.
  const voltage_error = Math.sqrt(square(x_voltage_error) + square(y_voltage_error));


  // Return the complete readout
  // ---------------------------

  return {
    ...bare_readout,
    local_time,
    // Index
    readout_number,
    readout_number_diff,
    readout_index,
    time,
    dt,
    cycle_start_tick,
    cycle_end_tick,

    // Reference readout in adc units
    ref_readout,

    // Readouts converted to physical dimensions
    x_current, y_current,
    x_current_diff, y_current_diff,
    
    u_current, v_current, w_current, zero_current,


    prev_u_current, prev_v_current, prev_w_current,
    prev_x_current, prev_y_current,
    prev_current_magnitude,
    u_current_diff, v_current_diff, w_current_diff,
    u_pwm, v_pwm, w_pwm,
    u_drive_voltage, v_drive_voltage, w_drive_voltage,
    x_drive_voltage, y_drive_voltage,
    drive_voltage_angle, drive_voltage_angle_offset,
    drive_voltage_magnitude,
    angle,
    predicted_angle,
    angle_adjustment,
    angular_speed,
    vcc_voltage,
    emf_voltage_angle, emf_voltage_angle_offset,
    emf_angle_stdev,
    emf_voltage_angular_speed,
    emf_voltage_magnitude, 
    emf_magnitude_stdev,
    emf_speed_constant,
    emf_speed_constant_stdev,
    x_emf_voltage, y_emf_voltage,
    current_angle, current_angle_offset,
    current_magnitude,
    live_max_pwm,
    
    battery_current,
    total_power_average,
    resistive_power_average,
    rotations,

    emf_stable_magnitude,
    active_pwm,
    active_angle,
    probing_angle,


    // Calibration deduced voltages
    x_resistive_voltage,
    y_resistive_voltage,
    x_inductance_saliency_voltage,
    y_inductance_saliency_voltage,
    x_inductance_base_voltage,
    y_inductance_base_voltage,
    x_iron_loss_voltage,
    y_iron_loss_voltage,
    x_voltage_error,
    y_voltage_error,
    voltage_error
  };
}


export function parse_unit_test_output(bare_unit_test) {
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

export function parse_version(bare_version) {
  // Convert number array to Uint8Array.
  const uint8_array = new Uint8Array(bare_version);

  // Get dataview from test data buffer.
  const data_view = new DataView(uint8_array.buffer);

  // Get the length of the null terminated string.
  let len = 0;
  while (data_view.getUint8(len) !== 0 && len < VERSION_STRING_SIZE) len++;

  // Return the output as an utf-8 string.
  return new TextDecoder().decode(data_view.buffer.slice(0, len));
}

export function parse_info(bare_info) {
  // Are we giving up on hall sensors completely?
  // const {
  //   hall_state,
  //   emf_detected,
  //   emf_fix,
  //   current_detected,
  // } = parse_state_flags(state_flags);

  // const hall_u = Boolean(hall_state & 0b001) * 1;
  // const hall_v = Boolean(hall_state & 0b010) * 1;
  // const hall_w = Boolean(hall_state & 0b100) * 1;

  // const hall_sector = get_hall_sector({hall_u, hall_v, hall_w});

  return {
    ...bare_info,
    temperature: calculate_temperature(bare_info.temperature),
  };
}

export function parse_calibration(bare_calibration) {
  return {
    ...bare_calibration,
    u_current_zero: bare_calibration.u_current_zero / CURRENT_UNITS_PER_AMP,
    v_current_zero: bare_calibration.v_current_zero / CURRENT_UNITS_PER_AMP,
    w_current_zero: bare_calibration.w_current_zero / CURRENT_UNITS_PER_AMP,
  };
} 

export function make_calibration(calibration){
  return {
    ...calibration,
    u_current_zero: calibration.u_current_zero * CURRENT_UNITS_PER_AMP,
    v_current_zero: calibration.v_current_zero * CURRENT_UNITS_PER_AMP,
    w_current_zero: calibration.w_current_zero * CURRENT_UNITS_PER_AMP,
  };
}

export function parse_calibration_instance(bare_calibration_instance){
  return {
    ...bare_calibration_instance,
    target_speed: speed_units_to_rotations_per_millisecond(bare_calibration_instance.target_speed),
    avg_drive_voltage: bare_calibration_instance.avg_drive_voltage / VOLTAGE_UNITS_PER_VOLT,
    avg_current_magnitude: bare_calibration_instance.avg_current_magnitude / CURRENT_UNITS_PER_AMP,
    avg_error: Math.sqrt(bare_calibration_instance.avg_square_error) / VOLTAGE_UNITS_PER_VOLT,
    iron_loss: bare_calibration_instance.iron_loss / VOLTAGE_UNITS_PER_VOLT,
    saliency_angle: angle_units_to_radians(bare_calibration_instance.saliency_angle),
    iron_loss_angle: angle_units_to_radians(bare_calibration_instance.iron_loss_angle),
  }
}

export function parse_control_parameters(bare_control_parameters){
  return {
    ...bare_control_parameters,
    angle_zero: angle_units_to_radians(bare_control_parameters.angle_zero),
  };
}

export function make_control_parameters(control_parameters){
  return {
    ...control_parameters,
    angle_zero: radians_to_angle_units(control_parameters.angle_zero),
  };
}