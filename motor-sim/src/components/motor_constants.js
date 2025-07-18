// Motor driver constants copied from the C++ code.
import {normalize_degrees, positive_degrees} from "./angular_math.js";

// Motor phase constants
// ---------------------


// Inductance per phase in Henries. Assuming the motor is a 3 phase star connected motor.
export const phase_inductance = 0.000_145; // 290 uH measured with LCR meter across phase pairs.

// Resistance per phase in Ohms. Assuming the motor is a 3 phase star connected motor.
export const phase_resistance = 2.00 * 2/3; // 2.00 Ohm measured with voltmeter between 1 phase and the other 2 in parallel.

// Time constant of the phase inductor circuit.
export const phase_time_constant = phase_inductance / phase_resistance;

// Timing constants
// ----------------

export const ticks_per_millisecond = 72000;

// PWM motor cycles per millisecond.
export const cycles_per_millisecond = 23437.5 / 1000.0; // 23437.5 cycles per second: 72MHz / (2*1536) / 1000.0

// Millisecond (fractions) per PWM motor cycle.
export const millis_per_cycle = 1.0/cycles_per_millisecond;

// Angle units
// -----------

export const angle_base = 1024;

// Convert degrees to angle units.
export function degrees_to_angle_units(degrees){
  return Math.round(positive_degrees(degrees) * angle_base / 360.0);
}

export function angle_units_to_degrees(angle){
  return normalize_degrees(unbounded_angle_units_to_degrees(angle));
}

export function unbounded_degrees_to_angle_units(degrees){
  return degrees * angle_base / 360.0;
}

export function unbounded_angle_units_to_degrees(angle){
  return angle * 360.0 / angle_base;
}


// Speed units
// -----------

export const speed_fixed_point = 128;

export function speed_units_to_degrees_per_millisecond(speed){
  return unbounded_angle_units_to_degrees(speed / speed_fixed_point) * cycles_per_millisecond;
}

export function degrees_per_millisecond_to_speed_units(speed){
  return unbounded_degrees_to_angle_units(speed * speed_fixed_point * millis_per_cycle);
}

export const acceleration_fixed_point = 512;

export function acceleration_units_to_degrees_per_millisecond_squared(acceleration){
  return speed_units_to_degrees_per_millisecond(acceleration / acceleration_fixed_point) * cycles_per_millisecond;
}

export function degrees_per_millisecond_squared_to_acceleration_units(acceleration){
  return degrees_per_millisecond_to_speed_units(acceleration * acceleration_fixed_point / cycles_per_millisecond);
}

// Bit handling constants
// ----------------------
export const hall_state_bit_offset = 0;
export const emf_detected_bit_offset = 11;
export const emf_fix_bit_offset = 10;
export const current_detected_bit_offset = 9;
export const current_fix_bit_offset = 8;

export const hall_state_bit_mask = 0b111 << hall_state_bit_offset;

// Motor control constants
// -----------------------

export const pwm_base = 1536; // 0x0600
export const pwm_period = 2 * pwm_base;
export const max_timeout = 0xFFFF;
export const history_size = 360;
export const readout_base = 0x10000;

export const voltage_reference = 3.3; // V
export const adc_base = 4096; // 12 bits
export const vcc_divider = 10.0/110.0; // 10k/110k divider

// Constants for the temperature sensor. This sensor isn't very accurate.
export const voltage_at_reference_temperature = 1.43; // V (varies between 1.34 and 1.52)
export const temperature_slope = 4.3; // mV/C (varies between 4.0 and 4.6)
export const temperature_celsius_reference = 25.0; // C

export function calculate_temperature(adc_reading){
  return (voltage_at_reference_temperature - adc_reading * voltage_reference / adc_base) * 1000 / temperature_slope + temperature_celsius_reference;
}

export function calculate_voltage(adc_reading){
  return adc_reading * voltage_reference / adc_base / vcc_divider;
}

export const adc_voltage_reference = 3.3;
export const motor_shunt_resistance = 0.010;
export const amplifier_gain = 20.0;
export const current_conversion = adc_voltage_reference / (adc_base * motor_shunt_resistance * amplifier_gain);

const power_fixed_point = 448; // Fixed point for power calculations.

// Convert power units to Watts.
export function convert_power_to_watts(power){
  return power / power_fixed_point;
}

// Maximum current we can measure in Amperes.
export const max_measurable_current = adc_base * current_conversion / 2; // Halved because we can measure negative current too.

export const expected_ref_readout = 2048; // Half of 12 bit ADC range. It should be half the circuit voltage, but... it ain't.

export const current_calibration_base = 1024; // Base for fixed point multiplication.

export const default_current_calibration = {
  u_factor: 1.0,
  v_factor: 1.0,
  w_factor: 1.0,
  inductance_factor: 1.0,
};


export const default_pid_parameters = {
  "current_angle_gains": {
    "kp": 128,
    "ki": 8,
    "kd": 64,
    "max_output": 256
  },
  "torque_gains": {
    "kp": 0,
    "ki": 8,
    "kd": 0,
    "max_output": 1450
  },
  "battery_power_gains": {
    "kp": 0,
    "ki": 8,
    "kd": 0,
    "max_output": 1450
  },
  "angular_speed_gains": {
    "kp": 1337,
    "ki": 16,
    "kd": 8,
    "max_output": 228
  },
  "position_gains": {
    "kp": 128,
    "ki": 16,
    "kd": 8,
    "max_output": 512
  }
};