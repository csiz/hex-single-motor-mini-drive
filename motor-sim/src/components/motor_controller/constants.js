// Motor driver constants copied from the C++ code.

export {HISTORY_SIZE} from "hex-mini-drive-interface";

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

export const ticks_per_millisecond = 144000;

export const pwm_cycles_per_second = 23437;

// PWM motor cycles per millisecond.
export const cycles_per_millisecond = pwm_cycles_per_second / 1000.0;

// Millisecond (fractions) per PWM motor cycle.
export const millis_per_cycle = 1.0/cycles_per_millisecond;

// Angle units
// -----------

export const angle_base = 1048576;

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

export function speed_units_to_degrees_per_millisecond(speed){
  return unbounded_angle_units_to_degrees(speed) * cycles_per_millisecond;
}

export function degrees_per_millisecond_to_speed_units(speed){
  return unbounded_degrees_to_angle_units(speed / cycles_per_millisecond);
}

// Number of electrical revolutions per mechanical revolution. This is pole pairs times the number of slot triplets.
const rotor_revolutions_per_electric = 4;

const max_rpm = 32000 * rotor_revolutions_per_electric;

// Gear ratio of our chosen motor.
const gear_ratio = 6 * 6 * 6;

// Total ratio between the electrical angle and the output shaft angle.
const ratio = rotor_revolutions_per_electric * gear_ratio;

export const max_angular_speed = speed_units_to_degrees_per_millisecond(1.0 * max_rpm * angle_base / 60.0 / pwm_cycles_per_second);

export function acceleration_units_to_degrees_per_millisecond_squared(acceleration){
  return speed_units_to_degrees_per_millisecond(acceleration) * cycles_per_millisecond;
}

export function degrees_per_millisecond_squared_to_acceleration_units(acceleration){
  return degrees_per_millisecond_to_speed_units(acceleration / cycles_per_millisecond);
}


// Bit handling constants
// ----------------------
export const hall_state_bit_offset = 0;
export const emf_detected_bit_offset = 11;
export const emf_fix_bit_offset = 10;
export const current_detected_bit_offset = 9;
export const angle_fix_bit_offset = 8;
export const incorrect_rotor_angle_bit_offset = 7;
export const rotor_direction_flip_imminent_bit_offset = 6;

export const hall_state_bit_mask = 0b111 << hall_state_bit_offset;

export function parse_state_flags(state_flags){
  return {
    hall_state: (state_flags & hall_state_bit_mask) >> hall_state_bit_offset,
    emf_detected: (state_flags >> emf_detected_bit_offset) & 0b1,
    emf_fix: (state_flags >> emf_fix_bit_offset) & 0b1,
    current_detected: (state_flags >> current_detected_bit_offset) & 0b1,
    angle_fix: (state_flags >> angle_fix_bit_offset) & 0b1,
    incorrect_rotor_angle: (state_flags >> incorrect_rotor_angle_bit_offset) & 0b1,
    rotor_direction_flip_imminent: (state_flags >> rotor_direction_flip_imminent_bit_offset) & 0b1,
  };
}

// Motor control constants
// -----------------------

export const pwm_base = 3072;
export const pwm_period = 2 * pwm_base;
export const max_timeout = 0xFFFF;
export const readout_base = 0x10000;

export const voltage_reference = 2.9; // V
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

export const motor_shunt_resistance = 0.010;
export const amplifier_gain = 20.0;
export const current_conversion = voltage_reference / (adc_base * motor_shunt_resistance * amplifier_gain);

const power_fixed_point = 448; // Fixed point for power calculations.

// Convert power units to Watts.
export function convert_power_units_to_watts(power){
  return power / power_fixed_point;
}

// Convert Watts to power units.
export function convert_watts_to_power_units(watts){
  return watts * power_fixed_point;
}

// Maximum current we can measure in Amperes.
export const max_measurable_current = adc_base * current_conversion / 2; // Halved because we can measure negative current too.

export const max_drive_current = 6.0;

export const max_drive_power = 12;

// The reference voltage should be 3.3V/2. With the internal voltage reference at 2.9V 
// this means we expect to read 3.3/2 * 4096 / 2.9 = 2325.5. 
export const expected_ref_readout = 2326;

// Base for fixed point multiplication.
export const current_calibration_base = 1024;
