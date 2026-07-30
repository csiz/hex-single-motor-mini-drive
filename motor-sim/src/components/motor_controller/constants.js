// Motor driver constants to convert to standard units.
import {
  HISTORY_SIZE, 
  PWM_BASE,
  CLOCK_FREQUENCY,
  CURRENT_UNITS_PER_AMP,
  VOLTAGE_UNITS_PER_VOLT,
} from "hex-mini-drive-interface";

import {normalize_degrees, positive_degrees} from "./angular_math.js";


export {
  HISTORY_SIZE, 
  PWM_BASE, 
  CLOCK_FREQUENCY,  
  CURRENT_UNITS_PER_AMP,
  VOLTAGE_UNITS_PER_VOLT,
};

// Timing constants
// ----------------

export const pwm_period = 2 * PWM_BASE;

export const pwm_cycles_per_second = Math.floor(CLOCK_FREQUENCY / (2*PWM_BASE));

// PWM motor cycles per millisecond.
export const cycles_per_millisecond = pwm_cycles_per_second / 1000.0;

// Millisecond (fractions) per PWM motor cycle.
export const millis_per_cycle = 1.0/cycles_per_millisecond;

// Angle units
// -----------

export const angle_base = 1048576.0;

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

const max_rpm = 32000.0 * rotor_revolutions_per_electric;

// Gear ratio of our chosen motor.
const gear_ratio = 6 * 6 * 6;

// Total ratio between the electrical angle and the output shaft angle.
const ratio = rotor_revolutions_per_electric * gear_ratio;

// Maximum angular speed that we can command the driver.
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

export const max_timeout = 0xFFFF;

export const readout_base = 0x10000;

// Constants for the temperature sensor. This sensor isn't very accurate.
// TODO: do on chip side
const voltage_at_reference_temperature = 1.43; // V (varies between 1.34 and 1.52)
const temperature_slope = 4.3; // mV/C (varies between 4.0 and 4.6)
const temperature_celsius_reference = 25.0; // C

export function calculate_temperature(adc_reading){
  return (voltage_at_reference_temperature - adc_reading * 2.9 / 4096) * 1000 / temperature_slope + temperature_celsius_reference;
}

export const max_drive_current = 6.0;

export const max_drive_power = 12;