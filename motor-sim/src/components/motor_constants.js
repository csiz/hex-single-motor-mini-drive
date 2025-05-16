// Motor driver constants copied from the C++ code.
import {normalize_degrees} from "./angular_math.js";

// PWM motor cycles per millisecond.
export const cycles_per_millisecond = 23437.5 / 1000.0; // 23437.5 cycles per second: 72MHz / (2*1536) / 1000.0

// Millisecond (fractions) per PWM motor cycle.
export const millis_per_cycle = 1.0/cycles_per_millisecond;


export const angle_base = 1024;

export const pwm_base = 1536; // 0x0600
export const pwm_period = 2 * pwm_base;
export const max_timeout = 0xFFFF;
export const history_size = 360;
export const readout_base = 0x10000;

export const voltage_reference = 3.3; // V
export const adc_base = 4096; // 12 bits
export const VCC_divider = 10.0/110.0; // 10k/110k divider

// Constants for the temperature sensor. This sensor isn't very accurate.
export const voltage_at_reference_temperature = 1.43; // V (varies between 1.34 and 1.52)
export const temperature_slope = 4.3; // mV/C (varies between 4.0 and 4.6)
export const temperature_celsius_reference = 25.0; // C

export function calculate_temperature(adc_reading){
  return (voltage_at_reference_temperature - adc_reading * voltage_reference / adc_base) * 1000 / temperature_slope + temperature_celsius_reference;
}

export function calculate_voltage(adc_reading){
  return adc_reading * voltage_reference / adc_base / VCC_divider;
}

export const adc_voltage_reference = 3.3;
export const motor_shunt_resistance = 0.010;
export const amplifier_gain = 20.0;
export const current_conversion = adc_voltage_reference / (adc_base * motor_shunt_resistance * amplifier_gain);

export const expected_ref_readout = 2048; // Half of 12 bit ADC range. It should be half the circuit voltage, but... it ain't.


export const current_calibration_default = {
  u_positive: 1.0,
  u_negative: 1.0,
  v_positive: 1.0,
  v_negative: 1.0,
  w_positive: 1.0,
  w_negative: 1.0,
};


const hall_hysterisis = 10;
const transition_stdev = 15;

export const position_calibration_default = {
  sector_center_degrees: [0, 60, 120, 180, 240, 300].map(normalize_degrees),
  sector_center_stdev: [30, 30, 30, 30, 30, 30],
  sector_transition_degrees: [
    [- 30 + hall_hysterisis / 2, + 30 - hall_hysterisis / 2],
    [+ 30 + hall_hysterisis / 2, + 90 - hall_hysterisis / 2],
    [+ 90 + hall_hysterisis / 2, +150 - hall_hysterisis / 2],
    [+150 + hall_hysterisis / 2, -150 - hall_hysterisis / 2],
    [-150 + hall_hysterisis / 2, - 90 - hall_hysterisis / 2],
    [- 90 + hall_hysterisis / 2, - 30 - hall_hysterisis / 2],
  ],
  sector_transition_stdev: [
    [transition_stdev, transition_stdev],
    [transition_stdev, transition_stdev],
    [transition_stdev, transition_stdev],
    [transition_stdev, transition_stdev],
    [transition_stdev, transition_stdev],
    [transition_stdev, transition_stdev],
  ],
  accel_stdev: 360.0 / 5.0 / 50.0, // acceleration distribution up to (360 degrees per 5ms) per 50ms.
  initial_angular_speed_stdev: 0.05 * 360,
}