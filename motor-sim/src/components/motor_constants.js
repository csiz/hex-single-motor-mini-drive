// Motor driver constants copied from the C++ code.

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