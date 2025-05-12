// Motor driver constants copied from the C++ code.

export const ANGLE_BASE = 1024;

export const PWM_BASE = 1536; // 0x0600
export const PWM_PERIOD = 2 * PWM_BASE;
export const MAX_TIMEOUT = 0xFFFF; 
export const HISTORY_SIZE = 360;
export const READOUT_BASE = 0x10000;

export const V_REF = 3.3; // V
export const ADC_RESOLUTION = 4096; // 12 bits
export const VCC_DIVIDER = 10.0/110.0; // 10k/110k divider

// Constants for the temperature sensor. This sensor isn't very accurate.
export const TEMP_V_AT_REFERENCE_TEMP = 1.43; // V (varies between 1.34 and 1.52)
export const TEMP_AVG_SLOPE = 4.3; // mV/C (varies between 4.0 and 4.6)
export const TEMP_C_REFERENCE = 25.0; // C

export function calculate_temperature(adc_reading){
  return (TEMP_V_AT_REFERENCE_TEMP - adc_reading * V_REF / ADC_RESOLUTION) * 1000 / TEMP_AVG_SLOPE + TEMP_C_REFERENCE;
}

export function calculate_voltage(adc_reading){
  return adc_reading * V_REF / ADC_RESOLUTION / VCC_DIVIDER;
}