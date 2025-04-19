#pragma once

#include <cstdint>
#include <cstddef>


// Data storage available.
const size_t HISTORY_SIZE = 420;

// Current constants
// -----------------


// Voltage reference for the ADC; it's a filtered 3.3V that powers the board.
const float adc_voltage_reference = 3.3;
// Shunt resistance for the motor phase current sensing is 10mOhm, 500mW resistor.
const float motor_shunt_resistance = 0.010;
// The voltage on the shunt resistor is amplified by INA4181 Bidirectional, Low and 
// High Side Voltage Output, Current-Sense Amplifier.
const float amplifier_gain = 20.0;
// The ADC has a 12-bit resolution.
const uint16_t adc_max_value = 0xFFF; // 2^12 - 1 == 4095 == 0xFFF.
// The formula that determines the current from the ADC readout: 
//   Vout = (Iload * Rsense * GAIN) + Vref
// And the conversion from the ADC readout is given by:
//   Vout = adc_voltage_reference * (adc_current_readout / adc_max_value).
// So the current is:
//   Iload = (Vout - Vref) / (Rsense * GAIN) = adc_voltage_reference * (adc_readout_diff / adc_max_value) / (Rsense * GAIN).
// 
// Note: The minus sign is because of the way I wired up the INA4181 ...
const float current_conversion = -adc_voltage_reference / (adc_max_value * motor_shunt_resistance * amplifier_gain);




// Motor PWM constants
// -------------------

const uint16_t PWM_AUTORELOAD = 1535;
const uint16_t PWM_BASE = PWM_AUTORELOAD + 1;

// Maximum duty cycle for the high side mosfet needs to allow some off time for 
// the bootstrap capacitor to charge so it has enough voltage to turn mosfet on.
const uint16_t MIN_BOOTSTRAP_DUTY = 16; // 16/72MHz = 222ns
const uint16_t PWM_MAX = PWM_BASE - MIN_BOOTSTRAP_DUTY; // 1536/72MHz = 21.3us

const uint16_t PWM_MAX_HOLD = PWM_BASE * 2 / 10;

const uint16_t MAX_TIMEOUT = 0xFFFF;


// Position tracking defaults
// --------------------------

// Base and 1 over maximum value of the hall sector.
const uint8_t hall_sector_base = 6;


// Some useful functions to compute position. These need to be inlined for 
// efficiency, and constexpr to define a bunch more constants below.

// Square a number.
inline constexpr int square(int x){
    return x * x;
}

// Get the smaller between two numbers.
inline constexpr int min(int a, int b){
    return a < b ? a : b;
}

// Get the larger between two numbers.
inline constexpr int max(int a, int b){
    return a > b ? a : b;
}

// Clip a value between two limits; params are (low high value).
inline constexpr int clip_to(int low, int high, int value){
    return min(high, max(low, value));
}


// Define a lot of constants. First, we need to define angles as integers because 
// we can't (shouldn't) use floating point arithmetic in the interrupt handlers.
// Then define the time units. Then speed units, further scaled by a constant to
// have enough precision to represent it.

// Maximum value we can use in signed 32 bit multiplication.
const int max_16bit = 0x0000'8FFF; //0x0000'B504;

// Angle units of a full circle (2pi).
const int angle_base = 2048;
// Half a circle (pi).
const int half_circle = angle_base / 2;
// 3/2 of a circle (3pi/2).
const int one_and_half_circle = (3 * angle_base) / 2;

// Normalize to a positive angle (0 to 2pi).
inline constexpr int normalize_angle(int angle){
    return (angle + angle_base) % angle_base;
}
// Normalize a 0 centerd angle; keeping its sign (-pi to pi).
inline constexpr int signed_angle(int angle){
    return (angle + one_and_half_circle) % angle_base - half_circle;
}

// Scaling constant for time.
const int ticks_per_time_units = 256;
// Ticks per microsecond with the 72MHz clock.
const int ticks_per_microsecond = 72;
// Ticks per millisecond with the 72MHz clock.
const int ticks_per_millisecond = 72'000;
// Our time units per millisecond. Helpful scaling factor for the constants below.
const int time_units_per_millisecond = ticks_per_millisecond / ticks_per_time_units; 
// Time units per PWM cycle (2x because it counts up then down).
const int time_increment_per_cycle = 2 * static_cast<int>(PWM_BASE) / ticks_per_time_units;

// Another scaling factor: speed = distances * scale / time; acceleration = speed_change * scale;
const int scale = 128;
// Precomputed square of scale.
const int square_scale = square(scale);

// Reference for the maximum speed we should be able to represent.
const int max_speed = 20 * scale * angle_base / time_units_per_millisecond;

// The maximum time in our time units before we can no longer safely square the value.
// 
// Keep in mind that a single motor rotation takes at least 12 toggles (6 per electrical
//  revolution * 2 poles per phase).
const int max_time_between_observations = 100 * time_units_per_millisecond;


// Note:
// Speed and acceleration are written in degrees per ms and per ms^2 respectively.

// Initial speed estimate.
const int initial_angular_speed = 0;
// Start with a high speed variance.
const int initial_angular_speed_variance = square(scale * angle_base * 30 / 360 / time_units_per_millisecond);

// Precalculate the acceleration variance divided by 4. Note the scaled is squared twice.
const int angular_acceleration_variance_div_4 = square(square_scale * angle_base / 1 / 50 / time_units_per_millisecond / time_units_per_millisecond) / 4;

// Maximum distance to a trigger angle. Don't let the estimated angle deviate
// from the hall sensor angle by more than this value to keep the estimate
// within the half circle of the trigger so we don't switch sign.
const int sector_transition_confidence = 20 * angle_base / 360;

// Variance of the hall sensor; it doesn't seem to be consistent, even between two rotations.
const int default_sector_transition_variance = square(5 * angle_base / 360);
// Variance of a gaussian spread over the entire sector.
const int default_sector_center_variance = square(30 * angle_base / 360);

// The hall sensors trigger later than expected going each direction.
const int hysterisis = 5 * angle_base / 360;

// The angle at which we transition to this sector. The first is when rotating in the
// positive direction; second for the negative direction.
const int sector_transition_angles[6][2] = {
    {330 * angle_base / 360 + hysterisis,  30 * angle_base / 360 - hysterisis},
    { 30 * angle_base / 360 + hysterisis,  90 * angle_base / 360 - hysterisis},
    { 90 * angle_base / 360 + hysterisis, 150 * angle_base / 360 - hysterisis},
    {150 * angle_base / 360 + hysterisis, 210 * angle_base / 360 - hysterisis},
    {210 * angle_base / 360 + hysterisis, 270 * angle_base / 360 - hysterisis},
    {270 * angle_base / 360 + hysterisis, 330 * angle_base / 360 - hysterisis},     
};

// Variance of each sector transition; we can calibrate it.
const int sector_transition_variances[6][2] = {
    {default_sector_transition_variance, default_sector_transition_variance},
    {default_sector_transition_variance, default_sector_transition_variance},
    {default_sector_transition_variance, default_sector_transition_variance},
    {default_sector_transition_variance, default_sector_transition_variance},
    {default_sector_transition_variance, default_sector_transition_variance},
    {default_sector_transition_variance, default_sector_transition_variance},
};

// The center of each hall sector; the motor should rest at these poles.
const int sector_center_angles[6] = {
    (  0 * angle_base / 360),
    ( 60 * angle_base / 360),
    (120 * angle_base / 360),
    (180 * angle_base / 360),
    (240 * angle_base / 360),
    (300 * angle_base / 360),
};

// Variance of the centers.
const int sector_center_variances[6] = {
    default_sector_center_variance,
    default_sector_center_variance,
    default_sector_center_variance,
    default_sector_center_variance,
    default_sector_center_variance,
    default_sector_center_variance,
};