#pragma once

#include <cstdint>
#include <cstddef>


// Interface command codes
// -----------------------

const uint32_t READOUT = 0x80202020;
const uint32_t GET_READOUTS = 0x80202021;
const uint32_t GET_READOUTS_SNAPSHOT = 0x80202022;
const uint32_t SET_STATE_OFF = 0x80202030;
const uint32_t SET_STATE_DRIVE_POS = 0x80202031;
const uint32_t SET_STATE_TEST_ALL_PERMUTATIONS = 0x80202032;
const uint32_t SET_STATE_DRIVE_NEG = 0x80202033;
const uint32_t SET_STATE_FREEWHEEL = 0x80202034;

const uint32_t SET_STATE_TEST_GROUND_SHORT = 0x80202036;
const uint32_t SET_STATE_TEST_POSITIVE_SHORT = 0x80202037;

const uint32_t SET_STATE_TEST_U_DIRECTIONS = 0x80202039;
const uint32_t SET_STATE_TEST_U_INCREASING = 0x8020203A;
const uint32_t SET_STATE_TEST_U_DECREASING = 0x8020203B;
const uint32_t SET_STATE_TEST_V_INCREASING = 0x8020203C;
const uint32_t SET_STATE_TEST_V_DECREASING = 0x8020203D;
const uint32_t SET_STATE_TEST_W_INCREASING = 0x8020203E;
const uint32_t SET_STATE_TEST_W_DECREASING = 0x8020203F;

const uint32_t SET_STATE_HOLD_U_POSITIVE = 0x80203020;
const uint32_t SET_STATE_HOLD_V_POSITIVE = 0x80203021;
const uint32_t SET_STATE_HOLD_W_POSITIVE = 0x80203022;
const uint32_t SET_STATE_HOLD_U_NEGATIVE = 0x80203023;
const uint32_t SET_STATE_HOLD_V_NEGATIVE = 0x80203024;
const uint32_t SET_STATE_HOLD_W_NEGATIVE = 0x80203025;

const uint32_t SET_STATE_DRIVE_SMOOTH_POS = 0x80204030;
const uint32_t SET_STATE_DRIVE_SMOOTH_NEG = 0x80204031;


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
const int max_angle_unit = 2048;
// Half a circle (pi).
const int half_max_angle = max_angle_unit / 2;
// 3/2 of a circle (3pi/2).
const int one_and_half_max_angle = (3 * max_angle_unit) / 2;

// Normalize to a positive angle (0 to 2pi).
inline constexpr int normalize_angle(int angle){
    return (angle + max_angle_unit) % max_angle_unit;
}
// Normalize a 0 centerd angle; keeping its sign (-pi to pi).
inline constexpr int signed_angle(int angle){
    return (angle + one_and_half_max_angle) % max_angle_unit - half_max_angle;
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
const int max_speed = 20 * scale * max_angle_unit / time_units_per_millisecond;

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
const int initial_angular_speed_variance = square(scale * max_angle_unit * 30 / 360 / time_units_per_millisecond);

// Precalculate the acceleration variance divided by 4. Note the scaled is squared twice.
const int angular_acceleration_variance_div_4 = square(square_scale * max_angle_unit / 1 / 50 / time_units_per_millisecond / time_units_per_millisecond) / 4;

// Maximum distance to a trigger angle. Don't let the estimated angle deviate
// from the hall sensor angle by more than this value to keep the estimate
// within the half circle of the trigger so we don't switch sign.
const int sector_transition_confidence = 20 * max_angle_unit / 360;

// Variance of the hall sensor; it doesn't seem to be consistent, even between two rotations.
const int default_sector_transition_variance = square(5 * max_angle_unit / 360);
// Variance of a gaussian spread over the entire sector.
const int default_sector_center_variance = square(30 * max_angle_unit / 360);

// The hall sensors trigger later than expected going each direction.
const int hysterisis = 5 * max_angle_unit / 360;

// The angle at which we transition to this sector. The first is when rotating in the
// positive direction; second for the negative direction.
const int sector_transition_angles[6][2] = {
    {330 * max_angle_unit / 360 + hysterisis,  30 * max_angle_unit / 360 - hysterisis},
    { 30 * max_angle_unit / 360 + hysterisis,  90 * max_angle_unit / 360 - hysterisis},
    { 90 * max_angle_unit / 360 + hysterisis, 150 * max_angle_unit / 360 - hysterisis},
    {150 * max_angle_unit / 360 + hysterisis, 210 * max_angle_unit / 360 - hysterisis},
    {210 * max_angle_unit / 360 + hysterisis, 270 * max_angle_unit / 360 - hysterisis},
    {270 * max_angle_unit / 360 + hysterisis, 330 * max_angle_unit / 360 - hysterisis},     
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
    (  0 * max_angle_unit / 360),
    ( 60 * max_angle_unit / 360),
    (120 * max_angle_unit / 360),
    (180 * max_angle_unit / 360),
    (240 * max_angle_unit / 360),
    (300 * max_angle_unit / 360),
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