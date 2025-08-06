#pragma once

#include <cstdint>
#include <cstddef>

#include "type_definitions.hpp"
#include "math_utils.hpp"

// Data storage available.
const size_t history_size = 336;


// Unit definitions
// ----------------

// One over the maximum value we can use in signed 16 bit multiplication.
const int int_16bit_base = 1 << 15; // 32768

// Double the int base; number of values in an int16_t.
const int int_16bit_span = 1 << 16; // 65536

// Maximum value we can use in signed 32 bit multiplication.
const int max_16bit = int_16bit_base - 1; // 32767


// PWM base value; representing full on duty cycle.
const uint16_t pwm_base = 1536;

// Base and number of hall sectors.
const uint8_t hall_sector_base = 6;

// Angle units of a full circle: 1 angle unit = (tau = 2pi) / 1024.
// 
// Note that speed units and acceleration units are defined as angle units
// per pwm cycle. This allows for easy maths during updates because the time
// delta is always 1 pwm cycle per cycle.
const int angle_base = 1024;

// Bit mask for the angle reading.
const uint16_t angle_bit_mask = angle_base - 1; // 0xFFF;


// The ADC has a 12-bit resolution.
const uint16_t adc_max_value = 0xFFF; // 2^12 - 1 == 4095 == 0xFFF.

// Readout number base; we cycle back to 0 instead of reaching this value.
const int readout_number_base = 1 << 16;


// Readout bit packing
// -------------------

const size_t hall_state_bit_offset = 0;
const uint16_t hall_state_bit_mask = 0b111 << hall_state_bit_offset;

const size_t emf_detected_bit_offset = 11;
const uint16_t emf_detected_bit_mask = 0b1 << emf_detected_bit_offset;

const size_t emf_fix_bit_offset = 10;
const uint16_t emf_fix_bit_mask = 0b1 << emf_fix_bit_offset;

const size_t current_detected_bit_offset = 9;
const uint16_t current_detected_bit_mask = 0b1 << current_detected_bit_offset;

const size_t angle_fix_bit_offset = 8;
const uint16_t angle_fix_bit_mask = 0b1 << angle_fix_bit_offset;

const size_t incorrect_rotor_angle_bit_offset = 7;
const uint16_t incorrect_rotor_angle_bit_mask = 0b1 << incorrect_rotor_angle_bit_offset;

const size_t rotor_direction_flip_imminent_bit_offset = 6;
const uint16_t rotor_direction_flip_imminent_bit_mask = 0b1 << rotor_direction_flip_imminent_bit_offset;

// Position constants
// ------------------

// Number of electrical revolutions per mechanical revolution. This is pole pairs times the number of slot triplets.
const int rotor_revolutions_per_electric = 4;

// Gear ratio of our chosen motor.
const int gear_ratio = 6 * 6 * 6;

// Total ratio between the electrical angle and the output shaft angle.
const int ratio = rotor_revolutions_per_electric * gear_ratio;


// Electric constants
// ------------------


// Voltage reference for the ADC; it's a filtered 3.3V that powers the board.
const float adc_voltage_reference = 3.3;

// Shunt resistance for the motor phase current sensing is 10mOhm, 500mW resistor.
const float motor_shunt_resistance = 0.010;

// The voltage on the shunt resistor is amplified by INA4181 Bidirectional, Low and 
// High Side Voltage Output, Current-Sense Amplifier.
const float amplifier_gain = 20.0;


// The formula that determines the current from the ADC readout: 
//   Vout = (Iload * Rsense * GAIN) + Vref
// And the conversion from the ADC readout is given by:
//   Vout = adc_voltage_reference * (adc_current_readout / adc_max_value).
// So the current is:
//   Iload = (Vout - Vref) / (Rsense * GAIN) = adc_voltage_reference * (adc_readout_diff / adc_max_value) / (Rsense * GAIN).
const float current_conversion_float = adc_voltage_reference / (adc_max_value * motor_shunt_resistance * amplifier_gain);

// Current conversion: 1 current unit = 1/248 A.
const int current_fixed_point = 1 / current_conversion_float;

// 6A max DQ0 driving current.
const int max_drive_current = 6 * current_fixed_point;


// Resistance of the motor phase windings & mosfet; in Ohm.
// Inductance per phase in Henries. Assuming the motor is a 3 phase star connected motor.
// 290 uH measured with LCR meter across phase pairs.
const float phase_inductance_float = 0.000'145;

const int inductance_fixed_point = 1 << 22;

const int phase_inductance = phase_inductance_float * inductance_fixed_point;


// Resistance of the motor phase windings & mosfet; in Ohm. Assuming the motor is a 3 phase star connected motor.
const float phase_resistance_float = 2.00 * 2/3;


// Resistance conversion: 1 resistance unit = 1/1024 Ohm.
const int resistance_fixed_point = 1024;

// Phase resistance in fixed point representation.
const int16_t phase_resistance = static_cast<int16_t>(phase_resistance_float * resistance_fixed_point);

// We can use 0.86% of the voltage range; we need the inverse constant.
const int pwm_waveform_base = static_cast<int>(pwm_base / 0.866);

// Voltage divider between VCC and the ADC reference voltage: 10kohm/110kohm divider
const float vcc_divider = 10.0/110.0;

// Conversion factor for the voltage readout from the ADC.
const float voltage_conversion_float = adc_voltage_reference / (adc_max_value * vcc_divider);

// Voltage conversion: 1 voltage unit = 1/112 V.
const int16_t voltage_fixed_point = static_cast<int16_t>(1/voltage_conversion_float);

// Conversion factor between current and phase resistance voltage.
const int phase_current_to_voltage = phase_resistance * voltage_fixed_point / resistance_fixed_point;

// The drivers need over 8V to power the MOSFETs.
const int vcc_mosfet_driver_undervoltage = voltage_fixed_point * 8;

// Power conversion: 1 power unit = 1/224 W.
const int power_fixed_point = 4 * voltage_fixed_point;

// 12W max drive power.
const int max_drive_power = 12 * power_fixed_point;

// We need this to convert a few formulas using voltage to power.
const int power_div_voltage_fixed_point = power_fixed_point / voltage_fixed_point;

// Directly convert voltage * current to power in fixed point format.
const int voltage_current_div_power_fixed_point = current_fixed_point / power_div_voltage_fixed_point;

// Our dq0 transformation lead to a factor or 3/2 overestimation for the current and therefore power.
const int dq0_to_power_fixed_point = voltage_current_div_power_fixed_point * 3 / 2;

// Divisor used to limit the PWM per cycle.
const int limiting_divisor = 128;

// We also need the divisor minus one for ceiling division.
const int limiting_divisor_m1 = limiting_divisor - 1;


// Timing and PWM constants
// ------------------------

// Note ADC conversion time is = sample time + 12.5 cycles. The ADC clock is 12MHz (72MHz / 6). A cycle is 6 ticks.

// Temperature ADC conversion time: 12.5 cycles + 71.5 cycles = 84 cycles = 7us.
const int temperature_sample_time = (71.5 + 12.5)*6;

// Current ADC conversion time: 12.5 cycles + 1.5 cycles = 14 cycles = 1.16us.
const int current_sample_time = (1.5 + 12.5)*6;

const int current_sample_lead_time = (1.5 + 12.5 + 1.5)*6 / 2;

// The ADC will read the temperature first then 2 phase currents; try to time the sampling 
// time of the phase currents symmetrically around the peak of the PWM cycle.
const int sample_lead_time = temperature_sample_time + current_sample_lead_time;


// Ticks per second at 72MHz clock speed. Each tick is ~13.89ns.
const int ticks_per_second = 72'000'000;

// Auto-reload value for the PWM timer.
const int pwm_autoreload = pwm_base - 1;

// Number of MCU clock ticks per PWM cycle; counting up then down.
const int pwm_period = 2 * pwm_base; 

// Number of PWM cycles per second: 3072 ticks = 42.7us @ 72MHz = 23.4KHz
const int pwm_cycles_per_second = ticks_per_second / pwm_period;

// Maximum duty cycle for the high side MOSFETs needs to allow some off time for the
// bootstrap capacitor to charge so it has enough voltage to turn high side MOSFET on.
const int minimum_bootstrap_duty = 16; // 16/72MHz = 222ns

// Because of this version's electronic design (v0) the current measurement degrades if the
// phase is set exactly to 0; we'll set it to the lowest on value. The phase voltage 
// difference won't change at all. Therefore this setting doesn't affect our driving algorithm
// besides consuming another unit of the PWM range, but this is less than the bootstrapping...
const int pwm_min = 2;

// Maximum duty cycle for the high side mosfet. We need to allow some off time for the 
// bootstrap capacitor to charge so it has enough voltage to turn mosfet on. And also
// enough time to connect all low side mosfets to ground in order to sample phase currents.
const int pwm_max = pwm_base - max(current_sample_lead_time, minimum_bootstrap_duty) - pwm_min;

// Maximum duty for hold commands.
const int pwm_max_hold = pwm_base * 2 / 10;

// Maximum time (in pwm cycles) while a command is in effect.
const int max_timeout = 0xFFFF;



// Motor control tables
// --------------------

// Motor PWM settings for 6-step commutation in the positive direction (counter-clockwise).
const uint16_t motor_sector_driving_positive[6][3] = {
    {0,        pwm_base, 0       },
    {0,        pwm_base, pwm_base},
    {0,        0,        pwm_base},
    {pwm_base, 0,        pwm_base},
    {pwm_base, 0,        0       },
    {pwm_base, pwm_base, 0       },
};

// Motor PWM settings for 6-step commutation in the negative direction (clockwise).
const uint16_t motor_sector_driving_negative[6][3] = {
    {0,        0,        pwm_base},
    {pwm_base, 0,        pwm_base},
    {pwm_base, 0,        0       },
    {pwm_base, pwm_base, 0       },
    {0,        pwm_base, 0       },
    {0,        pwm_base, pwm_base},
};



// Position tracking defaults
// --------------------------

// Define a lot of constants. First, we need to define angles as integers because 
// we can't (shouldn't) use floating point arithmetic in the interrupt handlers.
// Then define the time units. Then speed units, further scaled by a constant to
// have enough precision to represent it.



// Half a circle (pi) aka 180 degrees.
const int half_circle = angle_base / 2;

// 3/2 of a circle (3pi/2) aka 540 degrees.
const int one_and_half_circle = (3 * angle_base) / 2;

// 2/3 of a circle (2pi/3) aka 240 degrees.
const int two_thirds_circle = (2 * angle_base) / 3;

// 1/3 of a circle (pi/3) aka 120 degrees.
const int third_circle = angle_base / 3;

// 3/4 of a circle (3pi/4) aka 270 degrees.
const int three_quarters_circle = (3 * angle_base) / 4;

// 1/4 of a circle (pi/4) aka 90 degrees.
const int quarter_circle = angle_base / 4;

// 1/8 of a circle (pi/8) aka 45 degrees.
const int eighth_circle = angle_base / 8;

// Conversion factor between angle units and radians.
const int half_circle_div_pi = static_cast<int>(half_circle / 3.14159265);

// Normalize to a positive angle (0 to 2pi).
inline constexpr int normalize_angle(int angle){
    return (angle + angle_base) & angle_bit_mask;
}

// Normalize a 0 centerd angle; keeping its sign (-pi to pi).
inline constexpr int signed_angle(int angle){
    return ((angle + one_and_half_circle) & angle_bit_mask) - half_circle;
}

// The angle units per hall sector; 60 degrees.
const int hall_sector_span = angle_base / hall_sector_base;

// Variance of the hall sensor; it doesn't seem to be consistent, even between two rotations.
const int default_sector_transition_variance = square(10 * angle_base / 360);

// Variance of a gaussian spread over the entire sector.
const int default_sector_center_variance = square(30 * angle_base / 360);

// Ensure that our biggest variance is small enough to be usable in gaussian updates while staying < max_16bit.
static_assert(default_sector_center_variance < max_16bit, "max_variance must be less than 32768 (max 16-bit signed int)");

// The hall sensors trigger later than expected going each direction.
const int default_hysterisis = 5 * angle_base / 360;



// Speed and acceleration constants
// --------------------------------

// Note speed values written in degrees per ms and converted to speed units.

// Maximum speed achievable by the motor; in electric revolutions per minute (RPM).
const int max_rpm = 32'000 * rotor_revolutions_per_electric;

// Speed needs more precision than angle. The speed is in angle units per pwm cycle / fixed point.
const int speed_fixed_point = 128;

static_assert(speed_fixed_point * quarter_circle - 1 <= max_16bit, "speed_fixed_point * 90 degrees must be less than 32768 (max 16-bit signed int)");

// Maximum angular speed that we want to represent in the fixed point representation.
const int max_angular_speed = max_rpm / 60 * angle_base * speed_fixed_point / pwm_cycles_per_second;

// Largest angle we could traverse take in a single cycle when going at the maximum speed.
const int max_angle_step = max_angular_speed / speed_fixed_point;

// Minimum speed in rotor revolutions per minute (RPM) that we can represent with our units.
const int min_rpm = 1 * pwm_cycles_per_second / angle_base * 60 / speed_fixed_point / rotor_revolutions_per_electric;

static_assert(max_angular_speed < max_16bit, "max_angular_speed must be less than 32768 (max 16-bit signed int)");


// Fixed point representation of the motor constant; units are V/(rad/s) = Volt * second.
const int motor_constant_fixed_point = 1 << 20;

// Conversion factor between our speed units and radians per second.
const int radians_per_sec_div_angle_base = pwm_cycles_per_second / half_circle_div_pi;

// Acceleration needs more precision than speed; it's in angle units per pwm cycle per pwm cycle / fixed point series.
const int acceleration_fixed_point = 512;


// Calibration and Control Parameters
// ----------------------------------

// By default assume the hall sensors are perfectly placed 120 degrees apart.
const PositionCalibration default_position_calibration = {
    // The angle at which we transition to this sector. The first is when rotating in the
    // positive direction; second for the negative direction.
    .sector_transition_angles = {{
        {330 * angle_base / 360 + default_hysterisis,  30 * angle_base / 360 - default_hysterisis},
        { 30 * angle_base / 360 + default_hysterisis,  90 * angle_base / 360 - default_hysterisis},
        { 90 * angle_base / 360 + default_hysterisis, 150 * angle_base / 360 - default_hysterisis},
        {150 * angle_base / 360 + default_hysterisis, 210 * angle_base / 360 - default_hysterisis},
        {210 * angle_base / 360 + default_hysterisis, 270 * angle_base / 360 - default_hysterisis},
        {270 * angle_base / 360 + default_hysterisis, 330 * angle_base / 360 - default_hysterisis},
    }},
    // Variance of each sector transition; we can calibrate it.
    .sector_transition_variances = {{
        {default_sector_transition_variance, default_sector_transition_variance},
        {default_sector_transition_variance, default_sector_transition_variance},
        {default_sector_transition_variance, default_sector_transition_variance},
        {default_sector_transition_variance, default_sector_transition_variance},
        {default_sector_transition_variance, default_sector_transition_variance},
        {default_sector_transition_variance, default_sector_transition_variance},
    }},
    // The center of each hall sector; the motor should rest at these poles.
    .sector_center_angles = {{
        (  0 * angle_base / 360),
        ( 60 * angle_base / 360),
        (120 * angle_base / 360),
        (180 * angle_base / 360),
        (240 * angle_base / 360),
        (300 * angle_base / 360),
    }},
    // Variance of the centers.
    .sector_center_variances = {{
        default_sector_center_variance,
        default_sector_center_variance,
        default_sector_center_variance,
        default_sector_center_variance,
        default_sector_center_variance,
        default_sector_center_variance,
    }},
};

// Fixed point representation of the current calibration factors.
const int16_t current_calibration_fixed_point = 1024;

// By default assume the current calibration is 1.0 for all phases and inductance.
const CurrentCalibration default_current_calibration = {
    .u_factor = current_calibration_fixed_point,
    .v_factor = current_calibration_fixed_point,
    .w_factor = current_calibration_fixed_point,
    .inductance_factor = current_calibration_fixed_point,
};

// Fixed point for most control parameters and most other floating point values.
const int16_t hires_fixed_point = 4096;

// Fixed point for the PID control values and PID output.
const int16_t seek_pid_fixed_point = 1024;

// The default control parameters should be set to reasonable values for any motor.
// 
// The reset button will reload these values.
const ControlParameters default_control_parameters = {

    .rotor_angle_ki = 1024,
    .rotor_angular_speed_ki = 64,
    .rotor_acceleration_ki = 32,
    .motor_constant_ki = 2,
    
    .resistance_ki = 0,
    .inductance_ki = 0,
    .max_pwm_change = 8,
    .max_angle_change = 8,

    .min_emf_voltage = voltage_fixed_point * 100 / 1000,
    .hall_angle_ki = 256,
    .lead_angle_control_ki = 4,
    .torque_control_ki = 32,

    .battery_power_control_ki = 8,
    .speed_control_ki = 8,
    .probing_angular_speed = speed_fixed_point / 2,
    .max_pwm_difference = pwm_max_hold,

    .emf_angle_error_variance_threshold = square(10 * angle_base / 360),
    .min_emf_for_motor_constant = voltage_fixed_point * 1,
    .max_resistive_power = power_fixed_point * 2,
    .resistive_power_ki = 1,

    .max_angular_speed = max_angular_speed,
    .max_power_draw = max_drive_power,
    .power_draw_ki = 1,
    .max_pwm = pwm_max,

    .seek_via_torque_k_prediction = 2048,
    .seek_via_torque_ki = 256,
    .seek_via_torque_kp = 1024,
    .seek_via_torque_kd = 256,

    .seek_via_power_k_prediction = 2048,
    .seek_via_power_ki = 64,
    .seek_via_power_kp = 1024,
    .seek_via_power_kd = 256,
};

// Maximum value for the lead angle control; we won't lead more than 60degrees ahead of the quadrature angle.
const int max_lead_angle_control = 60 * angle_base / 360 * hires_fixed_point;

// Hi resolution value for pwm_max, used for the 32bit control integral.
const int max_pwm_control = pwm_max * hires_fixed_point;

// Maximum rotations to use in the PID angle seeking loop (max rotations should imply max control when KP == 1.0).
const int max_seek_rotations_error = 128;

// The angle is too high resolution for seeking; we need to lower the resolution.
const int seek_angle_divisor = 32;

// Number of angle divisions per rotation; used to convert rotations into the seeking angle units.
const int seek_rotation_multiplier = angle_base / seek_angle_divisor;

const int seek_position_error_reference = max_seek_rotations_error * seek_rotation_multiplier;

// Maximum error in the seek angle; double the control output to overcome the differential term.
const int max_seek_error = 2 * seek_position_error_reference;

// Reference speed for the PID control (the derivative term is the speed).
const int seek_speed_error_reference = max_angular_speed / 4;

// Duration to maximize the seek integral at the maximum error.
const int seek_integral_duration = 32;

// The divisor for the seek integral; we accumulate the error over a few cycles and then divide by this value.
const int seek_integral_divisor = max_seek_error * seek_integral_duration;

// Cap the integral term to 1/2 of the maximum control output so it doesn't overpower the P and D terms.
const int max_seek_integral_control = (seek_pid_fixed_point / 2);

// Maximum value to accumulate in the seek integral.
const int max_seek_integral = seek_integral_divisor * max_seek_integral_control;


// Calculation precomputed constants
// ---------------------------------

// Conversion factor between current difference and the implied inductor voltage given the phase inductance (V = L * di/dt).
const int phase_diff_conversion = (
    inductance_fixed_point * current_fixed_point / phase_inductance * 
    current_calibration_fixed_point / pwm_cycles_per_second / voltage_fixed_point
);

// Conversion factor between the motor constant and our speed/voltage units.
const int emf_motor_constant_conversion = (
    motor_constant_fixed_point / voltage_fixed_point * speed_fixed_point / radians_per_sec_div_angle_base
);

// Cap for the high resolution angular speed observer. This caps the high resolution observer so that the 
// low resolution output is capped to the maximum angular speed.
const int max_angular_speed_observer = max_angular_speed * hires_fixed_point;


// Waveform and Trigonometric tables
// ---------------------------------

static_assert(angle_base == 1024, "angle_base must be 4096 for the sine and cosine lookup tables to work correctly");

// Waveform for driving the coil phases at the specifed angle. Check the `3 Phase Tricks` page for 
// the illustrated explanation and the generated table.
const uint16_t phases_waveform[1024] = {
    1331, 1335, 1340, 1345, 1349, 1354, 1358, 1362, 1367, 1371, 1375, 1379, 1384, 1388, 1392, 1396,
    1400, 1403, 1407, 1411, 1415, 1418, 1422, 1426, 1429, 1432, 1436, 1439, 1442, 1446, 1449, 1452,
    1455, 1458, 1461, 1464, 1467, 1469, 1472, 1475, 1477, 1480, 1483, 1485, 1487, 1490, 1492, 1494,
    1496, 1498, 1501, 1503, 1504, 1506, 1508, 1510, 1512, 1513, 1515, 1516, 1518, 1519, 1521, 1522,
    1523, 1525, 1526, 1527, 1528, 1529, 1530, 1531, 1531, 1532, 1533, 1533, 1534, 1534, 1535, 1535,
    1536, 1536, 1536, 1536, 1536, 1536, 1536, 1536, 1536, 1536, 1536, 1536, 1535, 1535, 1534, 1534,
    1533, 1533, 1532, 1531, 1530, 1529, 1528, 1527, 1526, 1525, 1524, 1523, 1522, 1520, 1519, 1517,
    1516, 1514, 1513, 1511, 1509, 1508, 1506, 1504, 1502, 1500, 1498, 1496, 1493, 1491, 1489, 1487,
    1484, 1482, 1479, 1477, 1474, 1471, 1469, 1466, 1463, 1460, 1457, 1454, 1451, 1448, 1445, 1441,
    1438, 1435, 1431, 1428, 1424, 1421, 1417, 1413, 1410, 1406, 1402, 1398, 1394, 1390, 1386, 1382,
    1378, 1374, 1370, 1365, 1361, 1357, 1352, 1348, 1343, 1339, 1334, 1329, 1324, 1320, 1315, 1310,
    1305, 1300, 1295, 1290, 1285, 1279, 1274, 1269, 1264, 1258, 1253, 1247, 1242, 1236, 1230, 1225,
    1219, 1213, 1208, 1202, 1196, 1190, 1184, 1178, 1172, 1166, 1159, 1153, 1147, 1141, 1134, 1128,
    1122, 1115, 1109, 1102, 1095, 1089, 1082, 1075, 1069, 1062, 1055, 1048, 1041, 1034, 1027, 1020,
    1013, 1006,  999,  992,  985,  977,  970,  963,  955,  948,  941,  933,  926,  918,  910,  903,
     895,  888,  880,  872,  864,  856,  849,  841,  833,  825,  817,  809,  801,  793,  785,  777,
     769,  760,  752,  744,  736,  727,  719,  711,  702,  694,  685,  677,  669,  660,  652,  643,
     634,  626,  617,  609,  600,  591,  582,  574,  565,  556,  547,  539,  530,  521,  512,  503,
     494,  485,  476,  467,  458,  449,  440,  431,  422,  413,  404,  395,  386,  377,  368,  358,
     349,  340,  331,  322,  312,  303,  294,  285,  275,  266,  257,  248,  238,  229,  220,  210,
     201,  192,  182,  173,  164,  154,  145,  135,  126,  117,  107,   98,   88,   79,   70,   60,
      51,   41,   32,   22,   13,    4,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    4,   13,   22,   32,   41,
      51,   60,   70,   79,   88,   98,  107,  117,  126,  135,  145,  154,  164,  173,  182,  192,
     201,  210,  220,  229,  238,  248,  257,  266,  275,  285,  294,  303,  312,  322,  331,  340,
     349,  358,  368,  377,  386,  395,  404,  413,  422,  431,  440,  449,  458,  467,  476,  485,
     494,  503,  512,  521,  530,  539,  547,  556,  565,  574,  582,  591,  600,  609,  617,  626,
     634,  643,  652,  660,  669,  677,  685,  694,  702,  711,  719,  727,  736,  744,  752,  760,
     769,  777,  785,  793,  801,  809,  817,  825,  833,  841,  849,  856,  864,  872,  880,  888,
     895,  903,  910,  918,  926,  933,  941,  948,  955,  963,  970,  977,  985,  992,  999, 1006,
    1013, 1020, 1027, 1034, 1041, 1048, 1055, 1062, 1069, 1075, 1082, 1089, 1095, 1102, 1109, 1115,
    1122, 1128, 1134, 1141, 1147, 1153, 1159, 1166, 1172, 1178, 1184, 1190, 1196, 1202, 1208, 1213,
    1219, 1225, 1230, 1236, 1242, 1247, 1253, 1258, 1264, 1269, 1274, 1279, 1285, 1290, 1295, 1300,
    1305, 1310, 1315, 1320, 1324, 1329, 1334, 1339, 1343, 1348, 1352, 1357, 1361, 1365, 1370, 1374,
    1378, 1382, 1386, 1390, 1394, 1398, 1402, 1406, 1410, 1413, 1417, 1421, 1424, 1428, 1431, 1435,
    1438, 1441, 1445, 1448, 1451, 1454, 1457, 1460, 1463, 1466, 1469, 1471, 1474, 1477, 1479, 1482,
    1484, 1487, 1489, 1491, 1493, 1496, 1498, 1500, 1502, 1504, 1506, 1508, 1509, 1511, 1513, 1514,
    1516, 1517, 1519, 1520, 1522, 1523, 1524, 1525, 1526, 1527, 1528, 1529, 1530, 1531, 1532, 1533,
    1533, 1534, 1534, 1535, 1535, 1536, 1536, 1536, 1536, 1536, 1536, 1536, 1536, 1536, 1536, 1536,
    1536, 1535, 1535, 1534, 1534, 1533, 1533, 1532, 1531, 1531, 1530, 1529, 1528, 1527, 1526, 1525,
    1523, 1522, 1521, 1519, 1518, 1516, 1515, 1513, 1512, 1510, 1508, 1506, 1504, 1503, 1501, 1498,
    1496, 1494, 1492, 1490, 1487, 1485, 1483, 1480, 1477, 1475, 1472, 1469, 1467, 1464, 1461, 1458,
    1455, 1452, 1449, 1446, 1442, 1439, 1436, 1432, 1429, 1426, 1422, 1418, 1415, 1411, 1407, 1403,
    1400, 1396, 1392, 1388, 1384, 1379, 1375, 1371, 1367, 1362, 1358, 1354, 1349, 1345, 1340, 1335
};

// Get the PWM fraction at the specified angle.
static inline int16_t get_phase_pwm(const int16_t angle) {
    return phases_waveform[normalize_angle(angle)];
}

// Lookup table for the sine function; 1024 entries for a full circle (2*pi). Table is also found
// on the `3 Phase Tricks` page.
const int16_t sin_lookup[1024] = {
        0,     6,    13,    19,    25,    31,    38,    44,    50,    57,    63,    69,    75,    82,    88,    94,
      100,   107,   113,   119,   125,   132,   138,   144,   150,   156,   163,   169,   175,   181,   187,   194,
      200,   206,   212,   218,   224,   230,   237,   243,   249,   255,   261,   267,   273,   279,   285,   291,
      297,   303,   309,   315,   321,   327,   333,   339,   345,   351,   357,   363,   369,   374,   380,   386,
      392,   398,   403,   409,   415,   421,   426,   432,   438,   443,   449,   455,   460,   466,   472,   477,
      483,   488,   494,   499,   505,   510,   516,   521,   526,   532,   537,   543,   548,   553,   558,   564,
      569,   574,   579,   584,   590,   595,   600,   605,   610,   615,   620,   625,   630,   635,   640,   645,
      650,   654,   659,   664,   669,   674,   678,   683,   688,   692,   697,   702,   706,   711,   715,   720,
      724,   729,   733,   737,   742,   746,   750,   755,   759,   763,   767,   771,   775,   779,   784,   788,
      792,   796,   799,   803,   807,   811,   815,   819,   822,   826,   830,   834,   837,   841,   844,   848,
      851,   855,   858,   862,   865,   868,   872,   875,   878,   882,   885,   888,   891,   894,   897,   900,
      903,   906,   909,   912,   915,   917,   920,   923,   926,   928,   931,   934,   936,   939,   941,   944,
      946,   948,   951,   953,   955,   958,   960,   962,   964,   966,   968,   970,   972,   974,   976,   978,
      980,   982,   983,   985,   987,   989,   990,   992,   993,   995,   996,   998,   999,  1000,  1002,  1003,
     1004,  1006,  1007,  1008,  1009,  1010,  1011,  1012,  1013,  1014,  1015,  1016,  1016,  1017,  1018,  1018,
     1019,  1020,  1020,  1021,  1021,  1022,  1022,  1022,  1023,  1023,  1023,  1024,  1024,  1024,  1024,  1024,
     1024,  1024,  1024,  1024,  1024,  1024,  1023,  1023,  1023,  1022,  1022,  1022,  1021,  1021,  1020,  1020,
     1019,  1018,  1018,  1017,  1016,  1016,  1015,  1014,  1013,  1012,  1011,  1010,  1009,  1008,  1007,  1006,
     1004,  1003,  1002,  1000,   999,   998,   996,   995,   993,   992,   990,   989,   987,   985,   983,   982,
      980,   978,   976,   974,   972,   970,   968,   966,   964,   962,   960,   958,   955,   953,   951,   948,
      946,   944,   941,   939,   936,   934,   931,   928,   926,   923,   920,   917,   915,   912,   909,   906,
      903,   900,   897,   894,   891,   888,   885,   882,   878,   875,   872,   868,   865,   862,   858,   855,
      851,   848,   844,   841,   837,   834,   830,   826,   822,   819,   815,   811,   807,   803,   799,   796,
      792,   788,   784,   779,   775,   771,   767,   763,   759,   755,   750,   746,   742,   737,   733,   729,
      724,   720,   715,   711,   706,   702,   697,   692,   688,   683,   678,   674,   669,   664,   659,   654,
      650,   645,   640,   635,   630,   625,   620,   615,   610,   605,   600,   595,   590,   584,   579,   574,
      569,   564,   558,   553,   548,   543,   537,   532,   526,   521,   516,   510,   505,   499,   494,   488,
      483,   477,   472,   466,   460,   455,   449,   443,   438,   432,   426,   421,   415,   409,   403,   398,
      392,   386,   380,   374,   369,   363,   357,   351,   345,   339,   333,   327,   321,   315,   309,   303,
      297,   291,   285,   279,   273,   267,   261,   255,   249,   243,   237,   230,   224,   218,   212,   206,
      200,   194,   187,   181,   175,   169,   163,   156,   150,   144,   138,   132,   125,   119,   113,   107,
      100,    94,    88,    82,    75,    69,    63,    57,    50,    44,    38,    31,    25,    19,    13,     6,
        0,    -6,   -13,   -19,   -25,   -31,   -38,   -44,   -50,   -57,   -63,   -69,   -75,   -82,   -88,   -94,
     -100,  -107,  -113,  -119,  -125,  -132,  -138,  -144,  -150,  -156,  -163,  -169,  -175,  -181,  -187,  -194,
     -200,  -206,  -212,  -218,  -224,  -230,  -237,  -243,  -249,  -255,  -261,  -267,  -273,  -279,  -285,  -291,
     -297,  -303,  -309,  -315,  -321,  -327,  -333,  -339,  -345,  -351,  -357,  -363,  -369,  -374,  -380,  -386,
     -392,  -398,  -403,  -409,  -415,  -421,  -426,  -432,  -438,  -443,  -449,  -455,  -460,  -466,  -472,  -477,
     -483,  -488,  -494,  -499,  -505,  -510,  -516,  -521,  -526,  -532,  -537,  -543,  -548,  -553,  -558,  -564,
     -569,  -574,  -579,  -584,  -590,  -595,  -600,  -605,  -610,  -615,  -620,  -625,  -630,  -635,  -640,  -645,
     -650,  -654,  -659,  -664,  -669,  -674,  -678,  -683,  -688,  -692,  -697,  -702,  -706,  -711,  -715,  -720,
     -724,  -729,  -733,  -737,  -742,  -746,  -750,  -755,  -759,  -763,  -767,  -771,  -775,  -779,  -784,  -788,
     -792,  -796,  -799,  -803,  -807,  -811,  -815,  -819,  -822,  -826,  -830,  -834,  -837,  -841,  -844,  -848,
     -851,  -855,  -858,  -862,  -865,  -868,  -872,  -875,  -878,  -882,  -885,  -888,  -891,  -894,  -897,  -900,
     -903,  -906,  -909,  -912,  -915,  -917,  -920,  -923,  -926,  -928,  -931,  -934,  -936,  -939,  -941,  -944,
     -946,  -948,  -951,  -953,  -955,  -958,  -960,  -962,  -964,  -966,  -968,  -970,  -972,  -974,  -976,  -978,
     -980,  -982,  -983,  -985,  -987,  -989,  -990,  -992,  -993,  -995,  -996,  -998,  -999, -1000, -1002, -1003,
    -1004, -1006, -1007, -1008, -1009, -1010, -1011, -1012, -1013, -1014, -1015, -1016, -1016, -1017, -1018, -1018,
    -1019, -1020, -1020, -1021, -1021, -1022, -1022, -1022, -1023, -1023, -1023, -1024, -1024, -1024, -1024, -1024,
    -1024, -1024, -1024, -1024, -1024, -1024, -1023, -1023, -1023, -1022, -1022, -1022, -1021, -1021, -1020, -1020,
    -1019, -1018, -1018, -1017, -1016, -1016, -1015, -1014, -1013, -1012, -1011, -1010, -1009, -1008, -1007, -1006,
    -1004, -1003, -1002, -1000,  -999,  -998,  -996,  -995,  -993,  -992,  -990,  -989,  -987,  -985,  -983,  -982,
     -980,  -978,  -976,  -974,  -972,  -970,  -968,  -966,  -964,  -962,  -960,  -958,  -955,  -953,  -951,  -948,
     -946,  -944,  -941,  -939,  -936,  -934,  -931,  -928,  -926,  -923,  -920,  -917,  -915,  -912,  -909,  -906,
     -903,  -900,  -897,  -894,  -891,  -888,  -885,  -882,  -878,  -875,  -872,  -868,  -865,  -862,  -858,  -855,
     -851,  -848,  -844,  -841,  -837,  -834,  -830,  -826,  -822,  -819,  -815,  -811,  -807,  -803,  -799,  -796,
     -792,  -788,  -784,  -779,  -775,  -771,  -767,  -763,  -759,  -755,  -750,  -746,  -742,  -737,  -733,  -729,
     -724,  -720,  -715,  -711,  -706,  -702,  -697,  -692,  -688,  -683,  -678,  -674,  -669,  -664,  -659,  -654,
     -650,  -645,  -640,  -635,  -630,  -625,  -620,  -615,  -610,  -605,  -600,  -595,  -590,  -584,  -579,  -574,
     -569,  -564,  -558,  -553,  -548,  -543,  -537,  -532,  -526,  -521,  -516,  -510,  -505,  -499,  -494,  -488,
     -483,  -477,  -472,  -466,  -460,  -455,  -449,  -443,  -438,  -432,  -426,  -421,  -415,  -409,  -403,  -398,
     -392,  -386,  -380,  -374,  -369,  -363,  -357,  -351,  -345,  -339,  -333,  -327,  -321,  -315,  -309,  -303,
     -297,  -291,  -285,  -279,  -273,  -267,  -261,  -255,  -249,  -243,  -237,  -230,  -224,  -218,  -212,  -206,
     -200,  -194,  -187,  -181,  -175,  -169,  -163,  -156,  -150,  -144,  -138,  -132,  -125,  -119,  -113,  -107,
     -100,   -94,   -88,   -82,   -75,   -69,   -63,   -57,   -50,   -44,   -38,   -31,   -25,   -19,   -13,    -6
};

// Get the sine value for the given angle.
static inline int16_t get_sin(const int16_t angle) {
    return sin_lookup[normalize_angle(angle)];
}

// Get the sine value for the angle and the 120 degrees phase shifts on either side.
static inline ThreePhase get_three_phase_sin(int16_t angle) {
    angle = normalize_angle(angle);
    return {
        sin_lookup[angle],
        sin_lookup[(angle + two_thirds_circle) & angle_bit_mask],
        sin_lookup[(angle + third_circle) & angle_bit_mask]
    };
}

// For cos lookup we can use the sin lookup table + 90 degrees (quarter_circle).
static inline int16_t get_cos(const int16_t angle) {
    return get_sin(angle + quarter_circle);
}

// Get the cosine value for the angle and the 120 degrees phase shifts on either side.
static inline ThreePhase get_three_phase_cos(int16_t angle) {
    return get_three_phase_sin(angle + quarter_circle);
}