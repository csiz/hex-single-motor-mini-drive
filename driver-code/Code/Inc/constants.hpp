#pragma once

#include <cstdint>
#include <cstddef>

#include "type_definitions.hpp"
#include "math_utils.hpp"

// Data storage available.
const size_t history_size = 360;


// Unit definitions
// ----------------

// Maximum value we can use in signed 32 bit multiplication.
const int max_16bit = (1 << 15) - 1; // 32767

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

// Round down angle units for the trig lookup tables. 14 to 10 bits.
const int angle_lookup_divider = angle_base / 1024;

// The ADC has a 12-bit resolution.
const uint16_t adc_max_value = 0xFFF; // 2^12 - 1 == 4095 == 0xFFF.

// Readout number base; we cycle back to 0 instead of reaching this value.
const int readout_number_base = 1 << 16;

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
const int16_t current_fixed_point = static_cast<int16_t>(1/current_conversion_float);

// Max driving current that we can command.
const int16_t max_drive_current = 4 * current_fixed_point;

// Minimum driving current to move the motor.
const int16_t min_drive_current = max_drive_current / 32;

// Resistance of the motor phase windings & mosfet; in Ohm.
// Inductance per phase in Henries. Assuming the motor is a 3 phase star connected motor.
// 290 uH measured with LCR meter across phase pairs.
const float phase_inductance_float = 0.000'145;

const int inductance_fixed_point = 1 << 22;

const int16_t phase_inductance = static_cast<int16_t>(phase_inductance_float * inductance_fixed_point);


// Resistance of the motor phase windings & mosfet; in Ohm. Assuming the motor is a 3 phase star connected motor.
const float phase_resistance_float = 2.00 * 2/3;


// Resistance conversion: 1 resistance unit = 1/1024 Ohm.
const int resistance_fixed_point = 1024;

// Phase resistance in fixed point representation.
const int16_t phase_resistance = static_cast<int16_t>(phase_resistance_float * resistance_fixed_point);




// Voltage divider between VCC and the ADC reference voltage: 10kohm/110kohm divider
const float vcc_divider = 10.0/110.0;

// Conversion factor for the voltage readout from the ADC.
const float voltage_conversion_float = adc_voltage_reference / (adc_max_value * vcc_divider);

// Voltage conversion: 1 voltage unit = 1/112 V.
const int16_t voltage_fixed_point = static_cast<int16_t>(1/voltage_conversion_float);


// Power conversion: 1 power unit = 1/224 W.
const int power_fixed_point = 2 * voltage_fixed_point;

// We need this to convert a few formulas using voltage to power.
const int power_div_voltage_fixed_point = power_fixed_point / voltage_fixed_point;



// Timing and PWM constants
// ------------------------

// Note ADC conversion time is = sample time + 12.5 cycles. The ADC clock is 12MHz (72MHz / 6). A cycle is 6 ticks.

// Temperature ADC conversion time: 12.5 cycles + 71.5 cycles = 84 cycles = 7us.
const uint16_t temperature_sample_time = (71.5 + 12.5)*6;

// Current ADC conversion time: 12.5 cycles + 1.5 cycles = 14 cycles = 1.16us.
const uint16_t current_sample_time = (1.5 + 12.5)*6;

const uint16_t current_sample_lead_time = (1.5 + 12.5 + 1.5)*6 / 2;

// The ADC will read the temperature first then 2 phase currents; try to time the sampling 
// time of the phase currents symmetrically around the peak of the PWM cycle.
const int16_t sample_lead_time = temperature_sample_time + current_sample_lead_time;


// Ticks per second at 72MHz clock speed. Each tick is ~13.89ns.
const int ticks_per_second = 72'000'000;

// Auto-reload value for the PWM timer.
const uint16_t pwm_autoreload = pwm_base - 1;

// Number of MCU clock ticks per PWM cycle; counting up then down.
const uint16_t pwm_period = 2 * pwm_base; 

// Number of PWM cycles per second: 3072 ticks = 42.7us @ 72MHz = 23.4KHz
const int pwm_cycles_per_second = ticks_per_second / pwm_period;

// Maximum duty cycle for the high side mosfet needs to allow some off time for 
// the bootstrap capacitor to charge so it has enough voltage to turn mosfet on.
const uint16_t minimum_bootstrap_duty = 16; // 16/72MHz = 222ns

// Maximum duty cycle for the high side mosfet. We need to allow some off time for the 
// bootstrap capacitor to charge so it has enough voltage to turn mosfet on. And also
// enough time to connect all low side mosfets to ground in order to sample phase currents.
const uint16_t pwm_max = pwm_base - max(current_sample_lead_time, minimum_bootstrap_duty); 

// Maximum duty for hold commands.
const uint16_t pwm_max_hold = pwm_base * 2 / 10;

// Maximum time (in pwm cycles) while a command is in effect.
const uint16_t max_timeout = 0xFFFF;


// Time dependent constants
// ------------------------

const int phase_readout_diff_per_cycle_to_voltage = (
    (pwm_cycles_per_second * phase_inductance * voltage_fixed_point + inductance_fixed_point / 2) / 
    inductance_fixed_point
);

// Motor control tables
// --------------------

// Motor voltage fraction for the 6-step commutation.
const uint16_t motor_sector_driving_positive[6][3] = {
    {0,        pwm_base, 0       },
    {0,        pwm_base, pwm_base},
    {0,        0,        pwm_base},
    {pwm_base, 0,        pwm_base},
    {pwm_base, 0,        0       },
    {pwm_base, pwm_base, 0       },
};

// Surpirsingly good schedule for the 6-step commutation.
const uint16_t motor_sector_driving_negative[6][3] {
    {0,        0,        pwm_base},
    {pwm_base, 0,        pwm_base},
    {pwm_base, 0,        0       },
    {pwm_base, pwm_base, 0       },
    {0,        pwm_base, 0       },
    {0,        pwm_base, pwm_base},
};



// Position tracking defaults
// --------------------------


// Some useful functions to compute position. These need to be inlined for 
// efficiency, and constexpr to define a bunch more constants below.


// Mapping from hall state to sector number.
static inline uint8_t get_hall_sector(const uint8_t hall_state){
    // Get the hall sector from the state.
    switch (hall_state & 0b111) {
        case 0b000: // no hall sensors; either it's not ready or no magnet
            return hall_sector_base; // Out of range, indicates invalid.
        case 0b001: // hall U active; 0 degrees
            return 0;
        case 0b011: // hall U and hall V active; 60 degrees
            return 1;
        case 0b010: // hall V active; 120 degrees
            return 2;
        case 0b110: // hall V and hall W active; 180 degrees
            return 3;
        case 0b100: // hall W active; 240 degrees
            return 4;
        case 0b101: // hall U and hall W active; 300 degrees
            return 5;
        case 0b111: // all hall sensors active; this would be quite unusual; but carry on
            return hall_sector_base; // Out of range, indicates invalid.
    }
    // We shouldn't reach here.
    return hall_sector_base;
}

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

// Normalize to a positive angle (0 to 2pi).
inline constexpr int normalize_angle(int angle){
    return (angle + angle_base) % angle_base;
}
// Normalize a 0 centerd angle; keeping its sign (-pi to pi).
inline constexpr int signed_angle(int angle){
    return (angle + one_and_half_circle) % angle_base - half_circle;
}

// Scaling factor for variance.
const int variance_divider = 16;

// Note speed and acceleration are written in degrees per ms and per ms^2 respectively.

// Variance of the hall sensor; it doesn't seem to be consistent, even between two rotations.
const int default_sector_transition_variance = square(10 * angle_base / 360) / variance_divider;

// Variance of a gaussian spread over the entire sector.
const int default_sector_center_variance = square(30 * angle_base / 360) / variance_divider;

// Ensure that our biggest variance is small enough to be usable in gaussian updates while staying < max_16bit.
static_assert(default_sector_center_variance * 16 < max_16bit, "max_variance must be less than 32768 (max 16-bit signed int)");

// The hall sensors trigger later than expected going each direction.
const int hysterisis = 5 * angle_base / 360;

// Maximum speed achievable by the motor; in electric revolutions per minute (RPM).
const int max_rpm = 32'000 * rotor_revolutions_per_electric;

// Speed needs more precision than angle. The speed is in angle units per pwm cycle / fixed point.
const int speed_fixed_point = 128;

// Fixed point for speed variance with respect to angle units variance.
const int speed_variance_fixed_point = 256;

// Conversion factor between square speed and speed variance.
const int speed_variance_to_square_speed = square(speed_fixed_point) / speed_variance_fixed_point * variance_divider;

// Maximum angular speed that we can represent in the fixed point representation.
const int max_angular_speed = max_rpm / 60 * angle_base * speed_fixed_point / pwm_cycles_per_second;

// Cap the update counter between transitions; we'll consider the motor settled at this point.
const int max_updates_between_transitions = 16384;

// Minimum angular speed that we need to have to move at the max time between transitions.
const int min_angular_speed = 60 * angle_base / 360 * speed_fixed_point / max_updates_between_transitions;

static_assert(min_angular_speed > 0, "min_angular_speed must be greater than 0");
static_assert(min_angular_speed < 4, "min_angular_speed should be quite low");

// Minimum speed in rotor revolutions per minute (RPM) that we can represent with our units.
const int min_rpm = min_angular_speed * pwm_cycles_per_second / angle_base * 60 / speed_fixed_point / rotor_revolutions_per_electric;

static_assert(max_angular_speed < max_16bit, "max_angular_speed must be less than 32768 (max 16-bit signed int)");

// Threshold speed when we consider the motor to be moving; in angle units per pwm cycle.
const int threshold_speed = 10 * angle_base * speed_fixed_point / 360 / (pwm_cycles_per_second / 1000);

// Variance of the speed in degrees per ms; converted to angle units per pwm cycle, all squared.
const int default_speed_variance = square(15 * angle_base / 360 * 1000 * speed_fixed_point / pwm_cycles_per_second) / speed_variance_to_square_speed;

static_assert(default_speed_variance < max_16bit, "default_speed_variance must be less than max_variance");

// Acceleration fixed point with respect to speed units.
const int acceleration_fixed_point = 32;

// Fixed point for acceleration variance with respect to speed variance.
const int acceleration_variance_fixed_point = 32;

// Covnersion factor between square acceleration and acceleration variance.
const int acceleration_variance_to_square_acceleration = square(acceleration_fixed_point) / acceleration_variance_fixed_point * speed_variance_to_square_speed;

// Variance of the expected acceleration loads in degrees per ms^2; converted to acceleration units per pwm cycle, all squared.
const int angular_acceleration_div_2_variance = round_div(
    square(
        15 * angle_base / 360 * 
        speed_fixed_point * 1000 / pwm_cycles_per_second * 
        acceleration_fixed_point * 1000 / pwm_cycles_per_second / 
        2
    ),
    acceleration_variance_to_square_acceleration
);

const PositionCalibration default_position_calibration = {
    // The angle at which we transition to this sector. The first is when rotating in the
    // positive direction; second for the negative direction.
    .sector_transition_angles = {{
        {330 * angle_base / 360 + hysterisis,  30 * angle_base / 360 - hysterisis},
        { 30 * angle_base / 360 + hysterisis,  90 * angle_base / 360 - hysterisis},
        { 90 * angle_base / 360 + hysterisis, 150 * angle_base / 360 - hysterisis},
        {150 * angle_base / 360 + hysterisis, 210 * angle_base / 360 - hysterisis},
        {210 * angle_base / 360 + hysterisis, 270 * angle_base / 360 - hysterisis},
        {270 * angle_base / 360 + hysterisis, 330 * angle_base / 360 - hysterisis},     
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
    .initial_angular_speed_variance = default_speed_variance,
    // Precalculate the variance of the acceleration divided by 2.
    .angular_acceleration_div_2_variance = angular_acceleration_div_2_variance,
};

const int16_t current_calibration_fixed_point = 1024;

const CurrentCalibration default_current_calibration = {
    .u_factor = current_calibration_fixed_point,
    .v_factor = current_calibration_fixed_point,
    .w_factor = current_calibration_fixed_point,
    .inductance_factor = current_calibration_fixed_point,
};

const int16_t gains_fixed_point = 1024;

const PIDParameters default_pid_parameters = {
    .current_angle_gains = PIDGains{
        .kp = 32,
        .ki = 32,
        .kd = 32,
        .max_output = quarter_circle / 2
    },
    .torque_gains = PIDGains{
        .kp = 32,
        .ki = 32,
        .kd = 32,
        .max_output = pwm_base + 512 // Give some leeway for the integral to stay at max command.
    },
    .angular_speed_gains = PIDGains{
        .kp = gains_fixed_point / 8,
        .ki = gains_fixed_point / 64,
        .kd = gains_fixed_point / 128,
        .max_output = static_cast<int16_t>(sqrt(default_speed_variance * speed_variance_to_square_speed))
    },
    .position_gains = PIDGains{
        .kp = gains_fixed_point / 8,
        .ki = gains_fixed_point / 64,
        .kd = gains_fixed_point / 128,
        .max_output = half_circle
    }
};




static_assert(angle_base == 1024, "angle_base must be 1024 for the sine and cosine lookup tables to work correctly");

const uint16_t phases_waveform[1024] = {
    1330, 1335, 1340, 1344, 1349, 1353, 1358, 1362, 1366, 1371, 1375, 1379, 1383, 1387, 1391, 1395,
    1399, 1403, 1407, 1411, 1414, 1418, 1421, 1425, 1429, 1432, 1435, 1439, 1442, 1445, 1448, 1451,
    1454, 1457, 1460, 1463, 1466, 1469, 1472, 1474, 1477, 1480, 1482, 1484, 1487, 1489, 1491, 1494,
    1496, 1498, 1500, 1502, 1504, 1506, 1508, 1509, 1511, 1513, 1514, 1516, 1517, 1519, 1520, 1522,
    1523, 1524, 1525, 1526, 1527, 1528, 1529, 1530, 1531, 1532, 1532, 1533, 1533, 1534, 1534, 1535,
    1535, 1535, 1536, 1536, 1536, 1536, 1536, 1536, 1536, 1536, 1535, 1535, 1535, 1534, 1534, 1533,
    1533, 1532, 1531, 1531, 1530, 1529, 1528, 1527, 1526, 1525, 1524, 1522, 1521, 1520, 1518, 1517,
    1515, 1514, 1512, 1511, 1509, 1507, 1505, 1503, 1501, 1499, 1497, 1495, 1493, 1491, 1488, 1486,
    1484, 1481, 1479, 1476, 1473, 1471, 1468, 1465, 1462, 1459, 1456, 1453, 1450, 1447, 1444, 1441,
    1438, 1434, 1431, 1427, 1424, 1420, 1417, 1413, 1409, 1405, 1402, 1398, 1394, 1390, 1386, 1382,
    1378, 1373, 1369, 1365, 1361, 1356, 1352, 1347, 1343, 1338, 1333, 1329, 1324, 1319, 1314, 1309,
    1304, 1299, 1294, 1289, 1284, 1279, 1274, 1268, 1263, 1258, 1252, 1247, 1241, 1236, 1230, 1224,
    1219, 1213, 1207, 1201, 1195, 1189, 1183, 1177, 1171, 1165, 1159, 1153, 1147, 1140, 1134, 1127,
    1121, 1115, 1108, 1102, 1095, 1088, 1082, 1075, 1068, 1061, 1055, 1048, 1041, 1034, 1027, 1020,
    1013, 1006,  999,  991,  984,  977,  970,  962,  955,  947,  940,  933,  925,  918,  910,  902,
     895,  887,  879,  872,  864,  856,  848,  840,  832,  824,  816,  808,  800,  792,  784,  776,
     768,  760,  752,  743,  735,  727,  719,  710,  702,  693,  685,  677,  668,  660,  651,  642,
     634,  625,  617,  608,  599,  591,  582,  573,  565,  556,  547,  538,  529,  520,  512,  503,
     494,  485,  476,  467,  458,  449,  440,  431,  422,  413,  404,  395,  385,  376,  367,  358,
     349,  340,  330,  321,  312,  303,  293,  284,  275,  266,  256,  247,  238,  228,  219,  210,
     200,  191,  182,  172,  163,  154,  144,  135,  126,  116,  107,   97,   88,   79,   69,   60,
      50,   41,   31,   22,   13,    3,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
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
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    3,   13,   22,   31,   41,
      50,   60,   69,   79,   88,   97,  107,  116,  126,  135,  144,  154,  163,  172,  182,  191,
     200,  210,  219,  228,  238,  247,  256,  266,  275,  284,  293,  303,  312,  321,  330,  340,
     349,  358,  367,  376,  385,  395,  404,  413,  422,  431,  440,  449,  458,  467,  476,  485,
     494,  503,  512,  520,  529,  538,  547,  556,  565,  573,  582,  591,  599,  608,  617,  625,
     634,  642,  651,  660,  668,  677,  685,  693,  702,  710,  719,  727,  735,  743,  752,  760,
     768,  776,  784,  792,  800,  808,  816,  824,  832,  840,  848,  856,  864,  872,  879,  887,
     895,  902,  910,  918,  925,  933,  940,  947,  955,  962,  970,  977,  984,  991,  999, 1006,
    1013, 1020, 1027, 1034, 1041, 1048, 1055, 1061, 1068, 1075, 1082, 1088, 1095, 1102, 1108, 1115,
    1121, 1127, 1134, 1140, 1147, 1153, 1159, 1165, 1171, 1177, 1183, 1189, 1195, 1201, 1207, 1213,
    1219, 1224, 1230, 1236, 1241, 1247, 1252, 1258, 1263, 1268, 1274, 1279, 1284, 1289, 1294, 1299,
    1304, 1309, 1314, 1319, 1324, 1329, 1333, 1338, 1343, 1347, 1352, 1356, 1361, 1365, 1369, 1373,
    1378, 1382, 1386, 1390, 1394, 1398, 1402, 1405, 1409, 1413, 1417, 1420, 1424, 1427, 1431, 1434,
    1438, 1441, 1444, 1447, 1450, 1453, 1456, 1459, 1462, 1465, 1468, 1471, 1473, 1476, 1479, 1481,
    1484, 1486, 1488, 1491, 1493, 1495, 1497, 1499, 1501, 1503, 1505, 1507, 1509, 1511, 1512, 1514,
    1515, 1517, 1518, 1520, 1521, 1522, 1524, 1525, 1526, 1527, 1528, 1529, 1530, 1531, 1531, 1532,
    1533, 1533, 1534, 1534, 1535, 1535, 1535, 1536, 1536, 1536, 1536, 1536, 1536, 1536, 1536, 1535,
    1535, 1535, 1534, 1534, 1533, 1533, 1532, 1532, 1531, 1530, 1529, 1528, 1527, 1526, 1525, 1524,
    1523, 1522, 1520, 1519, 1517, 1516, 1514, 1513, 1511, 1509, 1508, 1506, 1504, 1502, 1500, 1498,
    1496, 1494, 1491, 1489, 1487, 1484, 1482, 1480, 1477, 1474, 1472, 1469, 1466, 1463, 1460, 1457,
    1454, 1451, 1448, 1445, 1442, 1439, 1435, 1432, 1429, 1425, 1421, 1418, 1414, 1411, 1407, 1403,
    1399, 1395, 1391, 1387, 1383, 1379, 1375, 1371, 1366, 1362, 1358, 1353, 1349, 1344, 1340, 1335
};

static inline int16_t get_phase_pwm(const int16_t angle) {
    return phases_waveform[normalize_angle(angle) / angle_lookup_divider];
}

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

static inline int16_t get_sin(const int16_t angle) {
    return sin_lookup[normalize_angle(angle) / angle_lookup_divider];
}

// For cos lookup we can use the sin lookup table + 90 degrees (quarter_circle).
static inline int16_t get_cos(const int16_t angle) {
    return sin_lookup[normalize_angle(angle + quarter_circle) / angle_lookup_divider];
}