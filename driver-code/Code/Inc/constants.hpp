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
const int angle_base = 1 << 12; // 4096

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

const size_t current_detected_bit_offset = 9;
const uint16_t current_detected_bit_mask = 0b1 << current_detected_bit_offset;


// Observer constants
// ------------------

const int16_t observer_fixed_point = 4096;


const ObserverParameters default_observer_parameters = {
    .rotor_angle_ki = observer_fixed_point / 4,
    .rotor_angular_speed_ki = observer_fixed_point / 16,
    .inductor_angle_ki = observer_fixed_point / 4,
    .inductor_angular_speed_ki = observer_fixed_point / 16,
    .resistance_ki = 16,
    .inductance_ki = 16,
    .motor_constant_ki = 16,
    .magnetic_resistance_ki = 16,
    .rotor_mass_ki = 16,
    .rotor_torque_ki = 16
};


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

// 4A max DQ0 driving current.
const int16_t max_drive_current = 4 * current_fixed_point;


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

// Conversion factor between current and phase resistance voltage.
const int phase_current_to_voltage = round_div(phase_resistance * voltage_fixed_point, resistance_fixed_point);


// Power conversion: 1 power unit = 1/224 W.
const int power_fixed_point = 4 * voltage_fixed_point;

// 12W max drive power.
const int max_drive_power = 12 * power_fixed_point;

// We need this to convert a few formulas using voltage to power.
const int power_div_voltage_fixed_point = power_fixed_point / voltage_fixed_point;

// Directly convert voltage * current to power in fixed point format.
const int voltage_current_div_power_fixed_point = current_fixed_point / power_div_voltage_fixed_point;

const int dq0_to_power_fixed_point = voltage_current_div_power_fixed_point * 3 / 2;

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

// Save a bit of PWM capacity to compensate for sudden rotor acceleration.
// 
// The rotor usually has a gearbox and gears wiggle and bump into each other. That
// means the rotor experiences large accelerations and decelerations from the tooth
// collisions; they aren't just bad data, those acceleration spikes are real. We need
// to be able to compensate for the increased back EMF while we wait for the gears to
// get back into sync. Therefore we need to save some spare PWM capacity.
const uint16_t pwm_max_smooth = pwm_max - pwm_base / 10;

// Maximum duty for hold commands.
const uint16_t pwm_max_hold = pwm_base * 2 / 10;

// Maximum time (in pwm cycles) while a command is in effect.
const uint16_t max_timeout = 0xFFFF;



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

// Note speed values written in degrees per ms and converted to speed units.

// Maximum speed achievable by the motor; in electric revolutions per minute (RPM).
const int max_rpm = 32'000 * rotor_revolutions_per_electric;

// Speed needs more precision than angle. The speed is in angle units per pwm cycle / fixed point.
const int speed_fixed_point = 16;

// Maximum angular speed that we can represent in the fixed point representation.
const int max_angular_speed = max_rpm / 60 * angle_base * speed_fixed_point / pwm_cycles_per_second;

// Minimum speed in rotor revolutions per minute (RPM) that we can represent with our units.
const int min_rpm = 1 * pwm_cycles_per_second / angle_base * 60 / speed_fixed_point / rotor_revolutions_per_electric;

static_assert(max_angular_speed < max_16bit, "max_angular_speed must be less than 32768 (max 16-bit signed int)");

const int emf_base = static_cast<int>(pwm_base * 1.16);

// Fixed point representation of the motor constant; units are V/(rad/s) = Volt * second.
const int motor_constant_fixed_point = 1 << 20;

const int radians_per_sec_div_angle_base = pwm_cycles_per_second / half_circle_div_pi;


// Time dependent constants
// ------------------------

const int phase_readout_diff_per_cycle_to_voltage = round_div(
    pwm_cycles_per_second * phase_inductance * voltage_fixed_point,
    inductance_fixed_point
);


const int emf_change_rotor_voltage_conversion = (
    motor_constant_fixed_point / voltage_fixed_point * speed_fixed_point / radians_per_sec_div_angle_base
);


// Calibration and PID constants
// -----------------------------


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
        .kp = 128,
        .ki = 8,
        .kd = 64,
        .max_output = quarter_circle
    },
    .torque_gains = PIDGains{
        .kp = 0,
        .ki = 8,
        .kd = 0,
        .max_output = pwm_max - 40
    },
    .battery_power_gains = PIDGains{
        .kp = 0,
        .ki = 2,
        .kd = 0,
        .max_output = pwm_max - 40
    },
    .angular_speed_gains = PIDGains{
        .kp = gains_fixed_point / 8,
        .ki = gains_fixed_point / 64,
        .kd = gains_fixed_point / 128,
        .max_output = max_angular_speed
    },
    .position_gains = PIDGains{
        .kp = gains_fixed_point / 8,
        .ki = gains_fixed_point / 64,
        .kd = gains_fixed_point / 128,
        .max_output = half_circle
    }
};


static_assert(angle_base == 4096, "angle_base must be 4096 for the sine and cosine lookup tables to work correctly");

const uint16_t phases_waveform[1024] = {
    1330, 1333, 1335, 1337, 1340, 1342, 1344, 1346, 1349, 1351, 1353, 1355, 1358, 1360, 1362, 1364,
    1366, 1368, 1371, 1373, 1375, 1377, 1379, 1381, 1383, 1385, 1387, 1389, 1391, 1393, 1395, 1397,
    1399, 1401, 1403, 1405, 1407, 1409, 1411, 1412, 1414, 1416, 1418, 1420, 1421, 1423, 1425, 1427,
    1429, 1430, 1432, 1434, 1435, 1437, 1439, 1440, 1442, 1444, 1445, 1447, 1448, 1450, 1451, 1453,
    1454, 1456, 1457, 1459, 1460, 1462, 1463, 1465, 1466, 1468, 1469, 1470, 1472, 1473, 1474, 1476,
    1477, 1478, 1480, 1481, 1482, 1483, 1484, 1486, 1487, 1488, 1489, 1490, 1491, 1493, 1494, 1495,
    1496, 1497, 1498, 1499, 1500, 1501, 1502, 1503, 1504, 1505, 1506, 1507, 1508, 1509, 1509, 1510,
    1511, 1512, 1513, 1514, 1514, 1515, 1516, 1517, 1517, 1518, 1519, 1520, 1520, 1521, 1522, 1522,
    1523, 1523, 1524, 1525, 1525, 1526, 1526, 1527, 1527, 1528, 1528, 1529, 1529, 1530, 1530, 1530,
    1531, 1531, 1532, 1532, 1532, 1533, 1533, 1533, 1533, 1534, 1534, 1534, 1534, 1535, 1535, 1535,
    1535, 1535, 1535, 1536, 1536, 1536, 1536, 1536, 1536, 1536, 1536, 1536, 1536, 1536, 1536, 1536,
    1536, 1536, 1536, 1535, 1535, 1535, 1535, 1535, 1535, 1535, 1534, 1534, 1534, 1534, 1533, 1533,
    1533, 1532, 1532, 1532, 1531, 1531, 1531, 1530, 1530, 1529, 1529, 1528, 1528, 1527, 1527, 1526,
    1526, 1525, 1525, 1524, 1524, 1523, 1522, 1522, 1521, 1521, 1520, 1519, 1518, 1518, 1517, 1516,
    1515, 1515, 1514, 1513, 1512, 1511, 1511, 1510, 1509, 1508, 1507, 1506, 1505, 1504, 1503, 1502,
    1501, 1500, 1499, 1498, 1497, 1496, 1495, 1494, 1493, 1492, 1491, 1490, 1488, 1487, 1486, 1485,
    1484, 1482, 1481, 1480, 1479, 1477, 1476, 1475, 1473, 1472, 1471, 1469, 1468, 1467, 1465, 1464,
    1462, 1461, 1459, 1458, 1456, 1455, 1453, 1452, 1450, 1449, 1447, 1446, 1444, 1442, 1441, 1439,
    1438, 1436, 1434, 1433, 1431, 1429, 1427, 1426, 1424, 1422, 1420, 1418, 1417, 1415, 1413, 1411,
    1409, 1407, 1405, 1404, 1402, 1400, 1398, 1396, 1394, 1392, 1390, 1388, 1386, 1384, 1382, 1380,
    1378, 1376, 1373, 1371, 1369, 1367, 1365, 1363, 1361, 1358, 1356, 1354, 1352, 1349, 1347, 1345,
    1343, 1340, 1338, 1336, 1333, 1331, 1329, 1326, 1324, 1321, 1319, 1317, 1314, 1312, 1309, 1307,
    1304, 1302, 1299, 1297, 1294, 1292, 1289, 1287, 1284, 1281, 1279, 1276, 1274, 1271, 1268, 1266,
    1263, 1260, 1258, 1255, 1252, 1249, 1247, 1244, 1241, 1238, 1236, 1233, 1230, 1227, 1224, 1221,
    1219, 1216, 1213, 1210, 1207, 1204, 1201, 1198, 1195, 1192, 1189, 1186, 1183, 1180, 1177, 1174,
    1171, 1168, 1165, 1162, 1159, 1156, 1153, 1150, 1147, 1143, 1140, 1137, 1134, 1131, 1127, 1124,
    1121, 1118, 1115, 1111, 1108, 1105, 1102, 1098, 1095, 1092, 1088, 1085, 1082, 1078, 1075, 1072,
    1068, 1065, 1061, 1058, 1055, 1051, 1048, 1044, 1041, 1037, 1034, 1030, 1027, 1023, 1020, 1016,
    1013, 1009, 1006, 1002,  999,  995,  991,  988,  984,  980,  977,  973,  970,  966,  962,  959,
     955,  951,  947,  944,  940,  936,  933,  929,  925,  921,  918,  914,  910,  906,  902,  899,
     895,  891,  887,  883,  879,  875,  872,  868,  864,  860,  856,  852,  848,  844,  840,  836,
     832,  828,  824,  820,  816,  812,  808,  804,  800,  796,  792,  788,  784,  780,  776,  772,
     768,  764,  760,  756,  752,  748,  743,  739,  735,  731,  727,  723,  719,  714,  710,  706,
     702,  698,  693,  689,  685,  681,  677,  672,  668,  664,  660,  655,  651,  647,  642,  638,
     634,  630,  625,  621,  617,  612,  608,  604,  599,  595,  591,  586,  582,  578,  573,  569,
     565,  560,  556,  551,  547,  543,  538,  534,  529,  525,  520,  516,  512,  507,  503,  498,
     494,  489,  485,  480,  476,  471,  467,  462,  458,  453,  449,  444,  440,  435,  431,  426,
     422,  417,  413,  408,  404,  399,  395,  390,  385,  381,  376,  372,  367,  363,  358,  353,
     349,  344,  340,  335,  330,  326,  321,  317,  312,  307,  303,  298,  293,  289,  284,  280,
     275,  270,  266,  261,  256,  252,  247,  242,  238,  233,  228,  224,  219,  214,  210,  205,
     200,  196,  191,  186,  182,  177,  172,  168,  163,  158,  154,  149,  144,  140,  135,  130,
     126,  121,  116,  111,  107,  102,   97,   93,   88,   83,   79,   74,   69,   64,   60,   55,
      50,   46,   41,   36,   31,   27,   22,   17,   13,    8,    3,    0,    0,    0,    0,    0,
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
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0
};

static inline int16_t get_phase_pwm(const int16_t angle) {
    const int norm_angle = normalize_angle(angle);
    return (
        norm_angle < half_circle ? 
        phases_waveform[norm_angle >> 1] : 
        phases_waveform[(angle_base - norm_angle - 1) >> 1]
    );
}

const uint16_t sin_lookup[1024] = {
        0,     6,    13,    19,    25,    31,    38,    44,    50,    57,    63,    69,    75,    82,    88,    94,
      101,   107,   113,   119,   126,   132,   138,   144,   151,   157,   163,   170,   176,   182,   188,   195,
      201,   207,   214,   220,   226,   232,   239,   245,   251,   257,   264,   270,   276,   283,   289,   295,
      301,   308,   314,   320,   326,   333,   339,   345,   351,   358,   364,   370,   376,   383,   389,   395,
      401,   408,   414,   420,   426,   433,   439,   445,   451,   458,   464,   470,   476,   483,   489,   495,
      501,   508,   514,   520,   526,   533,   539,   545,   551,   557,   564,   570,   576,   582,   589,   595,
      601,   607,   613,   620,   626,   632,   638,   644,   651,   657,   663,   669,   675,   682,   688,   694,
      700,   706,   713,   719,   725,   731,   737,   744,   750,   756,   762,   768,   774,   781,   787,   793,
      799,   805,   811,   818,   824,   830,   836,   842,   848,   854,   861,   867,   873,   879,   885,   891,
      897,   904,   910,   916,   922,   928,   934,   940,   946,   953,   959,   965,   971,   977,   983,   989,
      995,  1001,  1007,  1014,  1020,  1026,  1032,  1038,  1044,  1050,  1056,  1062,  1068,  1074,  1080,  1086,
     1092,  1099,  1105,  1111,  1117,  1123,  1129,  1135,  1141,  1147,  1153,  1159,  1165,  1171,  1177,  1183,
     1189,  1195,  1201,  1207,  1213,  1219,  1225,  1231,  1237,  1243,  1249,  1255,  1261,  1267,  1273,  1279,
     1285,  1291,  1297,  1303,  1309,  1315,  1321,  1327,  1332,  1338,  1344,  1350,  1356,  1362,  1368,  1374,
     1380,  1386,  1392,  1398,  1404,  1409,  1415,  1421,  1427,  1433,  1439,  1445,  1451,  1457,  1462,  1468,
     1474,  1480,  1486,  1492,  1498,  1503,  1509,  1515,  1521,  1527,  1533,  1538,  1544,  1550,  1556,  1562,
     1567,  1573,  1579,  1585,  1591,  1596,  1602,  1608,  1614,  1620,  1625,  1631,  1637,  1643,  1648,  1654,
     1660,  1666,  1671,  1677,  1683,  1689,  1694,  1700,  1706,  1711,  1717,  1723,  1729,  1734,  1740,  1746,
     1751,  1757,  1763,  1768,  1774,  1780,  1785,  1791,  1797,  1802,  1808,  1813,  1819,  1825,  1830,  1836,
     1842,  1847,  1853,  1858,  1864,  1870,  1875,  1881,  1886,  1892,  1898,  1903,  1909,  1914,  1920,  1925,
     1931,  1936,  1942,  1947,  1953,  1958,  1964,  1970,  1975,  1981,  1986,  1992,  1997,  2002,  2008,  2013,
     2019,  2024,  2030,  2035,  2041,  2046,  2052,  2057,  2062,  2068,  2073,  2079,  2084,  2090,  2095,  2100,
     2106,  2111,  2117,  2122,  2127,  2133,  2138,  2143,  2149,  2154,  2159,  2165,  2170,  2175,  2181,  2186,
     2191,  2197,  2202,  2207,  2213,  2218,  2223,  2228,  2234,  2239,  2244,  2249,  2255,  2260,  2265,  2270,
     2276,  2281,  2286,  2291,  2296,  2302,  2307,  2312,  2317,  2322,  2328,  2333,  2338,  2343,  2348,  2353,
     2359,  2364,  2369,  2374,  2379,  2384,  2389,  2394,  2399,  2405,  2410,  2415,  2420,  2425,  2430,  2435,
     2440,  2445,  2450,  2455,  2460,  2465,  2470,  2475,  2480,  2485,  2490,  2495,  2500,  2505,  2510,  2515,
     2520,  2525,  2530,  2535,  2540,  2545,  2550,  2555,  2559,  2564,  2569,  2574,  2579,  2584,  2589,  2594,
     2598,  2603,  2608,  2613,  2618,  2623,  2628,  2632,  2637,  2642,  2647,  2652,  2656,  2661,  2666,  2671,
     2675,  2680,  2685,  2690,  2694,  2699,  2704,  2709,  2713,  2718,  2723,  2727,  2732,  2737,  2741,  2746,
     2751,  2755,  2760,  2765,  2769,  2774,  2779,  2783,  2788,  2792,  2797,  2802,  2806,  2811,  2815,  2820,
     2824,  2829,  2833,  2838,  2843,  2847,  2852,  2856,  2861,  2865,  2870,  2874,  2878,  2883,  2887,  2892,
     2896,  2901,  2905,  2910,  2914,  2918,  2923,  2927,  2932,  2936,  2940,  2945,  2949,  2953,  2958,  2962,
     2967,  2971,  2975,  2979,  2984,  2988,  2992,  2997,  3001,  3005,  3009,  3014,  3018,  3022,  3026,  3031,
     3035,  3039,  3043,  3048,  3052,  3056,  3060,  3064,  3068,  3073,  3077,  3081,  3085,  3089,  3093,  3097,
     3102,  3106,  3110,  3114,  3118,  3122,  3126,  3130,  3134,  3138,  3142,  3146,  3150,  3154,  3158,  3162,
     3166,  3170,  3174,  3178,  3182,  3186,  3190,  3194,  3198,  3202,  3206,  3210,  3214,  3217,  3221,  3225,
     3229,  3233,  3237,  3241,  3244,  3248,  3252,  3256,  3260,  3264,  3267,  3271,  3275,  3279,  3282,  3286,
     3290,  3294,  3297,  3301,  3305,  3309,  3312,  3316,  3320,  3323,  3327,  3331,  3334,  3338,  3342,  3345,
     3349,  3352,  3356,  3360,  3363,  3367,  3370,  3374,  3378,  3381,  3385,  3388,  3392,  3395,  3399,  3402,
     3406,  3409,  3413,  3416,  3420,  3423,  3426,  3430,  3433,  3437,  3440,  3444,  3447,  3450,  3454,  3457,
     3461,  3464,  3467,  3471,  3474,  3477,  3481,  3484,  3487,  3490,  3494,  3497,  3500,  3504,  3507,  3510,
     3513,  3516,  3520,  3523,  3526,  3529,  3532,  3536,  3539,  3542,  3545,  3548,  3551,  3555,  3558,  3561,
     3564,  3567,  3570,  3573,  3576,  3579,  3582,  3585,  3588,  3591,  3594,  3597,  3600,  3603,  3606,  3609,
     3612,  3615,  3618,  3621,  3624,  3627,  3630,  3633,  3636,  3639,  3642,  3644,  3647,  3650,  3653,  3656,
     3659,  3661,  3664,  3667,  3670,  3673,  3675,  3678,  3681,  3684,  3686,  3689,  3692,  3695,  3697,  3700,
     3703,  3705,  3708,  3711,  3713,  3716,  3719,  3721,  3724,  3727,  3729,  3732,  3734,  3737,  3739,  3742,
     3745,  3747,  3750,  3752,  3755,  3757,  3760,  3762,  3765,  3767,  3770,  3772,  3775,  3777,  3779,  3782,
     3784,  3787,  3789,  3791,  3794,  3796,  3798,  3801,  3803,  3805,  3808,  3810,  3812,  3815,  3817,  3819,
     3822,  3824,  3826,  3828,  3831,  3833,  3835,  3837,  3839,  3842,  3844,  3846,  3848,  3850,  3852,  3854,
     3857,  3859,  3861,  3863,  3865,  3867,  3869,  3871,  3873,  3875,  3877,  3879,  3881,  3883,  3885,  3887,
     3889,  3891,  3893,  3895,  3897,  3899,  3901,  3903,  3905,  3907,  3909,  3910,  3912,  3914,  3916,  3918,
     3920,  3921,  3923,  3925,  3927,  3929,  3930,  3932,  3934,  3936,  3937,  3939,  3941,  3943,  3944,  3946,
     3948,  3949,  3951,  3953,  3954,  3956,  3958,  3959,  3961,  3962,  3964,  3965,  3967,  3969,  3970,  3972,
     3973,  3975,  3976,  3978,  3979,  3981,  3982,  3984,  3985,  3987,  3988,  3989,  3991,  3992,  3994,  3995,
     3996,  3998,  3999,  4001,  4002,  4003,  4005,  4006,  4007,  4008,  4010,  4011,  4012,  4014,  4015,  4016,
     4017,  4019,  4020,  4021,  4022,  4023,  4024,  4026,  4027,  4028,  4029,  4030,  4031,  4032,  4034,  4035,
     4036,  4037,  4038,  4039,  4040,  4041,  4042,  4043,  4044,  4045,  4046,  4047,  4048,  4049,  4050,  4051,
     4052,  4053,  4053,  4054,  4055,  4056,  4057,  4058,  4059,  4060,  4060,  4061,  4062,  4063,  4064,  4064,
     4065,  4066,  4067,  4067,  4068,  4069,  4070,  4070,  4071,  4072,  4072,  4073,  4074,  4074,  4075,  4076,
     4076,  4077,  4077,  4078,  4079,  4079,  4080,  4080,  4081,  4081,  4082,  4082,  4083,  4083,  4084,  4084,
     4085,  4085,  4086,  4086,  4087,  4087,  4088,  4088,  4088,  4089,  4089,  4089,  4090,  4090,  4090,  4091,
     4091,  4091,  4092,  4092,  4092,  4092,  4093,  4093,  4093,  4093,  4094,  4094,  4094,  4094,  4094,  4095,
     4095,  4095,  4095,  4095,  4095,  4095,  4096,  4096,  4096,  4096,  4096,  4096,  4096,  4096,  4096,  4096
};

static inline int16_t get_sin(const int16_t angle) {
    const int norm_angle = normalize_angle(angle);
    
    if (norm_angle < half_circle) {
        if (norm_angle < quarter_circle) {
            return sin_lookup[norm_angle];
        } else {
            return sin_lookup[half_circle - norm_angle - 1];
        }
    } else {
        if (norm_angle < three_quarters_circle) {
            return -sin_lookup[norm_angle - half_circle];
        } else {
            return -sin_lookup[angle_base - norm_angle - 1];
        }
    }
}

// For cos lookup we can use the sin lookup table + 90 degrees (quarter_circle).
static inline int16_t get_cos(const int16_t angle) {
    return get_sin(angle + quarter_circle);
}