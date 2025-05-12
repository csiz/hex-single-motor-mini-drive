#pragma once

#include <cstdint>
#include <cstddef>

#include "type_definitions.hpp"


// Data storage available.
const size_t HISTORY_SIZE = 360;


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
const float current_conversion = adc_voltage_reference / (adc_max_value * motor_shunt_resistance * amplifier_gain);

const int16_t current_int_scale = static_cast<int16_t>(1/current_conversion);

const float phase_resistance = 2.0; // 2 Ohm resistance of the motor phase windings & mosfet.

const int16_t phase_int_resistance = static_cast<int16_t>(phase_resistance);

// Note ADC conversion time is = sample time + 12.5 cycles. The ADC clock is 12MHz (72MHz / 6). A cycle is 6 ticks.

// Temperature ADC conversion time: 12.5 cycles + 71.5 cycles = 84 cycles = 7us.
const uint16_t TEMP_SAMPLE_TIME = (71.5 + 12.5)*6;
// Current ADC conversion time: 12.5 cycles + 1.5 cycles = 14 cycles = 1.16us.
const uint16_t CURR_SAMPLE_TIME = (1.5 + 12.5)*6;

const uint16_t CURR_SAMPLE_LEAD_TIME = (1.5 + 12.5 + 1.5)*6 / 2;

// The ADC will read the temperature first then 2 phase currents; try to time the sampling 
// time of the phase currents symmetrically around the peak of the PWM cycle.
const int16_t SAMPLE_LEAD_TIME = TEMP_SAMPLE_TIME + CURR_SAMPLE_LEAD_TIME;


// Motor PWM constants
// -------------------

const uint16_t PWM_AUTORELOAD = 1535;
const uint16_t PWM_BASE = PWM_AUTORELOAD + 1;
const uint16_t PWM_PERIOD = 2 * PWM_BASE; // 3072 ticks = 42.7us @ 72MHz = 23.4KHz

// Maximum duty cycle for the high side mosfet needs to allow some off time for 
// the bootstrap capacitor to charge so it has enough voltage to turn mosfet on.
const uint16_t MIN_BOOTSTRAP_DUTY = 16; // 16/72MHz = 222ns

const uint16_t PWM_MAX = PWM_BASE - (CURR_SAMPLE_LEAD_TIME > MIN_BOOTSTRAP_DUTY ? CURR_SAMPLE_LEAD_TIME : MIN_BOOTSTRAP_DUTY); 

const uint16_t PWM_MAX_HOLD = PWM_BASE * 2 / 10;

const uint16_t MAX_TIMEOUT = 0xFFFF;



// Motor control tables
// --------------------

// Motor voltage fraction for the 6-step commutation.
const uint16_t motor_sector_driving_pos[6][3] = {
    {0,        PWM_BASE, 0       },
    {0,        PWM_BASE, PWM_BASE},
    {0,        0,        PWM_BASE},
    {PWM_BASE, 0,        PWM_BASE},
    {PWM_BASE, 0,        0       },
    {PWM_BASE, PWM_BASE, 0       },
};

// Surpirsingly good schedule for the 6-step commutation.
const uint16_t motor_sector_driving_neg[6][3] {
    {0,        0,        PWM_BASE},
    {PWM_BASE, 0,        PWM_BASE},
    {PWM_BASE, 0,        0       },
    {PWM_BASE, PWM_BASE, 0       },
    {0,        PWM_BASE, 0       },
    {0,        PWM_BASE, PWM_BASE},
};



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
const int angle_base = 1024;
// Half a circle (pi).
const int half_circle = angle_base / 2;
// 3/2 of a circle (3pi/2).
const int one_and_half_circle = (3 * angle_base) / 2;

// 2/3 of a circle (2pi/3).
const int two_thirds_circle = (2 * angle_base) / 3;

// 1/3 of a circle (pi/3).
const int third_circle = angle_base / 3;

// 3/4 of a circle (3pi/4).
const int three_quarters_circle = (3 * angle_base) / 4;

// 1/4 of a circle (pi/4).
const int quarter_circle = angle_base / 4;

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

const PositionCalibration default_position_calibration = {
    // The angle at which we transition to this sector. The first is when rotating in the
    // positive direction; second for the negative direction.
    .trigger_angles = {{
        {330 * angle_base / 360 + hysterisis,  30 * angle_base / 360 - hysterisis},
        { 30 * angle_base / 360 + hysterisis,  90 * angle_base / 360 - hysterisis},
        { 90 * angle_base / 360 + hysterisis, 150 * angle_base / 360 - hysterisis},
        {150 * angle_base / 360 + hysterisis, 210 * angle_base / 360 - hysterisis},
        {210 * angle_base / 360 + hysterisis, 270 * angle_base / 360 - hysterisis},
        {270 * angle_base / 360 + hysterisis, 330 * angle_base / 360 - hysterisis},     
    }},
    // Variance of each sector transition; we can calibrate it.
    .trigger_angle_variances = {{
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

const int16_t calibration_base = 256;

const CurrentCalibration default_current_calibration = {
    .u_factor = calibration_base,
    .v_factor = calibration_base,
    .w_factor = calibration_base,
};




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