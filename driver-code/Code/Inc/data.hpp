#pragma once

#include <cstdint>
#include <cstddef>

// Interface command codes
// -----------------------

const uint32_t STATE_READOUT = 0x80202020;
const uint32_t GET_STATE_READOUTS = 0x80202021;
const uint32_t SET_STATE_OFF = 0x80202030;
const uint32_t SET_STATE_DRIVE = 0x80202031;
const uint32_t SET_STATE_TEST_ALL_PERMUTATIONS = 0x80202032;
const uint32_t SET_STATE_TEST_SINGLE_PHASE_POSITIVE = 0x80202033;
const uint32_t SET_STATE_TEST_DOUBLE_PHASE_POSITIVE = 0x80202034;
const uint32_t SET_STATE_TEST_ALL_SHORTED = 0x80202035;
const uint32_t SET_STATE_TEST_LONG_GROUNDED_SHORT = 0x80202036;
const uint32_t SET_STATE_TEST_LONG_POSITIVE_SHORT = 0x80202037;

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


extern uint32_t adc_update_number;
extern uint32_t tim1_update_number;
extern uint32_t tim2_update_number;
extern uint32_t tim2_cc1_number;

// Electrical state
// ----------------

struct UpdateReadout{
    uint32_t readout_number;
    uint32_t pwm_commands;
    uint16_t u_readout;
    uint16_t v_readout;
    uint16_t w_readout;
    uint16_t ref_readout;
};

extern UpdateReadout state_readout;
const size_t HISTORY_SIZE = 384;
extern UpdateReadout state_readouts[HISTORY_SIZE];
extern size_t state_readouts_index;
extern uint32_t state_updates_to_send;

// Hall sensors
// ------------

extern bool hall_1, hall_2, hall_3;

// Use 8 bit electric angle (1.4 degrees per LSB) to track motor position; we'll use this
// value as a lookup into the sine table to generate the PWM duty cycles.
extern uint8_t motor_electric_angle;
// Same as the angle, but split into the 6 commutation steps.
extern uint8_t motor_electric_phase;
extern bool hall_sensor_valid;
extern uint8_t hall_state;


// Motor Currents
// --------------

// Computed motor phase currents.
extern float current_u, current_v, current_w;

// Voltage reference for the ADC; it's a filtered 3.3V that power the board.
const float adc_voltage_reference = 3.3;
// Shunt resistance for the motor phase current sensing are 10mOhm, 500mW resistors.
const float motor_shunt_resistance = 0.010;
// The voltage on the shunt resistor is amplified by INA4181 Bidirectional, Low and 
// High Side Voltage Output, Current-Sense Amplifier.
const float amplifier_gain = 20.0;
// The ADC has a 12-bit resolution.
const uint16_t adc_max_value = 0xFFF;
// The formula that determines the current from the ADC readout: 
//   Vout = (Iload * Rsense * GAIN) + Vref
//   Vout = adc_current_readout / adc_max_value * adc_voltage_reference.
// The minus sign is because of the way the INA4181 is wired up...
const float current_conversion = -adc_voltage_reference / (adc_max_value * motor_shunt_resistance * amplifier_gain);


// Motor control
// -------------

enum struct DriverState {
    OFF,
    DRIVE,
    HOLD_U_POSITIVE,
    HOLD_V_POSITIVE,
    HOLD_W_POSITIVE,
    HOLD_U_NEGATIVE,
    HOLD_V_NEGATIVE,
    HOLD_W_NEGATIVE,
    TEST_ALL_PERMUTATIONS,
    TEST_SINGLE_PHASE_POSITIVE,
    TEST_DOUBLE_PHASE_POSITIVE,
    TEST_ALL_SHORTED,
    TEST_LONG_GROUNDED_SHORT,
    TEST_LONG_POSITIVE_SHORT,
    TEST_U_DIRECTIONS,
    TEST_U_INCREASING,
    TEST_U_DECREASING,
    TEST_V_INCREASING,
    TEST_V_DECREASING,
    TEST_W_INCREASING,
    TEST_W_DECREASING
};

const uint16_t PWM_AUTORELOAD = 1535;
const uint16_t PWM_BASE = PWM_AUTORELOAD + 1;

// Maximum duty cycle for the high side mosfet needs to allow some off time for 
// the bootstrap capacitor to charge so it has enough voltage to turn mosfet on.
const uint16_t MIN_BOOTSTRAP_DUTY = 4; // 4/(72MHz) = 55.5ns
const uint16_t PWM_MAX = PWM_BASE - MIN_BOOTSTRAP_DUTY; // 1024/72MHz = 14.2us
// Sentinel value to indicate that the phase output should be floating.
const uint16_t PWM_FLOAT = PWM_BASE - 1;

const uint16_t PWM_HOLD = PWM_BASE / 5;

// Motor control state
extern DriverState driver_state;
extern uint16_t motor_u_pwm_duty;
extern uint16_t motor_v_pwm_duty;
extern uint16_t motor_w_pwm_duty;
extern bool motor_register_update_needed;



// Functions
// ---------

// Read the hall sensors and update the motor rotation angle.
void read_motor_hall_sensors();

// Compute motor phase currents using latest ADC readouts.
void calculate_motor_phase_currents_gated();

