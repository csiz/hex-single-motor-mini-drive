#pragma once

#include "FreeRTOS.h"
#include "queue.h"

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

extern UpdateReadout latest_readout;
const size_t HISTORY_SIZE = 420;
const size_t READOUT_ITEMSIZE = sizeof(UpdateReadout);

extern QueueHandle_t readouts_queue;
extern StaticQueue_t readouts_queue_storage;
extern uint8_t readouts_queue_buffer[HISTORY_SIZE * READOUT_ITEMSIZE];
extern uint32_t readouts_missed;

extern bool readouts_allow_missing;
extern bool readouts_allow_sending;

extern int32_t readouts_to_send;

// Hall sensors
// ------------

extern bool hall_1, hall_2, hall_3;

// Use 8 bit electric angle (1.4 degrees per LSB) to track motor position; we'll use this
// value as a lookup into the sine table to generate the PWM duty cycles.
extern uint8_t motor_electric_angle;
// Same as the angle, but split into the 6 commutation steps.
extern uint8_t hall_sector;
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



// Functions
// ---------

void data_init();

// Read the hall sensors and update the motor rotation angle.
void read_hall_sensors();

// Compute motor phase currents using latest ADC readouts.
void calculate_motor_phase_currents_gated();

