#pragma once

#include <FreeRTOS.h>
#include <queue.h>

const uint32_t ADC_READOUT = 0x80202020;
const uint32_t GET_ADC_READOUTS = 0x80202021;


void data_init();

extern uint32_t adc_update_number;
extern uint32_t tim1_update_number;
extern uint32_t tim2_update_number;
extern uint32_t tim2_cc1_number;

struct UpdateReadout{
    uint32_t readout_number;
    uint32_t pwm_commands;
    uint16_t u_readout;
    uint16_t v_readout;
    uint16_t w_readout;
    uint16_t ref_readout;
};
#define HISTORY_SIZE 384
#define ADC_QUEUE_SIZE 32
#define ADC_ITEMSIZE sizeof(struct UpdateReadout)



extern QueueHandle_t adc_queue;
extern StaticQueue_t adc_queue_storage;
extern uint8_t adc_queue_buffer[ADC_QUEUE_SIZE * ADC_ITEMSIZE];

extern struct UpdateReadout adc_readouts[HISTORY_SIZE];
extern uint32_t adc_readouts_index;
extern uint32_t adc_processed_number;

size_t move_adc_readouts(bool overwrite);



extern bool hall_1, hall_2, hall_3;

// Use 8 bit electric angle (1.4 degrees per LSB) to track motor position; we'll use this
// value as a lookup into the sine table to generate the PWM duty cycles.
extern uint8_t motor_electric_angle;
// Same as the angle, but split into the 6 commutation steps.
extern uint8_t motor_electric_phase;
extern bool hall_sensor_valid;
extern uint8_t hall_state;

void read_motor_hall_sensors();


// Computed motor phase currents.
extern float current_u, current_v, current_w;


// Motor Currents
// --------------

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

// Compute motor phase currents using latest ADC readouts.
void update_motor_phase_currents();


// Motor control
// -------------

enum struct DriverState {
    OFF,
    DRIVE,
    MEASURE_CURRENT,
};

extern DriverState driver_state;

extern volatile bool motor_register_update_needed;

const uint32_t PWM_AUTORELOAD = 1535;

// Maximum duty cycle for the high side mosfet needs to allow some off time for 
// the bootstrap capacitor to charge so it has enough voltage to turn mosfet on.
const uint16_t min_bootstrap_duty = 4; // 4/(72MHz) = 55.5ns
const uint16_t max_pwm_duty = PWM_AUTORELOAD + 1 - min_bootstrap_duty; // 1024/72MHz = 14.2us
const uint16_t floating_pwm_duty = 0b1000'0000'0000'0000;

extern DriverState driver_state;

// Motor control state
extern uint16_t motor_u_pwm_duty;
extern uint16_t motor_v_pwm_duty;
extern uint16_t motor_w_pwm_duty;

extern volatile bool motor_register_update_needed;




// Motor voltage fraction for the 6-step commutation.
const float motor_voltage_table[6][3] = {
    {0.5, 1.0, 0.0},
    {0.0, 1.0, 0.5},
    {0.0, 0.5, 1.0},
    {0.5, 0.0, 1.0},
    {1.0, 0.0, 0.5},
    {1.0, 0.5, 0.0}
};

const uint16_t measure_current_table[12][3] = {
    {0,            0,            0},
    {max_pwm_duty, 0,            0}, // Positive U
    {0,            0,            0},
    {0,            max_pwm_duty, 0}, // Positive V
    {0,            0,            0},
    {0,            0,            max_pwm_duty}, // Positive W
    {0,            0,            0},
    {0,            max_pwm_duty, max_pwm_duty}, // Negative U
    {0,            0,            0},
    {max_pwm_duty, 0,            max_pwm_duty}, // Negative V
    {0,            0,            0},
    {max_pwm_duty, max_pwm_duty, 0} // Negative W
};

const size_t measure_current_steps = HISTORY_SIZE / 12;

extern size_t measure_current_stage;
extern size_t measure_current_counter;


