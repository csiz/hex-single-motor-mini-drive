#include <stm32f1xx_ll_adc.h>
#include <stm32f1xx_ll_tim.h>

#include <cmath>

#include "motor_control.hpp"
#include "io.hpp"
#include "interrupts.hpp"

// Maximum duty cycle for the high side mosfet needs to allow some off time for 
// the bootstrap capacitor to charge so it has enough voltage to turn mosfet on.
const uint16_t min_bootstrap_duty = 4; // 4/(72MHz) = 55.5ns
const uint16_t max_pwm_duty = PWM_AUTORELOAD + 1 - min_bootstrap_duty; // 1024/72MHz = 14.2us
const uint16_t floating_pwm_duty = 0b1000'0000'0000'0000;
DriverState driver_state = DriverState::DRIVE;

// Motor control state
uint16_t motor_u_pwm_duty = 0;
uint16_t motor_v_pwm_duty = 0;
uint16_t motor_w_pwm_duty = 0;

volatile bool motor_register_update_needed = false;


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
size_t measure_current_stage = 0;
size_t measure_current_counter = 0;


void disable_motor_ouputs(){
    disable_motor_u_output();
    disable_motor_v_output();
    disable_motor_w_output();
}

bool check_overcurrent(){
    const float overcurrent_threshold = 5.0;
    return abs(current_u) > overcurrent_threshold || 
           abs(current_v) > overcurrent_threshold || 
           abs(current_w) > overcurrent_threshold;
}

void update_motor_control(){
    const float pwm_fraction = 0.2;
    // Use the hall sensor state to determine the motor position and commutation.
    if (hall_sensor_valid) {
        const float voltage_phase_u = motor_voltage_table[motor_electric_phase][0];
        const float voltage_phase_v = motor_voltage_table[motor_electric_phase][1];
        const float voltage_phase_w = motor_voltage_table[motor_electric_phase][2];

        if (voltage_phase_u == 0.5) {
            motor_u_pwm_duty = floating_pwm_duty;
        } else {
            motor_u_pwm_duty = voltage_phase_u * pwm_fraction * max_pwm_duty;
        }

        if (voltage_phase_v == 0.5) {
            motor_v_pwm_duty = floating_pwm_duty;
        } else {
            motor_v_pwm_duty = voltage_phase_v * pwm_fraction * max_pwm_duty;
        }

        if (voltage_phase_w == 0.5) {
            motor_w_pwm_duty = floating_pwm_duty;
        } else {
            motor_w_pwm_duty = voltage_phase_w * pwm_fraction * max_pwm_duty;
        }
    } else {
        motor_u_pwm_duty = floating_pwm_duty;
        motor_v_pwm_duty = floating_pwm_duty;
        motor_w_pwm_duty = floating_pwm_duty;
        disable_motor_ouputs();
    }

    motor_register_update_needed = true;
}

void drive_motor(){
    if(motor_u_pwm_duty == floating_pwm_duty){
        disable_motor_u_output();
    } else {
        enable_motor_u_output();
        set_motor_u_pwm_duty_cycle(motor_u_pwm_duty);
    }

    if(motor_v_pwm_duty == floating_pwm_duty){
        disable_motor_v_output();
    } else {
        enable_motor_v_output();
        set_motor_v_pwm_duty_cycle(motor_v_pwm_duty);
    }

    if(motor_w_pwm_duty == floating_pwm_duty){
        disable_motor_w_output();
    } else {
        enable_motor_w_output();
        set_motor_w_pwm_duty_cycle(motor_w_pwm_duty);
    }
}

void measure_current(){
    const uint16_t u_duty = measure_current_table[measure_current_stage][0];
    const uint16_t v_duty = measure_current_table[measure_current_stage][1];
    const uint16_t w_duty = measure_current_table[measure_current_stage][2];

    set_motor_u_pwm_duty_cycle(u_duty);
    set_motor_v_pwm_duty_cycle(v_duty);
    set_motor_w_pwm_duty_cycle(w_duty);

    measure_current_counter += 1;
    if (measure_current_counter >= measure_current_steps) {
        measure_current_stage += 1;
        measure_current_counter = 0;
    }

    if (measure_current_stage >= 12) {
        driver_state = DriverState::OFF;
    }
}

void update_motor_control_registers(){
    motor_register_update_needed = false;
    switch (driver_state) {
        case DriverState::OFF:
            disable_motor_ouputs();
            break;
        case DriverState::DRIVE:
            drive_motor();
            break;
        case DriverState::MEASURE_CURRENT:
            measure_current();
            break;
    }

}
