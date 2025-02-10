#include "motor_control.hpp"
#include "io.hpp"
#include "data.hpp"

#include <stm32f1xx_ll_adc.h>
#include <stm32f1xx_ll_tim.h>

#include <FreeRTOS.h>
#include <queue.h>

#include <cmath>


void disable_motor_ouputs(){
    disable_motor_u_output();
    disable_motor_v_output();
    disable_motor_w_output();

    motor_u_pwm_duty = set_floating_duty;
    motor_v_pwm_duty = set_floating_duty;
    motor_w_pwm_duty = set_floating_duty;
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
            motor_u_pwm_duty = set_floating_duty;
        } else {
            motor_u_pwm_duty = voltage_phase_u * pwm_fraction * max_pwm_duty;
        }

        if (voltage_phase_v == 0.5) {
            motor_v_pwm_duty = set_floating_duty;
        } else {
            motor_v_pwm_duty = voltage_phase_v * pwm_fraction * max_pwm_duty;
        }

        if (voltage_phase_w == 0.5) {
            motor_w_pwm_duty = set_floating_duty;
        } else {
            motor_w_pwm_duty = voltage_phase_w * pwm_fraction * max_pwm_duty;
        }
    } else {
        disable_motor_ouputs();
        motor_u_pwm_duty = set_floating_duty;
        motor_v_pwm_duty = set_floating_duty;
        motor_w_pwm_duty = set_floating_duty;
        driver_state = DriverState::OFF;
    }
}

void drive_motor(){
    if(motor_u_pwm_duty == set_floating_duty){
        disable_motor_u_output();
    } else {
        enable_motor_u_output();
        set_motor_u_pwm_duty_cycle(motor_u_pwm_duty);
    }

    if(motor_v_pwm_duty == set_floating_duty){
        disable_motor_v_output();
    } else {
        enable_motor_v_output();
        set_motor_v_pwm_duty_cycle(motor_v_pwm_duty);
    }

    if(motor_w_pwm_duty == set_floating_duty){
        disable_motor_w_output();
    } else {
        enable_motor_w_output();
        set_motor_w_pwm_duty_cycle(motor_w_pwm_duty);
    }
}

void measure_current(){
    motor_u_pwm_duty = measure_current_table[measure_current_stage][0];
    motor_v_pwm_duty = measure_current_table[measure_current_stage][1];
    motor_w_pwm_duty = measure_current_table[measure_current_stage][2];


    set_motor_u_pwm_duty_cycle(motor_u_pwm_duty);
    set_motor_v_pwm_duty_cycle(motor_v_pwm_duty);
    set_motor_w_pwm_duty_cycle(motor_w_pwm_duty);



    measure_current_counter += 1;
    if (measure_current_counter >= measure_current_steps) {
        measure_current_stage += 1;
        measure_current_counter = 0;
    }

    if (measure_current_stage >= 12) {
        measure_current_stage = 0;
        measure_current_counter = 0;
        driver_state = DriverState::OFF;
        state_updates_to_send = HISTORY_SIZE;
    }

    motor_register_update_needed = true;
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
