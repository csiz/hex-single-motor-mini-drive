#include "motor_control.hpp"
#include "io.hpp"
#include "data.hpp"

#include <stm32f1xx_ll_adc.h>
#include <stm32f1xx_ll_tim.h>


#include <cmath>

size_t schedule_counter = 0;
size_t schedule_stage = 0;
const PWMSchedule * active_schedule = nullptr;

// Motor control state
DriverState driver_state = DriverState::OFF;
uint16_t motor_u_pwm_duty = 0;
uint16_t motor_v_pwm_duty = 0;
uint16_t motor_w_pwm_duty = 0;
bool motor_register_update_needed = true;
uint16_t duration_till_timeout = 0;

const float TRI = -1.0;

// Motor voltage fraction for the 6-step commutation.
const float motor_3phase_voltage_table[6][3] = {
    {1.0, 0.0, 0.0},
    {1.0, 1.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 1.0, 1.0},
    {0.0, 0.0, 1.0},
    {1.0, 0.0, 1.0}
};

// Motor voltage fraction for the 6-step commutation.
const float motor_2phase_voltage_table[6][3] = {
    {TRI, 1.0, 0.0},
    {0.0, 1.0, TRI},
    {0.0, TRI, 1.0},
    {TRI, 0.0, 1.0},
    {1.0, 0.0, TRI},
    {1.0, TRI, 0.0}
};

float pwm_fraction = 0.0;

uint32_t get_combined_motor_pwm_duty(){
    return motor_u_pwm_duty * PWM_BASE * PWM_BASE + motor_v_pwm_duty * PWM_BASE + motor_w_pwm_duty;
}

void write_motor_registers(){
    if(motor_u_pwm_duty == PWM_FLOAT){
        disable_motor_u_output();
    } else {
        enable_motor_u_output();
        set_motor_u_pwm_duty_cycle(motor_u_pwm_duty);
    }

    if(motor_v_pwm_duty == PWM_FLOAT){
        disable_motor_v_output();
    } else {
        enable_motor_v_output();
        set_motor_v_pwm_duty_cycle(motor_v_pwm_duty);
    }

    if(motor_w_pwm_duty == PWM_FLOAT){
        disable_motor_w_output();
    } else {
        enable_motor_w_output();
        set_motor_w_pwm_duty_cycle(motor_w_pwm_duty);
    }
}


void disable_motor_ouputs(){
    driver_state = DriverState::OFF;

    disable_motor_u_output();
    disable_motor_v_output();
    disable_motor_w_output();

    motor_u_pwm_duty = PWM_FLOAT;
    motor_v_pwm_duty = PWM_FLOAT;
    motor_w_pwm_duty = PWM_FLOAT;

    write_motor_registers();
}


bool check_overcurrent(){
    const float overcurrent_threshold = 5.0;
    return abs(current_u) > overcurrent_threshold || 
           abs(current_v) > overcurrent_threshold || 
           abs(current_w) > overcurrent_threshold;
}

void set_motor_pwm_gated(uint16_t u, uint16_t v, uint16_t w){
    // Disable the Tim1 update interrupt to prevent the motor from getting an intermediate state.
    NVIC_DisableIRQ(TIM1_UP_IRQn);
    motor_u_pwm_duty = u;
    motor_v_pwm_duty = v;
    motor_w_pwm_duty = w;
    NVIC_EnableIRQ(TIM1_UP_IRQn);
}


void hold_motor(uint16_t u, uint16_t v, uint16_t w, uint16_t timeout){
    driver_state = DriverState::HOLD;
    set_motor_pwm_gated(u > PWM_MAX_HOLD ? PWM_MAX_HOLD : u, 
                        v > PWM_MAX_HOLD ? PWM_MAX_HOLD : v, 
                        w > PWM_MAX_HOLD ? PWM_MAX_HOLD : w);
    duration_till_timeout = timeout > MAX_TIMEOUT ? MAX_TIMEOUT : timeout;
    motor_register_update_needed = true;
}

void update_motor_control(){
    // Update motor control registers only if actively driving.

    const float (*motor_voltage_table)[3] = nullptr;
    if (driver_state == DriverState::DRIVE_2PHASE) motor_voltage_table = motor_2phase_voltage_table;
    else if(driver_state == DriverState::DRIVE_3PHASE) motor_voltage_table = motor_3phase_voltage_table;
    else return;

    // Note: The registers need to be left unchanged whilst running in the calibration modes.

    // Use the hall sensor state to determine the motor position and commutation.
    if (not hall_sensor_valid) {
        disable_motor_ouputs();
        return;
    }

    const float voltage_phase_u = motor_voltage_table[motor_electric_phase][0];
    const float voltage_phase_v = motor_voltage_table[motor_electric_phase][1];
    const float voltage_phase_w = motor_voltage_table[motor_electric_phase][2];

    if (voltage_phase_u == TRI) {
        motor_u_pwm_duty = PWM_FLOAT;
    } else {
        motor_u_pwm_duty = voltage_phase_u * pwm_fraction * PWM_BASE;
    }

    if (voltage_phase_v == TRI) {
        motor_v_pwm_duty = PWM_FLOAT;
    } else {
        motor_v_pwm_duty = voltage_phase_v * pwm_fraction * PWM_BASE;
    }

    if (voltage_phase_w == TRI) {
        motor_w_pwm_duty = PWM_FLOAT;
    } else {
        motor_w_pwm_duty = voltage_phase_w * pwm_fraction * PWM_BASE;
    }

    motor_register_update_needed = true;
}

void drive_motor_2phase(uint16_t pwm, uint16_t timeout){
    driver_state = DriverState::DRIVE_2PHASE;
    pwm_fraction = (pwm > PWM_MAX ? PWM_MAX : pwm) / static_cast<float>(PWM_BASE);
    duration_till_timeout = timeout > MAX_TIMEOUT ? MAX_TIMEOUT : timeout;
    update_motor_control();
}

void drive_motor_3phase(uint16_t pwm, uint16_t timeout){
    driver_state = DriverState::DRIVE_3PHASE;
    pwm_fraction = (pwm > PWM_MAX ? PWM_MAX : pwm) / static_cast<float>(PWM_BASE);
    duration_till_timeout = timeout > MAX_TIMEOUT ? MAX_TIMEOUT : timeout;
    update_motor_control();
}


void start_test(const PWMSchedule & schedule){
    schedule_counter = 0;
    schedule_stage = 0;
    state_updates_to_send = 0;

    active_schedule = &schedule;

    driver_state = DriverState::TEST_SCHEDULE;
    motor_register_update_needed = true;
}

void test_procedure(){ 
    if(active_schedule == nullptr) return;

    const PWMSchedule & schedule = *active_schedule;


    motor_u_pwm_duty = schedule[schedule_stage].u;
    motor_v_pwm_duty = schedule[schedule_stage].v;
    motor_w_pwm_duty = schedule[schedule_stage].w;

    
    schedule_counter += 1;
    if (schedule_counter >= schedule[schedule_stage].duration) {
        schedule_stage += 1;
        schedule_counter = 0;
    }
    
    if (schedule_stage >= SCHEDULE_SIZE) {
        disable_motor_ouputs();
        state_updates_to_send = HISTORY_SIZE;
    }
    
    write_motor_registers();

    motor_register_update_needed = true;
}

void update_motor_control_registers(){
    if (duration_till_timeout > 0) {
        duration_till_timeout -= 1;
    } else {
        switch (driver_state) {
            case DriverState::HOLD:
            case DriverState::DRIVE_2PHASE:
            case DriverState::DRIVE_3PHASE:
                disable_motor_ouputs();       
                return;
            case DriverState::OFF:
            case DriverState::TEST_SCHEDULE:
                break; // continue despite timeout
        }
    }
    if (not motor_register_update_needed) return;

    motor_register_update_needed = false;
    switch (driver_state) {
        case DriverState::OFF:
            disable_motor_ouputs();
            break;
        case DriverState::TEST_SCHEDULE:
            test_procedure();
            break;
        case DriverState::DRIVE_2PHASE:
        case DriverState::DRIVE_3PHASE:
            write_motor_registers();
            break;
        case DriverState::HOLD:
            write_motor_registers();
            break;
    }

}
