#include "motor_control.hpp"

DriverState driver_state = DriverState::OFF;
uint16_t hold_u_pwm_duty = 0;
uint16_t hold_v_pwm_duty = 0;
uint16_t hold_w_pwm_duty = 0;

uint16_t pwm_command = 0;
uint16_t duration_till_timeout = 0;

uint8_t leading_angle = 0;


PWMSchedule const* active_schedule = nullptr;



void motor_break(){
    driver_state = DriverState::OFF;
}

void motor_freewheel(){
    driver_state = DriverState::FREEWHEEL;
}


void motor_hold(uint16_t u, uint16_t v, uint16_t w, uint16_t timeout){
    hold_u_pwm_duty = u > PWM_MAX_HOLD ? PWM_MAX_HOLD : u;
    hold_v_pwm_duty = v > PWM_MAX_HOLD ? PWM_MAX_HOLD : v;
    hold_w_pwm_duty = w > PWM_MAX_HOLD ? PWM_MAX_HOLD : w;
    duration_till_timeout = timeout > MAX_TIMEOUT ? MAX_TIMEOUT : timeout;

    driver_state = DriverState::HOLD;
}




void motor_drive_neg(uint16_t pwm, uint16_t timeout){
    pwm_command = pwm > PWM_MAX ? PWM_MAX : pwm;
    duration_till_timeout = timeout > MAX_TIMEOUT ? MAX_TIMEOUT : timeout;
    driver_state = DriverState::DRIVE_NEG;
}

void motor_drive_pos(uint16_t pwm, uint16_t timeout){
    pwm_command = pwm > PWM_MAX ? PWM_MAX : pwm;
    duration_till_timeout = timeout > MAX_TIMEOUT ? MAX_TIMEOUT : timeout;
    driver_state = DriverState::DRIVE_POS;
}



void motor_drive_smooth_pos(uint16_t pwm, uint16_t timeout){
    pwm_command = pwm > PWM_MAX ? PWM_MAX : pwm;
    // Fix timeout to experiment with drive angle.
    duration_till_timeout = MAX_TIMEOUT / 2;
    leading_angle = 256 * static_cast<int>(timeout) / MAX_TIMEOUT;
    // duration_till_timeout = timeout > MAX_TIMEOUT ? MAX_TIMEOUT : timeout;
    driver_state = DriverState::DRIVE_SMOOTH_POS;
}

void motor_drive_smooth_neg(uint16_t pwm, uint16_t timeout){
    pwm_command = pwm > PWM_MAX ? PWM_MAX : pwm;
    duration_till_timeout = MAX_TIMEOUT / 2;
    leading_angle = 256 * static_cast<int>(timeout) / MAX_TIMEOUT;
    // duration_till_timeout = timeout > MAX_TIMEOUT ? MAX_TIMEOUT : timeout;
    driver_state = DriverState::DRIVE_SMOOTH_NEG;
}


void motor_start_test(PWMSchedule const& schedule){
    // Don't start a new test if we're already running one.
    if (active_schedule != nullptr) return;

    active_schedule = &schedule;

    driver_state = DriverState::TEST_SCHEDULE;
}




