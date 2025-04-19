#include "motor_control.hpp"

#include "error_handler.hpp"

DriverState driver_state = DriverState::OFF;

uint16_t hold_u_pwm_duty = 0;
uint16_t hold_v_pwm_duty = 0;
uint16_t hold_w_pwm_duty = 0;

uint16_t pwm_command = 0;
uint16_t duration_till_timeout = 0;

uint8_t leading_angle = 0;

PWMSchedule const* schedule_queued = nullptr;


void motor_break(){
    driver_state = DriverState::OFF;
}

void motor_freewheel(){
    driver_state = DriverState::FREEWHEEL;
}


void motor_hold(uint16_t u, uint16_t v, uint16_t w, uint16_t timeout){
    hold_u_pwm_duty = clip_to(0, PWM_MAX_HOLD, u);
    hold_v_pwm_duty = clip_to(0, PWM_MAX_HOLD, v);
    hold_w_pwm_duty = clip_to(0, PWM_MAX_HOLD, w);
    duration_till_timeout = clip_to(0, MAX_TIMEOUT, timeout);

    driver_state = DriverState::HOLD;
}




void motor_drive_neg(uint16_t pwm, uint16_t timeout){
    pwm_command = clip_to(0, PWM_MAX, pwm);
    duration_till_timeout = clip_to(0, MAX_TIMEOUT, timeout);
    driver_state = DriverState::DRIVE_NEG;
}

void motor_drive_pos(uint16_t pwm, uint16_t timeout){
    pwm_command = clip_to(0, PWM_MAX, pwm);
    duration_till_timeout = clip_to(0, MAX_TIMEOUT, timeout);
    driver_state = DriverState::DRIVE_POS;
}



void motor_drive_smooth_pos(uint16_t pwm, uint16_t timeout, uint16_t new_leading_angle){
    pwm_command = clip_to(0, PWM_MAX, pwm);
    leading_angle = clip_to(0, 255, new_leading_angle);
    duration_till_timeout = clip_to(0, MAX_TIMEOUT, timeout);
    driver_state = DriverState::DRIVE_SMOOTH_POS;
}

void motor_drive_smooth_neg(uint16_t pwm, uint16_t timeout, uint16_t new_leading_angle){
    pwm_command = clip_to(0, PWM_MAX, pwm);
    leading_angle = clip_to(0, 255, new_leading_angle);
    duration_till_timeout = clip_to(0, MAX_TIMEOUT, timeout);
    driver_state = DriverState::DRIVE_SMOOTH_NEG;
}


void motor_start_schedule(PWMSchedule const& schedule){
    schedule_queued = &schedule;

    driver_state = DriverState::SCHEDULE;
}




