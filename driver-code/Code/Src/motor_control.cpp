#include "motor_control.hpp"

#include "interrupts_motor.hpp"

#include "error_handler.hpp"



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




