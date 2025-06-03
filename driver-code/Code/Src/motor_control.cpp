#include "motor_control.hpp"

#include "interrupts_motor.hpp"

#include "error_handler.hpp"

bool is_motor_stopped(){
    // Check if the motor is stopped.
    return (driver_state == DriverState::OFF) || (driver_state == DriverState::FREEWHEEL);
}

void motor_break(){
    // Short all motor phases to ground.
    set_motor_u_pwm_duty(0);
    set_motor_v_pwm_duty(0);
    set_motor_w_pwm_duty(0);

    // Immediately update and enable the motor outputs.
    enable_motor_outputs();

    driver_state = DriverState::OFF;
}

void motor_freewheel(){
    // Reset motor phases to 0.
    set_motor_u_pwm_duty(0);
    set_motor_v_pwm_duty(0);
    set_motor_w_pwm_duty(0);

    // Disable the motor outputs; leaving them floating voltage/tristate.
    disable_motor_outputs();

    driver_state = DriverState::FREEWHEEL;
}

void motor_hold(uint16_t u, uint16_t v, uint16_t w, uint16_t timeout){
    hold_u_pwm_duty = clip_to(0, pwm_max_hold, u);
    hold_v_pwm_duty = clip_to(0, pwm_max_hold, v);
    hold_w_pwm_duty = clip_to(0, pwm_max_hold, w);
    duration_till_timeout = clip_to(0, max_timeout, timeout);

    driver_state = DriverState::HOLD;
}


void motor_drive_neg(uint16_t pwm, uint16_t timeout){
    pwm_command = clip_to(0, pwm_max, pwm);
    duration_till_timeout = clip_to(0, max_timeout, timeout);
    driver_state = DriverState::DRIVE_NEG;
}

void motor_drive_pos(uint16_t pwm, uint16_t timeout){
    pwm_command = clip_to(0, pwm_max, pwm);
    duration_till_timeout = clip_to(0, max_timeout, timeout);
    driver_state = DriverState::DRIVE_POS;
}



void motor_drive_smooth_pos(uint16_t pwm, uint16_t timeout, uint16_t new_leading_angle){
    pwm_command = clip_to(0, pwm_max, pwm);
    leading_angle = clip_to(0, angle_units_per_circle-1, new_leading_angle);
    duration_till_timeout = clip_to(0, max_timeout, timeout);
    driver_state = DriverState::DRIVE_SMOOTH_POS;
}

void motor_drive_smooth_neg(uint16_t pwm, uint16_t timeout, uint16_t new_leading_angle){
    pwm_command = clip_to(0, pwm_max, pwm);
    leading_angle = clip_to(0, angle_units_per_circle-1, new_leading_angle);
    duration_till_timeout = clip_to(0, max_timeout, timeout);
    driver_state = DriverState::DRIVE_SMOOTH_NEG;
}


void motor_start_schedule(PWMSchedule const& schedule){
    schedule_queued = &schedule;

    driver_state = DriverState::SCHEDULE;
}




