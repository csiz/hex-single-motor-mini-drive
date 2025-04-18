#include "motor_control.hpp"

#include "interface.hpp"
#include "interrupts.hpp"

DriverState driver_state = DriverState::OFF;
uint16_t hold_u_pwm_duty = 0;
uint16_t hold_v_pwm_duty = 0;
uint16_t hold_w_pwm_duty = 0;

uint16_t pwm_command = 0;
uint16_t duration_till_timeout = 0;

uint8_t leading_angle = 0;


PWMSchedule const* test_schedule_pointer = nullptr;
size_t test_schedule_counter = 0;
size_t test_schedule_stage = 0;


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


void motor_start_test(PWMSchedule const& schedule){
    // Don't start a new test if we're already running one.
    if (driver_state == DriverState::TEST_SCHEDULE) return;

    // Stop adding readouts to the queue until the test starts in the interrupt handler.
    readouts_to_send = 0;
    // Stop emptying the readouts queue; we want to keep the test data.
    readouts_allow_sending = false;
    
    // Clear the readouts buffer of old data.
    xQueueReset(readouts_queue);

    test_schedule_counter = 0;
    test_schedule_stage = 0;
    test_schedule_pointer = &schedule;

    driver_state = DriverState::TEST_SCHEDULE;
}




