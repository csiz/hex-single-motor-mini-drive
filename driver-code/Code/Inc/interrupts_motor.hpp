#pragma once

#include "motor_control.hpp"

#include "constants.hpp"
#include "error_handler.hpp"
#include "io.hpp"

// Motor Control
// -------------


static inline bool update_and_check_timeout(){
    if (duration_till_timeout > 0) {
        duration_till_timeout -= 1;
        return false;
    } else {
        return true;
    }
}

static inline void control_motor_break(){
    // Short all motor phases to ground.
    set_motor_u_pwm_duty(0);
    set_motor_v_pwm_duty(0);
    set_motor_w_pwm_duty(0);

    // Immediately update and enable the motor outputs.
    enable_motor_outputs();
}

static inline void control_motor_freewheel(){
    // Reset motor phases to 0.
    set_motor_u_pwm_duty(0);
    set_motor_v_pwm_duty(0);
    set_motor_w_pwm_duty(0);

    // Disable the motor outputs; leaving them floating voltage/tristate.
    disable_motor_outputs();
}

static inline void control_motor_hold(){
    if(update_and_check_timeout()) return motor_break();

    // Set the duty cycle and hold.
    set_motor_u_pwm_duty(hold_u_pwm_duty);
    set_motor_v_pwm_duty(hold_v_pwm_duty);
    set_motor_w_pwm_duty(hold_w_pwm_duty);

    enable_motor_outputs();
}

static inline void control_motor_sector(const uint8_t hall_sector, const uint16_t (* motor_sector_driving_table)[3]){
    if (motor_sector_driving_table == nullptr) return error();
    
    if(update_and_check_timeout()) return motor_break();

    // Use the hall sensor state to determine the motor position and commutation.
    if (hall_sector >= hall_sector_base) return motor_break();


    // Get the voltage for the three phases from the table.

    const uint16_t voltage_phase_u = motor_sector_driving_table[hall_sector][0];
    const uint16_t voltage_phase_v = motor_sector_driving_table[hall_sector][1];
    const uint16_t voltage_phase_w = motor_sector_driving_table[hall_sector][2];


    set_motor_u_pwm_duty(voltage_phase_u * pwm_command / PWM_BASE);
    set_motor_v_pwm_duty(voltage_phase_v * pwm_command / PWM_BASE);
    set_motor_w_pwm_duty(voltage_phase_w * pwm_command / PWM_BASE);

    enable_motor_outputs();
}

static inline void control_motor_smooth(const bool angle_valid, const uint8_t angle, const int direction){
    if(update_and_check_timeout()) return motor_break();
    
    if (not angle_valid) return motor_break();

    const int target_angle = (256 + static_cast<int>(angle) + direction * static_cast<int>(leading_angle)) % 256;

    const uint16_t voltage_phase_u = phases_waveform[target_angle];
    const uint16_t voltage_phase_v = phases_waveform[(target_angle + 256 - 85) % 256];
    const uint16_t voltage_phase_w = phases_waveform[(target_angle + 256 - 170) % 256];

    set_motor_u_pwm_duty(voltage_phase_u * pwm_command / PWM_BASE);
    set_motor_v_pwm_duty(voltage_phase_v * pwm_command / PWM_BASE);
    set_motor_w_pwm_duty(voltage_phase_w * pwm_command / PWM_BASE);

    enable_motor_outputs();
}

// Quickly update the PWM settings from the test schedule.
static inline void control_motor_test(){
    if(schedule_pointer == nullptr) return error();

    // Check if we reached the end of the schedule.
    if (schedule_stage >= SCHEDULE_SIZE) {
        // Reset the schedule stage and counter.
        schedule_stage = 0;
        schedule_counter = 0;
        schedule_pointer = nullptr;
        // Stop the test by breaking the motor.
        motor_break();
    }

    const PWMSchedule & schedule = *schedule_pointer;

    set_motor_u_pwm_duty(schedule[schedule_stage].u);
    set_motor_v_pwm_duty(schedule[schedule_stage].v);
    set_motor_w_pwm_duty(schedule[schedule_stage].w);

    enable_motor_outputs();

    // Go to the next step in the schedule.
    schedule_counter += 1;
    if (schedule_counter >= schedule[schedule_stage].duration) {
        schedule_stage += 1;
        schedule_counter = 0;
    }
}