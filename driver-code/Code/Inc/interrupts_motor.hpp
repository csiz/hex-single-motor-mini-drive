#pragma once

#include "interrupts_data.hpp"

#include "motor_control.hpp"

#include "constants.hpp"
#include "type_definitions.hpp"
#include "error_handler.hpp"
#include "io.hpp"


// Driver State
// ------------

enum struct DriverState {
    OFF,
    FREEWHEEL,
    DRIVE_POS,
    DRIVE_NEG,
    DRIVE_SMOOTH_POS,
    DRIVE_SMOOTH_NEG,
    HOLD,
    SCHEDULE,
};

// Motor driver state.
extern DriverState driver_state;

// Duration until current command expires.
extern uint16_t duration_till_timeout;

extern uint16_t hold_u_pwm_duty;
extern uint16_t hold_v_pwm_duty;
extern uint16_t hold_w_pwm_duty;

extern uint16_t pwm_command;

extern int16_t leading_angle;

extern PWMSchedule const* schedule_queued;

// Active test schedule.
extern PWMSchedule const* schedule_active;
extern size_t schedule_counter;
extern size_t schedule_stage;


// Current calibration factors.
extern CurrentCalibration current_calibration;


// Count down until the timeout expires; return true if the timeout expired.
static inline bool update_and_check_timeout(){
    if (duration_till_timeout > 0) {
        duration_till_timeout -= 1;
        return false;
    } else {
        return true;
    }
}

static inline PWMSchedule const* get_and_reset_schedule_queued(){
    PWMSchedule const* schedule = schedule_queued;
    schedule_queued = nullptr;
    return schedule;
}

// Motor Control
// -------------



static inline void update_motor_hold(){
    if(update_and_check_timeout()) return motor_break();

    // Set the duty cycle and hold.
    set_motor_u_pwm_duty(hold_u_pwm_duty);
    set_motor_v_pwm_duty(hold_v_pwm_duty);
    set_motor_w_pwm_duty(hold_w_pwm_duty);

    enable_motor_outputs();
}

static inline void update_motor_sector(const uint8_t hall_sector, const uint16_t (* motor_sector_driving_table)[3]){
    if (motor_sector_driving_table == nullptr) return error();
    
    if(update_and_check_timeout()) return motor_break();

    // Use the hall sensor state to determine the motor position and commutation.
    if (hall_sector >= hall_sector_base) return motor_break();


    // Get the voltage for the three phases from the table.

    const uint16_t voltage_phase_u = motor_sector_driving_table[hall_sector][0];
    const uint16_t voltage_phase_v = motor_sector_driving_table[hall_sector][1];
    const uint16_t voltage_phase_w = motor_sector_driving_table[hall_sector][2];


    set_motor_u_pwm_duty(voltage_phase_u * pwm_command / pwm_base);
    set_motor_v_pwm_duty(voltage_phase_v * pwm_command / pwm_base);
    set_motor_w_pwm_duty(voltage_phase_w * pwm_command / pwm_base);

    enable_motor_outputs();
}

static inline void update_motor_smooth(const bool angle_valid, const int angle, const int direction){
    if(update_and_check_timeout()) return motor_break();
    
    if (not angle_valid) return motor_break();

    const int target_angle = normalize_angle(angle + direction * leading_angle);

    const uint16_t voltage_phase_u = get_phase_pwm(target_angle);
    const uint16_t voltage_phase_v = get_phase_pwm(target_angle - third_circle);
    const uint16_t voltage_phase_w = get_phase_pwm(target_angle - two_thirds_circle);

    set_motor_u_pwm_duty(voltage_phase_u * pwm_command / pwm_base);
    set_motor_v_pwm_duty(voltage_phase_v * pwm_command / pwm_base);
    set_motor_w_pwm_duty(voltage_phase_w * pwm_command / pwm_base);

    enable_motor_outputs();
}

// Quickly update the PWM settings from the test schedule.
static inline void update_motor_schedule(){
    // Start a new schedule if the old one is finished.
    if(schedule_active == nullptr){
        // Get the queued schedule and reset the variable for a new command.
        schedule_active = get_and_reset_schedule_queued();

        
        // We should not enter testing mode without a valid schedule.
        if (schedule_active == nullptr) return error();
        
        // Clear the readouts buffer of old data.
        readout_history_reset();
        
        // Reset the schedule counter.
        schedule_stage = 0;
        schedule_counter = 0;
    }

    // Check if we reached the end of the schedule; and stop.
    if (schedule_stage >= schedule_size) {
        schedule_active = nullptr;   
        return motor_break();
    }

    const PWMSchedule & schedule = *schedule_active;

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