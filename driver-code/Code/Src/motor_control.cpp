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
uint16_t duration_till_timeout = 0;

bool test_procedure_start = true;

// Motor voltage fraction for the 6-step commutation.
const float motor_voltage_table_pos[6][3] = {
    {0.0, 1.0, 0.0},
    {0.0, 1.0, 1.0},
    {0.0, 0.0, 1.0},
    {1.0, 0.0, 1.0},
    {1.0, 0.0, 0.0},
    {1.0, 1.0, 0.0},
};

// Surpirsingly good schedule for the 6-step commutation.
const float motor_voltage_table_neg[6][3] {
    {0.0, 0.0, 1.0},
    {1.0, 0.0, 1.0},
    {1.0, 0.0, 0.0},
    {1.0, 1.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 1.0, 1.0},
};

float pwm_fraction = 0.0;

uint32_t get_combined_motor_pwm_duty(){
    return motor_u_pwm_duty * PWM_BASE * PWM_BASE + motor_v_pwm_duty * PWM_BASE + motor_w_pwm_duty;
}

static inline void write_motor_registers(){
    set_motor_u_pwm_duty_cycle(motor_u_pwm_duty);
    set_motor_v_pwm_duty_cycle(motor_v_pwm_duty);
    set_motor_w_pwm_duty_cycle(motor_w_pwm_duty);
}


void motor_break(){
    driver_state = DriverState::OFF;

    motor_u_pwm_duty = 0;
    motor_v_pwm_duty = 0;
    motor_w_pwm_duty = 0;
    
    // Immediately update and enable the motor outputs.
    write_motor_registers();
    enable_motor_outputs();
}

void motor_freewheel(){
    driver_state = DriverState::FREEWHEEL;

    disable_motor_outputs();
    motor_u_pwm_duty = 0;
    motor_v_pwm_duty = 0;
    motor_w_pwm_duty = 0;
}




bool check_overcurrent(){
    const float overcurrent_threshold = 5.0;
    return abs(current_u) > overcurrent_threshold || 
           abs(current_v) > overcurrent_threshold || 
           abs(current_w) > overcurrent_threshold;
}



void hold_motor(uint16_t u, uint16_t v, uint16_t w, uint16_t timeout){
    driver_state = DriverState::HOLD;

    motor_u_pwm_duty = u > PWM_MAX_HOLD ? PWM_MAX_HOLD : u;
    motor_v_pwm_duty = v > PWM_MAX_HOLD ? PWM_MAX_HOLD : v;
    motor_w_pwm_duty = w > PWM_MAX_HOLD ? PWM_MAX_HOLD : w;

    duration_till_timeout = timeout > MAX_TIMEOUT ? MAX_TIMEOUT : timeout;
    enable_motor_outputs();
}

void update_motor_control(){
    // Update motor control registers only if actively driving.

    const float (*motor_voltage_table)[3] = nullptr;
    if (driver_state == DriverState::DRIVE_NEG) motor_voltage_table = motor_voltage_table_neg;
    else if(driver_state == DriverState::DRIVE_POS) motor_voltage_table = motor_voltage_table_pos;
    else return;

    // Note: The registers need to be left unchanged whilst running in the calibration modes.

    // Use the hall sensor state to determine the motor position and commutation.
    if (not hall_sensor_valid) {
        motor_break();
        return;
    }

    const float voltage_phase_u = motor_voltage_table[motor_electric_phase][0];
    const float voltage_phase_v = motor_voltage_table[motor_electric_phase][1];
    const float voltage_phase_w = motor_voltage_table[motor_electric_phase][2];


    motor_u_pwm_duty = voltage_phase_u * pwm_fraction * PWM_BASE;
    motor_v_pwm_duty = voltage_phase_v * pwm_fraction * PWM_BASE;
    motor_w_pwm_duty = voltage_phase_w * pwm_fraction * PWM_BASE;

}

void drive_motor_neg(uint16_t pwm, uint16_t timeout){
    driver_state = DriverState::DRIVE_NEG;
    pwm_fraction = (pwm > PWM_MAX ? PWM_MAX : pwm) / static_cast<float>(PWM_BASE);
    duration_till_timeout = timeout > MAX_TIMEOUT ? MAX_TIMEOUT : timeout;
    update_motor_control();
    enable_motor_outputs();
}

void drive_motor_pos(uint16_t pwm, uint16_t timeout){
    driver_state = DriverState::DRIVE_POS;
    pwm_fraction = (pwm > PWM_MAX ? PWM_MAX : pwm) / static_cast<float>(PWM_BASE);
    duration_till_timeout = timeout > MAX_TIMEOUT ? MAX_TIMEOUT : timeout;
    update_motor_control();
    enable_motor_outputs();
}


void start_test(const PWMSchedule & schedule){
    // Don't start a new test if we're already running one.
    if (active_schedule != nullptr) return;

    active_schedule = &schedule;

    driver_state = DriverState::TEST_SCHEDULE;
}

void test_procedure(){
    if(active_schedule == nullptr) return;

    if (test_procedure_start) {
        test_procedure_start = false;
        schedule_counter = 0;
        schedule_stage = 0;
            
        readouts_to_send = 0;
        readouts_allow_sending = false;
        readouts_allow_missing = false;
        // Clear the readouts buffer.
        UpdateReadout discard_readout = {};
        while(xQueueReceiveFromISR(readouts_queue, &discard_readout, NULL) == pdPASS) /* Discard all readouts up to test start. */;
        // Store exactly HISTORY_SIZE readouts; until we fully transmit the test.

        enable_motor_outputs();
    }

    const PWMSchedule & schedule = *active_schedule;


    motor_u_pwm_duty = schedule[schedule_stage].u;
    motor_v_pwm_duty = schedule[schedule_stage].v;
    motor_w_pwm_duty = schedule[schedule_stage].w;

    // Go to the next step in the schedule.
    schedule_counter += 1;
    if (schedule_counter >= schedule[schedule_stage].duration) {
        schedule_stage += 1;
        schedule_counter = 0;
    }
    // Next stage, unless we're at the end of the schedule.
    if (schedule_stage >= SCHEDULE_SIZE) {
        
        motor_break();
        readouts_to_send = HISTORY_SIZE;
        readouts_allow_sending = true;
        // Reset the first run flag.
        test_procedure_start = true;
        // Reset active schedule so we can start another test.
        active_schedule = nullptr;
    }
}

void update_motor_control_registers(){
    switch (driver_state) {
        case DriverState::OFF:
        case DriverState::FREEWHEEL:
            break;
    
        case DriverState::TEST_SCHEDULE:
            // Quickly update the PWM settings from the test schedule.
            test_procedure();
            break;
            
        case DriverState::DRIVE_NEG:
        case DriverState::DRIVE_POS:
        case DriverState::HOLD:
            if (duration_till_timeout <= 0) {
                motor_break();
            } else {
                duration_till_timeout -= 1;
            }
            // The PWM settings are updated in the main loop; we only need to write the registers.
            break;
    }

    write_motor_registers();
}
