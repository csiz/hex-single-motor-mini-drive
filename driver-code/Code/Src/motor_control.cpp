#include "motor_control.hpp"
#include "io.hpp"
#include "data.hpp"

#include <stm32f1xx_ll_adc.h>
#include <stm32f1xx_ll_tim.h>


#include <cmath>



// Motor voltage fraction for the 6-step commutation.
const float motor_voltage_table[6][3] = {
    {0.5, 1.0, 0.0},
    {0.0, 1.0, 0.5},
    {0.0, 0.5, 1.0},
    {0.5, 0.0, 1.0},
    {1.0, 0.0, 0.5},
    {1.0, 0.5, 0.0}
};

// Calibration procedures
// ----------------------

const uint16_t PWM_TEST = PWM_BASE / 2;

struct PWMTestStage {
    uint16_t duration;
    uint16_t u;
    uint16_t v;
    uint16_t w;
};

template <size_t N>
struct PWMTestSchedule {
    const PWMTestStage stages[N];
    
    size_t stage = 0;
    size_t counter = 0;

    PWMTestSchedule(const PWMTestStage (&stages)[N]) : stages(stages) {}
};

const size_t short_duration = HISTORY_SIZE / 12;

PWMTestSchedule<12> test_all_permutations = {{
    {short_duration, 0,         0,         0},
    {short_duration, PWM_TEST,  0,         0}, // Positive U
    {short_duration, 0,         0,         0},
    {short_duration, 0,         PWM_TEST,  0}, // Positive V
    {short_duration, 0,         0,         0},
    {short_duration, 0,         0,         PWM_TEST}, // Positive W
    {short_duration, 0,         0,         0},
    {short_duration, 0,         PWM_TEST,  PWM_TEST}, // Negative U
    {short_duration, 0,         0,         0},
    {short_duration, PWM_TEST,  0,         PWM_TEST}, // Negative V
    {short_duration, 0,         0,         0},
    {short_duration, PWM_TEST,  PWM_TEST,  0} // Negative W
}};

const size_t medium_duration = HISTORY_SIZE / 6;

PWMTestSchedule<6> test_single_phase_positive = {{
    {medium_duration, 0,         0,         0},
    {medium_duration, PWM_TEST,  0,         0}, // Positive U
    {medium_duration, 0,         0,         0},
    {medium_duration, 0,         PWM_TEST,  0}, // Positive V
    {medium_duration, 0,         0,         0},
    {medium_duration, 0,         0,         PWM_TEST} // Positive W
}};

PWMTestSchedule<6> test_double_phase_positive = {{
    {medium_duration, 0,         0,         0},
    {medium_duration, 0,         PWM_TEST,  PWM_TEST}, // Negative U
    {medium_duration, 0,         0,         0},
    {medium_duration, PWM_TEST,  0,         PWM_TEST}, // Negative V
    {medium_duration, 0,         0,         0},
    {medium_duration, PWM_TEST,  PWM_TEST,  0} // Negative W
}};

PWMTestSchedule<6> test_all_shorted = {{
    {medium_duration, PWM_FLOAT, PWM_FLOAT, PWM_FLOAT},
    {medium_duration, 0,         0,         0},
    {medium_duration, PWM_FLOAT, PWM_FLOAT, PWM_FLOAT},
    {medium_duration, PWM_MAX,   PWM_MAX,   PWM_MAX},
    {medium_duration, PWM_FLOAT, PWM_FLOAT, PWM_FLOAT},
    {medium_duration, PWM_FLOAT, PWM_FLOAT, PWM_FLOAT}
}};

PWMTestSchedule<6> test_long_grounded_short = {{
    {medium_duration, PWM_FLOAT, PWM_FLOAT, PWM_FLOAT},
    {medium_duration, 0,         0,         0},
    {medium_duration, 0,         0,         0},
    {medium_duration, 0,         0,         0},
    {medium_duration, 0,         0,         0},
    {medium_duration, PWM_FLOAT, PWM_FLOAT, PWM_FLOAT}
}};

PWMTestSchedule<6> test_long_positive_short = {{
    {medium_duration, PWM_FLOAT, PWM_FLOAT, PWM_FLOAT},
    {medium_duration, PWM_MAX,   PWM_MAX,   PWM_MAX},
    {medium_duration, PWM_MAX,   PWM_MAX,   PWM_MAX},
    {medium_duration, PWM_MAX,   PWM_MAX,   PWM_MAX},
    {medium_duration, PWM_FLOAT, PWM_FLOAT, PWM_FLOAT},
    {medium_duration, PWM_FLOAT, PWM_FLOAT, PWM_FLOAT}
}};

PWMTestSchedule<12> test_u_directions = {{
    {short_duration, PWM_FLOAT, PWM_FLOAT, PWM_FLOAT},
    {short_duration, 0,         PWM_TEST,  PWM_TEST},
    {short_duration, 0,         0,         0},
    {short_duration, PWM_TEST,  0,         0},
    {short_duration, 0,         0,         0},
    {short_duration, 0,         PWM_TEST,  PWM_TEST},
    {short_duration, 0,         0,         0},
    {short_duration, PWM_TEST,  0,         0},
    {short_duration, 0,         0,         0},
    {short_duration, 0,         PWM_TEST,  PWM_TEST},
    {short_duration, 0,         0,         0},
    {short_duration, PWM_FLOAT, PWM_FLOAT, PWM_FLOAT}
}};

PWMTestSchedule<12> test_u_increasing = {{
    {short_duration, PWM_FLOAT,           PWM_FLOAT, PWM_FLOAT},
    {short_duration, PWM_BASE * 1 / 10,   0,         0},
    {short_duration, PWM_BASE * 2 / 10,   0,         0},
    {short_duration, PWM_BASE * 3 / 10,   0,         0},
    {short_duration, PWM_BASE * 4 / 10,   0,         0},
    {short_duration, PWM_BASE * 5 / 10,   0,         0},
    {short_duration, PWM_BASE * 6 / 10,   0,         0},
    {short_duration, PWM_BASE * 7 / 10,   0,         0},
    {short_duration, PWM_BASE * 8 / 10,   0,         0},
    {short_duration, PWM_BASE * 9 / 10,   0,         0},
    {short_duration, 0,                   0,         0},
    {short_duration, PWM_FLOAT,           PWM_FLOAT, PWM_FLOAT}
}};

PWMTestSchedule<12> test_u_decreasing = {{
    {short_duration, PWM_FLOAT,           PWM_FLOAT, PWM_FLOAT},
    {short_duration, 0,                   PWM_BASE * 1 / 10, PWM_BASE * 1 / 10},
    {short_duration, 0,                   PWM_BASE * 2 / 10, PWM_BASE * 2 / 10},
    {short_duration, 0,                   PWM_BASE * 3 / 10, PWM_BASE * 3 / 10},
    {short_duration, 0,                   PWM_BASE * 4 / 10, PWM_BASE * 4 / 10},
    {short_duration, 0,                   PWM_BASE * 5 / 10, PWM_BASE * 5 / 10},
    {short_duration, 0,                   PWM_BASE * 6 / 10, PWM_BASE * 6 / 10},
    {short_duration, 0,                   PWM_BASE * 7 / 10, PWM_BASE * 7 / 10},
    {short_duration, 0,                   PWM_BASE * 8 / 10, PWM_BASE * 8 / 10},
    {short_duration, 0,                   PWM_BASE * 9 / 10, PWM_BASE * 9 / 10},
    {short_duration, 0,                   0,                 0},
    {short_duration, PWM_FLOAT,           PWM_FLOAT,         PWM_FLOAT}
}};

PWMTestSchedule<12> test_v_increasing = {{
    {short_duration, PWM_FLOAT,           PWM_FLOAT, PWM_FLOAT},
    {short_duration, 0,                   PWM_BASE * 1 / 10, 0},
    {short_duration, 0,                   PWM_BASE * 2 / 10, 0},
    {short_duration, 0,                   PWM_BASE * 3 / 10, 0},
    {short_duration, 0,                   PWM_BASE * 4 / 10, 0},
    {short_duration, 0,                   PWM_BASE * 5 / 10, 0},
    {short_duration, 0,                   PWM_BASE * 6 / 10, 0},
    {short_duration, 0,                   PWM_BASE * 7 / 10, 0},
    {short_duration, 0,                   PWM_BASE * 8 / 10, 0},
    {short_duration, 0,                   PWM_BASE * 9 / 10, 0},
    {short_duration, 0,                   0,                 0},
    {short_duration, PWM_FLOAT,           PWM_FLOAT,         PWM_FLOAT}
}};

PWMTestSchedule<12> test_v_decreasing = {{
    {short_duration, PWM_FLOAT,           PWM_FLOAT, PWM_FLOAT},
    {short_duration, PWM_BASE * 1 / 10,   0,         PWM_BASE * 1 / 10},
    {short_duration, PWM_BASE * 2 / 10,   0,         PWM_BASE * 2 / 10},
    {short_duration, PWM_BASE * 3 / 10,   0,         PWM_BASE * 3 / 10},
    {short_duration, PWM_BASE * 4 / 10,   0,         PWM_BASE * 4 / 10},
    {short_duration, PWM_BASE * 5 / 10,   0,         PWM_BASE * 5 / 10},
    {short_duration, PWM_BASE * 6 / 10,   0,         PWM_BASE * 6 / 10},
    {short_duration, PWM_BASE * 7 / 10,   0,         PWM_BASE * 7 / 10},
    {short_duration, PWM_BASE * 8 / 10,   0,         PWM_BASE * 8 / 10},
    {short_duration, PWM_BASE * 9 / 10,   0,         PWM_BASE * 9 / 10},
    {short_duration, 0,                   0,         0},
    {short_duration, PWM_FLOAT,           PWM_FLOAT, PWM_FLOAT}
}};

PWMTestSchedule<12> test_w_increasing = {{
    {short_duration, PWM_FLOAT,           PWM_FLOAT, PWM_FLOAT},
    {short_duration, 0,                   0,         PWM_BASE * 1 / 10},
    {short_duration, 0,                   0,         PWM_BASE * 2 / 10},
    {short_duration, 0,                   0,         PWM_BASE * 3 / 10},
    {short_duration, 0,                   0,         PWM_BASE * 4 / 10},
    {short_duration, 0,                   0,         PWM_BASE * 5 / 10},
    {short_duration, 0,                   0,         PWM_BASE * 6 / 10},
    {short_duration, 0,                   0,         PWM_BASE * 7 / 10},
    {short_duration, 0,                   0,         PWM_BASE * 8 / 10},
    {short_duration, 0,                   0,         PWM_BASE * 9 / 10},
    {short_duration, 0,                   0,         0},
    {short_duration, PWM_FLOAT,           PWM_FLOAT, PWM_FLOAT}
}};

PWMTestSchedule<12> test_w_decreasing = {{
    {short_duration, PWM_FLOAT,           PWM_FLOAT, PWM_FLOAT},
    {short_duration, PWM_BASE * 1 / 10,   PWM_BASE * 1 / 10, 0},
    {short_duration, PWM_BASE * 2 / 10,   PWM_BASE * 2 / 10, 0},
    {short_duration, PWM_BASE * 3 / 10,   PWM_BASE * 3 / 10, 0},
    {short_duration, PWM_BASE * 4 / 10,   PWM_BASE * 4 / 10, 0},
    {short_duration, PWM_BASE * 5 / 10,   PWM_BASE * 5 / 10, 0},
    {short_duration, PWM_BASE * 6 / 10,   PWM_BASE * 6 / 10, 0},
    {short_duration, PWM_BASE * 7 / 10,   PWM_BASE * 7 / 10, 0},
    {short_duration, PWM_BASE * 8 / 10,   PWM_BASE * 8 / 10, 0},
    {short_duration, PWM_BASE * 9 / 10,   PWM_BASE * 9 / 10, 0},
    {short_duration, 0,                   0,         0},
    {short_duration, PWM_FLOAT,           PWM_FLOAT, PWM_FLOAT}
}};

void disable_motor_ouputs(){
    disable_motor_u_output();
    disable_motor_v_output();
    disable_motor_w_output();

    motor_u_pwm_duty = PWM_FLOAT;
    motor_v_pwm_duty = PWM_FLOAT;
    motor_w_pwm_duty = PWM_FLOAT;
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

void update_motor_control(){
    const float pwm_fraction = 0.2;
    // Use the hall sensor state to determine the motor position and commutation.
    if (hall_sensor_valid) {
        const float voltage_phase_u = motor_voltage_table[motor_electric_phase][0];
        const float voltage_phase_v = motor_voltage_table[motor_electric_phase][1];
        const float voltage_phase_w = motor_voltage_table[motor_electric_phase][2];

        if (voltage_phase_u == 0.5) {
            motor_u_pwm_duty = PWM_FLOAT;
        } else {
            motor_u_pwm_duty = voltage_phase_u * pwm_fraction * PWM_MAX;
        }

        if (voltage_phase_v == 0.5) {
            motor_v_pwm_duty = PWM_FLOAT;
        } else {
            motor_v_pwm_duty = voltage_phase_v * pwm_fraction * PWM_MAX;
        }

        if (voltage_phase_w == 0.5) {
            motor_w_pwm_duty = PWM_FLOAT;
        } else {
            motor_w_pwm_duty = voltage_phase_w * pwm_fraction * PWM_MAX;
        }
    } else {
        disable_motor_ouputs();
        motor_u_pwm_duty = PWM_FLOAT;
        motor_v_pwm_duty = PWM_FLOAT;
        motor_w_pwm_duty = PWM_FLOAT;
        driver_state = DriverState::OFF;
    }
}

void drive_motor(){
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

template<size_t N>
void start_test_procedure(PWMTestSchedule<N>& schedule){
    schedule.stage = 0;
    schedule.counter = 0;
    state_updates_to_send = 0;
}

void start_test(){
    switch (driver_state) {
        case DriverState::TEST_ALL_PERMUTATIONS:
            start_test_procedure(test_all_permutations);
            break;
        case DriverState::TEST_SINGLE_PHASE_POSITIVE:
            start_test_procedure(test_single_phase_positive);
            break;
        case DriverState::TEST_DOUBLE_PHASE_POSITIVE:
            start_test_procedure(test_double_phase_positive);
            break;
        case DriverState::TEST_ALL_SHORTED:
            start_test_procedure(test_all_shorted);
            break;
        case DriverState::TEST_LONG_GROUNDED_SHORT:
            start_test_procedure(test_long_grounded_short);
            break;
        case DriverState::TEST_LONG_POSITIVE_SHORT:
            start_test_procedure(test_long_positive_short);
            break;

        case DriverState::TEST_U_DIRECTIONS:
            start_test_procedure(test_u_directions);
            break;

        case DriverState::TEST_U_INCREASING:
            start_test_procedure(test_u_increasing);
            break;
        case DriverState::TEST_U_DECREASING:
            start_test_procedure(test_u_decreasing);
            break;
        case DriverState::TEST_V_INCREASING:
            start_test_procedure(test_v_increasing);
            break;
        case DriverState::TEST_V_DECREASING:
            start_test_procedure(test_v_decreasing);
            break;
        case DriverState::TEST_W_INCREASING:
            start_test_procedure(test_w_increasing);
            break;
        case DriverState::TEST_W_DECREASING:
            start_test_procedure(test_w_decreasing);
            break;

        
        case DriverState::OFF:
        case DriverState::DRIVE:
        case DriverState::HOLD_U_POSITIVE:
        case DriverState::HOLD_V_POSITIVE:
        case DriverState::HOLD_W_POSITIVE:
        case DriverState::HOLD_U_NEGATIVE:
        case DriverState::HOLD_V_NEGATIVE:
        case DriverState::HOLD_W_NEGATIVE:
            // Not a test, do nothing.
            break;
    }
}

template<size_t N>
void test_procedure(PWMTestSchedule<N>& schedule){ 
    motor_u_pwm_duty = schedule.stages[schedule.stage].u;
    motor_v_pwm_duty = schedule.stages[schedule.stage].v;
    motor_w_pwm_duty = schedule.stages[schedule.stage].w;

    drive_motor();

    schedule.counter += 1;
    if (schedule.counter >= schedule.stages[schedule.stage].duration) {
        schedule.stage += 1;
        schedule.counter = 0;
    }

    if (schedule.stage >= N) {
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
        case DriverState::TEST_ALL_PERMUTATIONS:
            test_procedure(test_all_permutations);
            break;
        case DriverState::TEST_SINGLE_PHASE_POSITIVE:
            test_procedure(test_single_phase_positive);
            break;
        case DriverState::TEST_DOUBLE_PHASE_POSITIVE:
            test_procedure(test_double_phase_positive);
            break;
        case DriverState::TEST_ALL_SHORTED:
            test_procedure(test_all_shorted);
            break;

        case DriverState::TEST_LONG_GROUNDED_SHORT:
            test_procedure(test_long_grounded_short);
            break;

        case DriverState::TEST_LONG_POSITIVE_SHORT:
            test_procedure(test_long_positive_short);
            break;

        case DriverState::TEST_U_DIRECTIONS:
            test_procedure(test_u_directions);
            break;

        case DriverState::TEST_U_INCREASING:
            test_procedure(test_u_increasing);
            break;
        case DriverState::TEST_U_DECREASING:
            test_procedure(test_u_decreasing);
            break;
        case DriverState::TEST_V_INCREASING:
            test_procedure(test_v_increasing);
            break;
        case DriverState::TEST_V_DECREASING:
            test_procedure(test_v_decreasing);
            break;
        case DriverState::TEST_W_INCREASING:
            test_procedure(test_w_increasing);
            break;
        case DriverState::TEST_W_DECREASING:
            test_procedure(test_w_decreasing);
            break;
        
        case DriverState::HOLD_U_POSITIVE:
        case DriverState::HOLD_V_POSITIVE:
        case DriverState::HOLD_W_POSITIVE:
        case DriverState::HOLD_U_NEGATIVE:
        case DriverState::HOLD_V_NEGATIVE:
        case DriverState::HOLD_W_NEGATIVE:
            drive_motor();
            break;
    }

}
