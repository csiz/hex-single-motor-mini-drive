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

uint16_t pwm_command = 0;


// Motor voltage fraction for the 6-step commutation.
const uint16_t motor_voltage_table_pos[6][3] = {
    {0,        PWM_BASE, 0       },
    {0,        PWM_BASE, PWM_BASE},
    {0,        0,        PWM_BASE},
    {PWM_BASE, 0,        PWM_BASE},
    {PWM_BASE, 0,        0       },
    {PWM_BASE, PWM_BASE, 0       },
};

// Surpirsingly good schedule for the 6-step commutation.
const uint16_t motor_voltage_table_neg[6][3] {
    {0,        0,        PWM_BASE},
    {PWM_BASE, 0,        PWM_BASE},
    {PWM_BASE, 0,        0       },
    {PWM_BASE, PWM_BASE, 0       },
    {0,        PWM_BASE, 0       },
    {0,        PWM_BASE, PWM_BASE},
};

const uint16_t phases_waveform[256] = {
	1330, 1349, 1366, 1383, 1399, 1414, 1429, 1442, 1455, 1466, 1477, 1487, 1496, 1504, 1511, 1518,
	1523, 1527, 1531, 1534, 1535, 1536, 1536, 1535, 1533, 1530, 1526, 1521, 1516, 1509, 1501, 1493,
	1484, 1474, 1462, 1450, 1438, 1424, 1409, 1394, 1378, 1361, 1343, 1324, 1304, 1284, 1263, 1241,
	1219, 1195, 1171, 1147, 1121, 1095, 1068, 1041, 1013,  984,  955,  925,  895,  864,  832,  800,
	 768,  735,  702,  668,  634,  599,  565,  529,  494,  458,  422,  385,  349,  312,  275,  238,
	 200,  163,  126,   88,   50,   13,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
	   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
	   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
	   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
	   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
	   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,   13,   50,   88,  126,  163,
	 200,  238,  275,  312,  349,  385,  422,  458,  494,  529,  565,  599,  634,  668,  702,  735,
	 768,  800,  832,  864,  895,  925,  955,  984, 1013, 1041, 1068, 1095, 1121, 1147, 1171, 1195,
	1219, 1241, 1263, 1284, 1304, 1324, 1343, 1361, 1378, 1394, 1409, 1424, 1438, 1450, 1462, 1474,
	1484, 1493, 1501, 1509, 1516, 1521, 1526, 1530, 1533, 1535, 1536, 1536, 1535, 1534, 1531, 1527,
	1523, 1518, 1511, 1504, 1496, 1487, 1477, 1466, 1455, 1442, 1429, 1414, 1399, 1383, 1366, 1349
};


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




void hold_motor(uint16_t u, uint16_t v, uint16_t w, uint16_t timeout){
    driver_state = DriverState::HOLD;

    motor_u_pwm_duty = u > PWM_MAX_HOLD ? PWM_MAX_HOLD : u;
    motor_v_pwm_duty = v > PWM_MAX_HOLD ? PWM_MAX_HOLD : v;
    motor_w_pwm_duty = w > PWM_MAX_HOLD ? PWM_MAX_HOLD : w;

    duration_till_timeout = timeout > MAX_TIMEOUT ? MAX_TIMEOUT : timeout;
    enable_motor_outputs();
}

static inline void sector_motor_control(){
    // Update motor control registers only if actively driving.

    const uint16_t (*motor_voltage_table)[3] = 
        driver_state == DriverState::DRIVE_POS ? motor_voltage_table_pos : 
        driver_state == DriverState::DRIVE_NEG ? motor_voltage_table_neg : 
        nullptr;
    if (motor_voltage_table == nullptr) return;

    // Note: The registers need to be left unchanged whilst running in the calibration modes.

    // Use the hall sensor state to determine the motor position and commutation.
    if (not hall_sector_valid) {
        motor_break();
        return;
    }

    const uint16_t voltage_phase_u = motor_voltage_table[hall_sector][0];
    const uint16_t voltage_phase_v = motor_voltage_table[hall_sector][1];
    const uint16_t voltage_phase_w = motor_voltage_table[hall_sector][2];


    motor_u_pwm_duty = voltage_phase_u * pwm_command / PWM_BASE;
    motor_v_pwm_duty = voltage_phase_v * pwm_command / PWM_BASE;
    motor_w_pwm_duty = voltage_phase_w * pwm_command / PWM_BASE;
}

int drive_angle = 0;

static inline void smooth_motor_control(uint8_t angle){
    const int direction = 
        driver_state == DriverState::DRIVE_SMOOTH_POS ? +1 : 
        driver_state == DriverState::DRIVE_SMOOTH_NEG ? -1 : 
        0;
    if (direction == 0) return;

    if (not angle_valid) {
        motor_break();
        return;
    }


    const int target_angle = (256 + static_cast<int>(angle) + direction * drive_angle) % 256;

    const uint16_t voltage_phase_u = phases_waveform[target_angle];
    const uint16_t voltage_phase_v = phases_waveform[(256 + target_angle - 85) % 256];
    const uint16_t voltage_phase_w = phases_waveform[(256 + target_angle - 170) % 256];

    motor_u_pwm_duty = voltage_phase_u * pwm_command / PWM_BASE;
    motor_v_pwm_duty = voltage_phase_v * pwm_command / PWM_BASE;
    motor_w_pwm_duty = voltage_phase_w * pwm_command / PWM_BASE;
}

static inline void radial_motor_control(){
    if (not angle_valid) {
        motor_break();
        return;
    }

    // TODO:
}

void drive_motor_neg(uint16_t pwm, uint16_t timeout){
    driver_state = DriverState::DRIVE_NEG;
    pwm_command = pwm > PWM_MAX ? PWM_MAX : pwm;
    duration_till_timeout = timeout > MAX_TIMEOUT ? MAX_TIMEOUT : timeout;
    sector_motor_control();
    enable_motor_outputs();
}

void drive_motor_pos(uint16_t pwm, uint16_t timeout){
    driver_state = DriverState::DRIVE_POS;
    pwm_command = pwm > PWM_MAX ? PWM_MAX : pwm;
    duration_till_timeout = timeout > MAX_TIMEOUT ? MAX_TIMEOUT : timeout;
    sector_motor_control();
    enable_motor_outputs();
}



void drive_motor_smooth_pos(uint16_t pwm, uint16_t timeout){
    driver_state = DriverState::DRIVE_SMOOTH_POS;
    pwm_command = pwm > PWM_MAX ? PWM_MAX : pwm;
    // Fix timeout to experiment with drive angle.
    duration_till_timeout = MAX_TIMEOUT / 2;
    drive_angle = 256 * static_cast<int>(timeout) / MAX_TIMEOUT;
    // duration_till_timeout = timeout > MAX_TIMEOUT ? MAX_TIMEOUT : timeout;
    enable_motor_outputs();
}

void drive_motor_smooth_neg(uint16_t pwm, uint16_t timeout){
    driver_state = DriverState::DRIVE_SMOOTH_NEG;
    pwm_command = pwm > PWM_MAX ? PWM_MAX : pwm;
    duration_till_timeout = MAX_TIMEOUT / 2;
    drive_angle = 256 * static_cast<int>(timeout) / MAX_TIMEOUT;
    // duration_till_timeout = timeout > MAX_TIMEOUT ? MAX_TIMEOUT : timeout;
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

static inline void update_timeout(){
    if (duration_till_timeout > 0) {
        duration_till_timeout -= 1;
    } else {
        motor_break();
    }
}

void update_motor_control_registers(uint8_t angle){
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
            sector_motor_control();
            update_timeout();
            break;
        case DriverState::DRIVE_SMOOTH_POS:
        case DriverState::DRIVE_SMOOTH_NEG:
            smooth_motor_control(angle);
            update_timeout();
            break;
        case DriverState::HOLD:
            update_timeout();
            break;
    }

    write_motor_registers();
}
