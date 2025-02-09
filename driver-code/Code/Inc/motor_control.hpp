#pragma once


enum struct DriverState {
    OFF,
    DRIVE,
    MEASURE_CURRENT,
};

extern DriverState driver_state;

extern volatile bool motor_register_update_needed;

const uint32_t PWM_AUTORELOAD = 1535;

void motor_control_init();

void update_motor_control();

void update_motor_control_registers();