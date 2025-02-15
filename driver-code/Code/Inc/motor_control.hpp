#pragma once

#include <cstdint>

void motor_control_init();

void set_motor_pwm_gated(uint16_t u, uint16_t v, uint16_t w);

void drive_motor();

void update_motor_control();

void update_motor_control_registers();

void start_test();