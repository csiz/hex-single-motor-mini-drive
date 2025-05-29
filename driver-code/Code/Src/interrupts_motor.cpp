#include "interrupts_motor.hpp"

#include "user_data.hpp"

DriverState driver_state = DriverState::OFF;

uint16_t hold_u_pwm_duty = 0;
uint16_t hold_v_pwm_duty = 0;
uint16_t hold_w_pwm_duty = 0;

uint16_t pwm_command = 0;
uint16_t duration_till_timeout = 0;

int16_t leading_angle = 0;

PWMSchedule const* schedule_queued = nullptr;


PWMSchedule const* schedule_active = nullptr;
size_t schedule_counter = 0;
size_t schedule_stage = 0;


CurrentCalibration current_calibration = get_current_calibration();
FloatyCurrentCalibration floaty_current_calibration = make_floaty_current_calibration(current_calibration);
