#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void app_init();
void app_tick();
void show_error();

extern uint32_t main_loop_update_number;
const uint16_t min_pwm_duty = 64; // 64/1024 = 6.25% duty cycle
// Maximum duty cycle for the high side mosfet needs to allow some off time for 
// the bootstrap capacitor to charge so it has enough voltage to turn mosfet on.
const uint16_t min_bootstrap_duty = 4; // 4/(72MHz) = 55.5ns
const uint16_t max_pwm_duty = 1024 - min_bootstrap_duty; // 1024/72MHz = 14.2us

#ifdef __cplusplus
}
#endif