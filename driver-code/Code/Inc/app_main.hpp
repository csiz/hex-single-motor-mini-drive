#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void app_init();
void app_tick();
void show_error();

extern uint32_t main_loop_update_number;

#ifdef __cplusplus
}
#endif