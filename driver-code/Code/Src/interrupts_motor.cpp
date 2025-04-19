#include "interrupts_motor.hpp"

PWMSchedule const* schedule_active = nullptr;
size_t schedule_counter = 0;
size_t schedule_stage = 0;