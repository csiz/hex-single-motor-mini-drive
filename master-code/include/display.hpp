#pragma once

#include <cstdint>

const int display_width = 240;
const int display_height = 300;

void setup_display_and_lvgl();

void update_display(int64_t current_time_ms);