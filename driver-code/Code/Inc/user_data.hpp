#pragma once

#include <cstdint>
#include <cstddef>

const size_t user_data_size = 2048;

extern uint8_t user_data[user_data_size] __attribute__((__section__(".user_data")));

