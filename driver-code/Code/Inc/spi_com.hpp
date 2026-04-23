#pragma once


#ifdef __cplusplus
#include <cstdint>
#include <cstddef>
#include <functional>
extern "C" {
#else
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#endif

void spi_init();

#ifdef __cplusplus
}

bool spi_update(uint8_t * tx_data, size_t tx_size, std::function<void(uint8_t * buffer, size_t size)> process_received_data);

void spi_reset();

#endif