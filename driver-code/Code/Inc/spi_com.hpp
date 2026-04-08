#pragma once


#ifdef __cplusplus
#include <cstdint>
#include <cstddef>
extern "C" {
#else
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#endif

void spi_init();

#ifdef __cplusplus
}

#endif