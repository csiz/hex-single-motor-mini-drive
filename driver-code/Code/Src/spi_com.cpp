#include "spi_com.hpp"
#include "main.h"
#include <cstring>

extern SPI_HandleTypeDef hspi3;

static constexpr uint16_t spi_buffer_size = 128;

static uint8_t spi_receive_buffer[spi_buffer_size];
static uint8_t spi_transmit_buffer[spi_buffer_size];

static void spi_receive_buffer_reset() {
    memset(spi_receive_buffer, 0, spi_buffer_size);
}

static void spi_transmit_buffer_init() {
    for (uint16_t i = 0; i < spi_buffer_size; i++) {
        spi_transmit_buffer[i] = static_cast<uint8_t>(i);
    }
}

static void spi_start_transfer() {
    if (HAL_SPI_TransmitReceive_IT(&hspi3, spi_transmit_buffer, spi_receive_buffer, spi_buffer_size) != HAL_OK) {
        Error_Handler();
    }
}

void spi_init() {
    // Note: SPI3 is already initialized properly in main.c by CubeMX.

    // Fill transmit buffer with 0..127
    spi_transmit_buffer_init();
    spi_receive_buffer_reset();

    // Start the first SPI transfer (full-duplex: transmit reply while receiving data).
    spi_start_transfer();
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI3) {
        // Process spi_receive_buffer here if needed.

        // Restart the next transfer.
        spi_receive_buffer_reset();
        spi_start_transfer();
    }
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI3) {
        // Restart on error.
        spi_receive_buffer_reset();
        spi_start_transfer();
    }
}