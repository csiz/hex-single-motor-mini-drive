#include "spi_com.hpp"
#include "main.h"

#include "error_handler.hpp"

#include "hex_mini_drive_interface.hpp"
#include "stm32g4xx_hal_spi.h"
#include <cstddef>

extern SPI_HandleTypeDef hspi3;

// We made a mistake in our circuit, so we need to read/write 3 extra bytes and discard them.
static constexpr size_t SPI_BUFFER_SIZE = hex_mini_drive::MAX_MESSAGE_SIZE + 3;

uint8_t spi_receive_buffer[SPI_BUFFER_SIZE] = {0};
uint8_t spi_transmit_buffer[SPI_BUFFER_SIZE] = {0};

volatile bool spi_done = false;
volatile bool spi_error = false;

static void spi_receive_buffer_reset() {
  for (size_t i = 0; i < SPI_BUFFER_SIZE; i++) {
    spi_receive_buffer[i] = 0;
  }
}

static void spi_transmit_buffer_reset() {
  for (size_t i = 0; i < SPI_BUFFER_SIZE; i++) {
    spi_transmit_buffer[i] = 0;
  }
}

static void spi_start_transfer() {
	if (HAL_SPI_TransmitReceive_DMA(&hspi3, spi_transmit_buffer, spi_receive_buffer, SPI_BUFFER_SIZE) != HAL_OK) {
		Error_Handler();
	}
}



extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI3) {
		spi_done = true;
	}
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI3) {
		spi_error = true;
	}
}

void spi_init() {
	// Note: SPI3 is already initialized properly in main.c by CubeMX.
	spi_receive_buffer_reset();

	// Start the first SPI transfer (full-duplex: transmit reply while receiving data).
	spi_start_transfer();
}

// Run a tick of the SPI comms.
bool spi_update(uint8_t * tx_data, size_t tx_size, std::function<void(uint8_t * buffer, size_t size)> process_received_data){
	if (spi_error) {
    // We had an error; reset the buffers and start a new transfer.
    spi_error = false;
    spi_receive_buffer_reset();
    spi_transmit_buffer_reset();
    spi_start_transfer();
    return false;
  }

  if (spi_done) {
    spi_done = false;

    // Process the received data.
    process_received_data(spi_receive_buffer, hex_mini_drive::MAX_MESSAGE_SIZE);

    if (tx_size > hex_mini_drive::MAX_MESSAGE_SIZE) error();

    for (size_t i = 0; i < tx_size; i++) {
      spi_transmit_buffer[i] = tx_data[i];
    }
    for (size_t i = tx_size; i < SPI_BUFFER_SIZE; i++) {
      spi_transmit_buffer[i] = 0;
    }

    // Queue up a new transfer.
    spi_start_transfer();

    return true;
  }

  return false;
}

void spi_reset() {
  HAL_SPI_Abort(&hspi3);
  spi_receive_buffer_reset();
  spi_transmit_buffer_reset();
  spi_start_transfer();
}
