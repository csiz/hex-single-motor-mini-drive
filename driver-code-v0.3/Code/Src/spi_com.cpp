#include "spi_com.hpp"
#include "main.h"

#include "error_handler.hpp"

#include "hex_mini_drive_interface.hpp"
#include "stm32g4xx_hal_spi.h"
#include <cstddef>

extern SPI_HandleTypeDef hspi3;

// We made a mistake in our circuit, so we need to read/write 3 extra bytes and discard them.
using hex_mini_drive::SPI_TRANSACTION_SIZE;
using hex_mini_drive::MAX_MESSAGE_SIZE;

uint8_t spi_receive_buffer[SPI_TRANSACTION_SIZE] = {0};
uint8_t spi_transmit_buffer[2][SPI_TRANSACTION_SIZE] = {0};
volatile size_t spi_transmit_buffer_index = 0;
size_t spi_transmit_buffer_head = 0;

volatile bool spi_done = false;
volatile bool spi_error = false;

static void spi_receive_buffer_reset() {
  for (size_t i = 0; i < SPI_TRANSACTION_SIZE; i++) {
    spi_receive_buffer[i] = 0;
  }
}

static void spi_transmit_buffer_reset() {
  for (size_t i = 0; i < SPI_TRANSACTION_SIZE; i++) {
    spi_transmit_buffer[0][i] = 0;
    spi_transmit_buffer[1][i] = 0;
  }
  spi_transmit_buffer_head = 0;
}

static void spi_start_transfer() {
	if (HAL_SPI_TransmitReceive_DMA(&hspi3, spi_transmit_buffer[spi_transmit_buffer_index], spi_receive_buffer, SPI_TRANSACTION_SIZE) != HAL_OK) {
		Error_Handler();
	}
  // Switch to the other transmit buffer for the next transfer, so we can prepare data while the current transfer
  spi_transmit_buffer_head = 0;
  spi_transmit_buffer_index = (spi_transmit_buffer_index + 1) % 2;
  for (size_t i = 0; i < SPI_TRANSACTION_SIZE; i++) {
    spi_transmit_buffer[spi_transmit_buffer_index][i] = 0;
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
  }

  // Queue up data to be transmitted:
  const bool can_queue_transmit = tx_size <= SPI_TRANSACTION_SIZE - spi_transmit_buffer_head;
  if (can_queue_transmit) {
    for (size_t i = 0; i < tx_size; i++) {
      spi_transmit_buffer[spi_transmit_buffer_index][spi_transmit_buffer_head + i] = tx_data[i];
    }
    spi_transmit_buffer_head += tx_size;
  }

  if (spi_done) {
    spi_done = false;

    // Process the received data.
    process_received_data(spi_receive_buffer, SPI_TRANSACTION_SIZE - 3);
    
    if (tx_size > MAX_MESSAGE_SIZE) error();

    // Queue up a new transfer.
    spi_start_transfer();
  }

  return can_queue_transmit;
}

void spi_reset() {
  HAL_SPI_Abort(&hspi3);
  spi_receive_buffer_reset();
  spi_transmit_buffer_reset();
  spi_start_transfer();
}
