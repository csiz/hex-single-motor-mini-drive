#include "usb_com.hpp"

#include "usbd_cdc_if.h"

#include "error_handler.hpp"

#include "circular_buffer.hpp"
#include <cstdint>
#include <cstring>


extern "C" uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern "C" uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

const int max_usb_packet = 512;

CircularBuffer usb_com_rx_buffer = {
  .buffer = UserRxBufferFS,
  .max_size = APP_RX_DATA_SIZE,
};

CircularBuffer usb_com_tx_buffer = {
  .buffer = UserTxBufferFS,
  .max_size = APP_TX_DATA_SIZE,
};


static_assert(APP_RX_DATA_SIZE == 2 * max_usb_packet, "The USB sizes must be twice the max USB packet size.");

extern "C" USBD_HandleTypeDef hUsbDeviceFS;

volatile bool usb_active = false;
volatile bool receiving_active = false;

volatile uint32_t usb_last_interact = 0;

const uint32_t usb_interact_timeout = 500; // milliseconds

volatile int usb_pending_send = 0;


void usb_init() {
  usb_last_interact = HAL_GetTick();
  usb_active = true;
  usb_prepare_receive();
}

void usb_deinit() {
  receiving_active = false;
  usb_active = false;
}

void usb_received(int rx_size) {
  // Ignore data we weren't prepared to receive.
  if (not receiving_active) return;

  if(buffer_mark_write(&usb_com_rx_buffer, rx_size)) error();

  usb_prepare_receive();
}

void usb_prepare_receive() {
  // Prepare to receive more data.
  uint8_t * const rx_head = buffer_reserve_write_head(&usb_com_rx_buffer, max_usb_packet);

  if (rx_head != nullptr) {
    // Tell the driver where to write the next incoming packet.
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, rx_head);
    // Receive the next packet
    receiving_active = USBD_CDC_ReceivePacket(&hUsbDeviceFS) == USBD_OK;
  } else {
    receiving_active = false;
  }
}

void usb_sent(int len) {
  usb_pending_send -= len;

  usb_last_interact = HAL_GetTick();

  if(buffer_mark_read(&usb_com_tx_buffer, len)) error();
}

void usb_prepare_send() {
  // Check if the USB driver is ready to send.
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0) return;
  
  // If it's ready to send, then the previous transmission is complete.
  if (usb_pending_send) usb_sent(usb_pending_send);

  if (usb_pending_send != 0) error();
  
  // Check if there is data to send.
  usb_pending_send = buffer_available_to_read(&usb_com_tx_buffer);
  if (not usb_pending_send) return;

  uint8_t * const tx_head = buffer_get_read_head(&usb_com_tx_buffer);
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, tx_head, usb_pending_send);
  USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}

bool usb_update(uint8_t * tx_buffer, int tx_size, BufferFunction process_received_data, void (* onreset)()) {

  // Read any data in the reading buffer.
  const int read_available = buffer_available_to_read(&usb_com_rx_buffer);
  if (read_available > 0) {
    process_received_data(usb_com_rx_buffer.buffer + usb_com_rx_buffer.read_head, read_available);
    if(buffer_mark_read(&usb_com_rx_buffer, read_available)) error();
  }
  
  if (not receiving_active) {
    usb_prepare_receive();
  }

  uint8_t * const tx_head = tx_size == 0 ? nullptr : buffer_reserve_write_head(&usb_com_tx_buffer, tx_size);

  const bool can_send = tx_head != nullptr;

  // Queue data if we have space to send it.
  if (can_send) {
    // There is enough space to queue the data.
    std::memcpy(tx_head, tx_buffer, tx_size);
    if(buffer_mark_write(&usb_com_tx_buffer, tx_size)) error();
  }

  usb_prepare_send();

  // Check for timeout.
  if (usb_active and (HAL_GetTick() - usb_last_interact > usb_interact_timeout)) {
    // No interaction for a while; disable USB.
    usb_deinit();

    // Reset the buffers.
    buffer_reset(&usb_com_rx_buffer);
    buffer_reset(&usb_com_tx_buffer);

    onreset();

    // Re-initialize USB.
    usb_init();
  }

  return can_send;
}