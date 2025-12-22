#include "usb_com.hpp"

#include "usbd_cdc_if.h"

#include "error_handler.hpp"

#include "circular_buffer.hpp"
#include <cstdint>
#include <cstring>


extern "C" uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern "C" uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

const int max_usb_packet = 512;

CircularBuffer usb_com_rx_buffer = {UserRxBufferFS,APP_RX_DATA_SIZE};

CircularBuffer usb_com_tx_buffer = {UserTxBufferFS, APP_TX_DATA_SIZE};


static_assert(APP_RX_DATA_SIZE == 2 * max_usb_packet, "The USB sizes must be twice the max USB packet size.");

extern "C" USBD_HandleTypeDef hUsbDeviceFS;

volatile bool usb_active = false;
volatile bool receiving_active = false;
volatile size_t usb_pending_send = 0;


static inline void usb_prepare_receive() {
  // Prepare to receive more data.
  uint8_t * const rx_head = usb_com_rx_buffer.reserve_write_head(max_usb_packet);

  if (rx_head != nullptr) {
    // Tell the driver where to write the next incoming packet.
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, rx_head);
    // Receive the next packet
    receiving_active = USBD_CDC_ReceivePacket(&hUsbDeviceFS) == USBD_OK;
  } else {
    receiving_active = false;
  }
}

void usb_init() {
  usb_pending_send = 0;
  usb_active = true;
  usb_prepare_receive();
}

void usb_deinit() {
  usb_pending_send = 0;
  receiving_active = false;
  usb_active = false;
}

void usb_received(int rx_size) {
  // Ignore data we weren't prepared to receive.
  if (not receiving_active) return;

  if(usb_com_rx_buffer.mark_write(rx_size)) error();

  usb_prepare_receive();
}


static inline void usb_prepare_send() {
  // Check if the USB driver is ready to send.
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

  if (hcdc->TxState != 0) return;
  
  // If it's ready to send, then the previous transmission is complete.
  if (usb_pending_send) {
    if(usb_com_tx_buffer.mark_read(usb_pending_send)) error();

    usb_pending_send = 0;
  }

  // Check if there is data to send.
  usb_pending_send = usb_com_tx_buffer.available_to_read();

  if (not usb_pending_send) return;

  uint8_t * const tx_head = usb_com_tx_buffer.get_read_head();

  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, tx_head, usb_pending_send);
  USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}

bool usb_update(uint8_t * tx_data, int tx_size, BufferFunction process_received_data) {
  if (not usb_active) return false;


  // Read any data in the reading buffer.
  const int read_available = usb_com_rx_buffer.available_to_read();
  if (read_available > 0) {
    process_received_data(usb_com_rx_buffer.get_read_head(), read_available);
    if(usb_com_rx_buffer.mark_read(read_available)) error();
  }
  
  if (not receiving_active) {
    usb_prepare_receive();
  }

  uint8_t * const tx_head = usb_com_tx_buffer.reserve_write_head(tx_size);

  const bool can_send = tx_head != nullptr;

  // Queue data if we have space to send it.
  if (can_send) {
    // There is enough space to queue the data.
    std::memcpy(tx_head, tx_data, tx_size);
    if(usb_com_tx_buffer.mark_write(tx_size)) error();
  }

  usb_prepare_send();

  return can_send;
}

void usb_reset(){
    usb_deinit();

    // Reset the buffers.
    usb_com_rx_buffer.reset();
    usb_com_tx_buffer.reset();

    // Re-initialize USB.
    usb_init();
}