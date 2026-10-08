#include "usb_com.hpp"

#include "usbd_cdc_if.h"

#include "error_handler.hpp"

#include "circular_buffer.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>


extern "C" uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern "C" uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

const int max_usb_packet = 512;

CircularBuffer<UserRxBufferFS> usb_com_rx_buffer;

CircularBuffer<UserTxBufferFS> usb_com_tx_buffer;


static_assert(APP_RX_DATA_SIZE == 2 * max_usb_packet, "The USB sizes must be twice the max USB packet size.");

extern "C" USBD_HandleTypeDef hUsbDeviceFS;

volatile bool usb_active = false;
volatile bool receiving_active = false;
volatile size_t usb_pending_send = 0;
volatile uint16_t usb_host_is_ready = 0;


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

void usb_received(size_t rx_size) {
  // Ignore data we weren't prepared to receive.
  if (not receiving_active) return;

  if(usb_com_rx_buffer.mark_written(rx_size)) error();

  usb_prepare_receive();
}

void usb_confirmed_send(size_t len){
  if (usb_pending_send == len) {
    if(usb_com_tx_buffer.mark_read(len)) error();
    usb_pending_send = 0;
  } else {
    // We might get a different length if the connection is interrupted.
    usb_reset();
  }
}


uint8_t * usb_reserve_send_pointer(size_t max_write_size){
  if (not usb_active) return nullptr;

  return usb_com_tx_buffer.reserve_write_head(max_write_size);
}

void usb_mark_written(size_t tx_size) {
  if(usb_com_tx_buffer.mark_written(tx_size)) error();
}

// Return the number of bytes available to read from the USB receive buffer.
size_t usb_read_available(){
  return usb_com_rx_buffer.read_available();
}

uint8_t const* usb_get_read_pointer() {
  return usb_com_rx_buffer.get_read_pointer();
}

void usb_mark_read(size_t read_size) {
  if(usb_com_rx_buffer.mark_read(read_size)) error();
}

void usb_update() {
  if (not usb_active) return;
  
  if (not receiving_active) {
    usb_prepare_receive();
  }
  
  // Wait until the previous transmission is complete.
  if (usb_pending_send) return;

  // Wait for the host to be ready.
  if (not usb_host_is_ready) return;

  // Check if there is data to send.
  usb_pending_send = usb_com_tx_buffer.read_available();

  if (not usb_pending_send) return;

  uint8_t * const tx_head = usb_com_tx_buffer.get_read_pointer();

  // Not ready yet.
  const USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0U) return;

  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, tx_head, usb_pending_send);
  USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}

void usb_reset(){
    usb_deinit();

    // Reset the buffers.
    usb_com_rx_buffer.reset();
    usb_com_tx_buffer.reset();

    // Re-initialize USB.
    usb_init();
}

void usb_set_control_state(uint16_t control_state) {
  usb_host_is_ready = control_state;
}