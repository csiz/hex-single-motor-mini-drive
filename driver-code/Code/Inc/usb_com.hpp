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

void usb_init();

void usb_deinit();

void usb_received(size_t len);

void usb_confirmed_send(size_t len);

void usb_set_control_state(uint16_t control_state);

#ifdef __cplusplus
}


// Get buffer where to write data of max_write_size; returns nullptr if no space available.
uint8_t * usb_reserve_send_pointer(size_t max_write_size);

// Mark how much data we have written to the send buffer.
void usb_mark_written(size_t tx_size);

// Get the number of bytes available to read from the receive buffer.
size_t usb_read_available();

// Get the pointer to the start of the readable data in the receive buffer.
uint8_t const* usb_get_read_pointer();

// Mark how many bytes we have completed reading so we can free up the space.
void usb_mark_read(size_t read_size);

// Update the USB internals so it can receive and send from the buffers.
void usb_update();

// Reset the USB interface and buffers.
void usb_reset();

#endif