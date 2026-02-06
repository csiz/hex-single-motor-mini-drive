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

// Reserve space in a buffer to which we write data to send.
uint8_t * usb_reserve_send_buffer(size_t size);

// Mark the reserved buffer as ready to send.
void usb_mark_send_buffer(size_t size);

// Run a tick of the USB comms.
bool usb_update(uint8_t * tx_data, size_t tx_size, void (*process_received_data)(uint8_t * buffer, size_t size));

void usb_reset();

#endif