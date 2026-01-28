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

// Alias for a generic function that reads a buffer of some size.
using BufferFunction = void (*)(uint8_t * buffer, int size);

// Run a tick of the USB comms.
// 
// Send data in the tx_buffer of size tx_size returning whether data was queued successfully.
// And call the process_received_data function to handle any received data. 
bool usb_update(uint8_t * tx_data, int tx_size, BufferFunction process_received_data);

void usb_reset();

#endif