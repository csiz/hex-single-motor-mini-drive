#pragma once


#ifdef __cplusplus
#include <cstdint>
#include <cstddef>
#include <functional>
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

// Run a tick of the USB comms.
bool usb_update(uint8_t * tx_data, size_t tx_size, std::function<void(uint8_t * buffer, size_t size)> process_received_data);

void usb_reset();

#endif