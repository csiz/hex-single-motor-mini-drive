#pragma once

#include <cstdint>
#include <cstddef>

// Callback type for WebSocket receive
typedef void (*ws_receive_callback_t)(uint8_t* buffer, size_t size);

void setup_server(ws_receive_callback_t receive_callback);
int ws_send_binary(const uint8_t* buffer, size_t size);


