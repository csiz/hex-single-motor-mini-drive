#pragma once

#include <cstdint>
#include <cstddef>

#include <functional>

// Callback type for WebSocket receive
typedef void (*ws_receive_callback_t)(uint8_t* buffer, size_t size);

void setup_server(int core_id, ws_receive_callback_t receive_callback);

bool update_server(int64_t current_time_ms, std::function<size_t(uint8_t * buffer, size_t max_size)> write_func);


