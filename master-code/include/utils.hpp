#pragma once

#include <cstdint>
#include <functional>

#include <freertos/FreeRTOS.h>
#include <freertos/message_buffer.h>

#include <esp_timer.h>
#include <esp_log.h>

static inline int64_t get_ms_time() {
  return esp_timer_get_time() / 1000; // Convert to milliseconds
}

static inline std::function<void(const int64_t current_time_ms)> interval(const int64_t interval_ms, std::function<void()> callback) {
  int64_t last_time = 0;
  return [interval_ms, callback, last_time](const int64_t current_time) mutable {
    if (current_time - last_time >= interval_ms) {
      callback();
      last_time = current_time;
    }
  };
}

struct RunningStats {
  int loop_number = 0;
  float loop_frequency = 0.0;
  int updates_this_ms = 0;
  int64_t time_ms;

  RunningStats() {
    time_ms = get_ms_time();
  }

  void update(){
    const int64_t current_time_ms = get_ms_time();

    const int64_t loop_duration = current_time_ms - time_ms;

    time_ms = current_time_ms;

    // Calculate main loop frequency
    if (loop_duration == 0) {
      updates_this_ms += 1;
    } else {
      loop_frequency = loop_frequency * 0.99 + 0.01 * (updates_this_ms + 1) / (loop_duration / 1000.0);
      updates_this_ms = 0;
    }
  }
};

template<size_t N>
struct MessageBuffer {
  uint8_t data[N];
  StaticMessageBuffer_t storage;
  MessageBufferHandle_t handle;
  size_t discarded = 0;

  MessageBuffer() {
    handle = xMessageBufferCreateStatic(N, data, &storage);
    if (handle == nullptr) {
      ESP_LOGE("utils", "Failed to create message buffer of size %d", N);
      abort();
    }
  }

  size_t available() {
    return xMessageBufferSpacesAvailable(handle);
  }

  size_t send(uint8_t const* buffer, size_t size) {
    if (size > N) {
      ESP_LOGW("utils", "Attempted to send message of size %d to buffer of size %d, dropping message", size, N);
      return 0;
    }
    size_t sent = xMessageBufferSend(handle, buffer, size, 0);
    if (sent != size) {
      ESP_LOGW("utils", "Failed to send full message to buffer (sent %d/%d bytes)", sent, size);
    }
    return sent;
  }

  size_t receive(uint8_t * buffer, size_t size) {
    if (size > N) size = N;
    size_t received = xMessageBufferReceive(handle, buffer, size, 0);
    
    return received;
  }

  bool is_empty() {
    return xMessageBufferIsEmpty(handle) == pdTRUE;
  }
};