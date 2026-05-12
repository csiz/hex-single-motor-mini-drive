#include <stdio.h>

#include <functional>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/message_buffer.h>
#include <esp_timer.h>
#include <nvs_flash.h>
#include <esp_task_wdt.h>
#include <esp_log.h>

#include "display.hpp"
#include "io.hpp"
#include "wifi.hpp"
#include "https_server.hpp"

#include "hex_mini_drive_interface.hpp"

static const char* TAG = "main";

constexpr size_t spi_chunk_length = hex_mini_drive::MAX_MESSAGE_SIZE+3;

// FreeRTOS message buffer for motor messages
MessageBufferHandle_t motor_outgoing_messages = nullptr;
constexpr size_t motor_message_buffer_size = 4096;

uint8_t motor_message_buffer_data[motor_message_buffer_size];
StaticMessageBuffer_t motor_message_buffer_storage;

hex_mini_drive::ConsistentOverheadByteStuffing<spi_chunk_length> cobs_encoder = {};

uint8_t zero_buffer[spi_chunk_length] = {0}; 

static inline int64_t get_ms_time() {
  return esp_timer_get_time() / 1000; // Convert to milliseconds
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



void initialise_nvs_flash() {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
}

std::function<void(const int64_t current_time_ms)> interval(const int64_t interval_ms, std::function<void()> callback) {
  int64_t last_time = 0;
  return [interval_ms, callback, last_time](const int64_t current_time) mutable {
    if (current_time - last_time >= interval_ms) {
      callback();
      last_time = current_time;
    }
  };
}



const auto blink_status_led = interval(1000, []() {
  static bool status_state = false;

  status_state = !status_state;
  uint32_t duty = status_state ? 128 : 32;
  set_status_led_brightness(duty);
});

void wifi_connected(){
  ESP_LOGI(TAG, "Wi-Fi connected callback called");

  setup_server([](uint8_t* buffer, size_t size) {
    ESP_LOGI(TAG, "Received WebSocket message of size %d", size);
    if (motor_outgoing_messages == nullptr) {
      ESP_LOGE(TAG, "motor_outgoing_messages buffer not initialized!");
      return;
    }
    if (size > hex_mini_drive::MAX_MESSAGE_SIZE) {
      ESP_LOGW(TAG, "Received message size %d exceeds maximum of %d, dropping message", size, hex_mini_drive::MAX_MESSAGE_SIZE);
      return;
    }

    size_t sent = xMessageBufferSend(motor_outgoing_messages, buffer, size, 0);

    if (sent != size) {
      ESP_LOGW(TAG, "Failed to send full message to buffer (sent %d/%d bytes)", sent, size);
    }
  });
}




void motor_update(int64_t current_time_ms) {
  if (motor_outgoing_messages == nullptr) {
    ESP_LOGE(TAG, "motor_outgoing_messages buffer not initialized!");
    return;
  }

  uint8_t read_buffer[spi_chunk_length] = {0};

  // Check if there's a message in the buffer
  if (xMessageBufferIsEmpty(motor_outgoing_messages) == pdFALSE) {
    uint8_t message_buffer[hex_mini_drive::MAX_MESSAGE_SIZE];
    size_t received = xMessageBufferReceive(motor_outgoing_messages, message_buffer, sizeof(message_buffer), 0);
    
    if (received == 0) {
      ESP_LOGW(TAG, "Failed to receive message from buffer");
      return;
    }

    cobs_encoder.encode_message(message_buffer, received);

    motor_spi_transaction(0, cobs_encoder.encoding_buffer, read_buffer, spi_chunk_length);

    cobs_encoder.encode_reset();
  } else {
    motor_spi_transaction(0, zero_buffer, read_buffer, spi_chunk_length);
  }

  cobs_encoder.decode_chunk(read_buffer, spi_chunk_length, [](uint8_t* buffer, size_t size) {
    ws_send_binary(buffer, size);
  });
}

void core_0_task(void *arg) {
  // Register this task with the watchdog timer.
  esp_task_wdt_add(NULL);
  
  start_wifi_provisioning(wifi_connected);
  
  auto stats = RunningStats();

  auto log_loop_frequency = interval(5000, [&stats]() {
    ESP_LOGI(TAG, "Core 0 loop frequency: %.2f Hz", stats.loop_frequency);
  });

  while (1) {
    stats.update();

    // We need to reset the watchdog timer regularly.
    esp_task_wdt_reset();

    log_loop_frequency(stats.time_ms);

    // Sleep for 1 ms.
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void core_1_task(void *arg) {
  // Register this task with the watchdog timer.
  esp_task_wdt_add(NULL);


  // Setup everything
  setup_status_led();
  setup_spi_busses();
  setup_display_and_lvgl();

  update_display(get_ms_time());

  setup_output_pins();
  setup_shift_register_bank1();
  setup_shift_register_bank2();

  uint8_t bank2_defaults[7] = {
    0b0000'0000,
    0, 
    0, 
    0, 
    0, 
    0, 
    0b0000'0010, // Enable first motor by setting the reset pin high.
  };

  set_shift_register_bank2(bank2_defaults);

  uint8_t bank1_defaults[3] = {
    0xFF,
    0xFF, 
    0xFF, 
  };

  set_shift_register_bank1(bank1_defaults);

  enable_shift_register_outputs();

  // BIG oof, this burns the circuit, if the USB output flags are set wrong.
  connect_vbus_to_vcc();

  auto stats = RunningStats();

  auto log_loop_frequency = interval(5000, [&stats]() {
    ESP_LOGI(TAG, "Core 1 loop frequency: %.2f Hz", stats.loop_frequency);
  });

  while (1) {
    stats.update();

    // We need to reset the watchdog timer regularly.
    esp_task_wdt_reset();
    
    // Blink the built-in status LED to show the system is running
    blink_status_led(stats.time_ms);

    // Handle displaya and LVGL tasks.
    update_display(stats.time_ms);

    log_loop_frequency(stats.time_ms);

    motor_update(stats.time_ms);

    // Sleep for 1ms.
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  
}

extern "C" void app_main() {
  //Initialize NVS, used for WiFi provisioning and other settings.
  initialise_nvs_flash();

  // Create message buffer for motor outgoing messages
  motor_outgoing_messages = xMessageBufferCreateStatic(
    motor_message_buffer_size,
    motor_message_buffer_data,
    &motor_message_buffer_storage);

  if (motor_outgoing_messages == nullptr) {
    ESP_LOGE(TAG, "Failed to create motor_outgoing_messages buffer!");
  }

  // Start main task pinned to core 1 as core 0 is used by Wi-Fi and other system tasks.
  xTaskCreatePinnedToCore(core_1_task, "core_1_task", 16384 , NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(core_0_task, "core_0_task", 16384 , NULL, 5, NULL, 0);
}
