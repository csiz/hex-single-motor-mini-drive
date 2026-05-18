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
#include <esp_err.h>

#include "display.hpp"
#include "io.hpp"
#include "wifi.hpp"
#include "https_server.hpp"
#include "utils.hpp"

#include "hex_mini_drive_interface.hpp"

static const char* TAG = "main";


using hex_mini_drive::SPI_TRANSACTION_SIZE;
using hex_mini_drive::MAX_MESSAGE_SIZE;

constexpr size_t max_buffer_size = 8 * 1024; // 8 KB buffer size for motor messages, adjust as needed


// Create an FreeRTOS byte stream buffer for data received from the motor to be sent via wifi.
MessageBuffer<max_buffer_size> received_from_motor;

// FreeRTOS message buffer for motor messages
MessageBuffer<max_buffer_size> sending_to_motor;


uint8_t zero_buffer[SPI_TRANSACTION_SIZE] = {0};

volatile bool wifi_is_connected = false;


const auto blink_status_led = interval(1000, []() {
  static bool status_state = false;

  status_state = !status_state;
  uint32_t duty = status_state ? 128 : 32;
  set_status_led_brightness(duty);
});


void wifi_connected(){
  ESP_LOGI(TAG, "Wi-Fi connected callback called");

  setup_server(/* core_id = */ 0, [](uint8_t* buffer, size_t size) {
    if (size > MAX_MESSAGE_SIZE) {
      ESP_LOGW(TAG, "Received message size %d exceeds maximum of %d, dropping message", size, MAX_MESSAGE_SIZE);
      return;
    }

    size_t sent = sending_to_motor.send(buffer, size);

    if (sent != size) {
      ESP_LOGW(TAG, "Failed to send full message to buffer (sent %d/%d bytes)", sent, size);
    }
  });

  wifi_is_connected = true;
}


// Our messages are 0 delimited, with extra 0s allowed. We want to compact them to a
// series of messages delimited by a single 0. We'll count these bytes as data received.
size_t compactify_data_buffer(uint8_t * buffer, size_t size) {
  // The first byte is correct even as a 0. Start pruning from the second byte.
  size_t write_index = 1;
  
  for (size_t read_index = 1; read_index < size; ++read_index) {
    if (buffer[read_index] != 0 or buffer[read_index - 1] != 0) {
      buffer[write_index++] = buffer[read_index];
    }
  }
  return write_index;
}

void motor_update(int64_t current_time_ms) {
  uint8_t write_buffer[SPI_TRANSACTION_SIZE] = {0};
  uint8_t read_buffer[SPI_TRANSACTION_SIZE] = {0};

  // Check if there's a message in the buffer
  if (not sending_to_motor.is_empty()) {
    const auto bytes_to_send = sending_to_motor.receive(write_buffer, MAX_MESSAGE_SIZE);
    if (bytes_to_send == 0) {
      ESP_LOGE(TAG, "Failed to receive message from buffer");
      return abort();
    }
  }

  // Run the spi transaction with motor data, or zeroes.
  motor_spi_transaction(0, write_buffer, read_buffer, SPI_TRANSACTION_SIZE);

  bool all_ff = true;
  for (size_t i = 0; i < SPI_TRANSACTION_SIZE; ++i) {
    if (read_buffer[i] != 0xFF) {
      all_ff = false;
      break;
    }
  }
  if (all_ff) {
    // Only bits of 1 is probably the line being held low at reset.
    return;
  }

  size_t read_size = compactify_data_buffer(read_buffer, SPI_TRANSACTION_SIZE);

  // Check if the message is bigger than a single 0.
  if (read_size <= 1) {
    return;
  }

  if (received_from_motor.available() < read_size) {
    received_from_motor.discarded += read_size;
    ESP_LOGW(TAG, "Discarded %d bytes from motor buffer due to insufficient space", read_size);
    return;
  }

  const size_t queued_size = received_from_motor.send(read_buffer, read_size);
  if (queued_size != read_size) {
    ESP_LOGE(TAG, "Failed to send full message to received_from_motor (sent less than %d bytes)", read_size);
    return abort();
  }
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

    if (wifi_is_connected) {
      update_server(stats.time_ms, [](uint8_t * buffer, size_t max_size) -> size_t {
        size_t total_to_send = 0;
        // This lambda is called by the server when it's ready to send data.
        while(total_to_send < max_size) {
          size_t bytes_received = received_from_motor.receive(buffer + total_to_send, max_size - total_to_send);
          if (bytes_received == 0) break;
          total_to_send += bytes_received;
        }
        return total_to_send;
      });
    }

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

  auto log_loop_frequency = interval(10000, [&stats]() {
    ESP_LOGI(TAG, "Core 1 loop frequency: %.2f Hz", stats.loop_frequency);
  });

  while (1) {
    stats.update();

    // // We need to reset the watchdog timer regularly.
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
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Start main task pinned to core 1 as core 0 is used by Wi-Fi and other system tasks.
  xTaskCreatePinnedToCore(core_1_task, "core_1_task", 16384 , NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(core_0_task, "core_0_task", 16384 , NULL, 1, NULL, 0);
}
