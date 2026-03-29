#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <nvs_flash.h>
#include <esp_task_wdt.h>
#include "esp_log.h"

#include "display.hpp"
#include "io.hpp"
#include "wifi.hpp"
#include "https_server.hpp"

#include "hex_mini_drive/interface.hpp"

static const char* TAG = "main";


int loop_number = 0;

void initialise_nvs_flash() {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
}

void blink_status_led(int64_t current_time_ms) {
  static int64_t last_blink = 0;
  static bool status_state = false;

  if (current_time_ms - last_blink > 1000) {
    status_state = !status_state;
    uint32_t duty = status_state ? 128 : 32;
    set_status_led_brightness(duty);
    last_blink = current_time_ms;
  }
}

void wifi_connected(){
  ESP_LOGI(TAG, "Wi-Fi connected callback called");

  setup_server([](uint8_t* buffer, size_t size) {
    ESP_LOGI(TAG, "Received WebSocket message of size %d", size);
    
    ws_send_binary(buffer, size);
  });
}

void receive_message(uint8_t* buffer, size_t size) {
  ESP_LOGI(TAG, "Received WebSocket message of size %d", size);
  
  ws_send_binary(buffer, size);
}

static inline int64_t get_ms_time() {
  return esp_timer_get_time() / 1000; // Convert to milliseconds
}


void main_task(void *arg) {
  // Register this task with the watchdog timer.
  esp_task_wdt_add(NULL);

  //Initialize NVS, used for WiFi provisioning and other settings.
  initialise_nvs_flash();

  
  // Setup everything
  setup_status_led();
  setup_spi_busses();
  setup_display_and_lvgl();

  update_display(get_ms_time());

  setup_output_pins();
  setup_shift_register_bank2();

  set_shift_register_bank2((uint8_t[7]){
    0b0000'0000,
    0, 
    0, 
    0, 
    0, 
    0, 
    0,
  });

  enable_shift_register_outputs();

  // BIG oof, this burns the circuit.
  // connect_vbus_to_vcc();

  start_wifi_provisioning(wifi_connected);
  
  while (1) {
    // Blink the built-in status LED to show the system is running
    int64_t current_time_ms = get_ms_time();

    // Handle displaya and LVGL tasks.
    update_display(current_time_ms);
    
    blink_status_led(current_time_ms);

    // We need to reset the watchdog timer regularly.
    esp_task_wdt_reset();
    
    // Must sleep at last 1ms to let the idle task handle the watchdog and other housekeeping.
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  
}

extern "C" void app_main() {
  xTaskCreate(main_task, "main_task", 16384 , NULL, 5, NULL);
}
