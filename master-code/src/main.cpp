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

void main_task(void *arg) {
  // Register this task with the watchdog timer.
  esp_task_wdt_add(NULL);

  //Initialize NVS, used for WiFi provisioning and other settings.
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  
  // Setup everything
  setup_status_led();
  setup_spi_busses();
  setup_display_and_lvgl();

  start_wifi_provisioning();
  
  setup_server([](uint8_t* buffer, size_t size) {
    ESP_LOGI(TAG, "Received WebSocket message of size %d", size);
    
    ws_send_binary(buffer, size);
  });
  
  int64_t last_blink = 0;
  bool status_state = false;
  
  while (1) {
    // Blink the built-in status LED to show the system is running
    int64_t current_time = esp_timer_get_time() / 1000; // Convert to milliseconds

    // Handle displaya and LVGL tasks.
    update_display(current_time);
    
    
    if (current_time - last_blink > 1000) {
      status_state = !status_state;
      uint32_t duty = status_state ? 128 : 32;
      set_status_led_brightness(duty);
      last_blink = current_time;
    }

    // We need to reset the watchdog timer regularly.
    esp_task_wdt_reset();
    
    // Must sleep at last 1ms to let the idle task handle the watchdog and other housekeeping.
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  
}

extern "C" void app_main() {
  xTaskCreate(main_task, "main_task", 16384 , NULL, 5, NULL);
}
