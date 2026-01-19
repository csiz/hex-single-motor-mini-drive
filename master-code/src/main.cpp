#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <nvs_flash.h>

#include "display.hpp"
#include "io.hpp"
#include "wifi.hpp"

static const char* TAG = "main";


int loop_number = 0;

void main_task(void *arg) {
  //Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  dpp_enrollee_init();

  // Setup everything
  setup_status_led();
  setup_display_and_lvgl();
  
  int64_t lastBlink = 0;
  bool statusState = false;
  
  while (1) {
    // Handle displaya and LVGL tasks.
    update_display();
    
    // Blink the built-in status LED to show the system is running
    int64_t currentTime = esp_timer_get_time() / 1000; // Convert to milliseconds
    
    if (currentTime - lastBlink > 1000) {
      statusState = !statusState;
      uint32_t duty = statusState ? 128 : 32;
      set_status_led_brightness(duty);
      lastBlink = currentTime;
    }
    
    vTaskDelay(pdMS_TO_TICKS(5)); // LVGL needs frequent updates
  }

  
}

extern "C" void app_main() {
  xTaskCreate(main_task, "main_task", 16384 , NULL, 5, NULL);
}
