#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>

#include "display.hpp"
#include "io.hpp"

static const char* TAG = "main";


int loop_number = 0;

void main_task(void *arg) {
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
  xTaskCreate(main_task, "main_task", 4096, NULL, 5, NULL);
}
