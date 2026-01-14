#include <stdio.h>
#include <driver/spi_master.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_ops.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>

static const char* TAG = "main";

const gpio_num_t status_pin = GPIO_NUM_35;
const int display_width = 240;
const int display_height = 280;

const gpio_num_t display_mosi = GPIO_NUM_11; // MOSI pin
const gpio_num_t display_miso = GPIO_NUM_13; // MISO pin
const gpio_num_t display_sclk = GPIO_NUM_12; // CLK pin  
const gpio_num_t display_cs   = GPIO_NUM_37; // Chip select control pin
const gpio_num_t display_dc   = GPIO_NUM_38; // Data Command control pin
const gpio_num_t display_rst  = GPIO_NUM_18; // Reset pin

// ESP-IDF LCD handles
esp_lcd_panel_handle_t panel_handle = NULL;
esp_lcd_panel_io_handle_t io_handle = NULL;

void setup_status_led() {
  // Configure LEDC for PWM on status pin
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 1000,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  ledc_channel_config_t ledc_channel = {
    .gpio_num = status_pin,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .timer_sel = LEDC_TIMER_0,
    .duty = 2,
    .hpoint = 0
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

  ESP_LOGI(TAG, "Initializing ST7789 Display with ESP-IDF LCD...");

  // Configure SPI bus
  spi_bus_config_t buscfg = {
    .mosi_io_num = display_mosi,
    .miso_io_num = display_miso,
    .sclk_io_num = display_sclk,
    .quadwp_io_num = GPIO_NUM_NC,
    .quadhd_io_num = GPIO_NUM_NC,
    .max_transfer_sz = display_width * display_height * sizeof(uint16_t),
  };
  
  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

  // Configure panel I/O
  esp_lcd_panel_io_spi_config_t io_config = {
    .cs_gpio_num = display_cs,
    .dc_gpio_num = display_dc,
    .spi_mode = 0,
    .pclk_hz = 40 * 1000 * 1000,
    .trans_queue_depth = 10,
    .lcd_cmd_bits = 8,
    .lcd_param_bits = 8,
  };
  
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io_handle));

  // Configure ST7789 panel
  esp_lcd_panel_dev_config_t panel_config = {
    .reset_gpio_num = display_rst,
    .color_space = ESP_LCD_COLOR_SPACE_BGR,
    .bits_per_pixel = 16,
  };
  
  ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
  
  // Initialize the panel
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
  
  // Set orientation
  ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
  ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, true));
  
  // Turn on display
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
  
  // Setup the status pin to half brightness
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 32));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
  
  // Clear the screen with blue background
  uint16_t blue_color = 0x001F; // Blue in RGB565
  for(int y = 0; y < display_height; y++) {
    for(int x = 0; x < display_width; x++) {
      ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, x, y, x+1, y+1, &blue_color));
    }
  }
  ESP_LOGI(TAG, "Display initialized with ESP-IDF LCD!");
}

int loop_number = 0;

void main_task(void *arg) {
  // Setup everything
  setup_status_led();
  
  int64_t lastBlink = 0;
  bool statusState = false;
  
  while (1) {
    // Blink the built-in status LED to show the system is running
    int64_t currentTime = esp_timer_get_time() / 1000; // Convert to milliseconds
    
    if (currentTime - lastBlink > 2000) {
      statusState = !statusState;
      uint32_t duty = statusState ? 128 : 32;
      ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
      ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
      lastBlink = currentTime;
    }
    
    // ESP_LOGI(TAG, "Main loop iteration: %d", loop_number++);
    vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent watchdog issues
  }
}

extern "C" void app_main() {
  xTaskCreate(main_task, "main_task", 4096, NULL, 5, NULL);
}
