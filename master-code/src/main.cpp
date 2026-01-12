#include <Arduino.h>
#include <driver/spi_master.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_ops.h>
#include <driver/gpio.h>

const int status_pin = 35;
const int display_width = 240;
const int display_height = 280;

const int display_mosi = 11; // MOSI pin
const int display_miso = 13; // MISO pin
const int display_sclk = 12; // CLK pin  
const int display_cs   = 37; // Chip select control pin
const int display_dc   = 38; // Data Command control pin
const int display_rst  = 18; // Reset pin

// ESP-IDF LCD handles
esp_lcd_panel_handle_t panel_handle = NULL;
esp_lcd_panel_io_handle_t io_handle = NULL;

void setup(){
  // Setup the status pin to half brightness 0 to 255 pwm mode.
  pinMode(status_pin, OUTPUT);
  analogWrite(status_pin, 2);

  // Initialize serial for debugging
  Serial.begin(115200);
  Serial.println("Initializing ST7789 Display with ESP-IDF LCD...");

  // Configure SPI bus
  spi_bus_config_t buscfg = {
    .mosi_io_num = display_mosi,
    .miso_io_num = display_miso,
    .sclk_io_num = display_sclk,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
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
  analogWrite(status_pin, 32);
  
  // Clear the screen with blue background
  uint16_t blue_color = 0x001F; // Blue in RGB565
  for(int y = 0; y < display_height; y++) {
    for(int x = 0; x < display_width; x++) {
      ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, x, y, x+1, y+1, &blue_color));
    }
  }
  Serial.println("Display initialized with ESP-IDF LCD!");
}

int loop_number = 0;

void loop(){
  // Blink the built-in status LED to show the system is running
  static unsigned long lastBlink = 0;
  static bool statusState = false;
  
  if (millis() - lastBlink > 1000) {
    statusState = !statusState;
    analogWrite(status_pin, statusState ? 128 : 32);
    lastBlink = millis();
  }
  
  // Serial.println("Main loop iteration: " + String(loop_number++));
}
