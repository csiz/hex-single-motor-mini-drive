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
#include <lvgl.h>

#include <esp_lcd_panel_st7789.h>

static const char* TAG = "main";

// LVGL flush callback
void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    
    // Copy color data to the display
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map));
    
    // Tell LVGL that we are ready
    lv_disp_flush_ready(drv);
}

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

// LVGL display buffer
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[display_width * display_height];
static lv_disp_drv_t disp_drv;

void setup_display_and_lvgl() {
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

  ESP_LOGI(TAG, "Initializing ST7789 Display with LVGL...");

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
  ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));
  ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, true));
  
  // Turn on display
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
  
  // Setup the status pin to half brightness
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 32));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
  
  ESP_LOGI(TAG, "Display hardware initialized, setting up LVGL...");
  
  // Initialize LVGL
  lv_init();
  
  // Initialize the display buffer
  lv_disp_draw_buf_init(&draw_buf, buf1, NULL, display_width * display_height);
  
  // Initialize the display driver
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = display_width;
  disp_drv.ver_res = display_height;
  disp_drv.flush_cb = lvgl_flush_cb;
  disp_drv.draw_buf = &draw_buf;
  disp_drv.user_data = panel_handle;
  lv_disp_drv_register(&disp_drv);
  
  ESP_LOGI(TAG, "LVGL initialized successfully!");
  
  // Create hello world label
  lv_obj_t * label = lv_label_create(lv_scr_act());
  lv_label_set_text(label, "Hello World!");
  lv_obj_set_style_text_color(label, lv_color_white(), LV_PART_MAIN);
  lv_obj_set_style_text_font(label, &lv_font_montserrat_24, LV_PART_MAIN);
  lv_obj_center(label);
  
  // Set background color to blue
  lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x001F), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, LV_PART_MAIN);
  
  ESP_LOGI(TAG, "Hello World label created!");
}

int loop_number = 0;

void main_task(void *arg) {
  // Setup everything
  setup_display_and_lvgl();
  
  int64_t lastBlink = 0;
  bool statusState = false;
  
  while (1) {
    // Handle LVGL tasks
    lv_timer_handler();
    
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
    vTaskDelay(pdMS_TO_TICKS(5)); // LVGL needs frequent updates
  }
}

extern "C" void app_main() {
  xTaskCreate(main_task, "main_task", 4096, NULL, 5, NULL);
}
