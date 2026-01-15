#include "display.hpp"
#include "io.hpp"

#include <driver/spi_master.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_ops.h>
#include <lvgl.h>
#include <esp_lcd_panel_st7789.h>
#include "esp_log.h"

static const char* TAG = "display";


// ESP-IDF LCD handles
esp_lcd_panel_handle_t panel_handle = NULL;
esp_lcd_panel_io_handle_t io_handle = NULL;

// LVGL display buffer
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[display_width * display_height];
static lv_disp_drv_t disp_drv;

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

void setup_display_and_lvgl() {
  
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

void update_display() {
    // This function can be used to trigger LVGL tasks if needed
    lv_timer_handler();
}