#include "io.hpp"

#include <esp_log.h>

#include <driver/spi_master.h>

static const char* TAG = "io";

bool vbus_connected_to_vcc = false;
bool battery_connected_to_vcc = false;


void setup_status_led(){
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
    .gpio_num = status_led_pin,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .timer_sel = LEDC_TIMER_0,
    .duty = 2,
    .hpoint = 0
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void set_status_led_brightness(uint8_t brightness){
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, brightness));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

void setup_spi_busses(){
  ESP_LOGI(TAG, "Initializing SPI2...");

  // Configure SPI bus
  spi_bus_config_t buscfg = {
    .mosi_io_num = spi2_mosi_pin,
    .miso_io_num = spi2_miso_pin,
    .sclk_io_num = spi2_sclk_pin,
    .quadwp_io_num = GPIO_NUM_NC,
    .quadhd_io_num = GPIO_NUM_NC,
    .max_transfer_sz = display_width * display_height * sizeof(uint16_t),
  };
  
  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

  // ESP_LOGI(TAG, "Initializing SPI3...");
}

void setup_output_pins(){
  // Configure shift register output enable pin (active low)
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;


  // These gpio pins set flags to other components.
  const gpio_num_t output_pins[] = {
    shift_oe,
    shift_bank1_latch,
    shift_bank2_latch,
    connect_vbus_pin,
    connect_battery_pin
  };
  for (gpio_num_t pin : output_pins){
    io_conf.pin_bit_mask = (1ULL << pin);
    ESP_ERROR_CHECK(gpio_config(&io_conf));
  }
}

void connect_battery_to_vcc(){
  if (vbus_connected_to_vcc) disconnect_vbus_from_vcc();
  ESP_LOGI(TAG, "Connecting battery to VCC...");
  gpio_set_level(connect_battery_pin, 1);
  battery_connected_to_vcc = true;
}

void disconnect_battery_from_vcc(){
  ESP_LOGI(TAG, "Disconnecting battery from VCC...");
  gpio_set_level(connect_battery_pin, 0);
  battery_connected_to_vcc = false;
}

void connect_vbus_to_vcc(){
  if (battery_connected_to_vcc) disconnect_battery_from_vcc();
  ESP_LOGI(TAG, "Connecting VBUS to VCC...");
  gpio_set_level(connect_vbus_pin, 1);
  vbus_connected_to_vcc = true;
}

void disconnect_vbus_from_vcc(){
  ESP_LOGI(TAG, "Disconnecting VBUS from VCC...");
  gpio_set_level(connect_vbus_pin, 0);
  vbus_connected_to_vcc = false;
}