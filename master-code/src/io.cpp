#include "io.hpp"

#include <esp_log.h>

#include <driver/spi_master.h>
#include <rom/ets_sys.h>

static const char* TAG = "io";

bool vbus_connected_to_vcc = false;
bool battery_connected_to_vcc = false;

spi_device_handle_t shift_register_bank1_device = nullptr;
spi_device_handle_t shift_register_bank2_device = nullptr;


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


  ESP_LOGI(TAG, "Initializing SPI3...");

  buscfg.mosi_io_num = spi3_mosi_pin;
  buscfg.miso_io_num = spi3_miso_pin;
  buscfg.sclk_io_num = spi3_sclk_pin;
  buscfg.max_transfer_sz = 4096; // Shift registers only need small transfers.

  ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
}

void setup_output_pins(){
  disable_shift_register_outputs();

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

void enable_shift_register_outputs(){
  ESP_LOGI(TAG, "Enabling shift register outputs...");
  gpio_set_level(shift_oe, 0);
}

void disable_shift_register_outputs(){
  ESP_LOGI(TAG, "Disabling shift register outputs...");
  gpio_set_level(shift_oe, 1);
}

void setup_shift_register_bank1(){
  // Bank 1 is latched by shift_bank1_latch, and connected to SPI3.
  spi_device_interface_config_t devcfg = {};
  devcfg.clock_speed_hz = 10 * 1000 * 1000; // Clock at 10 MHz
  devcfg.mode = 0;                          // SPI mode 0
  devcfg.spics_io_num = GPIO_NUM_NC;
  devcfg.queue_size = 1;
  ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg, &shift_register_bank1_device));
}

bool set_shift_register_bank1(uint8_t data[3]){
  // Bank 1 is latched by shift_bank1_latch, and connected to SPI3.
  spi_transaction_t t = {};
  t.length = 3 * 8;
  t.tx_buffer = data;
  if(spi_device_transmit(shift_register_bank1_device, &t) != ESP_OK) return false;
  // Latch the data into the output registers.
  gpio_set_level(shift_bank1_latch, 1);
  gpio_set_level(shift_bank1_latch, 0);
  return true;
}

bool motor_spi_transaction(size_t motor_index, uint8_t* write_data, uint8_t* read_data, size_t length){
  if (length < 3) return false;

  // Assert chip select for the target motor (active low).
  // Bank 1 has 24 bits (3 bytes), one per motor. All bits high = all deasserted.
  uint8_t cs_data[3] = {0xFF, 0xFF, 0xFF};
  // Clear the bit corresponding to motor_index to assert CS.
  cs_data[2 - (motor_index / 8)] &= ~(1 << (motor_index % 8));
  // Bank 1 is latched by shift_bank1_latch, and connected to SPI3.
  if(!set_shift_register_bank1(cs_data)) return false;

  // Perform the SPI transaction on SPI3 (shared bus with shift registers).
  spi_transaction_t t = {};
  t.length = length * 8;
  t.tx_buffer = write_data;
  t.rx_buffer = read_data;

  // Last 3 bytes must set the shift register outputs high to deassert the nss line, yeah there's circuit error here...
  write_data[length - 1] = 0xFF;
  write_data[length - 2] = 0xFF;
  write_data[length - 3] = 0xFF;

  const esp_err_t transmit_result = spi_device_transmit(shift_register_bank1_device, &t);
  if(transmit_result != ESP_OK){
    ESP_LOGE(TAG, "SPI transaction failed for motor %d, err %d", motor_index, transmit_result);
    return false;
  }

  // Latch the data into the output registers.
  gpio_set_level(shift_bank1_latch, 1);
  gpio_set_level(shift_bank1_latch, 0);
  return true;
}




void setup_shift_register_bank2(){
  // Bank 2 is latched by shift_bank2_latch, and connected to SPI3.
  spi_device_interface_config_t devcfg = {};
  devcfg.clock_speed_hz = 10 * 1000 * 1000; // Clock at 10 MHz
  devcfg.mode = 0;                          // SPI mode 0
  devcfg.spics_io_num = GPIO_NUM_NC;
  devcfg.queue_size = 1;
  ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &shift_register_bank2_device));
}

bool set_shift_register_bank2(uint8_t data[7]){
  // Bank 2 is latched by shift_bank2_latch, and connected to SPI3.
  spi_transaction_t t = {};
  t.length = 7 * 8;
  t.tx_buffer = data;
  if(spi_device_transmit(shift_register_bank2_device, &t) != ESP_OK) return false;
  // Latch the data into the output registers.
  gpio_set_level(shift_bank2_latch, 1);
  gpio_set_level(shift_bank2_latch, 0);
  return true;
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