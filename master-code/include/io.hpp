#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"

const gpio_num_t status_led_pin = GPIO_NUM_35;

const gpio_num_t display_mosi = GPIO_NUM_11; // MOSI pin
const gpio_num_t display_miso = GPIO_NUM_13; // MISO pin
const gpio_num_t display_sclk = GPIO_NUM_12; // CLK pin  
const gpio_num_t display_cs   = GPIO_NUM_37; // Chip select control pin
const gpio_num_t display_dc   = GPIO_NUM_38; // Data Command control pin
const gpio_num_t display_rst  = GPIO_NUM_18; // Reset pin

const gpio_num_t shift_oe = GPIO_NUM_47;

const gpio_num_t shift_bank1_data_in = GPIO_NUM_9;
const gpio_num_t shift_bank1_clock = GPIO_NUM_21;
const gpio_num_t shift_bank1_latch = GPIO_NUM_46;

const gpio_num_t shift_bank2_data_in = GPIO_NUM_11;
const gpio_num_t shift_bank2_clock = GPIO_NUM_12;
const gpio_num_t shift_bank2_latch = GPIO_NUM_45;

const gpio_num_t vbus_voltage_pin = GPIO_NUM_4;
const gpio_num_t vbus_current_pin = GPIO_NUM_5;
const gpio_num_t connect_vbus_pin = GPIO_NUM_6;
const gpio_num_t connect_battery_pin = GPIO_NUM_7;
const gpio_num_t battery_current_pin = GPIO_NUM_15;
const gpio_num_t battery_voltage_pin = GPIO_NUM_16;

const gpio_num_t battery_1_pin = GPIO_NUM_17;
const gpio_num_t battery_2_pin = GPIO_NUM_48;

const gpio_num_t sensor_current_pin = GPIO_NUM_8;

const gpio_num_t can_tx_pin = GPIO_NUM_1;
const gpio_num_t can_rx_pin = GPIO_NUM_2;

const gpio_num_t touch_int_pin = GPIO_NUM_36;

const gpio_num_t boot0_pin = GPIO_NUM_0;

const gpio_num_t i2c1_sda_pin = GPIO_NUM_42;
const gpio_num_t i2c1_scl_pin = GPIO_NUM_41;
const gpio_num_t i2c2_scl_pin = GPIO_NUM_40;
const gpio_num_t i2c2_sda_pin = GPIO_NUM_39;

const gpio_num_t usb_dm_pin = GPIO_NUM_19;
const gpio_num_t usb_dp_pin = GPIO_NUM_20;

const gpio_num_t sd_cmd_pin = GPIO_NUM_11;
const gpio_num_t sd_clk_pin = GPIO_NUM_12;
const gpio_num_t sd_data0_pin = GPIO_NUM_13;
const gpio_num_t sd_cs_pin = GPIO_NUM_10;

const gpio_num_t spi3_miso_pin = GPIO_NUM_14;
const gpio_num_t spi3_mosi_pin = GPIO_NUM_9;
const gpio_num_t spi3_sclk_pin = GPIO_NUM_21;

const gpio_num_t spi2_mosi_pin = GPIO_NUM_11;
const gpio_num_t spi2_miso_pin = GPIO_NUM_13;
const gpio_num_t spi2_sclk_pin = GPIO_NUM_12;

const gpio_num_t reference_pin = GPIO_NUM_3;

void setup_status_led();

void set_status_led_brightness(uint8_t brightness);