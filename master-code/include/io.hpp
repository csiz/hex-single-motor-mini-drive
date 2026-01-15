#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"

const gpio_num_t status_pin = GPIO_NUM_35;
const gpio_num_t display_mosi = GPIO_NUM_11; // MOSI pin
const gpio_num_t display_miso = GPIO_NUM_13; // MISO pin
const gpio_num_t display_sclk = GPIO_NUM_12; // CLK pin  
const gpio_num_t display_cs   = GPIO_NUM_37; // Chip select control pin
const gpio_num_t display_dc   = GPIO_NUM_38; // Data Command control pin
const gpio_num_t display_rst  = GPIO_NUM_18; // Reset pin

void setup_status_led();

void set_status_led_brightness(uint8_t brightness);