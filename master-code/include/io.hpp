#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"

// Red LED on the board, used as a status indicator.
const gpio_num_t status_led_pin = GPIO_NUM_35;

// SPI pins and related devices
// ----------------------------

// SPI3 data input.
const gpio_num_t spi3_miso_pin = GPIO_NUM_14;
// SPI3 data output.
const gpio_num_t spi3_mosi_pin = GPIO_NUM_9;
// SPI3 clock pin.
const gpio_num_t spi3_sclk_pin = GPIO_NUM_21;

// SPI2 data input.
const gpio_num_t spi2_miso_pin = GPIO_NUM_13;
// SPI2 data output.
const gpio_num_t spi2_mosi_pin = GPIO_NUM_11;
// SPI2 clock pin.
const gpio_num_t spi2_sclk_pin = GPIO_NUM_12;

// ### Display pins

// Display data pin (MOSI).
const gpio_num_t display_mosi = spi2_mosi_pin;
// Display clock pin.
const gpio_num_t display_sclk = spi2_sclk_pin;
// Display chip select pin.
const gpio_num_t display_cs   = GPIO_NUM_37;
// Display data/command pin.
const gpio_num_t display_dc   = GPIO_NUM_38;
// Display reset pin.
const gpio_num_t display_rst  = GPIO_NUM_18;

// Pin for touch screen interrupt.
const gpio_num_t touch_int_pin = GPIO_NUM_36;

// Shift register output enable pin (active low).
const gpio_num_t shift_oe = GPIO_NUM_47;


// ### Bank 1: Registers for chip select lines
// 
// Bank 1 of shift registers is a series of 3 8-bit shift registers used
// to control the chip select lines for the 24 motor driver circuits.

// Bank 1 data pin.
const gpio_num_t shift_bank1_data_in = spi3_mosi_pin;
// Bank 1 clock pin.
const gpio_num_t shift_bank1_clock = spi3_sclk_pin;
// Bank 1 latch pin.
const gpio_num_t shift_bank1_latch = GPIO_NUM_46;

// ### Bank 2: Registers for BOOT0/STOP and NRST lines, and extra outputs
// 
// Bank 2 of shift registers is a series of 7 8-bit shift registers used
// to control the BOOT0 and NRST lines for the 24 motor driver circuits.
// The 7th and last shift register is used for the following outputs:
// 1: I2C1_NRST
// 2: I2C1_BOOT0/STOP
// 3: I2C2_NRST
// 4: I2C2_BOOT0/STOP
// 5: USB sensor NRST
// 6: USB sensor BOOT0/STOP
// 7: Connect USB sensor to 5V supply.
// 8: Connect USB sensor to VCC supply.

// Bank 2 data pin.
const gpio_num_t shift_bank2_data_in = spi2_mosi_pin;
// Bank 2 clock pin.
const gpio_num_t shift_bank2_clock = spi2_sclk_pin;
// Bank 2 latch pin.
const gpio_num_t shift_bank2_latch = GPIO_NUM_45;

// ### SD card pins

// SD card command pin (CMD).
const gpio_num_t sd_cmd_pin = spi2_mosi_pin;
// SD card clock pin.
const gpio_num_t sd_clk_pin = spi2_sclk_pin;
// SD card data pin 0 (D0).
const gpio_num_t sd_data0_pin = spi2_miso_pin;
// SD card chip select pin.
const gpio_num_t sd_cs_pin = GPIO_NUM_10;

// I2C, USB and CAN bus pins
// -------------------------
// 
// I2C1 is connected to external connector 1.
// 
// I2C2 is connected to external connector 2, and the touch sensor, 
// and IMU sensor, and USB source controller (to control power supply for attached sensor device).

const gpio_num_t i2c1_sda_pin = GPIO_NUM_42;
const gpio_num_t i2c1_scl_pin = GPIO_NUM_41;

const gpio_num_t i2c2_scl_pin = GPIO_NUM_40;
const gpio_num_t i2c2_sda_pin = GPIO_NUM_39;

const gpio_num_t usb_dm_pin = GPIO_NUM_19;
const gpio_num_t usb_dp_pin = GPIO_NUM_20;

const gpio_num_t can_tx_pin = GPIO_NUM_1;
const gpio_num_t can_rx_pin = GPIO_NUM_2;


// Power control and monitoring pins
// ---------------------------------

const gpio_num_t vbus_voltage_pin = GPIO_NUM_4;
const gpio_num_t vbus_current_pin = GPIO_NUM_5;
const gpio_num_t connect_vbus_pin = GPIO_NUM_6;
const gpio_num_t connect_battery_pin = GPIO_NUM_7;
const gpio_num_t battery_current_pin = GPIO_NUM_15;
const gpio_num_t battery_voltage_pin = GPIO_NUM_16;

const gpio_num_t battery_1_pin = GPIO_NUM_17;
const gpio_num_t battery_2_pin = GPIO_NUM_48;

const gpio_num_t sensor_current_pin = GPIO_NUM_8;

const gpio_num_t boot0_pin = GPIO_NUM_0;

const gpio_num_t reference_pin = GPIO_NUM_3;


// Constants for attached IO devices
// ---------------------------------

const int display_width = 240;
const int display_height = 300;


// Initialize the PWM for the status LED.
void setup_status_led();

// Set the brightness of the status LED (0-255).
void set_status_led_brightness(uint8_t brightness);

// Initialize the SPI busses for the display, SD card, and shift registers.
void setup_spi_busses();

void setup_output_pins();

void connect_battery_to_vcc();

void disconnect_battery_from_vcc();

void connect_vbus_to_vcc();

void disconnect_vbus_from_vcc();