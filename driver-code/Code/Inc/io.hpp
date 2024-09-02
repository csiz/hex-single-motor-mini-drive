#pragma once
/*
 * outputs.h
 *
 *  Created on: Sep 2, 2024
 *      Author: Calin
 */

#include <stm32f103xb.h>
#include <stm32f1xx_ll_gpio.h>

extern volatile bool hall_1, hall_2, hall_3;

void read_hall_sensors();
void initialise_LED_channels();
void set_LED_RGB_colours(uint8_t r, uint8_t g, uint8_t b);
