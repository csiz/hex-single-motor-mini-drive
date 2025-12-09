#pragma once


#include "io.hpp"
#include "constants.hpp"

#include <cstdint>


// Mapping from hall state to sector number.
static inline uint8_t get_hall_sector(const uint8_t hall_state){
    // Get the hall sector from the state.
    switch (hall_state & 0b111) {
        // No hall sensors; either it's not powered or no magnet
        case 0b000: return hall_sector_base; // Out of range, indicates invalid.
        // Hall U active; 0 degrees
        case 0b001: return 0;
        // Hall U and hall V active; angle is 60 degrees.
        case 0b011: return 1;
        // Hall V active; angle is 120 degrees.
        case 0b010: return 2;
        // Hall V and hall W active; angle is 180 degrees.
        case 0b110: return 3;
        // Hall W active; angle is 240 degrees.
        case 0b100: return 4;
        // Hall U and hall W active; angle is 300 degrees.
        case 0b101: return 5;
        // All hall sensors active; this would be quite unusual; but carry on.
        case 0b111: return hall_sector_base; // Out of range, indicates invalid.
    }
    // We shouldn't reach here.
    return hall_sector_base;
}

// Read the hall sensors and update the motor rotation angle. Sensor chips might be: SS360NT (can't read the inprint clearly).
static inline uint8_t read_hall_sensors_state(){
    // Grab the registers for the GPIO ports with the hall sensors.

    // uint16_t gpio_A_inputs = LL_GPIO_ReadInputPort(GPIOA);
    uint16_t gpio_B_inputs = LL_GPIO_ReadInputPort(GPIOB);
    uint16_t gpio_C_inputs = LL_GPIO_ReadInputPort(GPIOC);

    // Note: Hall sensors are active low!
    const bool hall_1 = !(gpio_C_inputs & (1<<6)); // Hall sensor 1, corresponding to phase V
    const bool hall_2 = !(gpio_C_inputs & (1<<7)); // Hall sensor 2, corresponding to phase W
    const bool hall_3 = !(gpio_B_inputs & (1<<0)); // Hall sensor 3, corresponding to phase U

    // Combine the hall sensor states into a single byte.
    // Note: Reorder the sensors according to the phase order; U on bit 0, V on bit 1, W on bit 2.
    const uint8_t hall_state = hall_3 | (hall_1 << 1) | (hall_2 << 2);

    return hall_state;
}


