#pragma once


#include "io.hpp"
#include "constants.hpp"

#include <cstdint>


// Mapping from hall state to sector number.
static inline uint8_t get_hall_sector(const uint8_t hall_state){
    // Get the hall sector from the state.
    switch (hall_state & 0b111) {
        case 0b000: // no hall sensors; either it's not ready or no magnet
            return hall_sector_base; // Out of range, indicates invalid.
        case 0b001: // hall U active; 0 degrees
            return 0;
        case 0b011: // hall U and hall V active; 60 degrees
            return 1;
        case 0b010: // hall V active; 120 degrees
            return 2;
        case 0b110: // hall V and hall W active; 180 degrees
            return 3;
        case 0b100: // hall W active; 240 degrees
            return 4;
        case 0b101: // hall U and hall W active; 300 degrees
            return 5;
        case 0b111: // all hall sensors active; this would be quite unusual; but carry on
            return hall_sector_base; // Out of range, indicates invalid.
    }
    // We shouldn't reach here.
    return hall_sector_base;
}

// Read the hall sensors and update the motor rotation angle. Sensor chips might be: SS360NT (can't read the inprint clearly).
static inline uint8_t read_hall_sensors_state(){
    // Grab the registers for the GPIO ports with the hall sensors.

    uint16_t gpio_A_inputs = LL_GPIO_ReadInputPort(GPIOA);
    uint16_t gpio_B_inputs = LL_GPIO_ReadInputPort(GPIOB);

    // Note: Hall sensors are active low!
    const bool hall_1 = !(gpio_A_inputs & (1<<0)); // Hall sensor 1, corresponding to phase V
    const bool hall_2 = !(gpio_A_inputs & (1<<1)); // Hall sensor 2, corresponding to phase W
    const bool hall_3 = !(gpio_B_inputs & (1<<10)); // Hall sensor 3, corresponding to phase U

    // Combine the hall sensor states into a single byte.
    // Note: Reorder the sensors according to the phase order; U on bit 0, V on bit 1, W on bit 2.
    const uint8_t hall_state = hall_3 | (hall_1 << 1) | (hall_2 << 2);

    return hall_state;
}


