#include "app_main.hpp"
#include "io.hpp"

void app_init() {
    initialise_LED_channels();
    //  TODO: set dead time for the motor phases pwm.
}

void app_tick() {
    
    read_hall_sensors();
    set_LED_RGB_colours(hall_1 ? 0xF0 : 0, hall_2 ? 0x40 : 0, hall_3 ? 0x80 : 0);
}
