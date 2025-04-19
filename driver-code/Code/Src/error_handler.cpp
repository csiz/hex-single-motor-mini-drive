#include "error_handler.hpp"

#include "main.h"

#include "io.hpp"


void error(){
    Error_Handler();
}

void show_error_on_led(){
    set_RED_LED(0xFF);
}
