#include "error_handler.hpp"

#include "io.hpp"
#include "main.h"



void error(){
    Error_Handler();
}

void show_error_on_led(){
    set_RED_LED(0xFF);
}
