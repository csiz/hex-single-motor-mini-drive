#include "error_handler.hpp"

#include "io.hpp"


void error(){
    set_BLUE_LED(0xFF);
    
    // ! Don't disable interrupts, the motor control loop must flow.
}
