#include "error_handler.hpp"

#include "io.hpp"


void error(){
    // TODO: motor stop

    set_RED_LED(0xFF);
    
    __disable_irq();

    while (true) {}
}
