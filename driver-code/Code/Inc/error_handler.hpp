#pragma once

#ifdef __cplusplus
extern "C" {
#endif


// Call Error_handler from C++ land.
void error();

// Called by Error_Handler to show an error on the RED LED. Use Error_Handler for errors!
void show_error_on_led();


#ifdef __cplusplus
}
#endif