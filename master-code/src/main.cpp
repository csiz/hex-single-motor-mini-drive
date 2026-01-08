#include <Arduino.h>

const int status_pin = 35;

void setup(){
  // Setup the status pin to half brightness 0 to 255 pwm mode.
  pinMode(status_pin, OUTPUT);
  analogWrite(status_pin, 32);
}

void loop(){
  // Your code here
}
