#ifndef Creality_V422_pins_h
  #define Creality_V422_pins_h
   #include "Creality_V422_pins.h"
#endif


#include "Arduino.h"





void set_Fan_speed(unsigned int Duty)
{
    pinMode(FAN_PIN, OUTPUT);
    analogWrite(FAN_PIN, Duty);
}