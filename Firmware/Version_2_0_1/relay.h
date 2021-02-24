/***************************************************
 Library for switching a bistable relais using set
 and reset pins

 Feel free to use the code as it is.

 Benjamin Völker, voelkerb@me.com
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

#ifndef RELAY_h
#define RELAY_h

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define HOLD_TIME_MS 20 // Relais must be held this ms until state is set

class Relay {
  public:
    Relay(int setPin, int resetPin);
    void set(bool value);
    void setCallback(void (*cb)(bool));
    int state;
  private:
    uint8_t _SET_PIN;
    uint8_t _RES_PIN;
    void (*_switchCB)(bool);
};

#endif
