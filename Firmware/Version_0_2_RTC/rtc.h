/***************************************************
 Library for rtc time handling.

 Feel free to use the code as it is.

 Benjamin Völker, voelkerb@me.com
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

#ifndef RTC_h
#define RTC_h

#include <Wire.h> // must be included here so that Arduino library object file references work
#include <RtcDS1307.h>
#include <TimeLib.h>

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class Rtc {
  public:
    Rtc();
    Rtc(int intPin);
    void init();
    void update();
    void enableInterrupt(int frequency, void (*cb)(void));
    void disableInterrupt();
    char * printTime(const RtcDateTime& dt);

  private:
    char datestring[20];
    int INT_PIN;

    void (*_intCB)(void);
};

#endif
