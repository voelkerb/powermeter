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
#include <RTClib.h>

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
    DateTime update();
    bool enableInterrupt(int frequency, void (*cb)(void));
    void disableInterrupt();
    void setTime(DateTime dt);
    char * timeStr(DateTime dt);

    bool connected;
    bool lost;

    DateTime _now;
  private:
    char datestring[20];
    int INT_PIN;
    char _timeStr[50];

    void (*_intCB)(void);
};

#endif
