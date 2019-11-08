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
#include "logger.h"

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class Rtc {
  public:
    Rtc(int intPin);
    bool init();
    DateTime update();
    bool enableInterrupt(int frequency, void (*cb)(void));
    void disableInterrupt();
    void setTime(DateTime dt);
    char * timeStr(DateTime dt);
    char * timeStr();

    bool connected;
    bool lost;

    DateTime _now;
  private:
    int INT_PIN;
    char _timeStr[50];

    MultiLogger &logger;
    void (*_intCB)(void);
};

#endif
