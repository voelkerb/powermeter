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

#define _MAX_RTC_TIME_STR_LENGTH 50

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class Rtc {
  public:
    Rtc(int8_t intPin, int8_t sda = -1, int8_t scl = -1);
    bool init();
    DateTime update();
    bool enableInterrupt(int frequency, void (*cb)(void));
    void disableInterrupt();
    void setTime(DateTime dt);
    char * timeStr(DateTime dt);
    char * timeStr();

    bool connected;
    bool lost;
    const int8_t _SDA;
    const int8_t _SCL;

    DateTime _now;
  private:
    int INT_PIN;
    char _timeStr[_MAX_RTC_TIME_STR_LENGTH];

    MultiLogger &logger;
    void (*_intCB)(void);
};

#endif
