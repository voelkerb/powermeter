/***************************************************
 Library for rtc time handling.

 Feel free to use the code as it is.

 Benjamin Völker, voelkerb@me.com
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

#include "rtc.h"

RtcDS1307<TwoWire> _rtc(Wire);

Rtc::Rtc(int intPin) {
  INT_PIN = intPin;
  _intCB = NULL;
  pinMode(INT_PIN, INPUT_PULLUP);
}

Rtc::Rtc() {
  INT_PIN = -1;
  _intCB = NULL;
}

void Rtc::init() {

  _rtc.Begin();

   RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);

   Serial.println(printTime(compiled));

   if (!_rtc.IsDateTimeValid()) {
    if (_rtc.LastError() != 0) {
      Serial.print("RTC communications error = ");
      Serial.println(_rtc.LastError());
    } else {
      Serial.println("RTC lost confidence in the DateTime!");
      _rtc.SetDateTime(compiled);
    }
   }

   if (!_rtc.GetIsRunning()) {
     Serial.println("RTC was not actively running, starting now");
     _rtc.SetIsRunning(true);
   }

   RtcDateTime now = _rtc.GetDateTime();
   if (now < compiled) {
     Serial.println("RTC is older than compile time!  (Updating DateTime)");
     _rtc.SetDateTime(compiled);
   } else if (now > compiled) {
     Serial.println("RTC is newer than compile time. (this is expected)");
   } else if (now == compiled) {
     Serial.println("RTC is the same as compile time! (not expected but all is fine)");
   }
}

void Rtc::enableInterrupt(int frequency, void (*cb)(void)) {
  if (INT_PIN == -1) {
    Serial.println("Need to init RTC with Pin to which RTC SQW out is connected");
    return;
  }
  // TODO: Frequency calculation and so on...
  // Currently 1 Hz
  _rtc.SetSquareWavePin(DS1307SquareWaveOut_1Hz);
  _intCB = cb;
  attachInterrupt(digitalPinToInterrupt(INT_PIN), _intCB, RISING);
}

void Rtc::disableInterrupt() {
  detachInterrupt(digitalPinToInterrupt(INT_PIN));
}

void Rtc::update() {

  if (!_rtc.IsDateTimeValid()) {
    if (_rtc.LastError() != 0) {
      Serial.print("RTC communications error = ");
      Serial.println(_rtc.LastError());
    } else {
      Serial.println("RTC lost confidence in the DateTime!");
    }
  }

  RtcDateTime now = _rtc.GetDateTime();

  Serial.println(printTime(now));
}

#define countof(a) (sizeof(a) / sizeof(a[0]))

char * Rtc::printTime(const RtcDateTime& dt) {
    snprintf_P(datestring,
            countof(datestring),
            PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
            dt.Month(),
            dt.Day(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    return &datestring[0];
}
