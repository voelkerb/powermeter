/***************************************************
 Library for rtc time handling.

 Feel free to use the code as it is.

 Benjamin Völker, voelkerb@me.com
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

#include "rtc.h"

RTC_DS3231 _rtc;

Rtc::Rtc(int intPin) {
  INT_PIN = intPin;
  _intCB = NULL;
  pinMode(INT_PIN, INPUT_PULLUP);
  lost = true;
  connected = false;
}

Rtc::Rtc() {
  INT_PIN = -1;
  _intCB = NULL;
  connected = false;
  lost = true;
}

void Rtc::init() {

  _rtc.begin();

  // Read temperature to see a DS3231 is connected
  float temp = _rtc.getTemperature();
  Serial.print("Info:Temp:");
  Serial.println(temp);
  // If temperature is beyond normal, consider RTC to be not present
  if (temp < 1 || temp > 70) {
    Serial.println("Info:No RTC connected");
    connected = false;
    return;
  } else {
    connected = true;
  }
  // Indicate power loss, s.t. others know they cannot trust time
  if (_rtc.lostPower()) {
    lost = true;
    // Set rtc time to compile time
    _rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

void Rtc::setTime(DateTime dt) {
  // Set the new time
  _rtc.adjust(dt);
  // Indicate time can be trusted now
  lost = false;
}

bool Rtc::enableInterrupt(int frequency, void (*cb)(void)) {
  if (INT_PIN == -1) {
    Serial.println("Info:Need to init RTC with Pin to which RTC SQW out is connected");
    return false;
  }
  // does this reset the sqwc counter?
  _rtc.adjust(_now);
  if (frequency == 1) {
    _rtc.writeSqwPinMode(DS3231_SquareWave1Hz);
  } else if (frequency == 1024) {
    _rtc.writeSqwPinMode(DS3231_SquareWave1kHz);
  } else if (frequency == 4096) {
    _rtc.writeSqwPinMode(DS3231_SquareWave4kHz);
  } else if (frequency == 8192) {
    _rtc.writeSqwPinMode(DS3231_SquareWave8kHz);
  } else {
    Serial.println("Unsupported RTC SQWV frequency");
    return false;
  }
  // TODO: Frequency calculation and so on...
  // Currently 1 Hz
  // _rtc.SetSquareWavePin(DS3231SquareWavePin_ModeClock);
  _intCB = cb;
  attachInterrupt(digitalPinToInterrupt(INT_PIN), _intCB, FALLING);
  return true;
}

void Rtc::disableInterrupt() {
  detachInterrupt(digitalPinToInterrupt(INT_PIN));
}

DateTime Rtc::update() {
  if (!connected) return DateTime(F(__DATE__), F(__TIME__));
  _now = _rtc.now();
  Serial.print("Info:");
  Serial.println(timeStr(_now));
  return _now;
}


char * Rtc::timeStr(DateTime dt) {
    sprintf(_timeStr, "%02u/%02u/%04u %02u:%02u:%02u", dt.month(), dt.day(),
            dt.year(),
            dt.hour(),
            dt.minute(),
            dt.second() );
    return &_timeStr[0];
}
