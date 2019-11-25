/***************************************************
 Library for rtc time handling.

 Feel free to use the code as it is.

 Benjamin Völker, voelkerb@me.com
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

#include "rtc.h"

RTC_DS3231 _rtc;

Rtc::Rtc(int intPin) : logger(MultiLogger::getInstance()) {
  INT_PIN = intPin;
  _intCB = NULL;
  lost = true;
  connected = false;
  _now = DateTime(0);
}


bool Rtc::init() {

  if (INT_PIN >= 0) pinMode(INT_PIN, INPUT_PULLUP);
  _rtc.begin();

  Wire.setDebugFlags(0xffff, 0xffff);
  // Read temperature to see a DS3231 is connected
  float temp = _rtc.getTemperature();
  logger.log("Temp: %.2f", temp);
  // If temperature is beyond normal, consider RTC to be not present
  if (temp < 1 || temp > 70) {
    logger.log("No RTC connected");
    connected = false;
    return false;
  } else {
    connected = true;
  }
  // Indicate power loss, s.t. others know they cannot trust time
  if (_rtc.lostPower()) {
    lost = true;
    // Set rtc time to compile time
    _rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  return true;
}

void Rtc::setTime(DateTime dt) {
  // Set the new time
  _rtc.adjust(dt);
  // Indicate time can be trusted now
  lost = false;
}

bool Rtc::enableInterrupt(int frequency, void (*cb)(void)) {
  if (INT_PIN == -1) {
    logger.log("Need to init RTC with Pin to which RTC SQW out is connected");
    return false;
  }
  // TODO: does this reset the sqwc counter?
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
    logger.log("Unsupported RTC SQWV frequency");
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
  // Serial.println(_now.timestamp());
  return _now;
}

char * Rtc::timeStr() {
  return timeStr(update());
}

char * Rtc::timeStr(DateTime dt) {
    sprintf(_timeStr, "%02u/%02u/%04u %02u:%02u:%02u", dt.month(), dt.day(),
            dt.year(),
            dt.hour(),
            dt.minute(),
            dt.second() );
    return &_timeStr[0];
}
