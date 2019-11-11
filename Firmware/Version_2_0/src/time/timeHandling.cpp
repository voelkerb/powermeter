/***************************************************
 Library for time keeping.

 Feel free to use the code as it is.

 Benjamin Völker, voelkerb@me.com
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

#include "timeHandling.h"

// _________________________________________________________________________
TimeHandler::TimeHandler(Rtc * rtc, const char * ntpServerName, int locationOffset, void (*ntpSyncCB)(void), DST dst) : logger(MultiLogger::getInstance()) {
  _rtc = rtc; 
  _ntpTaskHandle = NULL;
  _ntpServerName = ntpServerName;
  _locationOffset = locationOffset;
  _localNTPPort = 2390;
  _ntpValidMillis = 0;
  _ntpEpochSeconds = 0;
  _ntpMilliseconds = 0;
  _lastNTPTry = millis();
  _currentSeconds = 0;
  _currentMilliseconds = 0;
  _ntpSyncCB = ntpSyncCB;
  _dst = dst;
}


// _________________________________________________________________________
unsigned long TimeHandler::utc_seconds() {
  return _currentSeconds;
}

// _________________________________________________________________________
unsigned long TimeHandler::milliseconds() {
  return _currentMilliseconds;
}

// _________________________________________________________________________
bool TimeHandler::updateNTPTime(bool wait) {
  // NOTE: can anything bad happen if we do this multiple times?
  _udpNtp.begin(_localNTPPort);

  if (wait) {
    return _updateNTPTime();
  } else {
    if (_ntpTaskHandle == NULL) {
      // let it check on second core (not in loop)
      TimeHandler *obj = this;
      xTaskCreatePinnedToCore(
                _startUpdateNTPTime,
                // _updateNTPTime,   /* Function to implement the task */
                "_updateNTPTime", /* Name of the task */
                10000,      /* Stack size in words */
                (void*) obj,       /* Task input parameter */
                2,          /* Priority of the task */
                &_ntpTaskHandle,       /* Task handle. */
                0);  /* Core where the task should run */
    }
  }
  return true;
}

// _________________________________________________________________________
void TimeHandler::update() {
  int32_t delta = millis()-_ntpValidMillis;
  if (delta > NTP_UPDATE_INTV && millis()-_lastNTPTry > NTP_TRY_INTV) {
    _lastNTPTry = millis();
    if (Network::connected and not Network::apMode) {
      // TODO Uncomment, this is jut a try
      updateNTPTime(false);
    }
  }
  if (_ntpEpochSeconds != 0) {
    _currentMilliseconds = _ntpMilliseconds + delta%1000;
    _currentSeconds = _ntpEpochSeconds + delta/1000 + _currentMilliseconds/1000;
    _currentMilliseconds = _currentMilliseconds%1000;
  }
}

// _________________________________________________________________________
// Decorator function since we cannot use non static member function in freertos' createTask 
void TimeHandler::_startUpdateNTPTime(void * pvParameters) {
  ((TimeHandler*)pvParameters)->_updateNTPTime();
  vTaskDelete(NULL);
}

// _________________________________________________________________________
bool TimeHandler::_updateNTPTime() {
  bool success = _getTimeNTP();
  
  int32_t delta = millis()-_ntpValidMillis;
  _currentMilliseconds = _ntpMilliseconds + delta%1000;
  _currentSeconds = _ntpEpochSeconds + delta/1000 + _currentMilliseconds/1000;
  _currentMilliseconds = _currentMilliseconds%1000;
  DateTime ntpTime(_currentSeconds + _locationOffset);
  // Make sanity check here.
  if (ntpTime.year() < 2018 || ntpTime.year() > 2100 || ntpTime.month() > 12 || ntpTime.day() > 31) {
    logger.log(ERROR, "Strange NTP time %s", timeStr(ntpTime,0));
    success = false;
  }
  // Only on success do sth with the time
  if (success) {
    logger.log("Success NTP Time");
    // Call synced callback if there is any
    if (_ntpSyncCB != NULL) _ntpSyncCB();
    // if an rtc is connected and we are on core 1, maybe we can correct its time
    if (xPortGetCoreID() != 0 && _rtc->connected) {
      // Get current rtc time
      _rtc->update();
      // Check if RTC time is off
      // NOTE: the problem with checking for second is that this is off pretty often due to rounding, we don't 
      // want to reset the rtc time this often, do we?
      if (_rtc->lost or _rtc->_now.minute() != ntpTime.minute()) {
        logger.log(WARNING, "RTC updated old: %s", _rtc->timeStr());
        _rtc->setTime(ntpTime);
      }
    }
  }
  // Reset task handle for freertos so that this function can be called again
  _ntpTaskHandle = NULL;
  return success;
}

// _________________________________________________________________________
bool TimeHandler::_getTimeNTP() {
  logger.log("Sending NTP packet...");
  WiFi.hostByName(_ntpServerName, _timeServerIP);
  logger.log("To %s with IP: %s", _ntpServerName, _timeServerIP.toString().c_str());
  // Reset all bytes in the buffer to 0
  memset(_ntpBuffer, 0, PACKET_SIZE_NTP);
  // Initialize values needed to form NTP requestjvbasd
  _ntpBuffer[0] = 0b11100011; // LI, Version, Mode
  _ntpBuffer[1] = 0; // Stratum, or type of clock
  _ntpBuffer[2] = 6; // Polling Interval
  _ntpBuffer[3] = 0xEC; // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  _ntpBuffer[12]  = 49;
  _ntpBuffer[13]  = 0x4E;
  _ntpBuffer[14]  = 49;
  _ntpBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:

  long start = millis();
  _udpNtp.beginPacket(_timeServerIP, 123); //NTP requests are to port 123
  _udpNtp.write(_ntpBuffer, PACKET_SIZE_NTP);
  _udpNtp.endPacket();// should flush
  // Wait for packet to arrive with timeout of 2 seconds
  int cb = _udpNtp.parsePacket();
  while(!cb) {
    if (start + 2000 < millis()) break;
    cb = _udpNtp.parsePacket();
    yield();
  }
  if (!cb) {
    logger.log(WARNING, "No NTP response yet");
    return false;
  }

  _ntpValidMillis = millis();
  uint16_t tripDelayMs = (_ntpValidMillis-start)/2;
  _ntpValidMillis -= tripDelayMs;
  //Serial.print("Info:Received NTP packet, length=");
  //Serial.println(cb);
  // We've received a packet, read the data from it
  _udpNtp.read(_ntpBuffer, PACKET_SIZE_NTP);
  // read the packet into the buf
  unsigned long highWord = word(_ntpBuffer[40], _ntpBuffer[41]);
  unsigned long lowWord = word(_ntpBuffer[42], _ntpBuffer[43]);
  // combine the four bytes (two words) into a long integer
  // this is NTP time (seconds since Jan 1 1900):
  unsigned long secsSince1900 = highWord << 16 | lowWord;
  uint32_t frac  = (uint32_t) _ntpBuffer[44] << 24
                 | (uint32_t) _ntpBuffer[45] << 16
                 | (uint32_t) _ntpBuffer[46] <<  8
                 | (uint32_t) _ntpBuffer[47] <<  0;
  uint16_t mssec = ((uint64_t) frac *1000) >> 32;
  // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
  // subtract seventy years:
  unsigned long epoch = secsSince1900 - 2208988800UL;
  // Add the time we waited
  _ntpEpochSeconds = epoch;
  _ntpMilliseconds = mssec;
  logger.log("NTP Time: %s",  timeStr(_ntpEpochSeconds, _ntpMilliseconds));
  return true;
}

bool TimeHandler::valid() {
  if (_currentSeconds == 0) return false;
  else return true;
}
// _________________________________________________________________________
char * TimeHandler::timeStr(bool shortForm) {
  update();
  return timeStr(_currentSeconds, _currentMilliseconds, shortForm);
}

// _________________________________________________________________________
char * TimeHandler::timeStr(DateTime dt, unsigned long ms, bool shortForm) {
  if (shortForm) {
    sprintf(_timeString, "%02u/%02u %02u:%02u:%02u", dt.month(), dt.day(),
            dt.hour(), dt.minute(), dt.second());
  } else {
    sprintf(_timeString, "%02u/%02u/%04u %02u:%02u:%02u.%03u", dt.month(), dt.day(), dt.year(),
            dt.hour(), dt.minute(), dt.second(), (unsigned int)ms);
  }
  return _timeString;
}


// _________________________________________________________________________
int TimeHandler::_dow(int y, int m, int d) { 
  static int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  y -= m < 3; 
  return (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7; 
}

DateTime TimeHandler::timeZoneDST(unsigned long s) {
  DateTime ttime(s + _locationOffset);
  if (_dst.active) {
    int startDay = 31 - _dow(ttime.year(), _dst.startMon, 31);
    int stopDay = 31 - _dow(ttime.year(), _dst.stopMon, 31);
    DateTime d1(ttime.year(), _dst.startMon, startDay, 1, 0, 0);
    DateTime d2(ttime.year(), _dst.stopMon, stopDay, 1, 0, 0);
    uint32_t tsD1 = d1.unixtime();
    uint32_t tsD2 = d2.unixtime();
    if (s >= tsD1 && s <= tsD2) {
      return DateTime(s + _locationOffset +_dst.seconds);
    }
  }
  return ttime;
}



// _________________________________________________________________________
char * TimeHandler::timeStr(unsigned long s, unsigned long ms, bool shortForm) {
  // Not valid maybe just return default
  if (s == 0) {
    if (xPortGetCoreID() != 0) {
    // vTaskEnterCritical();
      _rtc->update(); 
    // vTaskExitCritical();
    }
    return timeStr(_rtc->_now, 0, shortForm);
  }
  return timeStr(timeZoneDST(s), ms, shortForm);
}