/***************************************************
 Library for time keeping.

 Feel free to use the code as it is.

 Benjamin Völker, voelkerb@me.com
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

#ifndef TIME_HANDLING_h
#define TIME_HANDLING_h

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "rtc.h"
#include <WiFi.h>
#include <time.h> 
#include "logger.h"
#include "network.h"
#include "FreeRTOS.h"

// NTP time stamp is in the first 48 bytes
#define PACKET_SIZE_NTP 48 

// Update interval to get new NTP time
#define NTP_UPDATE_INTV 60000
// Try NTP only if we have not tried last 5s
#define NTP_TRY_INTV 5000

struct DST {
  bool active;
  uint8_t startdayOfWeek;
  uint8_t startMon;
  uint8_t startAfterDay;
  uint8_t stopdayOfWeek;
  uint8_t stopMon;
  uint8_t stopAfterDay;
  int32_t seconds;
} ;

class TimeHandler {
  public:
    // Constructor
    TimeHandler(Rtc * rtc, const char * ntpServerName, int locationOffset, void (*ntpSyncCB)(void)=NULL,
                DST dst = { true, 7, 3, 25, 7, 10, 25, 3600} );

    // Function to call if NTP should be updated
    // If wait is false, the update is performed in a background task
    bool updateNTPTime(bool wait=false);

    // Various function to time to string with milliseconds support
    char * timeStr(bool shortForm=false);
    char * timeStr(unsigned long s, unsigned long ms, bool shortForm=false);
    char * timeStr(DateTime dt, unsigned long ms, bool shortForm=false);

    // get current seconds
    unsigned long utc_seconds();
    // get current milliseconds
    unsigned long milliseconds();
    // update _currentMilliseconds and _currentSeconds
    void update();
    // if ntp time is valid
    bool valid();
    DateTime timeZoneDST(unsigned long s);

  private:
    int _dow(int y, int m, int d);

    // Decorator function (static function required for freertos create task)
    // Effectively, this function calls _updateNTPTime() 
    static void _startUpdateNTPTime(void * pvParameters);
    // Updates ntp time, makes sanity check and updates rtc
    bool _updateNTPTime();

    // Try to get accurate time over NTP
    bool _getTimeNTP();

    // RTC object, which is updated and synced by this class
    Rtc * _rtc; // Rtc object 

    TaskHandle_t _ntpTaskHandle;

    // Millis at which the last ntp try happened
    unsigned long _lastNTPTry;
    // Time string stored, used by the tostring methods
    char _timeString[32];
    // IP address the time server is resolved to
    IPAddress _timeServerIP;
    // Use NTP server with hopefully a small ping
    const char* _ntpServerName;
    // local port to listen for UDP packets
    unsigned int _localNTPPort;
    // buffer to hold incoming and outgoing udp data
    byte _ntpBuffer[PACKET_SIZE_NTP]; 
    // A UDP instance to let us send and receive packets over UDP
    WiFiUDP _udpNtp;
    // Offset of device location to standard time (utc?)
    int _locationOffset;
    // The internal clock ms at which the last ntp ready was valid
    unsigned long _ntpValidMillis;
    // The last valid ntp seconds TODO: is this utc? or 0 Time
    unsigned long _ntpEpochSeconds;
    // The last valid ntp milliseconds
    unsigned long _ntpMilliseconds;

    // NOTE: In order to be accurate and to represent the current time, 
    // the update() or timeStr() method must be called
    // The current time in seconds (not localtime but the same timezone as ntp)
    unsigned long _currentSeconds;
    // The current milliseconds
    unsigned long _currentMilliseconds;
    DST _dst;

    MultiLogger &logger;

    // A callback to be called if external on sync stuff is required
    void (*_ntpSyncCB)(void);
};

#endif
