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

#include <WiFi.h>
#include <time.h> 
#include <FreeRTOS.h>
#include "logger.h"
#include "network.h"
#include "rtc.h"

#define _MAX_TIME_STR_LENGTH 42
#define _MAX_DOW_STR_LENGTH 10
// NTP time stamp is in the first 48 bytes
#define PACKET_SIZE_NTP 48 

// Update interval to get new NTP time
#define NTP_UPDATE_INTV 120000
// Try NTP only if we have not tried last 5s
#define NTP_TRY_INTV 30000

struct DST {
  bool active;
  uint8_t startdayOfWeek;
  uint8_t startMon;
  uint8_t startAfterDay;
  uint8_t stopdayOfWeek;
  uint8_t stopMon;
  uint8_t stopAfterDay;
  uint32_t seconds;
} ;


struct Timestamp {
  uint32_t seconds;
  uint32_t milliSeconds;
} ;

enum class Weekday{SUNDAY, MONDAY, TUESDAY, WEDNESDAY, THRUSDAY, FRIDAY, SATURDAY};

static const char * const WEEKDAYS[] = {
    "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"
};
static const char * const WEEKDAYS_SHORT[] = {
    "Su.", "Mo.", "Tu.", "We.", "Th.", "Fr.", "Sa."
};

class TimeHandler {

  public:
    // Constructor
    TimeHandler(char * ntpServerName, int locationOffset, 
                Rtc * rtc=NULL, // optional rtc 
                void (*ntpSyncCB)(void)=NULL, // optional ntp sync callback
                DST dst={ true, 7, 3, 25, 7, 10, 25, 3600} ); // optional Daylight saving time struct. Default: Germany

    // Function to call if NTP should be updated
    // If wait is false, the update is performed in a background task
    bool updateNTPTime(bool wait=false);

    // Various function to time to string with milliseconds support
    char * timeStr(bool shortForm=false);
    char * timeStr(Timestamp ts, bool shortForm=false);
    char * timeStr(unsigned long s, unsigned long ms, bool shortForm=false);
    char * timeStr(DateTime dt, unsigned long ms, bool shortForm=false);

    // Get a string of unix timestamp "<unix_seconds>.<ms>"
    char * timestampStr(bool shortForm=false);
    char * timestampStr(Timestamp, bool shortForm=false);
    char * timestampStr(unsigned long s, unsigned long ms, bool shortForm=false);

    Timestamp timestamp();

    // get current seconds
    unsigned long utc_seconds();
    // get current milliseconds
    unsigned long milliseconds();
    // update _currentMilliseconds and _currentSeconds
    void update();
    // if ntp time is valid
    bool valid();
    DateTime timeZoneDST(unsigned long s);

    Weekday dayOfTheWeek();
    Weekday dayOfTheWeek(Timestamp ts);
    Weekday dayOfTheWeek(DateTime dt);
    char * dayOfTheWeekStr(bool shortForm=false);
    char * dayOfTheWeekStr(Timestamp ts, bool shortForm=false);
    char * dayOfTheWeekStr(DateTime dt, bool shortForm=false);

  private:
    // Get day of week based on day given
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

    // Task handle for the background sync
    TaskHandle_t _ntpTaskHandle;

    // Millis at which the last ntp try happened
    unsigned long _lastNTPTry;
    // Time string stored, used by the tostring methods
    char _timeString[_MAX_TIME_STR_LENGTH];
    char _timeStampStr[_MAX_TIME_STR_LENGTH];
    char _dayOfWeekStr[_MAX_DOW_STR_LENGTH];
    // IP address the time server is resolved to
    IPAddress _timeServerIP;
    // Use NTP server with hopefully a small ping
    char* _ntpServerName;
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
    // The current time updated with _ntpTime + _ntpValidMillis
    Timestamp _currentTime;
    // The last valid ntp time 
    Timestamp _ntpTime;

    // NOTE: In order to be accurate and to represent the current time, 
    // the update() or timeStr() method must be called
    // The current time in seconds (not localtime but the same timezone as ntp)
    // Daylight saving time
    DST _dst;
    // utc timestamp between DST is active
    uint32_t _tsD1;
    uint32_t _tsD2;

    // logging instance (maybe use callback instead of object reference later)
    MultiLogger &logger;

    // A callback on NTP success called
    void (*_ntpSyncCB)(void);
};

#endif
