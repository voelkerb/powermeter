#include "logger.h"
#include "rtc.h"

#define SERIAL_SPEED 2000000
const char logPrefixSerial[] = "";
const char logPrefix[] = "Info:";

const int RTC_INT = 25;
Rtc rtc(RTC_INT);

// Serial logger
StreamLogger serialLog((Stream*)&Serial, &timeStr, &logPrefixSerial[0], ALL);

// TCP logger
StreamLogger streamLog((Stream*)NULL, &timeStr, &logPrefix[0], INFO);

const char * logFile = "/log.txt";
// SPIFFS
SPIFFSLogger spiffsLog(&logFile[0], &timeStr, &logPrefixSerial[0], WARNING);

// MultiLogger logger(&streamLog, &timeStr);
// Create singleton herer
MultiLogger& logger = MultiLogger::getInstance();


void setup() {
  rtc.init();
  // Init the logging module
  logger.setTimeGetter(&timeStr);
  // Setup serial communication
  Serial.begin(SERIAL_SPEED);
  // Add Serial logger
  logger.addLogger(&serialLog);
  spiffsLog.init();
  logger.addLogger(&spiffsLog);
}
#define MAX_LENGTH 200
char buffer[MAX_LENGTH] = {'\0'};

void loop() {
  Serial.println(rtc.timeStr(rtc.update()));
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'c':
      case 'C':
        spiffsLog.clear();
        break;
      case 'p':
      case 'P':
        spiffsLog.dumpLogfile();
        break;
      case 'q':
      case 'Q':
        spiffsLog.dumpLogfile(true);
        break;
      case 'w':
      case 'W':
        strcpy(&buffer[0], Serial.readStringUntil('\n').c_str());
        logger.log(WARNING, &buffer[0]);
        break;
      case 'e':
      case 'E':
        strcpy(&buffer[0], Serial.readStringUntil('\n').c_str());
        logger.log(ERROR, &buffer[0]);
        break;
      case 'i':
      case 'I':
        strcpy(&buffer[0], Serial.readStringUntil('\n').c_str());
        logger.log(INFO, &buffer[0]);
        break;
      case 'd':
      case 'D':
        strcpy(&buffer[0], Serial.readStringUntil('\n').c_str());
        logger.log(DEBUG, &buffer[0]);
        break;
      case 's':
      case 'S':
        spiffsLog.getFileSize();
        break;
      case 'l':
      case 'L':
        spiffsLog.listAllFiles();
        break;
      case 'a':
      case 'A': {
        bool hasRow = spiffsLog.nextRow(&buffer[0]);
        int row = 0;
        while(hasRow) {
          Serial.printf("#%i: ", row++);
          Serial.println(&buffer[0]);
          hasRow = spiffsLog.nextRow(&buffer[0]);
        }
        break;
      }
      default:
        break;
    }
  }
}

char * timeStr() {
  return rtc.timeStr();
}