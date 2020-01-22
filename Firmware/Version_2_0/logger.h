/***************************************************
 Library for logging.

 Feel free to use the code as it is.

 Benjamin Völker, voelkerb@me.com
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

#ifndef LOGGER_h
#define LOGGER_h

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "SPIFFS.h"
#include <rom/rtc.h>

#define MAX_LOG_STREAMS 5
#define MAX_LOG_TEXT_LENGTH 256
#define MAX_TIME_TEXT_LENGTH 30
#define MAX_TYPE_TEXT_LENGTH 10
#define MAX_TEXT_LENGTH (MAX_LOG_TEXT_LENGTH+MAX_TIME_TEXT_LENGTH)

static const char * const LOG_TYPE_TEXT[] = {"[A]", "[D]", "[I]", "[W]", "[E]",};
static const char LOG_ALL_TEXT[] = "[A]";
static const char LOG_DEBUG_TEXT[] = "[D]";
static const char LOG_INFO_TEXT[] = "[I]";
static const char LOG_WARNING_TEXT[] = "[W]";
static const char LOG_ERROR_TEXT[] = "[E]";

static const char NOTHING_TEXT[] = "";
static const char BYTES[] = "Bytes";
static const char KILO_BYTES[] = "kB";
static const char MEGA_BYTES[] = "MB";

static const char * const MY_RESET_REASON_TXT[] = {
    "NO_MEAN", "POWERON_RESET", "NO_MEAN", "SW_RESET", "OWDT_RESET", "DEEPSLEEP_RESET", "SDIO_RESET", "TG0WDT_SYS_RESET", "TG1WDT_SYS_RESET",
    "RTCWDT_SYS_RESET", "INTRUSION_RESET", "TGWDT_CPU_RESET", "SW_CPU_RESET", "RTCWDT_CPU_RESET", "EXT_CPU_RESET", "RTCWDT_BROWN_OUT_RESET",
    "RTCWDT_RTC_RESET",
};

// static const char TXT_NO_MEAN[] = "NO_MEAN";
// static const char TXT_POWERON_RESET[] = "POWERON_RESET";
// static const char TXT_SW_RESET[] = "SW_RESET";
// static const char TXT_OWDT_RESET[] = "OWDT_RESET";
// static const char TXT_DEEPSLEEP_RESET[] = "DEEPSLEEP_RESET";
// static const char TXT_SDIO_RESET[] = "SDIO_RESET";
// static const char TXT_TG0WDT_SYS_RESET[] = "TG0WDT_SYS_RESET";
// static const char TXT_TG1WDT_SYS_RESET[] = "TG1WDT_SYS_RESET";
// static const char TXT_RTCWDT_SYS_RESET[] = "RTCWDT_SYS_RESET";
// static const char TXT_INTRUSION_RESET[] = "INTRUSION_RESET";
// static const char TXT_TGWDT_CPU_RESET[] = "TGWDT_CPU_RESET";
// static const char TXT_SW_CPU_RESET[] = "SW_CPU_RESET";
// static const char TXT_RTCWDT_CPU_RESET[] = "RTCWDT_CPU_RESET";
// static const char TXT_EXT_CPU_RESET[] = "EXT_CPU_RESET";
// static const char TXT_RTCWDT_BROWN_OUT_RESET[] = "RTCWDT_BROWN_OUT_RESET";
// static const char TXT_RTCWDT_RTC_RESET[] = "RTCWDT_RTC_RESET";


enum LogType{ALL, DEBUG, INFO, WARNING, ERROR};
enum LoggerType{ABSTRACT_LOGGER, STREAM_LOGGER, SPIFFS_LOGGER};

// abstract 
class Logger {
  public:
    Logger(char * (*timeStrGetter)(void), const char * prefix, LogType type=ALL);
    virtual void write(const char * str);
    virtual void write(char * str);
    virtual void flush();
    virtual bool init();
    LoggerType loggerType;
    LogType _type;
    const char * _prefix;
    char * (*_timeStrGetter)(void);
};

class StreamLogger : public Logger {
  public:
    StreamLogger(Stream * stream, char * (*timeStrGetter)(void), const char * prefix, LogType type=ALL);
    // These are function which are required to be overloaded by this class
    void write(const char * str);
    void write(char * str);
    void flush();
    bool init();
    // Stream object which can be changed during runtime
    Stream * _stream;
};


class SPIFFSLogger : public Logger {
  #define DEFAULT_MAX_FILESIZE (1024*20) // 20kb
  #define BUFFERS 2
  #define BUF_SIZE 1024
  #define MAX_FILENAME_LENGTH 20
  #define NUM_FILES 2
  enum FileMode{Reading, Writing, Appending};

  public:
    // Open logger with file size limit in byte
    SPIFFSLogger(bool autoFlush, const char * fileName, char * (*timeStrGetter)(void), const char * prefix, LogType type, int32_t maxFileSize=DEFAULT_MAX_FILESIZE);
    bool init();
    bool dumpLogfile(bool file2=false);
    bool clear();
    uint32_t getFileSize();
    void listAllFiles();
    int logRows();
    bool nextRow(char * buffer);
    // These are function which are required to be overloaded by this class
    void write(const char * str);
    void write(char * str);
    void flush();
    
  private:
    void _write(char * str);
    uint32_t _getFileSize(File file);
    bool _openFor(int index, FileMode mode);
    int _getRowsInFile(File file);
    void _fileSizeLimitReached();
    void _dumpFile(File file);

    // If flushing should be performed automatically
    // If not you have to call it
    bool _autoFlush;
    SemaphoreHandle_t _mutex; 
    // Shows if flushing was not performed for some str
    bool _truncated;
    // Buffer str before flushing
    char buffer[BUFFERS][BUF_SIZE];
    size_t _bufferIndex;
    int _rowsFile[NUM_FILES];
    int _currentRow;
    int _currentFile;
    FileMode _fileMode[NUM_FILES];
    // The current active logfile
    File _file[NUM_FILES];
    // Filename of older logs
    char _fileName[NUM_FILES][MAX_FILENAME_LENGTH];
    // maximum filesize before copied over
    uint32_t _maxFileSize;
};


class MultiLogger {
  public:
    static MultiLogger& getInstance(){
      static MultiLogger instance;
      return instance;
    }
    // MultiLogger(Logger * logger, char * (*timeStrGetter)(void));
    bool init();
    bool addLogger(Logger * logger);
    bool removeLogger(Logger * logger);
    void setTimeGetter(char * (*timeStrGetter)(void));
    
    void flush(LogType type=INFO);
    // void log(const char* _log);
    void log(LogType type, char* _log, ...);
    void log(char* _log, ...);
    void log(LogType type, const char* _log, ...);
    void log(const char* _log, ...);
    void append(const char* _log, ...);
    void append(char* _log, ...);

    Logger * loggers[MAX_LOG_STREAMS];
  private:
    MultiLogger();
    MultiLogger(const MultiLogger &);             // Prevent copy constructor
    MultiLogger & operator=(const MultiLogger &); // prevents copy

    const char* _reset_reason(RESET_REASON reason);
    const char* _logTypeToStr(LogType type);
    SemaphoreHandle_t w_mutex; // During write, we are not allowed to add or remove streams from the logger class
    
	  char _logText[MAX_TEXT_LENGTH];
	  char _aLLStr[MAX_TEXT_LENGTH];
	  LogType _currentLog;
    char * (*_timeStrGetter)(void);
    uint32_t _index;
};

#endif
