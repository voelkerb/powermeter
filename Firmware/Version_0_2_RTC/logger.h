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

#define MAX_LOG_STREAMS 3
#define MAX_LOG_TEXT_LENGTH 256
#define MAX_TIME_TEXT_LENGTH 30
#define MAX_TYPE_TEXT_LENGTH 10
#define MAX_TEXT_LENGTH MAX_LOG_TEXT_LENGTH+MAX_TIME_TEXT_LENGTH

const char ERROR_TEXT[] = "[ERROR]";
const char WARNING_TEXT[] = "[WARNING]";
const char INFO_TEXT[] = "[INFO]";
const char DEBUG_TEXT[] = "[DEBUG]";
const char ALL_TEXT[] = "[ALL]";
const char NOTHING_TEXT[] = "";

enum LogType{ALL, DEBUG, INFO, WARNING, ERROR};

// abstract 
class Logger {
  public:
    Logger(char * (*timeStrGetter)(void), const char * prefix, LogType type=ALL) {
      _type = type;
      _prefix = prefix;
      _timeStrGetter = timeStrGetter;
    }
    virtual void write(const char * str);
    virtual void write(char * str);
    virtual void flush();
    const char * logTypeToStr(LogType type) {
      if (type == ALL) return &ALL_TEXT[0];
      else if (type == INFO) return &INFO_TEXT[0];
      else if (type == WARNING) return &WARNING_TEXT[0];
      else if (type == ERROR) return &ERROR_TEXT[0];
      else if (type == DEBUG) return &DEBUG_TEXT[0];
      else return &ALL_TEXT[0];
    }
    LogType _type;
    const char * _prefix;
    char * (*_timeStrGetter)(void);
};

class StreamLogger : public Logger {
  public:
    StreamLogger(Stream * stream, char * (*timeStrGetter)(void), const char * prefix, LogType type=ALL):Logger(timeStrGetter, prefix, type) {
      _stream = stream;
    }

    void write(const char * str) {
      _stream->write(str);
    }
    void write(char * str) {
      _stream->write(str);
    }
    void flush() {
      _stream->flush();
    }

    Stream * _stream;
};


class MultiLogger {
  
  public:
    static MultiLogger& getInstance(){
      static MultiLogger instance;
      return instance;
    }
    // MultiLogger(Logger * logger, char * (*timeStrGetter)(void));

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
    
    const char * logTypeToStr(LogType type);

  private:
    MultiLogger();
    MultiLogger(const MultiLogger &);             // Prevent copy constructor
    MultiLogger & operator=(const MultiLogger &); // prevents copy

    SemaphoreHandle_t w_mutex; // During write, we are not allowed to add or remove streams from the logger class
    Logger * loggers[MAX_LOG_STREAMS];
    
	  char _logText[MAX_TEXT_LENGTH];
	  char _aLLStr[MAX_TEXT_LENGTH];
	  LogType _currentLog;
    char * (*_timeStrGetter)(void);
    uint32_t _index;
};

#endif
