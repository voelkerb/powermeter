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
const char BYTES[] = "Bytes";
const char KILO_BYTES[] = "kB";
const char MEGA_BYTES[] = "MB";

enum LogType{ALL, DEBUG, INFO, WARNING, ERROR};

// abstract 
class Logger {
  public:
    Logger(char * (*timeStrGetter)(void), const char * prefix, LogType type=ALL);
    virtual void write(const char * str);
    virtual void write(char * str);
    virtual void flush();
    const char * logTypeToStr(LogType type);

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
    // Stream object which can be changed during runtime
    Stream * _stream;
};


class SPIFFSLogger : public Logger {
  #define DEFAULT_MAX_FILESIZE 1024*20 // 20kb
  #define MAX_FILENAME_LENGTH 20
  #define NUM_FILES 2
  enum FileMode{Reading, Writing, Appending};

  public:
    // Open logger with file size limit in byte
    SPIFFSLogger(const char * fileName, char * (*timeStrGetter)(void), const char * prefix, LogType type, int32_t maxFileSize=DEFAULT_MAX_FILESIZE);
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
    uint32_t _getFileSize(File file);
    bool _openFor(int index, FileMode mode);
    int _getRowsInFile(File file);
    void _fileSizeLimitReached();
    void _dumpFile(File file);

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
