/***************************************************
 Library for logging.

 Feel free to use the code as it is.

 Benjamin Völker, voelkerb@me.com
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

#include "logger.h"



// ********************************* Abstract LOGGER ************************************* //
Logger::Logger(char * (*timeStrGetter)(void), const char * prefix, LogType type) {
  _type = type;
  _prefix = prefix;
  _timeStrGetter = timeStrGetter;
  loggerType = ABSTRACT_LOGGER;
  _currentLog = INFO;
  _index = 0;
}

void Logger::setLogType(LogType type) {
  _type = type;
}


void Logger::append(const char* _log, ...) {
  va_list args;
  va_start(args, _log);
  _index += vsnprintf(&_logText[_index], _MAX_TEXT_LENGTH, _log, args);
  va_end(args);
}

void Logger::append(char* _log, ...) {
  va_list VAList;
  va_start(VAList, _log);
  _index += vsnprintf(&_logText[_index], _MAX_TEXT_LENGTH, _log, VAList);
  va_end(VAList);
}

void Logger::flushAppended(LogType type) {
  _currentLog = type;
  log("");
}

void Logger::log(LogType type, char* _log, ...) {
  _currentLog = type;
  log((const char*)_log);
}
void Logger::log(char* _log, ...) {
  log((const char*)_log);
}
void Logger::log(LogType type, const char* _log, ...) {
  _currentLog = type;
  log(_log);
}
void Logger::log(const char* _log, ...) {
  // Wait maximum time for semaphone to be available
    // Get time from time getter string
    char * timeStr = (char *)&NOTHING_TEXT[0];
    if (_timeStrGetter != NULL) {
      timeStr = _timeStrGetter();
    }
    va_list args;
    va_start(args, _log);
    vsnprintf(&_logText[_index], _MAX_TEXT_LENGTH, _log, args);
    va_end(args);

    const char * typeStr = LOG_TYPE_TEXT[_currentLog];
    // const char * typeStr = _logTypeToStr(_currentLog);
    if ((uint8_t)_type <= (uint8_t)_currentLog) {
      snprintf(_aLLStr, _MAX_TEXT_LENGTH, "%s%s%s: %s\r\n", _prefix, typeStr, timeStr, _logText);
      // Has to be a one liner since data might be written from somewhere else
      write(_aLLStr);
    }
    _index = 0;
    // Reset type string
    _currentLog = INFO;
}




// ********************************* STREAM LOGGER ************************************* //
StreamLogger::StreamLogger(Stream * stream, char * (*timeStrGetter)(void), const char * prefix, LogType type):Logger(timeStrGetter, prefix, type) {
  _stream = stream;
  loggerType = STREAM_LOGGER;
}
void StreamLogger::write(const char * str) {
  _stream->write(str);
}
void StreamLogger::write(char * str) {
  _stream->write(str);
}
void StreamLogger::flush() {
  _stream->flush();
}
bool StreamLogger::init() {
  return true;
}


//  #define DEBUG_SPIFFS_LOGGER
// ********************************* SPIFFS LOGGER ************************************* //
SPIFFSLogger::SPIFFSLogger(bool autoFlush, const char * fileName, char * (*timeStrGetter)(void), 
                           const char * prefix, LogType type, int32_t maxFileSize):Logger(timeStrGetter, prefix, type) {
  strcpy(_fileName[0], fileName);
  strcpy(_fileName[1], fileName);
  size_t index = strlen(fileName);
  strcpy(&_fileName[1][index], "1");
  _maxFileSize = maxFileSize;
  _truncated = false;
  _bufferIndex = 0;
  _autoFlush = autoFlush;
  _mutex = xSemaphoreCreateMutex();
  loggerType = SPIFFS_LOGGER;
  // Init buffers
  for (size_t i = 0; i < BUFFERS; i++) buffer[i][0] = '\0';
}

bool SPIFFSLogger::init() {
  if(SPIFFS.begin(true)) {
    #ifdef DEBUG_SPIFFS_LOGGER
    Serial.println("SPIFFS Initialisierung....OK");
    #endif
  } else {
    #ifdef DEBUG_SPIFFS_LOGGER
    Serial.println("SPIFFS Initialisierung...Fehler!");
    #endif
    return false;
  }
  #ifdef DEBUG_SPIFFS_LOGGER
  for (size_t i = 0; i < NUM_FILES; i++) {
    if (SPIFFS.exists(_fileName[i])) Serial.printf("File %s exists\n", _fileName[i]);
    else Serial.printf("File %s does not exist, t will be created\n", _fileName[i]);
  }
  #endif
  
  // This will create files if they do not exist
  for (size_t i = 0; i < NUM_FILES; i++) _openFor(i, Appending);
  for (size_t i = 0; i < NUM_FILES; i++) {
    _openFor(i, Reading);
    if (!_file[i]) {
      #ifdef DEBUG_SPIFFS_LOGGER
      Serial.printf("There was an error opening the file %s\n", _fileName[i]);
      #endif
      return false;
    }
  }
  
  _openFor(0, Appending);
  if (_timeStrGetter != NULL) {
    _file[0].print(_timeStrGetter());
    _file[0].print(": ");
  } 
  _file[0].println("Reboot - inited SPIFFS Logger");
  _file[0].flush();

  for (size_t i = 0; i < NUM_FILES; i++) _rowsFile[i] = -1;
  
  #ifdef DEBUG_SPIFFS_LOGGER
  for (size_t i = 0; i < NUM_FILES; i++) {
    _openFor(i, Reading);
    _rowsFile[i] = _getRowsInFile(_file[i]);
    Serial.printf("#Rows %s: %i \n", _file[i].name(), _rowsFile[i]);
  }
  #endif

  _currentFile = -1;
  return true;
}



bool SPIFFSLogger::_openFor(int file, FileMode mode) {
  if (file > NUM_FILES-1) return false;
  if (_fileMode[file] != mode) {
    // TODO: ist this important?
    _file[file].close();
  }
  if (mode == Reading) _file[file] = SPIFFS.open(_fileName[file], FILE_READ);
  if (mode == Writing) _file[file] = SPIFFS.open(_fileName[file], FILE_WRITE);
  if (mode == Appending) _file[file] = SPIFFS.open(_fileName[file], FILE_APPEND);
  _fileMode[file] = mode;
  return _file[file];
}

bool SPIFFSLogger::nextRow(char * buffer) {
  if (_currentFile <= -1) {
    _currentFile = NUM_FILES-1;
    _openFor(NUM_FILES-1, Reading);
  }
  if (_currentFile < 0) {
    return false;
  }
  if (!_file[_currentFile].available()) {
    _currentFile--;
    if (_currentFile < 0) {
      return false;
    }
    _openFor(_currentFile, Reading);
  }
  if (!_file[_currentFile].available()) {
    return false;
  }
  //Serial.print(_file[_currentFile].readStringUntil('\n'));
  strcpy(buffer, _file[_currentFile].readStringUntil('\n').c_str());
  // remove \r
  int length = strlen(buffer);
  if (length > 0) buffer[length-1] = '\0';
  return true;
}

int SPIFFSLogger::logRows() {
  int rows = 0;
  for (size_t i = 0; i < NUM_FILES; i++) {
    _openFor(i, Reading);
    rows += _getRowsInFile(_file[i]);
  }
  return rows;
}

int SPIFFSLogger::_getRowsInFile(File file) {
  #ifdef DEBUG_SPIFFS_LOGGER
  long ttime = millis();
  Serial.printf("Get rows in file: %s \n", file.name());
  #endif
  int rows = 0;
  while (file.available()) {
    if (file.read() == '\n') rows++;
  }
  #ifdef DEBUG_SPIFFS_LOGGER
  Serial.printf("Took: %lu ms\n", millis()-ttime);
  #endif
  return rows;
}

uint32_t SPIFFSLogger::getFileSize() {
  uint32_t fs = 0;
  for (size_t i = 0; i < NUM_FILES; i++) {
    _openFor(i, Appending);
    fs += _getFileSize(_file[i]);
  }
  return fs;
}

uint32_t SPIFFSLogger::_getFileSize(File file) {
  #ifdef DEBUG_SPIFFS_LOGGER
  float tsize = (float)file.size();
  const char * sur = &BYTES[0];
  bool asInt = true;
  if (tsize/(1024*1024) > 1) {
    asInt = false;
    tsize = tsize/(float)(1024*1024);
    sur = &MEGA_BYTES[0];
  } else if (tsize/(1024) > 1) {
    asInt = false;
    tsize = tsize/(float)(1024);
    sur = &KILO_BYTES[0];
  }
  if (asInt) {
    Serial.printf("File: %s - %u %s\n", file.name(), (uint32_t)tsize, sur);
  } else {
    Serial.printf("File: %s - %.2f %s\n", file.name(), tsize, sur);
  }
  #endif
  return file.size();
}

bool SPIFFSLogger::clear() {
  bool success = true;
  for (size_t i = 0; i < NUM_FILES; i++) {
    success &= SPIFFS.remove(_fileName[i]);
    #ifdef DEBUG_SPIFFS_LOGGER
    if (success) {
      Serial.printf("File %s cleared\n", _fileName[i]);
    } else {
      Serial.printf("Error clearing %s\n", _fileName[i]);
    }
    #endif
  }
  for (size_t i = 0; i < BUFFERS; i++) buffer[i][0] = '\0';
  return success;
}

void SPIFFSLogger::_dumpFile(File file) {
  if (!file.size()) {
    Serial.printf("File %s is empty\n", file.name());
  } else {
    Serial.printf("\n**********%s***********\n", file.name());
    while(file.available()){
      Serial.write(file.read());
    }
    Serial.println("************************\n");
  }
}

bool SPIFFSLogger::dumpLogfile(bool file2) {
  for (int i = NUM_FILES-1; i >= 0; i--) {
    if (i >= 1 && !file2) continue;
    _openFor(i, Reading);
    _dumpFile(_file[i]);
  }
  return true;
}

void SPIFFSLogger::write(const char * str) {
  write((char*)str);
}

void SPIFFSLogger::write(char * str) {
  if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) { 
    if (_autoFlush) {
      _write(str);
    } else {
      #ifdef DEBUG_SPIFFS_LOGGER
      Serial.println("entering buffered write");
      #endif
      // Get size of resulting buffer
      int n1 = strlen(&buffer[_bufferIndex][0]);
      int n2 = strlen(str);
      #ifdef DEBUG_SPIFFS_LOGGER
      Serial.printf("n1: %i, n2: %i\n", n1, n2);
      #endif
      // If would not fit, copy over
      if (n1+n2 >= BUF_SIZE-1) {
        _bufferIndex = (_bufferIndex + 1)%BUFFERS;
        #ifdef DEBUG_SPIFFS_LOGGER
        Serial.printf("_bufferIndex: %i\n", _bufferIndex);
        #endif
        // Indicate truncation
        if (strlen(&buffer[_bufferIndex][0]) > 0) {
          _truncated = true;
        }
        // TODO null terminate
        // Start is next buffer
        n1 = 0;
      }
      //if (n1+n2 >= BUF_SIZE-1) {
      memcpy(&buffer[_bufferIndex][n1], str, n2+1);
      //}
    }
    xSemaphoreGive(_mutex);
  }
}

void SPIFFSLogger::_write(char * str) {
  #ifdef DEBUG_SPIFFS_LOGGER
  long ttime = millis();
  Serial.println("Writing file");
  #endif
  _openFor(0, Appending);
  if (!_file[0]) {
    #ifdef DEBUG_SPIFFS_LOGGER
    Serial.println("error file could not be opened while writing");
    #endif
    return;
  }
  if (_file[0].size() > _maxFileSize) {
    _fileSizeLimitReached();
  }
  _file[0].print(str);
  _file[0].flush();
  // _file[0].close();
  #ifdef DEBUG_SPIFFS_LOGGER
  Serial.printf("Took: %lu ms\n", millis()-ttime);
  #endif
}

void SPIFFSLogger::flush() {
  if (!_autoFlush) {
  // Nothing to be done
    if (strlen(&buffer[0][0]) > 0) {
      if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) { 
        #ifdef DEBUG_SPIFFS_LOGGER
        Serial.println("flushing");
        #endif
        _bufferIndex = (_bufferIndex + 1)%BUFFERS;
        if (_truncated) {
          _write((char *)"Truncated...\n");
        }
        for (size_t i = 0; i < BUFFERS; i++) {
          size_t n = strlen(&buffer[_bufferIndex][0]);
          #ifdef DEBUG_SPIFFS_LOGGER
          Serial.printf("_bufferIndex %i, N: %i\n", _bufferIndex, n);
          #endif
          if (n > 0) {
            _write(&buffer[_bufferIndex][0]);
            // Reset buffer
            buffer[_bufferIndex][0] = '\0';
          }
          _bufferIndex = (_bufferIndex + 1)%BUFFERS;
        }
        _bufferIndex = 0;
        _truncated = false;
        #ifdef DEBUG_SPIFFS_LOGGER
        Serial.println("flushed");
        #endif
      }
      xSemaphoreGive(_mutex);
    }
  } else {
    _file[0].flush();
  }
}


void SPIFFSLogger::_fileSizeLimitReached() {
  #ifdef DEBUG_SPIFFS_LOGGER
  long ttime = millis();
  Serial.println("FileToolarge, copy over");
  #endif
  _file[0].close();
  for (size_t i = NUM_FILES-1; i >= 1; i--) {
    SPIFFS.remove(_fileName[i]);
    SPIFFS.rename(_fileName[i-1], _fileName[i]);
  }
  // open file again
  _openFor(0, Appending);
  #ifdef DEBUG_SPIFFS_LOGGER
  Serial.printf("Took: %lu ms\n", millis()-ttime);
  #endif
}

void SPIFFSLogger::listAllFiles() {
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
  while(file){
    _getFileSize(file);
    file = root.openNextFile();
  }
}


// ********************************* MULTI LOGGER ************************************* //

MultiLogger::MultiLogger() {
  for (size_t i = 0; i < _MAX_LOG_STREAMS; i++) loggers[i] = NULL;
  _timeStrGetter = NULL;
  w_mutex = xSemaphoreCreateMutex();
  _index = 0;
  _currentLog = INFO;
}

bool MultiLogger::init() {
  bool success = true;
  for (size_t i = 0; i < _MAX_LOG_STREAMS; i++) {
     if (loggers[i] != NULL) {
       success &= loggers[i]->init();
     }
  }
  for (size_t i = 0; i < _MAX_LOG_STREAMS; i++) {
     if (loggers[i] != NULL) {
       loggers[i]->flush();
     }
  }
  const char * r1 = _reset_reason(rtc_get_reset_reason(0));
  const char * r2 = _reset_reason(rtc_get_reset_reason(1));
  log(WARNING, "Resetted, reasons CPU 0: %s, 1: %s", r1, r2);

  for (size_t i = 0; i < _MAX_LOG_STREAMS; i++) {
     if (loggers[i] != NULL) {
       loggers[i]->flush();
     }
  }
  return success;
}

const char * MultiLogger::_logTypeToStr(LogType type) {
  switch(type) {
    case     ALL: return &LOG_ALL_TEXT[0];
    case   DEBUG: return &LOG_DEBUG_TEXT[0];
    case    INFO: return &LOG_INFO_TEXT[0];
    case WARNING: return &LOG_WARNING_TEXT[0];
    case   ERROR: return &LOG_ERROR_TEXT[0];
    default     : return &LOG_INFO_TEXT[0];
  }
}

const char * MultiLogger::_reset_reason(RESET_REASON reason) {
  // switch(reason) {
  //   case  1: return &TXT_POWERON_RESET[0];
  //   case  3: return &TXT_SW_RESET[0];
  //   case  4: return &TXT_OWDT_RESET[0];
  //   case  5: return &TXT_DEEPSLEEP_RESET[0];
  //   case  6: return &TXT_SDIO_RESET[0];
  //   case  7: return &TXT_TG0WDT_SYS_RESET[0];
  //   case  8: return &TXT_TG1WDT_SYS_RESET[0];
  //   case  9: return &TXT_RTCWDT_SYS_RESET[0];
  //   case 10: return &TXT_INTRUSION_RESET[0];
  //   case 11: return &TXT_TGWDT_CPU_RESET[0];
  //   case 12: return &TXT_SW_CPU_RESET[0];
  //   case 13: return &TXT_RTCWDT_CPU_RESET[0];
  //   case 14: return &TXT_EXT_CPU_RESET[0];
  //   case 15: return &TXT_RTCWDT_BROWN_OUT_RESET[0];
  //   case 16: return &TXT_RTCWDT_RTC_RESET[0];
  //   default: return &TXT_NO_MEAN[0];
  // }
  if (reason <= 16) return &MY_RESET_REASON_TXT[reason][0];
  else return &MY_RESET_REASON_TXT[0][0];
}


void MultiLogger::setTimeGetter(char * (*timeStrGetter)(void)) {
  _timeStrGetter = timeStrGetter;
}

bool MultiLogger::addLogger(Logger * logger) {
  bool success = false;
  for (size_t i = 0; i < _MAX_LOG_STREAMS; i++) {
    if (loggers[i] == NULL) {
      loggers[i] = logger;
      success = true;
      break;
    }
  }
  return success;
}

bool MultiLogger::removeLogger(Logger * logger) {
  bool success = false;
  for (size_t i = 0; i < _MAX_LOG_STREAMS; i++) {
    if (loggers[i] == logger) {
      loggers[i] = NULL;
      success = true;
      break;
    }
  }
return success;
}

void MultiLogger::append(const char* _log, ...) {
  va_list args;
  va_start(args, _log);
  _index += vsnprintf(&_logText[_index], _MAX_TEXT_LENGTH, _log, args);
  va_end(args);
}

void MultiLogger::append(char* _log, ...) {
  va_list VAList;
  va_start(VAList, _log);
  _index += vsnprintf(&_logText[_index], _MAX_TEXT_LENGTH, _log, VAList);
  va_end(VAList);
}

void MultiLogger::flushAppended(LogType type) {
  _currentLog = type;
  log("");
}

void MultiLogger::log(LogType type, char* _log, ...) {
  va_list args;
  va_start(args, _log);
  vsnprintf(&_logText[_index], _MAX_TEXT_LENGTH, _log, args);
  va_end(args);
  _currentLog = type;
  log(_logText);
}

void MultiLogger::log(char* _log, ...) {
  log((const char*)_log);
}


void MultiLogger::log(LogType type, const char* _log, ...) {
  va_list args;
  va_start(args, _log);
  vsnprintf(&_logText[_index], _MAX_TEXT_LENGTH, _log, args);
  va_end(args);
  _currentLog = type;
  log(_logText);
}

void MultiLogger::log(const char* _log, ...) {
  // Wait maximum time for semaphone to be available
  // Get time from time getter string
  char * timeStr = (char *)&NOTHING_TEXT[0];
  if (_timeStrGetter != NULL) {
    timeStr = _timeStrGetter();
  }
  va_list args;
  va_start(args, _log);
  vsnprintf(&_logText[_index], _MAX_TEXT_LENGTH, _log, args);
  va_end(args);

  const char * typeStr = LOG_TYPE_TEXT[_currentLog];
  // const char * typeStr = _logTypeToStr(_currentLog);

  for (size_t i = 0; i < _MAX_LOG_STREAMS; i++) {
    if (loggers[i] != NULL && (uint8_t)loggers[i]->_type <= (uint8_t)_currentLog) {
      snprintf(_aLLStr, _MAX_TEXT_LENGTH, "%s%s%s: %s\r\n", loggers[i]->_prefix, typeStr, timeStr, _logText);
      // Has to be a one liner since data might be written from somewhere else
      loggers[i]->write(_aLLStr);
    }
  }
  _index = 0;
  // Reset type string
  _currentLog = INFO;
}