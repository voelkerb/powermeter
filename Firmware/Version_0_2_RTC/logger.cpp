/***************************************************
 Library for logging.

 Feel free to use the code as it is.

 Benjamin Völker, voelkerb@me.com
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

#include "logger.h"


MultiLogger::MultiLogger() {
  for (int i = 0; i < MAX_LOG_STREAMS; i++) loggers[i] = NULL;
  _timeStrGetter = NULL;
  w_mutex = xSemaphoreCreateMutex();
  _index = 0;
  _currentLog = INFO;
}

void MultiLogger::setTimeGetter(char * (*timeStrGetter)(void)) {
  _timeStrGetter = timeStrGetter;
}

bool MultiLogger::addLogger(Logger * logger) {
  bool success = false;
  // Makes adding thread safe
  if (xSemaphoreTake(w_mutex, portMAX_DELAY) == pdTRUE) { 
    for (int i = 0; i < MAX_LOG_STREAMS; i++) {
      if (loggers[i] == NULL) {
        loggers[i] = logger;
        success = true;
        break;
      }
    }
    xSemaphoreGive(w_mutex);
  }
  return success;
}

bool MultiLogger::removeLogger(Logger * logger) {
  bool success = false;
  // Makes removing thread safe
  if (xSemaphoreTake(w_mutex, portMAX_DELAY) == pdTRUE) { 
    for (int i = 0; i < MAX_LOG_STREAMS; i++) {
      if (loggers[i] == logger) {
        loggers[i] = NULL;
        success = true;
        break;
      }
    }
    xSemaphoreGive(w_mutex);
  }
  return success;
}

const char * MultiLogger::logTypeToStr(LogType type) {
  if (type == ALL) return &ALL_TEXT[0];
  else if (type == INFO) return &INFO_TEXT[0];
  else if (type == WARNING) return &WARNING_TEXT[0];
  else if (type == ERROR) return &ERROR_TEXT[0];
  else if (type == DEBUG) return &DEBUG_TEXT[0];
  else return &ALL_TEXT[0];
}

void MultiLogger::append(const char* _log, ...) {
  va_list args;
  va_start(args, _log);
  _index += vsprintf(&_logText[_index], _log, args);
  va_end(args);
}

void MultiLogger::append(char* _log, ...) {
  va_list VAList;
  va_start(VAList, _log);
  _index += vsprintf(&_logText[_index], _log, VAList);
  va_end(VAList);
}

void MultiLogger::flush(LogType type) {
  _currentLog = type;
  log("");
}

void MultiLogger::log(LogType type, char* _log, ...) {
  va_list args;
  va_start(args, _log);
  vsprintf(&_logText[_index], _log, args);
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
  vsprintf(&_logText[_index], _log, args);
  va_end(args);
  _currentLog = type;
  log(_logText);
}

void MultiLogger::log(const char* _log, ...) {
  // Wait maximum time for semaphone to be available
  if (xSemaphoreTake(w_mutex, portMAX_DELAY) == pdTRUE) {
    // Get time from time getter string
    char * timeStr = (char *)&NOTHING_TEXT[0];
    if (_timeStrGetter != NULL) {
      timeStr = _timeStrGetter();
    }
    va_list args;
    va_start(args, _log);
    vsprintf(&_logText[_index], _log, args);
    va_end(args);

    const char * typeStr = logTypeToStr(_currentLog);
    for (int i = 0; i < MAX_LOG_STREAMS; i++) {
      if (loggers[i] != NULL && (uint8_t)loggers[i]->_type <= (uint8_t)_currentLog) {
        sprintf(_aLLStr, "%s%s%s: %s\r\n", loggers[i]->_prefix, typeStr, timeStr, _logText);
        // Has to be a one liner since data might be written from somewhere else
        loggers[i]->write(_aLLStr);
      }
    }
    _index = 0;
    // Reset type string
    _currentLog = INFO;
    xSemaphoreGive(w_mutex);
  } else {
    // Typically this should not happen
    // TODO: recovery from here?
  }
}