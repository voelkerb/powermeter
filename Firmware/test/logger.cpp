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
}
const char * Logger::logTypeToStr(LogType type) {
  if (type == ALL) return &ALL_TEXT[0];
  else if (type == INFO) return &INFO_TEXT[0];
  else if (type == WARNING) return &WARNING_TEXT[0];
  else if (type == ERROR) return &ERROR_TEXT[0];
  else if (type == DEBUG) return &DEBUG_TEXT[0];
  else return &ALL_TEXT[0];
}


// ********************************* STREAM LOGGER ************************************* //
StreamLogger::StreamLogger(Stream * stream, char * (*timeStrGetter)(void), const char * prefix, LogType type):Logger(timeStrGetter, prefix, type) {
  _stream = stream;
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


#define DEBUG_SPIFFS_LOGGER
// ********************************* SPIFFS LOGGER ************************************* //
SPIFFSLogger::SPIFFSLogger(const char * fileName, char * (*timeStrGetter)(void), const char * prefix, LogType type, int32_t maxFileSize):Logger(timeStrGetter, prefix, type) {
  strcpy(_fileName[0], fileName);
  strcpy(_fileName[1], fileName);
  int index = strlen(fileName);
  strcpy(&_fileName[1][index], "1");
  _maxFileSize = maxFileSize;
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
  for (int i = 0; i < NUM_FILES; i++) {
    if (SPIFFS.exists(_fileName[i])) Serial.printf("File %s exists\n", _fileName[i]);
    else Serial.printf("File %s does not exist, t will be created\n", _fileName[i]);
  }
  #endif
  
  // This will create files if they do not exist
  for (int i = 0; i < NUM_FILES; i++) _openFor(i, Appending);
  for (int i = 0; i < NUM_FILES; i++) {
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

  for (int i = 0; i < NUM_FILES; i++) _rowsFile[i] = -1;
  
  #ifdef DEBUG_SPIFFS_LOGGER
  for (int i = 0; i < NUM_FILES; i++) {
    _openFor(i, Reading);
    _rowsFile[i] = _getRowsInFile(_file[i]);
    Serial.printf("#Rows %s: %i \n", _file[i].name(), _rowsFile[i]);
  }
  #endif

  _currentRow = 0;
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
  return true;
}

int SPIFFSLogger::logRows() {
  int rows = 0;
  for (int i = 0; i < NUM_FILES; i++) {
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
  for (int i = 0; i < NUM_FILES; i++) {
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
  for (int i = 0; i < NUM_FILES; i++) {
    success &= SPIFFS.remove(_fileName[i]);
    #ifdef DEBUG_SPIFFS_LOGGER
    if (success) {
      Serial.printf("File %s cleared\n", _fileName[i]);
    } else {
      Serial.printf("Error clearing %s\n", _fileName[i]);
    }
    #endif
  }
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
  #ifdef DEBUG_SPIFFS_LOGGER
  Serial.printf("Took: %lu ms\n", millis()-ttime);
  #endif
}

void SPIFFSLogger::flush() {
  _file[0].flush();
}

void SPIFFSLogger::_fileSizeLimitReached() {
  #ifdef DEBUG_SPIFFS_LOGGER
  long ttime = millis();
  Serial.println("FileToolarge, copy over");
  #endif
  _file[0].close();
  for (int i = NUM_FILES-1; i >= 1; i--) {
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