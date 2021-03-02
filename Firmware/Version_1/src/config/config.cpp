/***************************************************
 Library for rtc time handling.

 Feel free to use the code as it is.

 Benjamin Völker, voelkerb@me.com
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

#include "config.h"

Configuration::Configuration() {
  sprintf(mqttServer, "");
  sprintf(streamServer, "");
  sprintf(timeServer, "");
  relayState = false;
  calV = calI = calP = 1.0;
}

void Configuration::init() {
  // Load the MDNS name from eeprom
  EEPROM.begin(EEPROM_SIZE);
  static_assert(EEPROM_SIZE <= 512, "Max eeprom size is 512");
}

void Configuration::makeDefault(bool resetName) {
  // We can do a reset but keep the name
  if (resetName) {
    sprintf(netConf.name, "powermeterX");
  }

  sprintf(mqttServer, NO_SERVER);
  sprintf(streamServer, NO_SERVER);
  sprintf(timeServer, "time.google.com");

  for (size_t i = 0; i < MAX_WIFI_APS; i++) {
    sprintf(&netConf.SSIDs[i][0], "");
    sprintf(&netConf.PWDs[i][0], "");
  }
  netConf.numAPs = 0;

  // TODO: Remove these credentials
  strcpy(netConf.SSIDs[netConf.numAPs], "esewifi");
  strcpy(netConf.PWDs[netConf.numAPs], "silkykayak943");
  netConf.numAPs++;
  strcpy(netConf.SSIDs[netConf.numAPs], "energywifi");
  strcpy(netConf.PWDs[netConf.numAPs], "silkykayak943");
  netConf.numAPs++;
  relayState = true;

  calV = calI = calP = 1.0;
}

void Configuration::load() {

  if (!loadString(NAME_START_ADDRESS, netConf.name, MAX_NAME_LEN)) {
    makeDefault();
    store();
  }
  loadWiFi();

  if (!loadString(MQTT_START_ADDRESS, mqttServer, MAX_IP_LEN)) {
    strcpy(mqttServer, NO_SERVER);
  }

  if (!loadString(STREAM_SERVER_ADDRESS, streamServer, MAX_IP_LEN)) {
    strcpy(streamServer, NO_SERVER);
  }

  if (!loadString(TIME_SERVER_ADDRESS, timeServer, MAX_DNS_LEN)) {
    strcpy(timeServer, NO_SERVER);
  }

  getRelayState();

  EEPROM.get(CALIBRATION_V_START_ADDRESS, calV);
  EEPROM.get(CALIBRATION_I_START_ADDRESS, calI);
  // sanity check
  if (isnan(calV) or calV == NULL or calV < 0.1 or calV > 10.0) calV = 1.0;
  if (isnan(calI) or calI == NULL or calI < 0.1 or calI > 10.0) calI = 1.0;
  calP = calV*calI;
}

void Configuration::store() {
  storeString(NAME_START_ADDRESS, netConf.name);
  storeWiFi();
  storeString(MQTT_START_ADDRESS, mqttServer);
  storeString(STREAM_SERVER_ADDRESS, streamServer);
  storeString(TIME_SERVER_ADDRESS, timeServer);
  EEPROM.put(RELAY_STATE_START_ADDRESS, relayState);
  EEPROM.put(CALIBRATION_V_START_ADDRESS, calV);
  EEPROM.put(CALIBRATION_I_START_ADDRESS, calI);
  EEPROM.commit();
}

void Configuration::setCalibration(float valueV, float valueI) {
  calV = valueV;
  calI = valueI;
  calP = calV*calI;
  EEPROM.put(CALIBRATION_V_START_ADDRESS, calV);
  EEPROM.put(CALIBRATION_I_START_ADDRESS, calI);
  EEPROM.commit();
}

bool Configuration::getRelayState() {
  EEPROM.get(RELAY_STATE_START_ADDRESS, relayState);
  return relayState;
}

void Configuration::setRelayState(bool value) {
  relayState = value;
  EEPROM.put(RELAY_STATE_START_ADDRESS, relayState);
  EEPROM.commit();
}

bool Configuration::removeWiFi(char * ssid) {
  int idx = -1;
  for (int ap = 0; ap < MAX_WIFI_APS; ap++) {
    if (strcmp(ssid, netConf.SSIDs[ap]) == 0) {
      idx = ap;
      break;
    }
  }
  if (idx == -1) return false;
  for (size_t i = idx; i < netConf.numAPs-1; i++) {
    strcpy(netConf.SSIDs[i], netConf.SSIDs[i+1]);
    strcpy(netConf.PWDs[i], netConf.PWDs[i+1]);
  }
  // Indicate end of array here
  strcpy(netConf.SSIDs[netConf.numAPs-1], "");
  strcpy(netConf.PWDs[netConf.numAPs-1], "");

  netConf.numAPs--;
  storeWiFi(); // Note a little bit overhead but who cares
  return true;
}

int Configuration::addWiFi(char * ssid, char * pwd) {
  int idx = -1;
  for (int ap = 0; ap < MAX_WIFI_APS; ap++) {
    if (strcmp(ssid, netConf.SSIDs[ap]) == 0) {
      idx = ap;
      break;
    }
  }
  if (idx != -1) return -1;
  if (netConf.numAPs >= MAX_WIFI_APS) return 0;
  strcpy(netConf.SSIDs[netConf.numAPs], ssid);
  strcpy(netConf.PWDs[netConf.numAPs], pwd);
  netConf.numAPs++;
  storeWiFi(); // NOTE: a little bit overhead but who cares
  return 1;
}

void Configuration::loadWiFi() {
  unsigned int address = WIFI_START_ADDRESS;
  netConf.numAPs = 0;
  for (int ap = 0; ap < MAX_WIFI_APS; ap++) {
    loadString(address, netConf.SSIDs[ap], MAX_SSID_LEN);
    if (strlen(netConf.SSIDs[ap]) == 0) break;
    address += MAX_SSID_LEN+1;
    loadString(address, netConf.PWDs[ap], MAX_PWD_LEN);
    address += MAX_PWD_LEN+1;
    netConf.numAPs++;
  }
}

void Configuration::storeWiFi() {
  unsigned int address = WIFI_START_ADDRESS;
  for (int ap = 0; ap < MAX_WIFI_APS; ap++) {
    storeString(address, netConf.SSIDs[ap]);
    address += MAX_SSID_LEN+1;
    storeString(address, netConf.PWDs[ap]);
    address += MAX_PWD_LEN+1;
  }
}


void Configuration::setMQTTServerAddress(char * serverAddress) {
  strcpy(mqttServer, serverAddress);
  storeString(MQTT_START_ADDRESS, mqttServer);
}

void Configuration::setTimeServerAddress(char * serverAddress) {
  strcpy(timeServer, serverAddress);
  storeString(TIME_SERVER_ADDRESS, timeServer);
}

void Configuration::setStreamServerAddress(char * serverAddress) {
  strcpy(streamServer, serverAddress);
  storeString(STREAM_SERVER_ADDRESS, streamServer);
}

void Configuration::setName(char * newName) {
  strcpy(netConf.name, newName);
  storeString(NAME_START_ADDRESS, netConf.name);
}

bool Configuration::loadString(unsigned int address, char * str, unsigned int max_len) {
  bool terminated = false;
  for (size_t i = 0; i < max_len; i++) {
    EEPROM.get(address+i, str[i]);
    if (str[i] == '\0') {
      terminated = true;
      break;
    }
  }
  if (!terminated) str[0] = '\0';
  return terminated;
}

bool Configuration::storeString(unsigned int address, char * str) {
  uint8_t chars = strlen(str);
  if (chars < MAX_STRING_LEN) {
    for (uint8_t i = 0; i < chars; i++) {
      EEPROM.put(address+i, str[i]);
    }
    EEPROM.put(address+chars, '\0');
    EEPROM.commit();
    return true;
  } else {
    return false;
  }
}
