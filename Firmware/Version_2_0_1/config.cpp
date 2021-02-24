/***************************************************
 Library for rtc time handling.

 Feel free to use the code as it is.

 Benjamin Völker, voelkerb@me.com
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

#include "config.h"

Configuration::Configuration() {
  for (size_t i = 0; i < MAX_NAME_LEN; i++) name[i] = '\0';
  for (size_t i = 0; i < MAX_WIFI_APS; i++) {
    for (int j = 0; j < MAX_SSID_LEN; j++) wifiSSIDs[i][j] = '\0';
    for (int j = 0; j < MAX_PWD_LEN; j++) wifiPWDs[i][j] = '\0';
  }
  for (size_t i = 0; i < MAX_IP_LEN; i++) mqttServer[i] = '\0';
  for (size_t i = 0; i < MAX_IP_LEN; i++) streamServer[i] = '\0';
  for (size_t i = 0; i < MAX_DNS_LEN; i++) timeServer[i] = '\0';
  numAPs = 0;
  relayState = false;
}

void Configuration::init() {
  // Load the MDNS name from eeprom
  EEPROM.begin(EEPROM_SIZE);
  static_assert(EEPROM_SIZE <= 512, "Max eeprom size is 512");
}

void Configuration::makeDefault(bool resetName) {
  // We can do a reset but keep the name
  if (resetName) {
    for (size_t i = 0; i < MAX_NAME_LEN; i++) name[i] = '\0';
    strcpy(name, "powermeterX");
  }

  for (size_t i = 0; i < MAX_IP_LEN; i++) mqttServer[i] = '\0';
  strcpy(mqttServer, NO_SERVER);

  for (size_t i = 0; i < MAX_IP_LEN; i++) streamServer[i] = '\0';
  strcpy(streamServer, NO_SERVER);
  
  for (size_t i = 0; i < MAX_DNS_LEN; i++) timeServer[i] = '\0';
  strcpy(timeServer, "de.pool.ntp.org");

  for (size_t i = 0; i < MAX_WIFI_APS; i++) {
    for (int j = 0; j < MAX_SSID_LEN; j++) wifiSSIDs[i][j] = '\0';
    for (int j = 0; j < MAX_PWD_LEN; j++) wifiPWDs[i][j] = '\0';
  }
  numAPs = 0;
  // TODO: Remove these credentials
  strcpy(wifiSSIDs[numAPs], "esewifi");
  strcpy(wifiPWDs[numAPs], "silkykayak943");
  numAPs++;
  strcpy(wifiSSIDs[numAPs], "energywifi");
  strcpy(wifiPWDs[numAPs], "silkykayak943");
  numAPs++;
  relayState = true;
}

void Configuration::load() {

  if (!loadString(NAME_START_ADDRESS, name, MAX_NAME_LEN)) {
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
}

void Configuration::store() {
  storeString(NAME_START_ADDRESS, name);
  storeWiFi();
  storeString(MQTT_START_ADDRESS, mqttServer);
  storeString(STREAM_SERVER_ADDRESS, streamServer);
  storeString(TIME_SERVER_ADDRESS, timeServer);
  EEPROM.put(RELAY_STATE_START_ADDRESS, relayState);
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
    if (strcmp(ssid, wifiSSIDs[ap]) == 0) {
      idx = ap;
      break;
    }
  }
  if (idx == -1) return false;
  for (size_t i = idx; i < numAPs-1; i++) {
    strcpy(wifiSSIDs[i], wifiSSIDs[i+1]);
    strcpy(wifiPWDs[i], wifiPWDs[i+1]);
  }
  // Indicate end of array here
  strcpy(wifiSSIDs[numAPs-1], "");
  strcpy(wifiPWDs[numAPs-1], "");

  numAPs--;
  storeWiFi(); // Note a little bit overhead but who cares
  return true;
}

int Configuration::addWiFi(char * ssid, char * pwd) {
  int idx = -1;
  for (int ap = 0; ap < MAX_WIFI_APS; ap++) {
    if (strcmp(ssid, wifiSSIDs[ap]) == 0) {
      idx = ap;
      break;
    }
  }
  if (idx != -1) return -1;
  if (numAPs >= MAX_WIFI_APS) return 0;
  strcpy(wifiSSIDs[numAPs], ssid);
  strcpy(wifiPWDs[numAPs], pwd);
  numAPs++;
  storeWiFi(); // NOTE: a little bit overhead but who cares
  return 1;
}

void Configuration::loadWiFi() {
  unsigned int address = WIFI_START_ADDRESS;
  numAPs = 0;
  for (int ap = 0; ap < MAX_WIFI_APS; ap++) {
    loadString(address, wifiSSIDs[ap], MAX_SSID_LEN);
    if (strlen(wifiSSIDs[ap]) == 0) break;
    address += MAX_SSID_LEN+1;
    loadString(address, wifiPWDs[ap], MAX_PWD_LEN);
    address += MAX_PWD_LEN+1;
    numAPs++;
  }
}

void Configuration::storeWiFi() {
  unsigned int address = WIFI_START_ADDRESS;
  for (int ap = 0; ap < MAX_WIFI_APS; ap++) {
    storeString(address, wifiSSIDs[ap]);
    address += MAX_SSID_LEN+1;
    storeString(address, wifiPWDs[ap]);
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
  strcpy(name, newName);
  storeString(NAME_START_ADDRESS, name);
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
