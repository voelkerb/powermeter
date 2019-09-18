/***************************************************
 Library for rtc time handling.

 Feel free to use the code as it is.

 Benjamin Völker, voelkerb@me.com
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

#include "config.h"

Configuration::Configuration() {
  for (int i = 0; i < MAX_NAME_LEN; i++) name[i] = '\0';
  for (int i = 0; i < MAX_WIFI_APS; i++) {
    for (int j = 0; j < MAX_SSID_LEN; j++) wifiSSIDs[i][j] = '\0';
    for (int j = 0; j < MAX_PWD_LEN; j++) wifiPWDs[i][j] = '\0';
  }
  numAPs = 0;
}

void Configuration::makeDefault() {
  for (int i = 0; i < MAX_NAME_LEN; i++) name[i] = '\0';
  for (int i = 0; i < MAX_WIFI_APS; i++) {
    for (int j = 0; j < MAX_SSID_LEN; j++) wifiSSIDs[i][j] = '\0';
    for (int j = 0; j < MAX_PWD_LEN; j++) wifiPWDs[i][j] = '\0';
  }
  numAPs = 0;
  strcpy(name, "powerMeterX");

  // TODO: Remove these credentials
  strcpy(wifiSSIDs[0], "esewifi");
  strcpy(wifiPWDs[0], "silkykayak943");
  numAPs++;
  strcpy(wifiSSIDs[1], "energywifi");
  strcpy(wifiPWDs[1], "silkykayak943");
  numAPs++;
  store();
}

void Configuration::load() {
  // Load the MDNS name from eeprom
  EEPROM.begin(EEPROM_SIZE);

  if (!loadString(NAME_START_ADDRESS, name, MAX_NAME_LEN)) {
    makeDefault();
    store();
  }
  loadWiFi();
}

void Configuration::store() {
  storeString(NAME_START_ADDRESS, name);
  storeWiFi();
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
  for (int i = idx; i < numAPs-1; i++) {
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

bool Configuration::addWiFi(char * ssid, char * pwd) {
  if (numAPs >= MAX_WIFI_APS) return false;
  strcpy(wifiSSIDs[numAPs], ssid);
  strcpy(wifiPWDs[numAPs], pwd);
  numAPs++;
  storeWiFi(); // NOTE: a little bit overhead but who cares
  return true;
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

void Configuration::setName(char * newName) {
  strcpy(name, newName);
  storeString(NAME_START_ADDRESS, name);
}

bool Configuration::loadString(unsigned int address, char * str, unsigned int max_len) {
  bool terminated = false;
  for (int i = 0; i < max_len; i++) {
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
    for (uint8_t i = 0; i <= chars; i++) {
      EEPROM.put(address+i, str[i]);
    }
    EEPROM.commit();
    return true;
  } else {
    return false;
  }
}
