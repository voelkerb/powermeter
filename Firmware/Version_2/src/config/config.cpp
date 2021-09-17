/***************************************************
 Library for Configuration handling.

 Feel free to use the code as it is.

 Benjamin Voelker, voelkerb@me.com
 Embedded Systems
 University of Freiburg, Institute of Computer Science
 ****************************************************/

#include "config.h"
#if __has_include("privateConfig.h")
#include "privateConfig.h"
#endif

Configuration::Configuration() {
  snprintf(myConf.mqttServer, MAX_DNS_LEN, NO_SERVER);
  snprintf(myConf.streamServer, MAX_DNS_LEN, NO_SERVER);
  snprintf(myConf.timeServer, MAX_DNS_LEN, NO_SERVER);
  myConf.relayState = false;
  myConf.calV = myConf.calI = 1.0;
  myConf.energy = 0.0;
  myConf.resetHour = myConf.resetMinute = -1;
}

void Configuration::init() {
  // Load the MDNS name from eeprom
  EEPROM.begin(EEPROM_SIZE);
  static_assert(EEPROM_SIZE <= 512, "Max eeprom size is 512");
}

void Configuration::makeDefault(bool resetName) {
  // We can do a reset but keep the name
  if (resetName) {
    snprintf(netConf.name, MAX_NAME_LEN, "powermeterX");
  }

  snprintf(myConf.mqttServer, MAX_DNS_LEN, NO_SERVER);
  snprintf(myConf.streamServer, MAX_DNS_LEN, NO_SERVER);
  snprintf(myConf.timeServer, MAX_DNS_LEN, "time.google.com");

  for (size_t i = 0; i < MAX_WIFI_APS; i++) {
    sprintf(&netConf.SSIDs[i][0], NO_SERVER);
    sprintf(&netConf.PWDs[i][0], NO_SERVER);
  }
  netConf.numAPs = 0;

  // #define in privateConfig
  #ifdef NUM_STANDARD_WIFIS
  for (int i = 0; i < NUM_STANDARD_WIFIS; i++) {
    strcpy(netConf.SSIDs[netConf.numAPs], MY_STANDARD_WIFIS[i*2]);
    strcpy(netConf.PWDs[netConf.numAPs], MY_STANDARD_WIFIS[i*2+1]);
    netConf.numAPs++;
  }
  #endif
  myConf.relayState = true;

  myConf.calV = myConf.calI = 1.0f;
  myConf.energy = 0.0;
  myConf.resetHour = myConf.resetMinute = -1;

  #ifdef SENSOR_BOARD
  sbConf->tempOffset = 0.0;
  sbConf->humOffset = 0.0;
  sbConf->brightness = 1.0;
  sbConf->lightCal = 1.0;
  sbConf->minLEDWatt = STD_MIN_LED_WATT;
  sbConf->maxLEDWatt = STD_MAX_LED_WATT;
  #endif
}


void Configuration::load() {

  uint32_t address = NAME_START_ADDRESS;
  EEPROM.get(address, netConf);
  // Spot invalid config
  if (netConf.numAPs > MAX_WIFI_APS) {
    makeDefault(false);
    store();
    return;
  }
  address += sizeof(netConf);

  EEPROM.get(address, myConf);
  address += sizeof(myConf);
  
  #ifdef SENSOR_BOARD
  EEPROM.get(address, *sbConf);
  address += sizeof(*sbConf);
  #endif
  // getRelayState();

  // sanity check
  if (isnan(myConf.calV) or !myConf.calV or myConf.calV < 0.1 or myConf.calV > 10.0) myConf.calV = 1.0;
  if (isnan(myConf.calI) or !myConf.calI or myConf.calI < 0.1 or myConf.calI > 10.0) myConf.calI = 1.0;
}

void Configuration::store() {
  uint32_t address = NAME_START_ADDRESS;
  EEPROM.put(address, netConf);
  address += sizeof(netConf);
  EEPROM.put(address, myConf);
  address += sizeof(myConf);
  #ifdef SENSOR_BOARD
  EEPROM.put(address, *sbConf);
  address += sizeof(*sbConf);
  #endif
  EEPROM.commit();
}

#ifdef SENSOR_BOARD
void Configuration::storeSensorBoard() {
  uint32_t address = NAME_START_ADDRESS + sizeof(netConf) + sizeof(myConf);
  EEPROM.put(address, *sbConf);
  EEPROM.commit();
}
#endif

void Configuration::storeMyConf() {
  uint32_t address = NAME_START_ADDRESS + sizeof(netConf);
  EEPROM.put(address, myConf);
  EEPROM.commit();
}

void Configuration::setCalibration(float valueV, float valueI) {
  myConf.calV = valueV;
  myConf.calI = valueI;

  uint32_t address = NAME_START_ADDRESS + sizeof(netConf);
  EEPROM.put(address, myConf);
  EEPROM.commit();
}

bool Configuration::getRelayState() {
  uint32_t address = NAME_START_ADDRESS + sizeof(netConf);
  EEPROM.get(address, myConf.relayState);
  return myConf.relayState;
}

void Configuration::setEnergy(double energy) {
  uint32_t address = NAME_START_ADDRESS + sizeof(netConf) + sizeof(myConf.relayState);
  myConf.energy = energy;
  EEPROM.put(address, myConf.energy);
  EEPROM.commit();
  // storeMyConf();
}

void Configuration::setRelayState(bool value) {
  uint32_t address = NAME_START_ADDRESS + sizeof(netConf);
  myConf.relayState = value;
  EEPROM.put(address, myConf.relayState);
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
  uint32_t address = NAME_START_ADDRESS;
  EEPROM.get(address, netConf);

}

void Configuration::storeWiFi() {
  uint32_t address = NAME_START_ADDRESS;
  EEPROM.put(address, netConf);
  EEPROM.commit();
}


void Configuration::setMQTTServerAddress(char * serverAddress) {
  snprintf(myConf.mqttServer, MAX_DNS_LEN, serverAddress);
  storeMyConf();
}

void Configuration::setTimeServerAddress(char * serverAddress) {
  snprintf(myConf.timeServer, MAX_DNS_LEN, serverAddress);
  storeMyConf();
}

void Configuration::setStreamServerAddress(char * serverAddress) {
  snprintf(myConf.streamServer, MAX_DNS_LEN, serverAddress);
  storeMyConf();
}

void Configuration::setName(char * newName) {
  snprintf(netConf.name, MAX_DNS_LEN, newName);
  EEPROM.put(NAME_START_ADDRESS, netConf);
  EEPROM.commit();
}