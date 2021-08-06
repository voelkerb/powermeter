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
  snprintf(myConf.mqttServer, MAX_DNS_LEN, "");
  snprintf(myConf.streamServer, MAX_DNS_LEN, "");
  snprintf(myConf.timeServer, MAX_DNS_LEN, "");
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
    sprintf(&netConf.SSIDs[i][0], "");
    sprintf(&netConf.PWDs[i][0], "");
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
}


void Configuration::load() {

  uint32_t address = NAME_START_ADDRESS;
  EEPROM.get(address, netConf);
  address += sizeof(netConf);

  EEPROM.get(address, myConf);
  address += sizeof(myConf);
  // getRelayState();

  // sanity check
  if (isnan(myConf.calV) or myConf.calV == NULL or myConf.calV < 0.1 or myConf.calV > 10.0) myConf.calV = 1.0;
  if (isnan(myConf.calI) or myConf.calI == NULL or myConf.calI < 0.1 or myConf.calI > 10.0) myConf.calI = 1.0;
}

void Configuration::store() {
  uint32_t address = NAME_START_ADDRESS;
  EEPROM.put(address, netConf);
  address += sizeof(netConf);
  EEPROM.put(address, myConf);
  address += sizeof(myConf);
  EEPROM.commit();
}

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
  // EEPROM.put(CALIBRATION_V_START_ADDRESS, myConf.calV);
  // EEPROM.put(CALIBRATION_I_START_ADDRESS, myConf.calI);
  EEPROM.commit();
}

bool Configuration::getRelayState() {
  uint32_t address = NAME_START_ADDRESS + sizeof(netConf);
  EEPROM.get(address, myConf.relayState);
  // EEPROM.get(RELAY_STATE_START_ADDRESS, myConf.relayState);
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
  // unsigned int address = WIFI_START_ADDRESS;
  // netConf.numAPs = 0;
  // for (int ap = 0; ap < MAX_WIFI_APS; ap++) {
  //   loadString(address, netConf.SSIDs[ap], MAX_SSID_LEN);
  //   if (strlen(netConf.SSIDs[ap]) == 0) break;
  //   address += MAX_SSID_LEN+1;
  //   loadString(address, netConf.PWDs[ap], MAX_PWD_LEN);
  //   address += MAX_PWD_LEN+1;
  //   netConf.numAPs++;
  // }
  uint32_t address = NAME_START_ADDRESS;
  EEPROM.get(address, netConf);

}

void Configuration::storeWiFi() {
  // unsigned int address = WIFI_START_ADDRESS;
  // for (int ap = 0; ap < MAX_WIFI_APS; ap++) {
  //   storeString(address, netConf.SSIDs[ap]);
  //   address += MAX_SSID_LEN+1;
  //   storeString(address, netConf.PWDs[ap]);
  //   address += MAX_PWD_LEN+1;
  // }
  uint32_t address = NAME_START_ADDRESS;
  EEPROM.put(address, netConf);
  EEPROM.commit();
}


void Configuration::setMQTTServerAddress(char * serverAddress) {
  snprintf(myConf.mqttServer, MAX_DNS_LEN, serverAddress);
  // storeString(MQTT_START_ADDRESS, myConf.mqttServer);
  storeMyConf();
}

void Configuration::setTimeServerAddress(char * serverAddress) {
  snprintf(myConf.timeServer, MAX_DNS_LEN, serverAddress);
  // storeString(TIME_SERVER_ADDRESS, myConf.timeServer);
  storeMyConf();
}

void Configuration::setStreamServerAddress(char * serverAddress) {
  snprintf(myConf.streamServer, MAX_DNS_LEN, serverAddress);
  // storeString(STREAM_SERVER_ADDRESS, myConf.streamServer);
  storeMyConf();
}

void Configuration::setName(char * newName) {
  snprintf(netConf.name, MAX_DNS_LEN, newName);
  // storeString(NAME_START_ADDRESS, netConf.name);
  store();
}

// bool Configuration::loadString(unsigned int address, char * str, unsigned int max_len) {
//   bool terminated = false;
//   for (size_t i = 0; i < max_len; i++) {
//     EEPROM.get(address+i, str[i]);
//     if (str[i] == '\0') {
//       terminated = true;
//       break;
//     }
//   }
//   if (!terminated) str[0] = '\0';
//   return terminated;
// }

// bool Configuration::storeString(unsigned int address, char * str) {
//   uint8_t chars = strlen(str);
//   if (chars < MAX_STRING_LEN) {
//     for (uint8_t i = 0; i < chars; i++) {
//       EEPROM.put(address+i, str[i]);
//     }
//     EEPROM.put(address+chars, '\0');
//     EEPROM.commit();
//     return true;
//   } else {
//     return false;
//   }
// }
