/***************************************************
 Library for rtc time handling.

 Feel free to use the code as it is.

 Benjamin Völker, voelkerb@me.com
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

#ifndef CONFIG_h
#define CONFIG_h

#include <EEPROM.h>

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// Include for network config
#include "../network/src/network.h"

// Define standard wifis settings such as in the following example
// In external privateConfig.h. It will be included automatically (see config.cpp) 
// #define NUM_STANDARD_WIFIS 2
// const char * MY_STANDARD_WIFIS[] = {
//     "SSID1",  "PWD1",
//     "SSID2",  "PWD2"
// };


// If you cahange any of these values, the config on all devices will be bricked
#define MAX_STRING_LEN 25
#define MAX_IP_LEN 17
#define MAX_DNS_LEN 25
#define MAX_NAME_LEN MAX_STRING_LEN

#define NAME_START_ADDRESS 0
#define EEPROM_SIZE sizeof(NetworkConf)+sizeof(MeterConfiguration)+2

#define NO_SERVER "-"

// packed required to store in EEEPROM efficiently
struct __attribute__((__packed__)) MeterConfiguration {
  //NOTE: Relay state must be first in configuration
  bool relayState = false;                  // State of the relay
  double energy = 0.0;                      // amount of energy consumed (Wh)
  float calI = 1.0f;                        // Calibration parameter for current
  float calV = 1.0f;                        // Calibration parameter for voltage
  char mqttServer[MAX_DNS_LEN] = {'\0'};    // MQTT Server DNS name
  char streamServer[MAX_DNS_LEN] = {'\0'};  // Stream Server DNS name
  char timeServer[MAX_DNS_LEN] = {'\0'};    // Time Server DNS name
  int8_t resetHour = -1;                    // Perform reliability reset (hour)
  int8_t resetMinute = -1;                  // Perform reliability reset (minute) -1 to disable
}; 

class Configuration {
  public:
    Configuration();
    void init();
    void load();
    void store();
    void makeDefault(bool resetName=true);

    void setName(char * newName);

    int addWiFi(char * ssid, char * pwd);
    bool removeWiFi(char * ssid);
    void loadWiFi();
    void storeWiFi();
    void setMQTTServerAddress(char * serverAddress);
    void setStreamServerAddress(char * serverAddress);
    void setTimeServerAddress(char * serverAddress);
    bool getRelayState();
    void setRelayState(bool value);
    void setEnergy(double energy);
    void setCalibration(float valueV, float valueI);

    NetworkConf netConf;
    MeterConfiguration myConf;

  private:
    void storeMyConf();
};

#endif
