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
#define MAX_WIFI_APS 4
#define MAX_STRING_LEN 25
#define MAX_IP_LEN 17
#define MAX_DNS_LEN 25
#define MAX_NAME_LEN MAX_STRING_LEN
#define MAX_SSID_LEN MAX_STRING_LEN
#define MAX_PWD_LEN MAX_STRING_LEN

#define NAME_START_ADDRESS 0
#define WIFI_START_ADDRESS NAME_START_ADDRESS+MAX_NAME_LEN+1
#define MQTT_START_ADDRESS WIFI_START_ADDRESS+MAX_WIFI_APS*(MAX_SSID_LEN+MAX_PWD_LEN+2)
#define RELAY_STATE_START_ADDRESS MQTT_START_ADDRESS+MAX_DNS_LEN+1
#define STREAM_SERVER_ADDRESS RELAY_STATE_START_ADDRESS+1
#define TIME_SERVER_ADDRESS STREAM_SERVER_ADDRESS+MAX_DNS_LEN+1

#define CALIBRATION_V_START_ADDRESS TIME_SERVER_ADDRESS+MAX_DNS_LEN+1
#define CALIBRATION_I_START_ADDRESS CALIBRATION_V_START_ADDRESS+sizeof(float)

// Add configs here
#define EEPROM_SIZE CALIBRATION_I_START_ADDRESS+sizeof(float)+2

#define NO_SERVER "-"
 
#define MAX_LOADER_SIZE 5

struct MeterConfiguration {
  //NOTE: Relay state must be first in configuration
  bool relayState = false;                  // State of the relay
  float calI = 1.0f;                        // Calibration parameter for current
  float calV = 1.0f;                        // Calibration parameter for voltage
  float energy = 0.0f;                      // amount of energy consumed (Wh)
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
    void setCalibration(float valueV, float valueI);

    NetworkConf netConf;
    MeterConfiguration myConf;

  private:
    void storeMyConf();
    // bool storeString(unsigned int address, char * str);
    // bool loadString(unsigned int address, char * str, unsigned int max_len);
    // bool loadBool(unsigned int address, char * str, unsigned int max_len);
    // bool relayState;
};

#endif
