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

#define MAX_WIFI_APS 4
#define MAX_STRING_LEN 16
#define MAX_NAME_LEN MAX_STRING_LEN
#define MAX_SSID_LEN MAX_STRING_LEN
#define MAX_PWD_LEN MAX_STRING_LEN
#define NAME_START_ADDRESS 0
#define WIFI_START_ADDRESS NAME_START_ADDRESS+MAX_NAME_LEN+1
#define EEPROM_SIZE WIFI_START_ADDRESS+MAX_WIFI_APS*(MAX_SSID_LEN+MAX_PWD_LEN+2)+1

class Configuration {
  public:
    Configuration();
    void load();
    void store();
    void makeDefault();

    void setName(char * newName);


    bool addWiFi(char * ssid, char * pwd);
    bool removeWiFi(char * ssid);
    void loadWiFi();
    void storeWiFi();

    char name[MAX_NAME_LEN];
    unsigned int numAPs;
    char wifiSSIDs[MAX_WIFI_APS][MAX_STRING_LEN];
    char wifiPWDs[MAX_WIFI_APS][MAX_STRING_LEN];

  private:

    bool storeString(unsigned int address, char * str);
    bool loadString(unsigned int address, char * str, unsigned int max_len);
};

#endif
