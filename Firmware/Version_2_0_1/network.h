/***************************************************
 Library for network stuff, connection, AP and so on.

 Feel free to use the code as it is.

 Benjamin Völker, voelkerb@me.com
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

#ifndef NETWORK_h
#define NETWORK_h

// Wifi stuff
#include <esp_wifi.h>
// Wifi and TCP/UDP Server stuff
#include <WiFi.h>
#include <WiFiAP.h>
// Ethernet stuff
#include <ETH.h>
// Both to set bt to sleep
#include <esp_sleep.h>
#include <esp_bt.h>
// Logging and configuration stuff
#include "logger.h"
#include "config.h"

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
// #define DEBUG_DEEP
#define CHECK_PERIODE_MS 60000

namespace Network {
    extern MultiLogger& logger;
    extern bool connected;
    extern bool apMode;
    extern bool ethernet;
    extern bool allowNetworkChange;

    void init(Configuration * config);
    void init(Configuration * config, void (*onConnect)(void), void (*onDisconnect)(void), bool usingEthernet=false);
    void initPHY(uint8_t addr, uint8_t pwr, uint8_t mdc, uint8_t mdio, eth_phy_type_t type, eth_clock_mode_t clk_mode);
    bool update();
    bool connect(char * network, char * pswd);
    void setupAP();
    IPAddress localIP();
    void wifiEvent(WiFiEvent_t event);
    void checkNetwork();
    void scanNetwork( void * pvParameters );

}

#endif
