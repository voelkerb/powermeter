/***************************************************
 Library for network stuff, connection, AP and so on.

 Feel free to use the code as it is.

 Benjamin Völker, voelkerb@me.com
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

#ifndef NETWORK_h
#define NETWORK_h

#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <Ticker.h>
#include "config.h"

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#define DEBUG
// #define DEBUG_DEEP
#define CHECK_PERIODE 10

namespace Network {
    static bool connected;
    static Configuration * _config;
    static void (*_onConnect)(void);
    static void (*_onDisconnect)(void);
    static bool _apMode;
    static Ticker checker;

    void init(Configuration * config);
    void init(Configuration * config, void (*onConnect)(void), void (*onDisconnect)(void));
    bool update();
    bool connect(char * network, char * pswd);
    void setupAP();
    void wifiEvent(WiFiEvent_t event);
    void checkNetwork();
    void scanNetwork( void * pvParameters );

    static uint16_t _scanCount;
    static void* _scanResult;
}

#endif
