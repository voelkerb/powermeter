#include <Arduino.h>
#include <ArduinoOTA.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <time.h>
#include <ArduinoJson.h>
#include <TimeLib.h>
#include <ESPmDNS.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
#include <esp_sleep.h>
#include <esp_bt.h>
#include <esp32-hal-cpu.h>

#include "constDefine.h"
#include "src/logger/logger.h"


char * timeStr();
void onWifiDisconnect();
void onWifiConnect();
void onClientConnect(WiFiClient *newClient);
void onClientDisconnect(WiFiClient *oldClient);

// Serial logger
StreamLogger serialLog((Stream*)&Serial, &timeStr, &LOG_PREFIX_SERIAL[0], ALL);

// MultiLogger logger(&streamLog, &timeStr);
// Create singleton here
MultiLogger& logger = MultiLogger::getInstance();


// TCP clients and current connected state, each client is assigned a logger
WiFiClient client;
// OTA Update in progress
bool clientConnected = false;

// tcp stuff is send over this client 
// Init getter to point to sth, might not work otherwise
Stream * newGetter = &Serial;

// Some timer stuff s.t. things are updated regularly and not at full speed
long lifenessUpdate = millis();
long tcpUpdate = millis();

// Current CPU speed
unsigned int coreFreq = 0;

// test stuff
long testMillis = 0;

// OTA Update in progress
bool updating = false;

bool wifiConnected = false;


// Command stuff send over what ever
char command[COMMAND_MAX_SIZE] = {'\0'};
char command2[COMMAND_MAX_SIZE] = {'\0'};
StaticJsonDocument<2*COMMAND_MAX_SIZE> docRcv;
StaticJsonDocument<2*COMMAND_MAX_SIZE> docSend;
StaticJsonDocument<COMMAND_MAX_SIZE> docSample;
String response = "";

/************************ SETUP *************************/
void setup() {
  // Setup serial communication
  Serial.begin(SERIAL_SPEED);

  // Init the logging module
  logger.setTimeGetter(&timeStr);
  // Add Serial logger
  logger.addLogger(&serialLog);
  // Init all loggers
  logger.init();

  // We do not need bluetooth, so disable it
  esp_bt_controller_disable();

  coreFreq = getCpuFrequencyMhz();

  logger.log(DEBUG, "%s @ firmware %s/%s", DEVICE_NAME, __DATE__, __TIME__);
  logger.log(DEBUG, "Core @ %u MHz", coreFreq);


  response.reserve(2*COMMAND_MAX_SIZE);
  initWifi();
  logger.log(ALL, "Setup done");
}

/************************ Loop *************************/
void loop() {
  // Arduino OTA
  ArduinoOTA.handle();
  if (updating) return;

  // Handle serial requests
  if (Serial.available()) {
    handleEvent(&Serial);
  }

  // Handle tcp requests
  if (client.available() > 0) {
    handleEvent(&client);
  }

  // Handle tcp clients connections

  if ((long)(millis() - tcpUpdate) >= 0) {
    tcpUpdate += TCP_UPDATE_INTERVAL;
    // Handle disconnect
    if (clientConnected and !client.connected()) {
      onClientDisconnect(&client);
      clientConnected = false;
    }
  }
}


void wifiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_SCAN_DONE:
      break;
    case SYSTEM_EVENT_STA_CONNECTED:
      wifiConnected = true;
      logger.log("STA_CONNECTED");
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED: {
      bool wasConnected = wifiConnected;
      // Recheck for wifi networks
      wifiConnected = false;
      // lets check for networks regularly
      logger.log("STA_DISCONNECTED");
      onWifiDisconnect();
      if (wasConnected) checkNetwork();
      break;
    }
    case SYSTEM_EVENT_STA_GOT_IP:
      wifiConnected = true;
      logger.log("STA_GOT_IP");
      onWifiConnect();
      break;
    default:
      break;
  }
}


void initWifi() {
  // Forget any previously set configuration
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
  // Disconnect if we were connected
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.onEvent(&wifiEvent);
  WiFi.setHostname(DEVICE_NAME);
  // Disable wifi power saving
  esp_wifi_set_ps(WIFI_PS_NONE);
  checkNetwork();
}

void checkNetwork() {
  // let it check for networks on second core (not in loop)
  xTaskCreatePinnedToCore(
                    scanNetwork,   /* Function to implement the task */
                    "scanTask", /* Name of the task */
                    10000,      /* Stack size in words */
                    NULL,       /* Task input parameter */
                    1,          /* Priority of the task */
                    NULL,       /* Task handle. */
                    1);  /* Core where the task should run */

}

void scanNetwork( void * pvParameters ) {
  // Indicating if we are connected to a wifi station
  bool staConnected = false;
  if (wifiConnected)  {
    staConnected = true;
  }

  while (not staConnected) {
    
    logger.log(INFO, "Scanning for Wifi Networks");
    int n = WiFi.scanNetworks();
    if (n == -2) {
      logger.log(ERROR, "Scan failed");
    } else if (n == -1) {
      logger.log(WARNING, "Scan already in progress");
    } else if (n == 0) {
      logger.log(INFO, "No network found");
    } else {
      logger.log("Scan done %i networks found", n);
      for (size_t i = 0; i < (size_t)n; ++i) {
        logger.log("%s (%i)", WiFi.SSID(i).c_str(), WiFi.RSSI(i));
      }
    }

    char testSSID[POWERMETER_MDNS_LENGHT+1] = {'\0'};
    int found = -1;
    // Only if we have found any network, we can search for a known one
    if (n > 0) {
      for (size_t i = 0; i < (size_t)n; ++i) {
        strncpy(&testSSID[0], WiFi.SSID(i).c_str(), POWERMETER_MDNS_LENGHT);

        if (strcmp(testSSID, POWERMETER_MDNS) == 0) {
          found = i;
          break; // Only break inner for loop, to check rest for better rssi
        }
      }
    }
    
    if (found != -1) {
      logger.log("Connecting to: %s", WiFi.SSID(found).c_str());
      WiFi.mode(WIFI_STA);
      WiFi.begin(WiFi.SSID(found).c_str());
      // Set the hostname
      WiFi.setHostname(DEVICE_NAME);
      long start = millis();
      while (WiFi.status() != WL_CONNECTED) {
        yield();
        // After trying to connect for 8s continue without wifi
        if (millis() - start > 8000) {
          logger.log(ERROR, "Connection to %s failed!", WiFi.SSID(found).c_str());
          break;
        }
      }
      if (WiFi.status() == WL_CONNECTED) staConnected = true;
    } else {
      logger.log(WARNING, "No known network");
    }
    // Delete results of previous scan
    WiFi.scanDelete();
    // If we are connected break, otherwise wait and continue with next scan
    if (staConnected) break;
    vTaskDelay(WIFI_CHECK_PERIODE_MS);
  }
  vTaskDelete( NULL );
}

char * noTime = "";
char * timeStr() { return &noTime[0]; }



void setNewSettings(WiFiClient *theClient) {
  logger.log("Setting new settings");
  logger.log(ALL, "IP: %s", theClient->remoteIP().toString().c_str(), theClient->remotePort());
  String settings = "{\"cmd\":\"addWifi\",\"payload\":{\"ssid\":\"FallObst\",\"pwd\":\"logitech\"}}\r\n";
  // response = "{\"cmd\":\"delWifi\",\"payload\":{\"ssid\":\"ssidName\"}}\r\n";

  logger.log("Settings: %s",settings.c_str());
  client.write(settings.c_str());

  delay(1000);

  settings = "{\"cmd\":\"restart\"}\r\n";
  logger.log("restart: %s",settings.c_str());
  client.write(settings.c_str());
  long start = millis();
  while (start + 3000 > millis()) {
    delay(10);
  }

  WiFi.disconnect();
}

/****************************************************
 * If ESP is connected to wifi successfully
 ****************************************************/
void onWifiConnect() {
  logger.log(ALL, "Wifi Connected");
  logger.log(ALL, "IP: %s", WiFi.localIP().toString().c_str());

  tryToConnectPowerMeterTCP();
}

/****************************************************
 * If ESP disconnected from wifi
 ****************************************************/
void onWifiDisconnect() {
  logger.log(ERROR, "Wifi Disconnected");
}

/****************************************************
 * If a tcp client connects.
 * We store them in list and add logger
 ****************************************************/
void onClientConnect(WiFiClient *newClient) {
  logger.log("Client with IP %s connected on port %u", newClient->remoteIP().toString().c_str(), newClient->remotePort());
}



/****************************************************
 * If a tcp client connects.
 * We store them in list and add logger
 ****************************************************/
void onPowerMeterConnected(WiFiClient *newClient) {
  logger.log("Powermeter with IP %s connected on port %u", newClient->remoteIP().toString().c_str(), newClient->remotePort());
  setNewSettings(newClient);
}

/****************************************************
 * If a tcp client disconnects.
 * We must remove the logger
 ****************************************************/
void onClientDisconnect(WiFiClient *oldClient) {
  logger.log("Client discconnected %s port %u", oldClient->remoteIP().toString().c_str(), oldClient->remotePort());
}


/****************************************************
 * Init the MDNs name from eeprom, only the number ist
 * stored in the eeprom, construct using prefix.
 ****************************************************/
void initMDNS() {
  // Setting up MDNs with the given Name
  logger.log("MDNS Name: %s", DEVICE_NAME);
  if (!MDNS.begin(String(DEVICE_NAME).c_str())) {             // Start the mDNS responder for esp8266.local
    logger.log(ERROR, "Setting up MDNS responder!");
  }
  MDNS.addService("_elec", "_tcp", STANDARD_TCP_PORT);
}

TaskHandle_t powerMeterClientConnectTaskHandle = NULL;
void connectPowermeterTCP(void * pvParameters) {
  while (not updating) {
    if (!client.connected()                         // If not already connected
        and wifiConnected) {  // and server is set
      if (client.connect(POWERMETER_IP, STANDARD_TCP_PORT)) {
        onPowerMeterConnected(&client);
      }
    }
    vTaskDelay(POWERMETER_CONNECT_UPDATE_INTERVAL);
  }
  vTaskDelete(powerMeterClientConnectTaskHandle); // destroy this task 
}

void tryToConnectPowerMeterTCP() {
  if (!client.connected()) {
    logger.log("Try to connect to device");
    // Handle reconnects
    if (powerMeterClientConnectTaskHandle == NULL) {
      xTaskCreate(
            connectPowermeterTCP,   /* Function to implement the task */
            "connectPowermeterTCP", /* Name of the task */
            10000,      /* Stack size in words */
            NULL,       /* Task input parameter */
            1,          /* Priority of the task */
            &powerMeterClientConnectTaskHandle);  /* Task handle */
    }
  }
}
