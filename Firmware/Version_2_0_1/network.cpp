/***************************************************
 Library for network stuff, connection, AP and so on.

 Feel free to use the code as it is.

 Benjamin Völker, voelkerb@me.com
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

#include "network.h"

namespace Network
{
  bool connected = false;
  bool preConnected = false;
  bool apMode = false;
  bool staConnected = false;
  bool allowNetworkChange = true;
  bool ethernet = false;
  MultiLogger& logger = MultiLogger::getInstance();

  static Configuration * _config;
  static void (*_onConnect)(void);
  static void (*_onDisconnect)(void);

  void wifiEvent(WiFiEvent_t event) {
    #ifdef DEBUG_DEEP
    Serial.printf("Info:[WiFi-event] event: %d\n", event);
    switch (event) {
      case SYSTEM_EVENT_WIFI_READY:
        Serial.println("Info:WiFi interface ready");
        break;
      case SYSTEM_EVENT_SCAN_DONE:
        Serial.println("Info:Completed scan for access points");
        break;
      case SYSTEM_EVENT_STA_START:
        Serial.println("Info:WiFi client started");
        break;
      case SYSTEM_EVENT_STA_STOP:
        Serial.println("Info:WiFi clients stopped");
        break;
      case SYSTEM_EVENT_STA_CONNECTED:
        Serial.println("Info:Connected to access point");
        break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("Info:Disconnected from WiFi access point");
        break;
      case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
        Serial.println("Info:Authentication mode of access point has changed");
        break;
      case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("Info:Obtained IP address");
        break;
      case SYSTEM_EVENT_STA_LOST_IP:
        Serial.println("Info:Lost IP address and IP address is reset to 0");
        break;
      case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
        Serial.println("Info:WiFi Protected Setup (WPS): succeeded in enrollee mode");
        break;
      case SYSTEM_EVENT_STA_WPS_ER_FAILED:
        Serial.println("Info:WiFi Protected Setup (WPS): failed in enrollee mode");
        break;
      case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
        Serial.println("Info:WiFi Protected Setup (WPS): timeout in enrollee mode");
        break;
      case SYSTEM_EVENT_STA_WPS_ER_PIN:
        Serial.println("Info:WiFi Protected Setup (WPS): pin code in enrollee mode");
        break;
      case SYSTEM_EVENT_AP_START:
        Serial.println("Info:WiFi access point started");
        break;
      case SYSTEM_EVENT_AP_STOP:
        Serial.println("Info:WiFi access point stopped");
        break;
      case SYSTEM_EVENT_AP_STACONNECTED:
        Serial.println("Info:Client connected");
        break;
      case SYSTEM_EVENT_AP_STADISCONNECTED:
        Serial.println("Info:Client disconnected");
        break;
      case SYSTEM_EVENT_AP_STAIPASSIGNED:
        Serial.println("Info:Assigned IP address to client");
        break;
      case SYSTEM_EVENT_AP_PROBEREQRECVED:
        Serial.println("Info:Received probe request");
        break;
      case SYSTEM_EVENT_GOT_IP6:
        Serial.println("Info:IPv6 is preferred");
        break;
      case SYSTEM_EVENT_ETH_START:
        Serial.println("Info:Ethernet started");
        break;
      case SYSTEM_EVENT_ETH_STOP:
        Serial.println("Info:Ethernet stopped");
        break;
      case SYSTEM_EVENT_ETH_CONNECTED:
        Serial.println("Info:Ethernet connected");
        break;
      case SYSTEM_EVENT_ETH_DISCONNECTED:
        Serial.println("Info:Ethernet disconnected");
        break;
      case SYSTEM_EVENT_ETH_GOT_IP:
        Serial.println("Info:Ethernet obtained IP address");
        break;
      default:
        Serial.println("Info:Unknown event");
    }
    #endif

    if (ethernet) {
      switch (event) {
        case SYSTEM_EVENT_ETH_START:
          logger.log("ETH Started");
          //set eth hostname here
          ETH.setHostname(_config->name);
          break;
        case SYSTEM_EVENT_ETH_CONNECTED:
          logger.log("ETH Connected");
          break;
        case SYSTEM_EVENT_ETH_GOT_IP:
          logger.log("ETH MAC: %s, IP: %s, Speed: %iMbps", ETH.macAddress().c_str(), ETH.localIP().toString().c_str(), ETH.linkSpeed());
          connected = true;
          if (_onConnect) _onConnect();
          break;
        case SYSTEM_EVENT_ETH_DISCONNECTED:
          logger.log("ETH Disconnected");
          connected = false;
          if (_onDisconnect) _onDisconnect();
          break;
        case SYSTEM_EVENT_ETH_STOP:
          logger.log("ETH Stopped");
          connected = false;
          break;
        default:
          break;
      }
    } else {
      switch (event) {
        case SYSTEM_EVENT_SCAN_DONE:
          break;
        case SYSTEM_EVENT_STA_CONNECTED:
          apMode = false;
          connected = false;
          preConnected = true;
          logger.log("STA_CONNECTED");
          break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
          // Recheck for wifi networks
          if (connected and apMode == false) checkNetwork();
          connected = false;
          preConnected = false;
          apMode = false;
          // lets check for networks regularly
          logger.log("STA_DISCONNECTED");
          if (_onDisconnect) _onDisconnect();
          break;
        case SYSTEM_EVENT_STA_GOT_IP:
          WiFi.softAPdisconnect(false);
          WiFi.mode(WIFI_STA);
          connected = true;
          staConnected = true;
          apMode = false;
          logger.log("STA_GOT_IP");
          if (_onConnect) _onConnect();
          break;
        case SYSTEM_EVENT_AP_START:
          if (apMode) break;
          connected = true;
          apMode = true;
          logger.log("AP_START");
          if (_onConnect) _onConnect();
          break;
        case SYSTEM_EVENT_AP_STOP:
          if (not apMode) break;
          connected = false;
          apMode = false;
          logger.log("AP_STOP");
          break;
        default:
          break;
      }
    }
  }

  void init(Configuration * config) {
    init(config, NULL, NULL);
  }


  void init(Configuration * config, void (*onConnect)(void), void (*onDisconnect)(void), bool usingEthernet) {
    _config = config;
    _onConnect = onConnect;
    _onDisconnect = onDisconnect;
    ethernet = usingEthernet;
    connected = false;
    // We do not need bluetooth, so disable it
    esp_bt_controller_disable();
    // Forget any previously set configuration
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
    // Disconnect if we were connected
    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    WiFi.onEvent(&wifiEvent);
    if (ethernet) {
      apMode = false;
      // ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);
    } else {
      WiFi.setHostname(_config->name);
      // Disable wifi power saving
      esp_wifi_set_ps(WIFI_PS_NONE);
      checkNetwork();
    }
  }

  void initPHY(uint8_t addr, uint8_t pwr, uint8_t mdc, uint8_t mdio, eth_phy_type_t type, eth_clock_mode_t clk_mode) {
      ETH.begin(addr, pwr, mdc, mdio, type, clk_mode);
  }

  IPAddress localIP() {
    if (ethernet) return ETH.localIP();
    else return WiFi.localIP(); 
  }

  bool connect(char * network, char * pswd) {
    logger.log("Connecting to: %s", network);
    WiFi.mode(WIFI_STA);
    WiFi.begin(network, pswd);
    // Set the hostname
    WiFi.setHostname(_config->name);
    long start = millis();
    while (WiFi.status() != WL_CONNECTED) {
      yield();
      // After trying to connect for 8s continue without wifi
      if (preConnected) {
        if (millis() - start > 8000) {
          logger.log(ERROR, "Cannot get IP from %s", network);
          return false;
        }
      } else {
        if (millis() - start > 5000) {
          logger.log(ERROR, "Connection to %s failed!", network);
          return false;
        }
      }
      delay(100);
    }
    return true;
  }

  void setupAP() {
    WiFi.mode(WIFI_AP_STA);
    logger.log("Setting up AP: %s", _config->name);
    WiFi.softAP(_config->name);
  }

  void scanNetwork( void * pvParameters ) {
    // Indicating if we are connected to a wifi station
    staConnected = false;
    bool apInited = false;

    while (not staConnected) {
      if (preConnected or not allowNetworkChange) {
        vTaskDelay(CHECK_PERIODE_MS); 
        continue;
      } 
      // If already connected
      // This is not true for AP mode
      if (WiFi.status() == WL_CONNECTED) {
        staConnected = true;
        break;
      }

      logger.log(INFO, "Scanning for Wifi Networks");
      int n = WiFi.scanNetworks();
      if (n == -2) {
        staConnected = false;
        apInited = false;
        logger.log(ERROR, "Scan failed");
        //  Give it up ...
        ESP.restart();
        // Reconfigure wifi
        // Delete results of previous scan
        WiFi.scanDelete();
        WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
        WiFi.disconnect();
        WiFi.mode(WIFI_STA);
        WiFi.onEvent(&wifiEvent);
        esp_wifi_set_ps(WIFI_PS_NONE);
      } else if (n == -1) {
        WiFi.scanDelete();
        logger.log(WARNING, "Scan already in progress");
        //  Give it up ...
        ESP.restart();
      } else if (n == 0) {
        WiFi.scanDelete();
        logger.log(INFO, "No network found");
      } else {
        logger.log("Scan done %i networks found", n);
        for (size_t i = 0; i < (size_t)n; ++i) {
          logger.log("%s (%i)", WiFi.SSID(i).c_str(), WiFi.RSSI(i));
        }
      }

      int found = -1;
      // Only if we have found any network, we can search for a known one
      if (n > 0) {
        int linkQuality = -1000; // The smaller the worse the quality (in dBm)
        for (size_t i = 0; i < (size_t)n; ++i) {
          for (size_t j = 0; j < _config->numAPs; j++) {
            if (strcmp(WiFi.SSID(i).c_str(), _config->wifiSSIDs[j]) == 0) {
              if (WiFi.RSSI(i) > linkQuality) {
                linkQuality = WiFi.RSSI(i);
                found = j;
              }
              break; // Only break inner for loop, to check rest for better rssi
            }
          }
        }
      }

      if (found != -1) {
        logger.log("Strongest known network: %s", _config->wifiSSIDs[found]);
        if (connect(_config->wifiSSIDs[found], _config->wifiPWDs[found])) {
          staConnected = true;
        } else {
          apInited = false;
        }
      } else {
        logger.log(WARNING, "No known network");
      }

      if (not staConnected and not apInited) {
        // We setup an access point
        setupAP();
        apInited = true;
      }
      // Delete results of previous scan
      WiFi.scanDelete();
      // If we are connected break, otherwise wait and continue with next scan
      if (staConnected) break;
      vTaskDelay(CHECK_PERIODE_MS);
    }
    vTaskDelete( NULL );
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
}
