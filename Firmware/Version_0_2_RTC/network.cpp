/***************************************************
 Library for network stuff, connection, AP and so on.

 Feel free to use the code as it is.

 Benjamin Völker, voelkerb@me.com
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

#include "network.h"

using namespace Network;

void Network::wifiEvent(WiFiEvent_t event) {
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

  switch (event) {
    case SYSTEM_EVENT_SCAN_DONE:
      break;
    case SYSTEM_EVENT_STA_CONNECTED:
      _apMode = false;
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      connected = false;
      _apMode = false;
      // lets check for networks regularly
      checker.attach(CHECK_PERIODE, checkNetwork);
      if (_onDisconnect) _onDisconnect();
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      connected = true;
      checker.detach();
      if (_onConnect) _onConnect();
      break;
    case SYSTEM_EVENT_AP_START:
      connected = true;
      _apMode = true;
      if (_onConnect) _onConnect();
      break;
    case SYSTEM_EVENT_AP_STOP:
      connected = false;
      _apMode = false;
      if (_onDisconnect) _onDisconnect();
      break;
    default:
      break;
  }
}

void Network::init(Configuration * config) {
  init(config, NULL, NULL);
}

void Network::init(Configuration * config, void (*onConnect)(void), void (*onDisconnect)(void)) {
  _config = config;
  _onConnect = onConnect;
  _onDisconnect = onDisconnect;
  WiFi.onEvent(wifiEvent);
  // At first we want to be in station and access point mode
  WiFi.setHostname(_config->name);
  WiFi.mode(WIFI_STA);
  // Disable wifi power saving
  esp_wifi_set_ps(WIFI_PS_NONE);
  checkNetwork();
  checker.attach(CHECK_PERIODE, checkNetwork);
}

bool Network::connect(char * network, char * pswd) {
  #ifdef DEBUG
  Serial.print(F("Info:Connecting to: "));
  Serial.println(network);
  #endif
  WiFi.mode(WIFI_STA);
  WiFi.begin(network, pswd);
  long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    yield();
    // After trying to connect for 8s continue without wifi
    if (millis() - start > 8000) {
      #ifdef DEBUG
      Serial.print(F("Info:Connection to "));
      Serial.print(network);
      Serial.println(F(" failed!"));
      #endif
      return false;
    }
  }
  return true;
}

void Network::setupAP() {
  WiFi.mode(WIFI_AP_STA);
  _apMode = true;
  #ifdef DEBUG
  Serial.print("Info:Setting up AP: ");
  Serial.println(_config->name);
  #endif
  WiFi.softAP(_config->name);
  // Check for known networks regularly
  checker.attach(CHECK_PERIODE, checkNetwork);
}

void Network::scanNetwork( void * pvParameters ) {
  // blocking call
  // NOTE: non blocking call did not work properly
  int n = WiFi.scanNetworks();
  int found = -1;
  #ifdef DEBUG
  Serial.print(F("Info:Scan done "));
  if (n == 0) {
    Serial.println(F(" no networks found"));
  } else {
    Serial.print(n); Serial.print(F(" networks found: "));
    for (int i = 0; i < n; ++i) {
      // Print SSID and RSSI for each network found
      Serial.print(WiFi.SSID(i)); Serial.print(F(" (")); Serial.print(WiFi.RSSI(i)); Serial.print(F(")"));
      if (i < n-1) Serial.print(", ");
    }
    Serial.println("");
  }
  #endif

  found = -1;
  // Only if we have found any network, we can search for a known one
  if (n != 0) {
    int linkQuality = -1000; // The smaller the worse the quality (in dBm)
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < _config->numAPs; j++) {
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
    #ifdef DEBUG
    Serial.print(F("Info:Strongest known network: "));
    Serial.println(_config->wifiSSIDs[found]);
    #endif
    // connect
    WiFi.mode(WIFI_STA);
    WiFi.begin(_config->wifiSSIDs[found], _config->wifiPWDs[found]);
  } else {
    #ifdef DEBUG
    Serial.println(F("Info:No known network"));
    #endif
    // We setup an access point
    setupAP();
  }

  vTaskDelete( NULL );
}

void Network::checkNetwork() {
  // let it check for networks on second core (not in loop)
  xTaskCreatePinnedToCore(
                    scanNetwork,   /* Function to implement the task */
                    "scanTask", /* Name of the task */
                    10000,      /* Stack size in words */
                    NULL,       /* Task input parameter */
                    0,          /* Priority of the task */
                    NULL,       /* Task handle. */
                    0);  /* Core where the task should run */

}
