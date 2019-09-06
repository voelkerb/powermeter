#include <SPI.h>
#include "STPM.h"
#include <esp_wifi.h>
#include <WiFi.h>
// #include <esp_wifi.h>
#include "esp_deep_sleep.h"
#include <time.h>
#include <ArduinoJson.h>
#include <TimeLib.h>
#include <ESPmDNS.h>
/* for normal hardware wire use below */
#include "esp_bt.h"
#include "relay.h"
#include "rtc.h"
#include "config.h"
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>

// Arduino Updater
#include <ArduinoOTA.h>


// Serial Speed and DEBUG option
#define SERIAL_SPEED 2000000
#define DEBUG

#define VERSION "0.2_x_rtc"
#define STANDARD_UDP_PORT 54323
#define STANDARD_TCP_SAMPLE_PORT 54321
#define STANDARD_TCP_STREAM_PORT 54322

Configuration config;

// Pins for STPM34 SPI Connection
const int STPM_CS = 5;
const int STPM_SYN = 14;
// Reset pin of STPM
const int STPM_RES = 12;

// Pins for 230V Relay
const int RELAY_PIN_S = 26;
const int RELAY_PIN_R = 27;

Relay relay(RELAY_PIN_S, RELAY_PIN_R);

const int RTC_INT = 25;
Rtc rtc(RTC_INT);

// STPM Object
STPM stpm34(STPM_RES, STPM_CS, STPM_SYN);

// counter holds # isr calls
volatile uint16_t counter = 0;
// Last micros() count of isr call
volatile long nowTs = 0;
volatile long lastTs = 0;

// Buffering stuff
// Number of bytes for one measurement
int MEASURMENT_BYTES = 8; //(16+2)
// Chunk sizes of data to send
const int PS_BUF_SIZE = 3*1024*1024;
#define NUM_RAM_BUFFERS 4
uint8_t* psdRamBuffer;
volatile uint32_t chunkSize = 256;
// Circular chunked buffer
// static uint8_t buffer[BUF_SIZE] = {0};
// Static vs dynamic allocation
#define MAX_SEND_SIZE 512 // 1024
static uint8_t buffer[NUM_RAM_BUFFERS][MAX_SEND_SIZE];
volatile bool flushBuffer[NUM_RAM_BUFFERS] = { false};
volatile uint8_t currentBuffer = 0;
// MAX_SEND_SIZE+"Data:"+4+2
static uint8_t sendbuffer[MAX_SEND_SIZE+16] = {0};
// Buffer read/write position
uint32_t psdReadPtr = 0;
uint32_t psdWritePtr = 0;
volatile uint32_t writePtr = 0;

// WLAN configuration
const char *ssids[] = {"energywifi", "esewifi"};
const char *passwords[] = {"silkykayak943", "silkykayak943"};

// Open two TCP ports, one for commands and sampling and one for a raw data stream
WiFiServer server(STANDARD_TCP_SAMPLE_PORT);
WiFiServer streamServer(STANDARD_TCP_STREAM_PORT);

// TIMER stuff
#define CLOCK 240 // clock frequency in MHz.
#define DEFAUL_SR 4000
unsigned int samplingRate = DEFAUL_SR; // Standard read frequency is 4kHz
// Calculate the number of cycles we have to wait
volatile uint32_t TIMER_CYCLES_FAST = (1000000) / samplingRate; // Cycles between HW timer inerrupts
volatile uint32_t timer_next;
volatile uint32_t timer_now;

// Sample state machine
#define CMD_SAMPLE "sample"
#define CMD_SWITCH "switch"
#define CMD_STOP "stop"
#define CMD_RESTART "restart"
#define CMD_RESET "factoryReset"
#define CMD_INFO "info"
#define CMD_MDNS "mdns"
#define CMD_NTP "ntp"
#define CMD_ADD_WIFI "addWifi"
#define CMD_REMOVE_WIFI "delWifi"

#define STATE_IDLE -1
#define STATE_SAMPLE_USB 0
#define STATE_SAMPLE_TCP 1
#define STATE_STREAM_TCP 2
#define STATE_SAMPLE_UDP 3
int state = STATE_IDLE;
int next_state = STATE_IDLE;


#define STATE_VI 0
#define STATE_PQ 1
#define STATE_VIPQ 2
volatile int measures = STATE_VI;

// Stuff for frequency calculation
volatile long freqCalcStart;
volatile long freqCalcNow;
volatile long freq = 0;


WiFiClient client;
WiFiClient streamClient;

#define COMMAND_MAX_SIZE 200
char command[COMMAND_MAX_SIZE] = {'\0'};
StaticJsonDocument<2*COMMAND_MAX_SIZE> docRcv;
StaticJsonDocument<2*COMMAND_MAX_SIZE> docSend;
String response = "";

#define LOCATION_OFFSET (2*60*60)
IPAddress timeServerIP; // time.nist.gov NTP server address
// Use local tcp server with hopefully a small ping
const char* ntpServerName = "time.uni-freiburg.de";//"0.de.pool.ntp.org";
unsigned int localNTPPort = 2390; // local port to listen for UDP packets
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes
byte ntpBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing
// A UDP instance to let us send and receive packets over UDP
WiFiUDP udpNtp;
unsigned long ntpValidMillis = 0;
unsigned long ntpEpochSeconds = 0;
unsigned long ntpMilliseconds = 0;
// UDP used for streaming

WiFiUDP udp;
IPAddress udpIP; // time.nist.gov NTP server address
uint16_t udpPort = STANDARD_UDP_PORT;

unsigned long currentSeconds = 0;
unsigned long currentMilliseconds = 0;

unsigned long samplingCountdown = 0;
unsigned long startSamplingMillis = 0;
unsigned long samplingDuration = 0;
unsigned long sentSamples = 0;
volatile unsigned long packetNumber = 0;
volatile unsigned long totalSamples = 0;

long lifenessUpdate = millis();
long mdnsUpdate = millis();



const char* host = "esp8266-webupdate";
const char* update_path = "/firmware";
const char* update_username = "admin";
const char* update_password = "admin";


// ESP8266WebServer httpServer(80);
// ESP8266HTTPUpdateServer httpUpdater;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;  // DOESN'T SEEM TO MAKE ANY DIFFERENCE

TaskHandle_t TaskCore2;
#include "esp32-hal-cpu.h"


extern "C" {
  #include <esp_spiram.h>
  #include <esp_himem.h>
}

TaskHandle_t TaskLoop2;

// Checks if motion was detected, sets LED HIGH and starts a timer
void IRAM_ATTR sqwvTriggered() {
  Serial.println("Pin interrupt");
}

/************************ SETUP *************************/
void setup() {
  // We do not need bluetooth, so disable it
  esp_bt_controller_disable();

   // Setup serial communication
  Serial.begin(SERIAL_SPEED);

  relay.set(true);

  if (psdRamBuffer = (uint8_t*)ps_malloc(PS_BUF_SIZE)) {
    Serial.printf("Info:Allocated %u Bytes of RAM\n", PS_BUF_SIZE);
  } else {
    Serial.println("Info:ps_malloc failed");
  }
  config.load();

  timer_init();

  int myfreq = getCpuFrequencyMhz();
  Serial.print("Core @ ");
  Serial.print(myfreq);
  Serial.println("MHz");
  // Setup STPM 32
  stpm34.init();

  rtc.init();
  rtc.enableInterrupt(1, sqwvTriggered);

#ifdef DEBUG
  Serial.println("Info:Connecting WLAN ");
#endif
  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
  setupOTA();
  //wifi_set_sleep_type(NONE_SLEEP_T);
  WiFi.setHostname(config.name);
  connectNetwork();
  WiFi.setHostname(config.name);

  initMDNS();
  // Start the TCP server
  server.begin();
  streamServer.begin();


  udpNtp.begin(localNTPPort);

  getTimeNTP();

  response.reserve(2*COMMAND_MAX_SIZE);

  // WiFi.setOutputPower(20.5);


  Serial.print(F("Info:SketchFree: "));
  Serial.println(ESP.getFreeSketchSpace());
  Serial.println(F("Info:Setup done"));

  lifenessUpdate = millis();
  mdnsUpdate = millis();
}

void createPSRAM_TASK() {
  xTaskCreatePinnedToCore(
        loop2, /* Function to implement the task */
        "Loop2", /* Name of the task */
        10000,  /* Stack size in words */
        NULL,  /* Task input parameter */
        1,  /* Priority of the task */
        &TaskLoop2,  /* Task handle. */
        0); /* Core where the task should run */
}

void loop2( void * pvParameters ) {
  while (true) {
    for(uint8_t a = 0; a < NUM_RAM_BUFFERS; a++){
      if(flushBuffer[a]) {
        Serial.print("RW Distance:");
        Serial.println(getReadWriteDistance());
        // move buffer to PSDRAM
        uint32_t end = (psdWritePtr+chunkSize)%PS_BUF_SIZE;

        //memcpy((uint8_t*)&psdRamBuffer[psdWritePtr],(uint8_t*)&buffer[a][0], chunkSize);

        if (end > psdWritePtr) {
          memcpy((uint8_t*)&psdRamBuffer[psdWritePtr],(uint8_t*)&buffer[a][0], chunkSize);
        } else {
          memcpy((uint8_t*)&psdRamBuffer[psdWritePtr],(uint8_t*)&buffer[a][0], PS_BUF_SIZE - psdWritePtr);
          memcpy((uint8_t*)&psdRamBuffer[psdWritePtr],(uint8_t*)&buffer[a][PS_BUF_SIZE - psdWritePtr], end);
        }
        psdWritePtr = end;
        flushBuffer[a] = false;
      }
    }
    vTaskDelay(2);
    yield();
  }
}

// the loop routine runs over and over again forever:
void loop() {


  if (WiFi.status() != WL_CONNECTED) {
    connectNetwork();
    lifenessUpdate = millis();
    mdnsUpdate = millis();
  }

  if (state == STATE_IDLE) {
    // HTTP Updater
    // httpServer.handleClient();
    // Arduino OTA
    ArduinoOTA.handle();

    // Update mdns only on idle
    //MDNS.update();
    // Re-advertise service
    if ((long)(millis() - mdnsUpdate) >= 0) {
      MDNS.addService("elec", "tcp", 54322);
      mdnsUpdate += 100000;
    }
    // Update lifeness only on idle every second
    if ((long)(millis() - lifenessUpdate) >= 0) {
      rtc.update();
      // MDNS.update();
      lifenessUpdate += 1000;
      #ifdef DEBUG
      char * cur = printCurrentTime();
      Serial.print(F("Info:"));
      Serial.println(cur);
      #endif
    }
  }

  // If we only have 100 ms before sampling should start, wait actively
  if (next_state != STATE_IDLE) {
    if (samplingCountdown != 0 and (samplingCountdown - millis()) < 1000) {
      // WiFi.setOutputPower(20.5);
      esp_wifi_set_ps(WIFI_PS_NONE);
      // Ramp up TCP Speed
      if (client.connected() and next_state == STATE_SAMPLE_TCP) {
        while (((long)(samplingCountdown) - millis()) > 100) {
          String res = "Info:KeepAlive";
          client.println(res);
          delay(10);
          yield();
        }
      }
      #ifdef DEBUG
      state = next_state;
      updateTime(false);
      char * cur = printCurrentTime();
      Serial.print(F("Info:StartSampling @ "));
      Serial.println(cur);
      #endif
      int32_t mydelta = samplingCountdown - millis();
      if (mydelta > 0) delay(mydelta-1);
      startSampling(true);
      samplingCountdown = 0;
    }
  }
  // Output frequency
  if (freq != 0) {
    long fr = freq;
    freq = 0;
    float frequency = (float)samplingRate/(float)((fr)/1000000.0);
    response = "Info:";
    response += frequency;
    response += "Hz";
    char * cur = printCurrentTime();
    response += "\r\nInfo:";
    response += cur;
    #ifdef DEBUG
    Serial.println(response);
    #endif
    #ifdef SENT_LIFENESS_TO_CLIENTS
    if (state != STATE_SAMPLE_UDP and client.connected()) {
      client.println(response);
    }
    #endif
  }

  // Handle serial events
  if (Serial.available()) {
    handleEvent(Serial);
  }


  // Handle client connections
  if (!client.connected()) {
    // Look for people connecting over the server
    client = server.available();
    udpIP = client.remoteIP();
    // We want that because we don't want each buffer part to be send immidiately
    client.setNoDelay(false);
    // Each buffer should be written directly
    // client.setNoDelay(false);
  } else {
    // Handle client events
    if (client.available() > 0) {
      handleEvent(client);
    }
  }

  if (state == STATE_IDLE) {
    // Look for people connecting over the server
    streamClient = streamServer.available();
    if (streamClient.connected()) {
      state = STATE_STREAM_TCP;
      samplingRate = DEFAUL_SR;
      startSampling();
    }
  } else if (state == STATE_SAMPLE_USB) {
    writeChunks(Serial, false);
  } else if (state == STATE_SAMPLE_TCP) {
    if (!client.connected()) {
      #ifdef DEBUG
      Serial.println(F("Info:TCP not connected"));
      #endif
      state = STATE_IDLE;
      stopSampling();
    } else {
      writeChunks(client, false);
    }
  } else if (state == STATE_SAMPLE_UDP) {
    if (!client.connected()) {
      #ifdef DEBUG
      Serial.println(F("Info:TCP/UDP not connected"));
      #endif
      state = STATE_IDLE;
      stopSampling();
    }
    while(getReadWriteDistance() > chunkSize) {
      udp.beginPacket(udpIP, udpPort);
      writeChunk(udp, chunkSize, true);
      udp.endPacket();
    }
    // udp.beginPacket(udp.remoteIP(), udp.remotePort());
    // udp.write(ReplyBuffer);
    // udp.endPacket();
  } else if (state == STATE_STREAM_TCP) {
    if (!streamClient.connected()) {
      #ifdef DEBUG
      Serial.println(F("Info:TCP Stream not connected"));
      #endif
      state = STATE_IDLE;
      stopSampling();
    } else {
      writeChunks(streamClient, false, false);
    }
  }
  yield();
}



void setupOTA() {

  // httpUpdater.setup(&httpServer, update_path, update_username, update_password);
  // httpServer.begin();
  // MDNS.addService("http", "tcp", 80);
  // Serial.printf("HTTPUpdateServer ready! Open http://%s.local%s in your browser and login with username '%s' and password '%s'\n", host, update_path, update_username, update_password);


  // No authentication by default
  ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    Serial.println("Start updating");
    // free(buffer);
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
    // No matter what happended, simply restart
    ESP.restart();
  });

  ArduinoOTA.begin();
}

void connectNetwork() {
  #ifdef DEBUG
  Serial.print(F("Info:Known Networks: "));
  for (int i = 0; i < config.numAPs; i++) {
    Serial.print(config.wifiSSIDs[i]);
    if (i < config.numAPs-1) Serial.print(", ");
  }
  Serial.println("");
  #endif

  int n = 0;
  while (WiFi.status() != WL_CONNECTED) {
    // WiFi.scanNetworks will return the number of networks found
    n = WiFi.scanNetworks();
    #ifdef DEBUG
    Serial.print(F("Info:Scan done "));
    if (n == 0) {
      Serial.println(F(" no networks found"));
      // TODO: Open wifi network here
    } else {
      Serial.print(n);
      Serial.println(F(" networks found"));
      for (int i = 0; i < n; ++i) {
        // Print SSID and RSSI for each network found
        Serial.print(F("Info:\t"));
        Serial.print(WiFi.SSID(i));
        Serial.print(F(" ("));
        Serial.print(WiFi.RSSI(i));
        Serial.println(F(")"));
      }
    }
    #endif

    if (n != 0) {
      int found = -1;
      int linkQuality = -1000; // The smaller the worse the quality (in dBm)
      for (int i = 0; i < n; ++i) {
        for (int j = 0; j < config.numAPs; j++) {
          if (strcmp(WiFi.SSID(i).c_str(), config.wifiSSIDs[j]) == 0) {
            if (WiFi.RSSI(i) > linkQuality) {
              linkQuality = WiFi.RSSI(i);
              found = j;
            }
            break;
          }
        }
      }
      if (found != -1) {
        #ifdef DEBUG
        Serial.print(F("Info:Known network with best quality: "));
        Serial.println(config.wifiSSIDs[found]);
        #endif
        WiFi.begin(config.wifiSSIDs[found], config.wifiPWDs[found]);
        long start = millis();
        while (WiFi.status() != WL_CONNECTED) {
          yield();
          // After trying to connect for 5s continue without wifi
          if (millis() - start > 8000) {
            #ifdef DEBUG
            Serial.print(F("Info:Connection to "));
            Serial.print(config.wifiSSIDs[found]);
            Serial.println(F(" failed!"));
            #endif
            break;
          }
        }
      }
    }
    if (WiFi.status() != WL_CONNECTED) delay(1000);
    yield();
  }
#ifdef DEBUG
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(F("Info:Connected"));
    Serial.print(F("Info:SSID:")); Serial.println(WiFi.SSID());
    Serial.print(F("Info:IP Address: ")); Serial.println(WiFi.localIP());
  }
#endif
}


uint32_t getReadWriteDistance() {
  if (psdWritePtr < psdReadPtr) return psdWritePtr + (PS_BUF_SIZE - psdReadPtr);
  else return psdWritePtr - psdReadPtr;
}

inline void writeChunks(Stream &getter, bool tail) {
  while(getReadWriteDistance() > chunkSize) writeChunk(getter, chunkSize); yield();
  if (tail) writeChunk(getter, getReadWriteDistance());
}

inline void writeChunks(Stream &getter, bool tail, bool prefix) {
  while(getReadWriteDistance() > chunkSize) writeChunk(getter, chunkSize, prefix); yield();
  if (tail) writeChunk(getter, getReadWriteDistance(), prefix);
}

inline void writeChunk(Stream &getter, uint16_t size) {
  writeChunk(getter, size, true);
}

char data_id[5] = {'D','a','t','a',':'};
uint8_t size_prefix[sizeof(uint16_t)];
uint8_t packet_prefix[sizeof(uint32_t)];
void writeChunk(Stream &getter, uint16_t size, bool prefix) {
  if (&getter == &client)
  if (size <= 0) return;
  uint32_t start = 0;
  uint32_t end = (psdReadPtr+size)%PS_BUF_SIZE;
  long ttime = millis();
  if (prefix) {
    // getter.print(F("Data:"));
    memcpy(&sendbuffer[start], (void*)&data_id[0], sizeof(data_id));
    start += sizeof(data_id);
    memcpy(&sendbuffer[start], (void*)&size, sizeof(uint16_t));
    start += sizeof(uint16_t);
    memcpy(&sendbuffer[start], (void*)&packetNumber, sizeof(uint32_t));
    start += sizeof(uint32_t);
    // memcpy(&size_prefix[0], (void*)&size, sizeof(uint16_t));
    // memcpy(&packet_prefix[0], (void*)&packetNumber, sizeof(uint32_t));
    // getter.write(size_prefix, sizeof(uint16_t));
    // getter.write(packet_prefix, sizeof(uint32_t));
    packetNumber += 1;
  }
  uint32_t sent = 0;

  if (end > psdReadPtr) {
    memcpy(&sendbuffer[start], &psdRamBuffer[psdReadPtr], size);
    // memcpy_P(&sendbuffer[0], &buffer[readPtr], size);
    // sent = sent + getter.write((uint8_t*)&buffer[readPtr], size);
  } else {
    memcpy(&sendbuffer[start], &psdRamBuffer[psdReadPtr], PS_BUF_SIZE - psdReadPtr);
    memcpy(&sendbuffer[start+(PS_BUF_SIZE - psdReadPtr)], &psdRamBuffer[0], end);
    // memcpy_P(&sendbuffer[0], &buffer[readPtr], BUF_SIZE - readPtr);
    // memcpy_P(&sendbuffer[BUF_SIZE - readPtr], &buffer[0], end);
    // sent = sent + getter.write((uint8_t*)&buffer[readPtr], BUF_SIZE - readPtr);
    // sent = sent + getter.write((uint8_t*)&buffer[0], end);
  }
  psdReadPtr = end;
  // Everything is sent at once (hopefully)
  sent = sent + getter.write((uint8_t*)&sendbuffer[0], size+start);
  if (sent > start) sentSamples += (sent-start)/MEASURMENT_BYTES;
}

volatile float values[4] = {0};

// FPU register state
uint32_t cp0_regs[18];

/****************************************************
 * ISR for sampling
 ****************************************************/
void IRAM_ATTR sample_ISR(){

  portENTER_CRITICAL_ISR(&timerMux);
  // get FPU state
  uint32_t cp_state = xthal_get_cpenable();

  // enable FPU if it is deactivated since we use floats
  if(cp_state) {
    // Save FPU registers
    xthal_save_cp0(cp0_regs);
  } else {
    // enable FPU
    xthal_set_cpenable(1);
  }

  // Vaiables for frequency count
  counter++;
  totalSamples++;
  if (counter >= samplingRate) {
    freqCalcNow = micros();
    freq = freqCalcNow-freqCalcStart;
    freqCalcStart = freqCalcNow;
    counter = 0;
  }

  if (measures == STATE_VI) {
    stpm34.readVoltageAndCurrent(1, (float*) &values[0], (float*) &values[1]);
  } else if (measures == STATE_PQ) {
    stpm34.readPower(1, (float*) &values[0], (float*) &values[2], (float*) &values[1], (float*) &values[2]);
  } else if (measures == STATE_VIPQ) {
    stpm34.readAll(1, (float*) &values[0], (float*) &values[1], (float*) &values[2], (float*) &values[3]);
  }
  memcpy(&buffer[currentBuffer][writePtr], (void*)&values[0], MEASURMENT_BYTES);
  // memcpy_P(&buffer[writePtr], (void*)&values[0], MEASURMENT_BYTES);

  writePtr = writePtr+MEASURMENT_BYTES;
  if (writePtr >= chunkSize) {
    flushBuffer[currentBuffer] = true;
    writePtr = 0;
    currentBuffer = (currentBuffer+1)%NUM_RAM_BUFFERS;
    if(flushBuffer[currentBuffer]) {
      Serial.println(F("Info:BufferOvf"));
      packetNumber += NUM_RAM_BUFFERS;
      currentBuffer = (currentBuffer+1)%NUM_RAM_BUFFERS;
    }
  }

  if(cp_state) {
     // Restore FPU registers
     xthal_restore_cp0(cp0_regs);
   } else {
     // turn it back off
     xthal_set_cpenable(0);
   }
   portEXIT_CRITICAL_ISR(&timerMux);
}

// _____________________________________________________________________________

/****************************************************
 * A serial event occured
 ****************************************************/
void handleEvent(Stream &getter) {
  if (!getter.available()) return;
  getter.readStringUntil('\n').toCharArray(command,COMMAND_MAX_SIZE);
  // Be backwards compatible to "?" command all the time
  if (command[0] == '?') {
    getter.println(F("Info:Setup done"));
    return;
  }
  #ifdef DEBUG
  Serial.print(F("Info:")); Serial.println(command);
  #endif

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(docRcv, command);
  // Test if parsing succeeds.
  if (error) {
    getter.print(F("Info:deserializeJson() failed: "));
    getter.println(error.c_str());
    #ifdef DEBUG
    if (&getter != &Serial) {
      Serial.print(F("Info:deserializeJson() failed: "));
      Serial.println(error.c_str());
    }
    #endif
    return;
  }
  //docSend.clear();
  JsonObject obj = docSend.to<JsonObject>();
  obj.clear();
  response = "";
  handleJSON();
  if (docSend.isNull() == false) {
    getter.flush();
    response = "";
    serializeJson(docSend, response);
    response = "Info:" + response;
    getter.println(response);

    #ifdef DEBUG
    if (&getter != &Serial) Serial.println(response);
    #endif
  }
  response = "";
}

void handleJSON() {
  // All commands look like the following:
  // {"cmd":{"name":"commandName", "payload":{<possible data>}}}
  // e.g. mdns

  const char* cmd = docRcv[F("cmd")]["name"];
  JsonObject root = docRcv.as<JsonObject>();
  if (cmd == nullptr) {
    docSend["msg"] = F("JSON does not match format, syntax: {\"cmd\":{\"name\":<commandName>, \"[payload\":{<possible data>}]}}");
    return;
  }

  /*********************** SAMPLING COMMAND ****************************/
  // e.g. {"cmd":{"name":"sample", "payload":{"type":"Serial", "rate":4000}}}
  if(strcmp(cmd, CMD_SAMPLE) == 0) {
    // For sampling we need type payload and rate payload
    const char* typeC = root["cmd"]["payload"]["type"];
    const char* measuresC = root["cmd"]["payload"]["measures"];
    int rate = docRcv["cmd"]["payload"]["rate"].as<int>();
    unsigned long ts = docRcv["cmd"]["payload"]["time"].as<unsigned long>();

    docSend["error"] = true;
    if (typeC == nullptr or rate == 0) {
      response = "Not a valid \"sample\" command";
      if (typeC == nullptr) response += ", \"type\" missing";
      if (rate == 0) response += ", \"rate\" missing";
      docSend["msg"] = response;
      return;
    }
    if (rate > 8000 || rate <= 0) {
      response = "SamplingRate could not be set to ";
      response += rate;
      docSend["msg"] = response;
      return;
    }
    if (measuresC == nullptr) {
      measures = STATE_VI;
      MEASURMENT_BYTES = 8;
    } else if (strcmp(measuresC, "v,i") == 0) {
      measures = STATE_VI;
      MEASURMENT_BYTES = 8;
    } else if (strcmp(measuresC, "p,q") == 0) {
      measures = STATE_PQ;
      MEASURMENT_BYTES = 8;
    } else if (strcmp(measuresC, "v,i,p,q") == 0) {
      measures = STATE_VIPQ;
      MEASURMENT_BYTES = 16;
    } else {
      response = "Unsupported measures";
      response += measuresC;
      docSend["msg"] = response;
      return;
    }
    // e.g. {"cmd":{"name":"sample", "payload":{"type":"Serial", "rate":4000}}}
    if (strcmp(typeC, "Serial") == 0) {
      next_state = STATE_SAMPLE_USB;
    // e.g. {"cmd":{"name":"sample", "payload":{"type":"TCP", "rate":4000}}}
    } else if (strcmp(typeC, "TCP") == 0) {
      next_state = STATE_SAMPLE_TCP;
    // e.g. {"cmd":{"name":"sample", "payload":{"type":"UDP", "rate":4000}}}
    } else if (strcmp(typeC, "UDP") == 0) {
      next_state = STATE_SAMPLE_UDP;
      int port = docRcv["cmd"]["payload"]["port"].as<int>();
      if (port > 80000 || port <= 0) {
        udpPort = STANDARD_UDP_PORT;
        response = "Unsupported UDP port";
        response += port;
        docSend["msg"] = response;
        return;
      } else {
        udpPort = port;
      }
      docSend["port"] = udpPort;
    } else if (strcmp(typeC, "FFMPEG") == 0) {
      bool success = tryConnectStream();
      if (success) {
        next_state = STATE_STREAM_TCP;
        response = F("Connected to TCP stream");
      } else {
        docSend["msg"] = F("Could not connect to TCP stream");
        return;
      }
    } else {
      response = F("Unsupported sampling type: ");
      response += typeC;
      docSend["msg"] = response;
      return;
    }
    // Set global sampling variable
    samplingRate = rate;
    TIMER_CYCLES_FAST = (1000000) / samplingRate; // Cycles between HW timer inerrupts
    calcChunkSize();

    docSend["sampling_rate"] = samplingRate;
    docSend["chunk_size"] = chunkSize;
    docSend["conn_type"] = typeC;
    docSend["timer_cycles"] = TIMER_CYCLES_FAST;
    docSend["cmd"] = CMD_SAMPLE;

    relay.set(true);

    if (ts != 0) {
      response += F("Should sample at: ");
      response += printTime(ts,0);
      updateTime(true);
      uint32_t delta = ts - currentSeconds;
      uint32_t nowMs = millis();
      delta *= 1000;
      delta -= currentMilliseconds;
      if (delta > 20000 or delta < 500) {
        response += F("//nCannot start sampling in: "); response += delta; response += F("ms");
        samplingCountdown = 0;
      } else {
        response += F("//nStart sampling in: "); response += delta; response += F("ms");
        samplingCountdown = nowMs + delta;
        docSend["error"] = false;
      }
      docSend["msg"] = String(response);
      return;
    }
    docSend["error"] = false;
    state = next_state;
    // UDP packets are not allowed to exceed 1500 bytes, so keep size reasonable
    startSampling(true);
  }

  /*********************** SWITCHING COMMAND ****************************/
  else if (strcmp(cmd, CMD_SWITCH) == 0) {
    // For switching we need value payload
    docSend["error"] = true;
    const char* payloadValue = docRcv["cmd"]["payload"]["value"];
    if (payloadValue == nullptr) {
      docSend["msg"] = F("Info:Not a valid \"switch\" command");
      return;
    }
    bool value = docRcv["cmd"]["payload"]["value"].as<bool>();
    docSend["msg"]["switch"] = value;
    response = F("Switching: ");
    response += value ? F("On") : F("Off");
    docSend["msg"] = response;
    docSend["error"] = false;
    relay.set(value);
  }

  /*********************** STOP COMMAND ****************************/
  // e.g. {"cmd":{"name":"stop"}}
  else if (strcmp(cmd, CMD_STOP) == 0) {
    // State is reset in stopSampling
    int prev_state = state;
    stopSampling();
    // Send remaining
    if (prev_state == STATE_SAMPLE_USB) writeChunks(Serial, true);
    else if (prev_state == STATE_SAMPLE_TCP) writeChunks(client, true);
    else if (prev_state == STATE_SAMPLE_UDP) {
      udp.beginPacket(udpIP, udpPort);
      writeChunks(udp, true);
      udp.endPacket();
    }
    docSend["msg"] = F("Received stop command");
    docSend["sample_duration"] = samplingDuration;
    docSend["samples"] = totalSamples;
    docSend["sent_samples"] = sentSamples;
    docSend["avg_rate"] = totalSamples/(samplingDuration/1000.0);
    docSend["cmd"] = CMD_STOP;
  }

  /*********************** RESTART COMMAND ****************************/
  // e.g. {"cmd":{"name":"restart"}}
  else if (strcmp(cmd, CMD_RESTART) == 0) {
    ESP.restart();
  }

  /*********************** RESTART COMMAND ****************************/
  // e.g. {"cmd":{"name":"factoryReset"}}
  else if (strcmp(cmd, CMD_RESET) == 0) {
    config.makeDefault();
    ESP.restart();
  }

  /*********************** INFO COMMAND ****************************/
  // e.g. {"cmd":{"name":"info"}}
  else if (strcmp(cmd, CMD_INFO) == 0) {
    docSend["msg"] = F("WIFI powermeter");
    docSend["version"] = VERSION;
    docSend["buffer_size"] = PS_BUF_SIZE;
    docSend["name"] = config.name;
    docSend["sampling_rate"] = samplingRate;
    String ssids = "[";
    for (int i = 0; i < config.numAPs; i++) {
      ssids += config.wifiSSIDs[i];
      ssids += ", ";
    }
    ssids += "]";
    docSend["ssids"] = ssids;
  }

  /*********************** MDNS COMMAND ****************************/
  // e.g. {"cmd":{"name":"mdns", "payload":{"name":"newName"}}}
  else if (strcmp(cmd, CMD_MDNS) == 0) {
    docSend["error"] = true;
    const char* newName = docRcv["cmd"]["payload"]["name"];
    if (newName == nullptr) {
      docSend["msg"] = F("MDNS name required in payload with key name");
      return;
    }
    if (strlen(newName) < MAX_NAME_LEN) {
      config.setName((char * )newName);
    } else {
      response = F("MDNS name too long, only string of size ");
      response += MAX_NAME_LEN;
      response += F(" allowed");
      docSend["msg"] = response;
      return;
    }
    char * name = config.name;
    response = F("Set MDNS name to: ");
    response += name;
    //docSend["msg"] = sprintf( %s", name);
    docSend["msg"] = response;
    docSend["mdns_name"] = name;
    docSend["error"] = false;
    initMDNS();
  }
  /*********************** ADD WIFI COMMAND ****************************/
  // e.g. {"cmd":{"name":"addWifi", "payload":{"ssid":"ssidName","pwd":"pwdName"}}}
  else if (strcmp(cmd, CMD_ADD_WIFI) == 0) {
    docSend["error"] = true;
    const char* newSSID = docRcv["cmd"]["payload"]["ssid"];
    const char* newPWD = docRcv["cmd"]["payload"]["pwd"];
    if (newSSID == nullptr or newPWD == nullptr) {
      docSend["msg"] = F("WiFi SSID and PWD required, for open networks, fill empty pwd");
      return;
    }
    bool success = false;
    if (strlen(newSSID) < MAX_SSID_LEN and strlen(newPWD) < MAX_PWD_LEN) {
      success = config.addWiFi((char * )newSSID, (char * )newPWD);
    } else {
      response = F("SSID or PWD too long, max: ");
      response += MAX_SSID_LEN;
      response += F(", ");
      response += MAX_PWD_LEN;
      docSend["msg"] = response;
      return;
    }
    if (success)  {
      char * name = config.wifiSSIDs[config.numAPs-1];
      char * pwd = config.wifiPWDs[config.numAPs-1];
      response = F("New Ap, SSID: ");
      response += name;
      response = F(", PW: ");
      response += pwd;
      //docSend["msg"] = sprintf( %s", name);
      docSend["ssid"] = name;
      docSend["pwd"] = pwd;
      docSend["error"] = false;
    } else {
      response = F("MAX # APs reached, need to delete first");
    }

    docSend["msg"] = response;
    String ssids = "[";
    for (int i = 0; i < config.numAPs; i++) {
      ssids += config.wifiSSIDs[i];
      ssids += ", ";
    }
    ssids += "]";
    docSend["ssids"] = ssids;
  }

  /*********************** ADD WIFI COMMAND ****************************/
  // e.g. {"cmd":{"name":"delWifi", "payload":{"ssid":"ssidName"}}}
  else if (strcmp(cmd, CMD_REMOVE_WIFI) == 0) {
    docSend["error"] = true;
    const char* newSSID = docRcv["cmd"]["payload"]["ssid"];
    if (newSSID == nullptr) {
      docSend["msg"] = F("Required SSID to remove");
      return;
    }
    bool success = false;
    if (strlen(newSSID) < MAX_SSID_LEN) {
      success = config.removeWiFi((char * )newSSID);
    } else {
      response = F("SSID too long, max: ");
      response += MAX_SSID_LEN;
      docSend["msg"] = response;
      return;
    }
    if (success)  {
      response = F("Removed SSID: ");
      response += newSSID;
      docSend["error"] = false;
    } else {
      response = F("SSID ");
      response += newSSID;
      response += F(" not found");
    }
    docSend["msg"] = response;
    String ssids = "[";
    for (int i = 0; i < config.numAPs; i++) {
      ssids += config.wifiSSIDs[i];
      ssids += ", ";
    }
    ssids += "]";
    docSend["ssids"] = ssids;
  }

  /*********************** NTP COMMAND ****************************/
  // e.g. {"cmd":{"name":"ntp"}}
  else if (strcmp(cmd, CMD_NTP) == 0) {
    if (getTimeNTP()) {
      docSend["msg"] = "Time synced";
      docSend["error"] = false;
    } else {
      docSend["msg"] = "Error syncing time";
      docSend["error"] = true;
    }
    char * timeStr = printTime(ntpEpochSeconds, ntpMilliseconds);
    docSend["ntp_time"] = timeStr;
    timeStr = printCurrentTime();
    docSend["current_time"] = timeStr;
  }
}

/****************************************************
 * Try to connect the streaming tcp client.
 ****************************************************/
bool tryConnectStream() {
  if (!streamClient.connected()) {
    // Look for people connecting over the streaming server
    streamClient = streamServer.available();
    if (streamClient && streamClient.connected()) {
      return true;
    } else {
      return false;
    }
  }
  return true;
}

/****************************************************
 * Stop Sampling will go into ide state, stop the
 * interrupt and if any stream client is connected,
 * will send EOF to the client
 ****************************************************/
void stopSampling() {
  vTaskDelete(TaskLoop2);
  state = STATE_IDLE;
  next_state = STATE_IDLE;
  turnInterrupt(false);
  samplingCountdown = 0;
  samplingDuration = millis() - startSamplingMillis;
  if (streamClient && streamClient.connected()) streamClient.stop();
  lifenessUpdate = millis();
  mdnsUpdate = millis();
}

void calcChunkSize() {
  chunkSize = min(max(MEASURMENT_BYTES, int(float(0.1*(float)(samplingRate*MEASURMENT_BYTES)))), MAX_SEND_SIZE);
  chunkSize--;
  chunkSize |= chunkSize >> 1;
  chunkSize |=chunkSize >> 2;
  chunkSize |= chunkSize >> 4;
  chunkSize |= chunkSize >> 8;
  chunkSize |= chunkSize >> 16;
  chunkSize++;
  // Typical
  chunkSize = min((int)chunkSize, MAX_SEND_SIZE);
  if (state == STATE_SAMPLE_UDP) {
    chunkSize = min((int)chunkSize, 512);
  }
}
/****************************************************
 * Start Sampling requires to start the interrupt and
 * to calculate the chunkSize size depending on
 * the samplingrate currently set. Furthermore,
 * all buffer indices are reset to the default values.
 ****************************************************/
inline void startSampling() {
  startSampling(false);
}

void startSampling(bool waitVoltage) {
  createPSRAM_TASK();
  counter = 0;
  sentSamples = 0;
  totalSamples = 0;
  TIMER_CYCLES_FAST = (1000000) / samplingRate; // Cycles between HW timer inerrupts
  calcChunkSize();
  writePtr = 0;
  psdReadPtr = 0;
  psdWritePtr = 0;
  samplingDuration = 0;
  packetNumber = 0;
  if (waitVoltage) {
    while(stpm34.readVoltage(1) > 0) {yield();}
    while(stpm34.readVoltage(1) < 0) {yield();}
  }
  startSamplingMillis = millis();
  turnInterrupt(true);

}

/****************************************************
 * Detach or attach intterupt
 ****************************************************/
void turnInterrupt(bool on) {
  cli();//stop interrupts
  if (on) {
    timer_init();
    timerAlarmWrite(timer, TIMER_CYCLES_FAST, true);
    timerAlarmEnable(timer);
  } else {
    timerAlarmDisable(timer);
    timer = NULL;
  }
  sei();
}


/****************************************************
 * Init the MDNs name from eeprom, only the number ist
 * stored in the eeprom, construct using prefix.
 ****************************************************/
void initMDNS() {
  char * name = config.name;
  if (strlen(name) == 0) {
    Serial.print(F("Info:Sth wrong with mdns"));
    strcpy(name,"powerMeterX");
  }
  // Setting up MDNs with the given Name
  Serial.print(F("Info:MDNS Name: ")); Serial.println(name);
  if (!MDNS.begin(String(name).c_str())) {             // Start the mDNS responder for esp8266.local
    Serial.println(F("Info:Error setting up MDNS responder!"));
  }
  MDNS.addService("elec", "tcp", 54322);
}

unsigned long lastTry = millis();
void updateTime(bool ntp) {
  int32_t delta = millis()-ntpValidMillis;
  if (ntp and delta > 60000 && millis()-lastTry > 10000) {
    getTimeNTP();
    lastTry = millis();
    delta = millis()-ntpValidMillis;
  }
  if (ntpEpochSeconds == 0) return;
  currentMilliseconds = ntpMilliseconds + delta%1000;
  currentSeconds = ntpEpochSeconds + delta/1000 + currentMilliseconds/1000;
  currentMilliseconds = currentMilliseconds%1000;
}

char timeString[32];
char * printCurrentTime() {
  // uint32_t delta = millis()-ntpValidMillis;
  // uint32_t milliSeconds = ntpMilliseconds + delta%1000;
  // unsigned long seconds = ntpEpochSeconds + delta/1000 + milliSeconds/1000;
  // milliSeconds = milliSeconds%1000;
  updateTime(false);
  return printTime(currentSeconds, currentMilliseconds);
}

char * printDuration(unsigned long ms) {
  sprintf(timeString, "%02d:%02d:%02d.%03d", hour(ms/1000), minute(ms/1000), second(ms/1000), ms%1000);
  return timeString;
}

char * printTime(unsigned long s, unsigned long ms) {
  unsigned long slocal = s + LOCATION_OFFSET;
  // Serial.print("Date: ");
  // Serial.print()%4d-%02d-%02d %02d:%02d:%02d\n", year(t_unix_date1), month(t_unix_date1), day(t_unix_date1), hour(t_unix_date1), minute(t_unix_date1), second(t_unix_date1));
  sprintf(timeString, "%4d-%02d-%02d %02d:%02d:%02d.%03d", year(slocal), month(slocal), day(slocal), hour(slocal), minute(slocal), second(slocal), ms);
  return timeString;
}
/****************************************************
 * Try to get accurate time over NTP
 ****************************************************/
bool getTimeNTP() {
  Serial.println(F("Info:Sending NTP packet..."));
  WiFi.hostByName(ntpServerName, timeServerIP);
  Serial.print(F("Info:To "));
  Serial.print(ntpServerName);
  Serial.print(F(" with IP: "));
  Serial.println(timeServerIP);
  // Reset all bytes in the buffer to 0
  memset(ntpBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  ntpBuffer[0] = 0b11100011; // LI, Version, Mode
  ntpBuffer[1] = 0; // Stratum, or type of clock
  ntpBuffer[2] = 6; // Polling Interval
  ntpBuffer[3] = 0xEC; // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  ntpBuffer[12]  = 49;
  ntpBuffer[13]  = 0x4E;
  ntpBuffer[14]  = 49;
  ntpBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:

  long start = millis();
  udpNtp.beginPacket(timeServerIP, 123); //NTP requests are to port 123
  udpNtp.write(ntpBuffer, NTP_PACKET_SIZE);
  udpNtp.endPacket();// should flush
  // Wait for packet to arrive with timeout of 2 seconds
  int cb = udpNtp.parsePacket();
  while(!cb) {
    if (start + 2000 < millis()) break;
    cb = udpNtp.parsePacket();
    yield();
  }
  if (!cb) {
    Serial.println(F("Info:No NTP response yet"));
    return false;
  }

  ntpValidMillis = millis();
  uint16_t tripDelayMs = (ntpValidMillis-start)/2;
  ntpValidMillis -= tripDelayMs;
  //Serial.print("Info:Received NTP packet, length=");
  //Serial.println(cb);
  // We've received a packet, read the data from it
  udpNtp.read(ntpBuffer, NTP_PACKET_SIZE);
  // read the packet into the buf
  unsigned long highWord = word(ntpBuffer[40], ntpBuffer[41]);
  unsigned long lowWord = word(ntpBuffer[42], ntpBuffer[43]);
  // combine the four bytes (two words) into a long integer
  // this is NTP time (seconds since Jan 1 1900):
  unsigned long secsSince1900 = highWord << 16 | lowWord;
  uint32_t frac  = (uint32_t) ntpBuffer[44] << 24
                 | (uint32_t) ntpBuffer[45] << 16
                 | (uint32_t) ntpBuffer[46] <<  8
                 | (uint32_t) ntpBuffer[47] <<  0;
  uint16_t mssec = ((uint64_t) frac *1000) >> 32;
  // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
  // subtract seventy years:
  unsigned long epoch = secsSince1900 - 2208988800UL;
  // Add the time we waited
  ntpEpochSeconds = epoch;
  ntpMilliseconds = mssec;
  #ifdef DEBUG
  Serial.print(F("Info:NTP Time:"));
  Serial.println(printTime(ntpEpochSeconds, ntpMilliseconds));
  #endif
  return true;
}


void timer_init() {
  // Timer base freq is 80Mhz
  timer = NULL;
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &sample_ISR, true);
}
