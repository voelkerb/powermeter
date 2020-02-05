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
#include <rom/rtc.h>
#include <PubSubClient.h>

#include "constDefine.h"
#include "src/logger/logger.h"
#include "src/stpm/STPM.h"
#include "src/relay/relay.h"
#include "src/network/network.h"
#include "src/rtc/rtc.h"
#include "src/config/config.h"
#include "src/time/timeHandling.h"
#include "src/ringbuffer/ringbuffer.h"



// Function prototypes required for e.g. platformio
void calcChunkSize();
bool getTimeNTP();
void handleEvent(Stream &getter);
void handleJSON();
void initMDNS();
void onIdle();
void onSampling();
void onWifiConnect();
void onWifiDisconnect();
char * printCurrentTime();
char * printDuration(unsigned long ms);
char * printTime(unsigned long s, unsigned long ms);
void IRAM_ATTR sample_ISR();
void setInfoString(char * str);
void setupOTA();
void IRAM_ATTR sqwvTriggered();
inline void startSampling();
void startSampling(bool waitVoltage);
void stopSampling();
void timer_init();
void turnInterrupt(bool on);
void updateTime(bool ntp);
void updateTimeInBG(void * pvParameters);
void writeChunks(bool tail);
void writeData(Stream &getter, uint16_t size);
char * timeStr();

TaskHandle_t xHandle = NULL;

// Serial logger
StreamLogger serialLog((Stream*)&Serial, &timeStr, &LOG_PREFIX_SERIAL[0], ALL);
// SPIFFS logger
SPIFFSLogger spiffsLog(false, &LOG_FILE[0], &timeStr, &LOG_PREFIX_SERIAL[0], WARNING);

// MultiLogger logger(&streamLog, &timeStr);
// Create singleton here
MultiLogger& logger = MultiLogger::getInstance();

Configuration config;

Relay relay(RELAY_PIN_S, RELAY_PIN_R);

Rtc rtc(RTC_INT);
TimeHandler myTime(&rtc, ntpServerName, LOCATION_TIME_OFFSET);

// STPM Object
STPM stpm34(STPM_RES, STPM_CS, STPM_SYN);

// counter holds # isr calls
volatile uint16_t counter = 0;
// Last micros() count of isr call
volatile long nowTs = 0;
volatile long lastTs = 0;

RingBuffer ringBuffer(PS_BUF_SIZE, true);

static uint8_t sendbuffer[MAX_SEND_SIZE+16] = {0};
// Buffer read/write position
uint32_t psdReadPtr = 0;
uint32_t psdWritePtr = 0;
volatile uint32_t writePtr = 0;

// Open two TCP ports, one for commands and sampling and one for a raw data stream
WiFiServer server(STANDARD_TCP_SAMPLE_PORT);
WiFiServer streamServer(STANDARD_TCP_STREAM_PORT);

// TIMER stuff
// Calculate the number of cycles we have to wait
volatile uint32_t TIMER_CYCLES_FAST = (1000000) / DEFAULT_SR; // Cycles between HW timer inerrupts
volatile uint32_t timer_next;
volatile uint32_t timer_now;


// Internal state machine for sampling
enum class SampleState{IDLE, SAMPLE};

SampleState state = SampleState::IDLE;
SampleState next_state = SampleState::IDLE;

// Available measures are VOLTAGE+CURRENT, ACTIVE+REACTIVE Power or both
enum class Measures{VI, PQ, VIPQ};
enum class StreamType{USB, TCP, UDP, TCP_RAW, MQTT};

struct StreamConfig {
  bool prefix = false;                      // Send data with "data:" prefix
  Measures measures = Measures::VI;             // The measures to send
  uint8_t measurementBytes = 8;             // Number of bytes for each measurement entry
  unsigned int samplingRate = DEFAULT_SR;   // The samplingrate
  StreamType stream = StreamType::USB;                  // Channel over which to send
  unsigned int countdown = 0;               // Start at specific time or immidiately
  uint32_t chunkSize = 0;                   // Chunksize of one packet sent
  IPAddress ip;                             // Ip address of data sink
  uint16_t port = STANDARD_TCP_SAMPLE_PORT; // Port of data sink
};

StreamConfig streamConfig;


// Stuff for frequency calculation
volatile long freqCalcStart;
volatile long freqCalcNow;
volatile long freq = 0;

// TCP clients and current connected state, each client is assigned a logger
WiFiClient client[MAX_CLIENTS];
bool clientConnected[MAX_CLIENTS] = {false};
StreamLogger * streamLog[MAX_CLIENTS];

// Strean client is for ffmpeg direct streaming
WiFiClient streamClient;
// tcp stuff is send over this client 
// Init getter to point to sth, might not work otherwise
Stream * newGetter = &Serial;

WiFiClient * sendClient;
// UDP used for streaming
WiFiUDP udpClient;

// Command stuff send over what ever
char command[COMMAND_MAX_SIZE] = {'\0'};
StaticJsonDocument<2*COMMAND_MAX_SIZE> docRcv;
StaticJsonDocument<2*COMMAND_MAX_SIZE> docSend;
String response = "";

// Information about the current sampling period
unsigned long samplingCountdown = 0;
unsigned long startSamplingMillis = 0;
unsigned long samplingDuration = 0;
unsigned long sentSamples = 0;
volatile unsigned long packetNumber = 0;
volatile unsigned long totalSamples = 0;

// Some timer stuff s.t. things are updated regularly and not at full speed
long lifenessUpdate = millis();
long mdnsUpdate = millis();
long tcpUpdate = millis();
long rtcUpdate = millis();
long mqttUpdate = millis();

// HW Timer and mutex for sapmpling ISR
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Current CPU speed
unsigned int coreFreq = 0;

// test stuff
long testMillis = 0;
long testMillis2 = 0;
uint16_t testSamples = 0;

// Mutex for 1s RTC Interrupt
portMUX_TYPE sqwMux = portMUX_INITIALIZER_UNLOCKED;
bool firstSqwv = true;
volatile int sqwCounter = 0;

volatile float values[4] = {0};

// FPU register state
uint32_t cp0_regs[18];

// OTA Update in progress
bool updating = false;

// How can we change those parameters in Arduino, possible?
// CONFIG_TCP_MSS=1436
// CONFIG_TCP_MSL=60000
// CONFIG_TCP_SND_BUF_DEFAULT=5744
// CONFIG_TCP_WND_DEFAULT=5744

char mqttTopicPubSwitch[MAX_MQTT_PUB_TOPIC_SWITCH+MAX_NAME_LEN] = {'\0'};
char mqttTopicPubSample[MAX_MQTT_PUB_TOPIC_SAMPLE+MAX_NAME_LEN] = {'\0'};
char mqttTopicPubInfo[MAX_MQTT_PUB_TOPIC_INFO+MAX_NAME_LEN] = {'\0'};
bool mqttConnected = false;
WiFiClient mqtt_client;
PubSubClient mqttClient(mqtt_client);

/************************ SETUP *************************/
void setup() {
  // Setup serial communication
  Serial.begin(SERIAL_SPEED);

  // At first init the rtc module to get 
  bool successAll = true;
  bool success = rtc.init();
  // Init the logging module
  logger.setTimeGetter(&timeStr);
  // Add Serial logger
  logger.addLogger(&serialLog);
  // Add spiffs logger
  logger.addLogger(&spiffsLog);
  // Init all loggers
  logger.init();
  // init the stream logger array
  for (size_t i = 0; i < MAX_CLIENTS; i++) {
    StreamLogger * theStreamLog = new StreamLogger(NULL, &timeStr, &LOG_PREFIX[0], INFO);
    streamLog[i] = theStreamLog;
  }

  if (!success) logger.log(ERROR, "Cannot init RTC");
  successAll &= success;
  relay.set(true);
  
  relay.setCallback(relayCB);

  // We do not need bluetooth, so disable it
  esp_bt_controller_disable();
  pinMode(ERROR_LED, OUTPUT);

  // Indicate lifeness / reset
  digitalWrite(ERROR_LED, HIGH);
  delay(200);
  digitalWrite(ERROR_LED, LOW);
  delay(200);
  digitalWrite(ERROR_LED, HIGH);

  config.init();

  // config.makeDefault();
  // config.store();
  
  
  config.load();

  relay.set(config.getRelayState());


  coreFreq = getCpuFrequencyMhz();
  logger.log(DEBUG, "%s @ firmware %s/%s", config.name, __DATE__, __TIME__);
  logger.log(DEBUG, "Core @ %u MHz", coreFreq);

  success = ringBuffer.init();
  if (!success) logger.log(ERROR, "PSRAM init failed");
  successAll &= success;

  // config.makeDefault();
  // config.store();

  timer_init();

  // Setup STPM 32
  success = stpm34.init();
  if (!success) logger.log(ERROR, "STPM Init Failed");
  successAll &= success;

  
   // Indicate error if there is any
  digitalWrite(ERROR_LED, !successAll);

  logger.log(ALL, "Connecting WLAN");

  Network::init(&config, onWifiConnect, onWifiDisconnect);
  setupOTA();

  response.reserve(2*COMMAND_MAX_SIZE);


  setInfoString(&command[0]);
  logger.log(&command[0]);

  logger.log(ALL, "Setup done");

  lifenessUpdate = millis();
  mdnsUpdate = millis();
  tcpUpdate = millis();
  rtcUpdate = millis();
  mqttUpdate = millis();
}

// the loop routine runs over and over again forever:
void loop() {
  if (updating) return;

  // Stuff done on idle
  if (state == SampleState::IDLE) {
    onIdle();
  // Stuff on sampling
  } else if (state == SampleState::SAMPLE) {
    onSampling();
  }

  onIdleOrSampling();

  // If we only have 200 ms before sampling should start, wait actively
  if (next_state != SampleState::IDLE) {
    if (streamConfig.countdown != 0 and (streamConfig.countdown - millis()) < 200) {
      // Disable any wifi sleep mode
      esp_wifi_set_ps(WIFI_PS_NONE);
      state = next_state;
      logger.log(DEBUG, "StartSampling @ %s", myTime.timeStr());
      // Calculate time to wait
      int32_t mydelta = streamConfig.countdown - millis();
      // Waiting here is not nice, but we wait for zero crossing
      if (mydelta > 0) delay(mydelta-1);
      startSampling(true);
      // Reset sampling countdown
      streamConfig.countdown = 0;
    }
  }

  // Watchdog
  yield();
}


char * timeStr() {
  char * ttime = myTime.timeStr(true);
  return ttime;
}

void onWifiConnect() {
  logger.log(ALL, "Wifi Connected");
  logger.log(ALL, "IP: %s", WiFi.localIP().toString().c_str());
  // The stuff todo if we have a network connection (and hopefully internet as well)
  if (Network::connected and not Network::apMode) {
    myTime.updateNTPTime();
  }

  // Reinit mdns
  initMDNS();
  // Start the TCP server
  server.begin();
  streamServer.begin();

  // Reset lifeness and MDNS update
  lifenessUpdate = millis();
  mdnsUpdate = millis();
  tcpUpdate = millis();
  rtcUpdate = millis();
  mqttUpdate = millis();
}

void onWifiDisconnect() {
  logger.log(ERROR, "Wifi Disconnected");

  if (state != SampleState::IDLE) {
    logger.log(ERROR, "Stop sampling (Wifi disconnect)");
    stopSampling();
  }
}

void onIdleOrSampling() {

  // MQTT loop
  mqttClient.loop();

  // Handle serial requests
  if (Serial.available()) {
    handleEvent(Serial);
  }

  // Handle tcp requests
  for (size_t i = 0; i < MAX_CLIENTS; i++) {
    if (client[i].available() > 0) {
      handleEvent(client[i]);
    }
  }

  // Handle tcp clients connections

  if ((long)(millis() - tcpUpdate) >= 0) {
    tcpUpdate += TCP_UPDATE_INTERVAL;
    // Handle disconnect
    for (size_t i = 0; i < MAX_CLIENTS; i++) {
      if (clientConnected[i] and !client[i].connected()) {
        onClientDisconnect(client[i], i);
        clientConnected[i] = false;
      }
    }

    // Handle connect
    WiFiClient newClient = server.available();
    if (newClient) {
      onClientConnect(newClient);
    }
  }
}


void onIdle() {
  spiffsLog.flush();
  // Arduino OTA
  ArduinoOTA.handle();

  // Re-advertise MDNS service service every 30s 
  // TODO: no clue why, but does not work properly for esp32 (maybe it is the mac side)
  if ((long)(millis() - mdnsUpdate) >= 0) {
    mdnsUpdate += MDNS_UPDATE_INTERVAL;
    // initMDNS();
    //MDNS.addService("_elec", "_tcp", STANDARD_TCP_STREAM_PORT);
  }
  // MQTT stuff, check connection status, on disconnect, try reconnect
  // TODO: no clue why, but does not work properly for esp32 (maybe it is the mac side)
  if ((long)(millis() - mqttUpdate) >= 0) {
    mqttUpdate += MQTT_UPDATE_INTERVAL;

    if (mqttConnected and !mqttClient.connected()) {
      logger.log(WARNING, "Disconnected from MQTT server");
      mqttConnected = false;
    }
    if (!mqttConnected) {
      initMQTT();
    }
  }

  if ((long)(millis() - rtcUpdate) >= 0) {
    rtcUpdate += RTC_UPDATE_INTERVAL;
    if (rtc.connected) rtc.update();
  }
    
  // Update lifeness only on idle every second
  if ((long)(millis() - lifenessUpdate) >= 0) {
    lifenessUpdate += LIFENESS_UPDATE_INTERVAL;
    logger.log("");
    logger.log("%s", config.getRelayState() ? "on":"off");
  }

  // Look for people connecting over the stream server
  // If one connected there, immidiately start streaming data
  if (!streamClient.connected()) {
    streamClient = streamServer.available();
    if (streamClient.connected()) {
      // Set everything to default settings
      streamConfig.stream = StreamType::TCP_RAW;
      streamConfig.prefix = false;
      streamConfig.samplingRate = DEFAULT_SR;
      streamConfig.measures = Measures::VI;
      streamConfig.ip = streamClient.remoteIP();
      streamConfig.port = STANDARD_TCP_SAMPLE_PORT;
      sendClient = &streamClient;
      startSampling();
    }
  }
}

void onClientConnect(WiFiClient &newClient) {
  logger.log("Client with IP %s connected on port %u", newClient.remoteIP().toString().c_str(), newClient.remotePort());
  
  // Loop over all clients and look where we can store the pointer... 
  for (size_t i = 0; i < MAX_CLIENTS; i++) {
    if (!clientConnected[i]) {
      client[i] = newClient;
      // Set connected flag
      clientConnected[i] = true;
      streamLog[i]->_type = INFO; // This might be later reset
      streamLog[i]->_stream = (Stream*)&client[i];
      logger.addLogger(streamLog[i]);
      return;
    }
  }
  logger.log("To much clients, could not add client");
  newClient.stop();
}

void onClientDisconnect(WiFiClient &oldClient, size_t i) {
  logger.log("Client discconnected %s port %u", oldClient.remoteIP().toString().c_str(), oldClient.remotePort());
  logger.removeLogger(streamLog[i]);
  streamLog[i]->_stream = NULL;
}

void onSampling() {
  // ______________ Send data to sink ________________
  writeChunks(false);

  // For error check during sampling
  if (sqwCounter) {
    portENTER_CRITICAL(&sqwMux);
    sqwCounter--;
    portEXIT_CRITICAL(&sqwMux);
    if (!firstSqwv and testSamples != 0) {
      response = "******** MISSED ";
      response += streamConfig.samplingRate - testSamples;
      response += " SAMPLES ********";
      logger.log(ERROR, response.c_str());
    }
    // If it is still > 0, our loop is too slow, 
    // reasons may be e.g. slow tcp write performance
    if (sqwCounter) {
      logger.log(WARNING, "Loop %us behind", sqwCounter);
      // Reset to 0
      portENTER_CRITICAL(&sqwMux);
      sqwCounter = 0;
      portEXIT_CRITICAL(&sqwMux);
    }
    // Ignore first sqwv showing missing samples since
    // sqwv did not start with sampling
    // NOTE: is this valid?
    // TODO: can we reset the sqwv somehow?
    if (firstSqwv) firstSqwv = false;
    testMillis = testMillis2;
  }

  // Output sampling frequency regularly
  if (freq != 0) {
    long fr = freq;
    freq = 0;
    float frequency = (float)streamConfig.samplingRate/(float)((fr)/1000000.0);
    logger.log("%.2fHz", frequency);
  }

  // ______________ Handle Disconnect ________________

  // NOTE: Cannot detect disconnect for USB
  if (streamConfig.stream == StreamType::USB) {
  } else if (streamConfig.stream == StreamType::TCP) {
    if (!sendClient->connected()) {
      logger.log(ERROR, "TCP disconnected while streaming");
      stopSampling();
    }
  // Disconnect of UDP means disconnecting from tcp port
  } else if (streamConfig.stream == StreamType::UDP) {
    if (!sendClient->connected()) {
      logger.log(ERROR, "TCP/UDP disconnected while streaming");
      stopSampling();
    }
  // Disconnect of raw stream means stop
  } else if (streamConfig.stream == StreamType::TCP_RAW) {
    // Check for intended connection loss
    if (!sendClient->connected()) {
      logger.log(INFO, "TCP Stream disconnected");
      stopSampling();
    }
  }
}

// Depending on data sink, send data
void writeChunks(bool tail) {
  while(ringBuffer.available() > streamConfig.chunkSize) {
    if (streamConfig.stream == StreamType::UDP) {
      udpClient.beginPacket(streamConfig.ip, streamConfig.port);
      writeData(udpClient, streamConfig.chunkSize);
      udpClient.endPacket();
    } else if (streamConfig.stream == StreamType::TCP) {
      writeData(*sendClient, streamConfig.chunkSize);
    } else if (streamConfig.stream == StreamType::TCP_RAW) {
      writeData(*sendClient, streamConfig.chunkSize);
    } else if (streamConfig.stream == StreamType::USB) {
      writeData(Serial, streamConfig.chunkSize);
    } else if (streamConfig.stream ==  StreamType::MQTT) {
      writeDataMQTT(streamConfig.chunkSize);
    }
  }
  if (tail) {
    if (streamConfig.stream == StreamType::UDP) {
      udpClient.beginPacket(streamConfig.ip, streamConfig.port);
      writeData(udpClient, ringBuffer.available());
      udpClient.endPacket();
    } else if (streamConfig.stream == StreamType::TCP) {
      writeData(*sendClient, ringBuffer.available());
    } else if (streamConfig.stream == StreamType::TCP_RAW) {
      writeData(*sendClient, ringBuffer.available());
    } else if (streamConfig.stream == StreamType::USB) {
      writeData(Serial, ringBuffer.available());
    } else if (streamConfig.stream ==  StreamType::MQTT) {
      writeDataMQTT(streamConfig.chunkSize);
    }
  }
}

void writeDataMQTT(uint16_t size) {
  if (size <= 0) return;
  size_t i = 0;
  while (i < size) { 
    i++;
  }
  logger.log(ERROR, "TODO implement");
}

// Data prefix 
const char data_id[5] = {'D','a','t','a',':'};
void writeData(Stream &getter, uint16_t size) {
  if (size <= 0) return;
  uint32_t start = 0;
  if (streamConfig.prefix) {
    memcpy(&sendbuffer[start], (void*)&data_id[0], sizeof(data_id));
    start += sizeof(data_id);
    memcpy(&sendbuffer[start], (void*)&size, sizeof(uint16_t));
    start += sizeof(uint16_t);
    memcpy(&sendbuffer[start], (void*)&packetNumber, sizeof(uint32_t));
    start += sizeof(uint32_t);
    packetNumber += 1;
  }

  ringBuffer.read(&sendbuffer[start], size);
  // Everything is sent at once (hopefully)
  uint32_t sent = getter.write((uint8_t*)&sendbuffer[0], size+start);
  if (sent > start) sentSamples += (sent-start)/streamConfig.measurementBytes;
}


static SemaphoreHandle_t timer_sem;
volatile uint32_t mytime = micros();

void IRAM_ATTR sample_ISR() {
  static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  //Serial.println("TIMERISR");
  mytime = micros();
  xSemaphoreGiveFromISR(timer_sem, &xHigherPriorityTaskWoken);
  if ( xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR(); // this wakes up sample_timer_task immediately
  }
}

void IRAM_ATTR sample_timer_task(void *param) {
  timer_sem = xSemaphoreCreateBinary();

  float data[4] = { 0.0 };

  int test = 0;

  while (state == SampleState::SAMPLE) {
    xSemaphoreTake(timer_sem, portMAX_DELAY);
    // test++;
    // if (test%100 == 0) {
    //   Serial.println(micros()-mytime);
    // }
    
    // Vaiables for frequency count
    counter++;
    totalSamples++;
    if (counter >= streamConfig.samplingRate) {
      freqCalcNow = micros();
      freq = freqCalcNow-freqCalcStart;
      freqCalcStart = freqCalcNow;
      counter = 0;
      if (rtc.connected) timerAlarmDisable(timer);
    }

    if (streamConfig.measures == Measures::VI) {
      stpm34.readVoltAndCurr((float*) &data[0]);
      // stpm34.readVoltageAndCurrent(1, (float*) &values[0], (float*) &values[1]);
    } else if (streamConfig.measures == Measures::PQ) {
      stpm34.readPower(1, (float*) &data[0], (float*) &data[2], (float*) &data[1], (float*) &data[2]);
    } else if (streamConfig.measures == Measures::VIPQ) {
      stpm34.readAll(1, (float*) &data[0], (float*) &data[1], (float*) &data[2], (float*) &data[3]);
    }

    ringBuffer.write((uint8_t*)&data[0], streamConfig.measurementBytes);

  }
  vTaskDelete( NULL );
}


// triggered each second to active a new generation of <samplingrate> samples 
void IRAM_ATTR sqwvTriggered() {
  // Disable timer if not already done in ISR
  timerAlarmDisable(timer);
  // Reset counter if not already done in ISR
  testSamples = counter;
  portENTER_CRITICAL_ISR(&timerMux);
  counter = 0;
  portEXIT_CRITICAL_ISR(&timerMux);
  
  // timerAlarmWrite(timer, TIMER_CYCLES_FAST, true);
  // We take one sample and enable the timing interrupt afterwards
  // this should allow the ISR e.g. at 4kHz to take 3999 samples during the next second
  timerWrite(timer, 0);
  sample_ISR();
  // Enable timer
  timerAlarmEnable(timer);
  // indicate to main that a second has passed and store time 
  portENTER_CRITICAL_ISR(&sqwMux);
  sqwCounter++;
  portEXIT_CRITICAL(&sqwMux);
  testMillis2 = millis();
}

// To display only full percent updates
unsigned int oldPercent = 0;
void setupOTA() {
  // Same name as mdns
  ArduinoOTA.setHostname(config.name);
  ArduinoOTA.setPassword("energy"); 
  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash(2"21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    updating = true;
    logger.log("Start updating");

    // Disconnecting all connected clients
    for (size_t i = 0; i < MAX_CLIENTS; i++) {
      if (clientConnected[i]) {
        onClientDisconnect(client[i], i);
        clientConnected[i] = false;
      }
    }
    if (streamClient.connected()) streamClient.stop();
    
    // Stopping all other tcp stuff
    streamServer.stop();
    streamServer.close();
    server.stop();
    server.close();
  });

  ArduinoOTA.onEnd([]() {
    logger.log("End");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    unsigned int percent = (progress / (total / 100));
    if (percent != oldPercent) {
      logger.log("Progress: %u%%\r", (progress / (total / 100)));
      oldPercent = percent;
    }
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    logger.append("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) logger.append("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) logger.append("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) logger.append("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) logger.append("Receive Failed");
    else if (error == OTA_END_ERROR) logger.append("End Failed");
    logger.flush(ERROR);
    // No matter what happended, simply restart
    ESP.restart();
  });
  // Enable OTA
  ArduinoOTA.begin();
}

void setBusyResponse() {
  if (sendClient != NULL) {
    response = "Device with IP: ";
    response += sendClient->localIP().toString();
    response += " currently sampling"; 
  } else {
    response = "Currently sampling";
  }
}
// _____________________________________________________________________________

/****************************************************
 * A request happended, handle it
 ****************************************************/
void handleEvent(Stream &getter) {
  if (!getter.available()) return;
  getter.readStringUntil('\n').toCharArray(command,COMMAND_MAX_SIZE);
  // Be backwards compatible to "?" command all the time
  if (command[0] == '?') {
    getter.println(F("Info:Setup done"));
    return;
  }
  #ifdef DEBUG_DEEP
  logger.log(INFO, command);
  #endif

  newGetter = (Stream*)&getter;

  response = "";

  parseCommand();
  handleJSON();

  if (docSend.isNull() == false) {
    getter.flush();
    response = "";
    serializeJson(docSend, response);
    response = "Info:" + response;
    getter.println(response);
    // This will be too long for the logger
    // logger.log(response.c_str());
  }
  response = "";
  command[0] = '\0';
}

void parseCommand() {
  // Deserialize the JSON document
  DeserializationError error = deserializeJson(docRcv, command);
  
  // Test if parsing succeeds.
  if (error) {
    // Remove all unallowed json characters to prevent error 
    uint32_t len = strlen(command);
    if (len > 10) len = 10;
    for (size_t i = 0; i < len; i++) {
      if (command[i] == '\r' || command[i] == '\n' || command[i] == '"' || command[i] == '}' || command[i] == '{') command[i] = '_';
    }
    logger.log(ERROR, "deserializeJson() failed: %.10s", &command[0]);
    return;
  }
  //docSend.clear();
  JsonObject obj = docSend.to<JsonObject>();
  obj.clear();
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
    if (state == SampleState::IDLE) {
      // For sampling we need type payload and rate payload
      const char* typeC = root["cmd"]["payload"]["type"];
      const char* measuresC = root["cmd"]["payload"]["measures"];
      int rate = docRcv["cmd"]["payload"]["rate"].as<int>();
      unsigned long ts = docRcv["cmd"]["payload"]["time"].as<unsigned long>();
      bool prefix = docRcv["cmd"]["payload"]["prefix"].as<bool>();
      JsonVariant prefixVariant = root["cmd"]["payload"]["prefix"];

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
        streamConfig.measures = Measures::VI;
        streamConfig.measurementBytes = 8;
      } else if (strcmp(measuresC, "v,i") == 0) {
        streamConfig.measures = Measures::VI;
        streamConfig.measurementBytes = 8;
      } else if (strcmp(measuresC, "p,q") == 0) {
        streamConfig.measures = Measures::PQ;
        streamConfig.measurementBytes = 8;
      } else if (strcmp(measuresC, "v,i,p,q") == 0) {
        streamConfig.measures = Measures::VIPQ;
        streamConfig.measurementBytes = 16;
      } else {
        response = "Unsupported measures";
        response += measuresC;
        docSend["msg"] = response;
        return;
      }
      streamConfig.prefix = true;
      // If we do not want a prefix, we have to disable this if not at extra port
      if (!prefixVariant.isNull()) {
        streamConfig.prefix = prefix;
      }
      // e.g. {"cmd":{"name":"sample", "payload":{"type":"Serial", "rate":4000}}}
      if (strcmp(typeC, "Serial") == 0) {
        streamConfig.stream = StreamType::USB;
      // e.g. {"cmd":{"name":"sample", "payload":{"type":"MQTT", "rate":4000}}}
      } else if (strcmp(typeC, "MQTT") == 0) {
        streamConfig.stream = StreamType::MQTT;
      // e.g. {"cmd":{"name":"sample", "payload":{"type":"TCP", "rate":4000}}}
      } else if (strcmp(typeC, "TCP") == 0) {
        sendClient = (WiFiClient*)newGetter; 
        streamConfig.stream = StreamType::TCP;
        streamConfig.port = STANDARD_TCP_SAMPLE_PORT;
        streamConfig.ip = sendClient->remoteIP();
      // e.g. {"cmd":{"name":"sample", "payload":{"type":"UDP", "rate":4000}}}
      } else if (strcmp(typeC, "UDP") == 0) {
        streamConfig.stream = StreamType::UDP;
        int port = docRcv["cmd"]["payload"]["port"].as<int>();
        if (port > 80000 || port <= 0) {
          streamConfig.port = STANDARD_UDP_PORT;
          response = "Unsupported UDP port";
          response += port;
          docSend["msg"] = response;
          return;
        } else {
          streamConfig.port = port;
        }
        sendClient = (WiFiClient*)newGetter;
        docSend["port"] = streamConfig.port;
        streamConfig.ip = sendClient->remoteIP();
      } else if (strcmp(typeC, "FFMPEG") == 0) {

        bool success = streamClient.connected();
        if (!success) {
          // Look for people connecting over the streaming server and connect them
          streamClient = streamServer.available();
          if (streamClient && streamClient.connected()) success = true;
        }
        if (success) {
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
      streamConfig.samplingRate = rate;
      TIMER_CYCLES_FAST = (1000000) / streamConfig.samplingRate; // Cycles between HW timer inerrupts
      calcChunkSize();


      docSend["sampling_rate"] = streamConfig.samplingRate;
      docSend["chunk_size"] = streamConfig.chunkSize;
      docSend["conn_type"] = typeC;
      docSend["prefix"] = streamConfig.prefix;
      docSend["timer_cycles"] = TIMER_CYCLES_FAST;
      docSend["cmd"] = CMD_SAMPLE;

      relay.set(true);

      next_state = SampleState::SAMPLE;

      if (ts != 0) {
        response += F("Should sample at: ");
        response += myTime.timeStr(ts, 0);
        // Update ntp time actively wait for finish
        myTime.updateNTPTime(true);
        uint32_t delta = ts - myTime.utc_seconds();
        uint32_t nowMs = millis();
        delta *= 1000;
        delta -= myTime.milliseconds();
        if (delta > 20000 or delta < 500) {
          response += F("//nCannot start sampling in: "); response += delta; response += F("ms");
          streamConfig.countdown = 0;
        } else {
          response += F("//nStart sampling in: "); response += delta; response += F("ms");
          streamConfig.countdown = nowMs + delta;
          docSend["error"] = false;
        }
        docSend["msg"] = String(response);
        return;
      }
      docSend["error"] = false;
      state = next_state;
      // UDP packets are not allowed to exceed 1500 bytes, so keep size reasonable
      startSampling(true);
    } else {
      setBusyResponse();
      docSend["msg"] = response;
      docSend["state"] = "busy";
    }
  }

  /*********************** SWITCHING COMMAND ****************************/
  // {"cmd":{"name":"switch", "payload":{"value":"true"}}}
  else if (strcmp(cmd, CMD_SWITCH) == 0) {
    // For switching we need value payload
    docSend["error"] = true;
    JsonVariant payloadValue = root["cmd"]["payload"]["value"];
    if (payloadValue.isNull()) {
      docSend["error"] = false;
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
    stopSampling();
    // Write remaining chunks with tail
    writeChunks(true);
    docSend["msg"] = F("Received stop command");
    docSend["sample_duration"] = samplingDuration;
    docSend["samples"] = totalSamples;
    docSend["sent_samples"] = sentSamples;
    docSend["ip"] = WiFi.localIP().toString();
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
    config.store();
    ESP.restart();
  }

  /*********************** INFO COMMAND ****************************/
  // e.g. {"cmd":{"name":"info"}}
  else if (strcmp(cmd, CMD_INFO) == 0) {
    docSend["cmd"] = "info";
    docSend["type"] = F("powermeter");
    docSend["version"] = VERSION;
    String compiled = __DATE__;
    compiled += " ";
    compiled += __TIME__;
    docSend["compiled"] = compiled;
    docSend["sys_time"] = myTime.timeStr();
    docSend["name"] = config.name;
    docSend["ip"] = WiFi.localIP().toString();
    docSend["mqtt_server"] = config.mqttServer;
    docSend["sampling_rate"] = streamConfig.samplingRate;
    docSend["buffer_size"] = ringBuffer.getSize();
    docSend["psram"] = ringBuffer.inPSRAM();
    docSend["rtc"] = rtc.connected;
    docSend["state"] = state != SampleState::IDLE ? "busy" : "idle";
    String ssids = "[";
    for (size_t i = 0; i < config.numAPs; i++) {
      ssids += config.wifiSSIDs[i];
      if (i < config.numAPs-1) ssids += ", ";
    }
    ssids += "]";
    docSend["ssids"] = ssids;
  }

  /*********************** MDNS COMMAND ****************************/
  // e.g. {"cmd":{"name":"mdns", "payload":{"name":"newName"}}}
  else if (strcmp(cmd, CMD_MDNS) == 0) {
    if (state == SampleState::IDLE) {
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
    } else {
      setBusyResponse();
      docSend["msg"] = response;
      docSend["state"] = "busy";
    }
  }

  /*********************** MQTT Server COMMAND ****************************/
  // e.g. {"cmd":{"name":"mqttServer", "payload":{"server":"<ServerAddress>"}}}
  else if (strcmp(cmd, CMD_MQTT_SERVER) == 0) {
    if (state == SampleState::IDLE) {
      docSend["error"] = true;
      const char* newServer = docRcv["cmd"]["payload"]["server"];
      if (newServer == nullptr) {
        docSend["msg"] = F("MQTTServer address required in payload with key server");
        return;
      }
      if (strlen(newServer) < MAX_IP_LEN) {
        config.setMQTTServerAddress((char * )newServer);
      } else {
        response = F("MQTTServer address too long, only string of size ");
        response += MAX_IP_LEN;
        response += F(" allowed");
        docSend["msg"] = response;
        return;
      }
      char * address = config.mqttServer;
      response = F("Set MQTTServer address to: ");
      response += address;
      //docSend["msg"] = sprintf( %s", name);
      docSend["msg"] = response;
      docSend["mqtt_server"] = address;
      docSend["error"] = false;
      initMQTT();
    } else {
      setBusyResponse();
      docSend["msg"] = response;
      docSend["state"] = "busy";
    }
  }
  /*********************** ADD WIFI COMMAND ****************************/
  // e.g. {"cmd":{"name":"addWifi", "payload":{"ssid":"ssidName","pwd":"pwdName"}}}
  else if (strcmp(cmd, CMD_ADD_WIFI) == 0) {
    if (state == SampleState::IDLE) {
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
        response += F(", PW: ");
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
      for (size_t i = 0; i < config.numAPs; i++) {
        ssids += config.wifiSSIDs[i];
        ssids += ", ";
      }
      ssids += "]";
      docSend["ssids"] = ssids;
    } else {
      setBusyResponse();
      docSend["msg"] = response;
      docSend["state"] = "busy";
    }
  }

  /*********************** DEl WIFI COMMAND ****************************/
  // e.g. {"cmd":{"name":"delWifi", "payload":{"ssid":"ssidName"}}}
  else if (strcmp(cmd, CMD_REMOVE_WIFI) == 0) {
    if (state == SampleState::IDLE) {
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
      for (size_t i = 0; i < config.numAPs; i++) {
        ssids += config.wifiSSIDs[i];
        ssids += ", ";
      }
      ssids += "]";
      docSend["ssids"] = ssids;
    } else {
      setBusyResponse();
      docSend["msg"] = response;
      docSend["state"] = "busy";
    }
  }

  /*********************** NTP COMMAND ****************************/
  // e.g. {"cmd":{"name":"ntp"}}
  else if (strcmp(cmd, CMD_NTP) == 0) {
    if (myTime.updateNTPTime(true)) {
      docSend["msg"] = "Time synced";
      docSend["error"] = false;
    } else {
      docSend["msg"] = "Error syncing time";
      docSend["error"] = true;
    }
    char * timeStr = myTime.timeStr();
    docSend["current_time"] = timeStr;
  }

  /*********************** Clear Log COMMAND ****************************/
  // e.g. {"cmd":{"name":"clearLog"}}
  else if (strcmp(cmd, CMD_CLEAR_LOG) == 0) {
    if (state == SampleState::IDLE) {
      docSend["error"] = false;
      spiffsLog.clear();
    } else {
      setBusyResponse();
      docSend["msg"] = response;
      docSend["state"] = "busy";
    }
  }

  /*********************** Get Log COMMAND ****************************/
  // e.g. {"cmd":{"name":"getLog"}}
  else if (strcmp(cmd, CMD_GET_LOG) == 0) {
    if (state == SampleState::IDLE) {
      spiffsLog.flush();
      docSend["error"] = false;
      bool hasRow = spiffsLog.nextRow(&command[0]);
      newGetter->printf("%s{\"cmd\":\"log\",\"msg\":\"", &LOG_PREFIX[0]);
      newGetter->printf("*** LOGFile *** //n");
      while(hasRow) {
        newGetter->printf("%s//n", &command[0]);
        hasRow = spiffsLog.nextRow(&command[0]);
      }
      newGetter->println("*** LOGFile *** \"}");
    } else {
      setBusyResponse();
      docSend["msg"] = response;
      docSend["state"] = "busy";
    }
  }
}


/****************************************************
 * Stop Sampling will go into ide state, stop the
 * interrupt and if any stream client is connected,
 * will send EOF to the client
 ****************************************************/
void stopSampling() {
  if (rtc.connected) rtc.disableInterrupt();
  state = SampleState::IDLE;
  next_state = SampleState::IDLE;
  // Stop the timer interrupt
  turnInterrupt(false);
  samplingDuration = millis() - startSamplingMillis;
  if (streamClient && streamClient.connected()) streamClient.stop();
  // Reset all variables
  streamConfig.countdown = 0;
  lifenessUpdate = millis();
  mdnsUpdate = millis();
  // If realy has toggled store its new value
  config.store();
}

/****************************************************
 * Calculate the chunk size depending on the
 * samplingrate and sample method
 ****************************************************/
void calcChunkSize() {
  uint32_t chunkSize = int(float(0.1*(float)(streamConfig.samplingRate*streamConfig.measurementBytes)));
  // Keep within bounds
  if (chunkSize > MAX_SEND_SIZE) chunkSize = MAX_SEND_SIZE;
  else if (chunkSize < streamConfig.measurementBytes) chunkSize = streamConfig.measurementBytes;
  // Calc next base two
  chunkSize--;
  chunkSize |= chunkSize >> 1;
  chunkSize |=chunkSize >> 2;
  chunkSize |= chunkSize >> 4;
  chunkSize |= chunkSize >> 8;
  chunkSize |= chunkSize >> 16;
  chunkSize++;
  // Upper bound is MAX_SEND_SIZE (see buffersize)
  chunkSize = min((int)chunkSize, MAX_SEND_SIZE);
  // For UDP we only allow 512, because no nagle algorithm will split
  // the data into subframes like in the tcp case
  if (streamConfig.stream == StreamType::UDP) {
    chunkSize = min((int)chunkSize, 512);
  } else if (streamConfig.stream == StreamType::USB) {
    // For mac we only have 1020 bytes
    chunkSize = min((int)chunkSize, 16);
  }
  streamConfig.chunkSize = chunkSize;
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
  // Reset all variables
  state = SampleState::SAMPLE;
  xTaskCreatePinnedToCore(  sample_timer_task,     /* Task function. */
    "Consumer",       /* String with name of task. */
    4096,            /* Stack size in words. */
    NULL,             /* Parameter passed as input of the task */
    10,                /* Priority of the task. */
    &xHandle,            /* Task handle. */
    1);

  counter = 0;
  sentSamples = 0;
  totalSamples = 0;
  TIMER_CYCLES_FAST = (1000000) / streamConfig.samplingRate; // Cycles between HW timer inerrupts
  calcChunkSize();
  ringBuffer.reset();
  samplingDuration = 0;
  packetNumber = 0;
  // This will reset the sqwv pin
  firstSqwv = true;
  sqwCounter = 0;
  if (rtc.connected) rtc.enableInterrupt(1, sqwvTriggered);
  // If we should wait for voltage to make positive zerocrossing
  if (waitVoltage) {
    while(stpm34.readVoltage(1) > 0) {yield();}
    while(stpm34.readVoltage(1) < 0) {yield();}
  }
  freqCalcNow = millis();
  freqCalcStart = millis();
  startSamplingMillis = millis();
  turnInterrupt(true);
}

/****************************************************
 * Detach or attach sampling interupt
 ****************************************************/
void turnInterrupt(bool on) {
  cli();//stop interrupts
  if (on) {
    // timer_init();
    // The timer runs at 80 MHZ, independent of cpu clk
    timerAlarmWrite(timer, TIMER_CYCLES_FAST, true);
    timerWrite(timer, 0);
    timerAlarmEnable(timer);
  } else {
    if (timer != NULL) timerAlarmDisable(timer);
    // timer = NULL;
  }
  sei();
}

void timer_init() {
  // Timer base freq is 80Mhz
  timer = NULL;
  // Make a 1 Mhz clk here
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &sample_ISR, true);
}

/****************************************************
 * Init the MDNs name from eeprom, only the number ist
 * stored in the eeprom, construct using prefix.
 ****************************************************/
void initMDNS() {
  char * name = config.name;
  if (strlen(name) == 0) {
    logger.log(ERROR, "Sth wrong with mdns");
    strcpy(name,"powerMeterX");
  }
  // Setting up MDNs with the given Name
  logger.log("MDNS Name: %s", name);
  if (!MDNS.begin(String(name).c_str())) {             // Start the mDNS responder for esp8266.local
    logger.log(ERROR, "Setting up MDNS responder!");
  }
  MDNS.addService("_elec", "_tcp", STANDARD_TCP_STREAM_PORT);
}

void mqttCallback(char* topic, byte* message, unsigned int length) {
  memcpy(&command[0], message, length);
  command[length] = '\0';
  logger.log("MQTT msg on topic: %s: %s", topic, command);

  // Search for last topic separator
  size_t topicLen = strlen(topic);
  // On not found, this will start from the beginning of the topic string
  int lastSep = -1;
  for (size_t i = 0; i < topicLen; i++) {
    if (topic[i] == '\0') break;
    if (topic[i] == MQTT_TOPIC_SEPARATOR) lastSep = i;
  }
  char * topicEnd = &topic[lastSep+1];


  if(strcmp(topicEnd, MQTT_TOPIC_SWITCH) == 0) {
    if(strcmp(command, MQTT_TOPIC_SWITCH_OFF) == 0) {
      relay.set(false);
    } else if(strcmp(command, MQTT_TOPIC_SWITCH_ON) == 0) {
      relay.set(true);
    }
  }
  else if(strcmp(topicEnd, MQTT_TOPIC_CMD) == 0) {
    // message was already copied to command array
    parseCommand();
    handleJSON();

    if (docSend.isNull() == false) {
      response = "";
      serializeJson(docSend, response);
      // This might be too long for the logger
      logger.log(response.c_str());
      mqttClient.publish(mqttTopicPubInfo, response.c_str());
    }
    response = "";
    command[0] = '\0';
  }
    
  else if(strcmp(topicEnd, MQTT_TOPIC_SAMPLE) == 0) {
    logger.log("MQTT wants sample");
    float value = -1.0;
    char unit[4] = {'\0'};
    if(strcmp(command, "v") == 0) {
      value = stpm34.readFundamentalVoltage(1);
      sprintf(unit, "V");
    }
    else if(strcmp(command, "i") == 0) {
      value = stpm34.readRMSCurrent(1);
      sprintf(unit, "mA");
    }
    else if(strcmp(command, "q") == 0) {
      value = stpm34.readReactivePower(1);
      sprintf(unit, "var");
    }
    else if(strcmp(command, "s") == 0) {
      value = stpm34.readApparentRMSPower(1);
      sprintf(unit, "VA");
    // default is active power
    } else {
      value = stpm34.readActivePower(1);
      sprintf(unit, "W");
    }
    JsonObject obj = docSend.to<JsonObject>();
    obj.clear();
    docSend["value"] = value;
    docSend["unit"] = unit;
    docSend["ts"] = myTime.timeStr();
    response = "";
    serializeJson(docSend, response);
    logger.log(response.c_str());
    mqttClient.publish(mqttTopicPubSample, response.c_str());
  }
}


/****************************************************
 * Init the MDNs name from eeprom, only the number ist
 * stored in the eeprom, construct using prefix.
 ****************************************************/
void initMQTT() {
  mqttConnected = false;
  char * serverAddress = config.mqttServer;
  if (strlen(serverAddress) == 0) {
    logger.log(ERROR, "Sth wrong with mqtt Server");
    return;
  }
  // Setting up MDNs with the given Name
  logger.log("Try to connect to MQTT Server: %s", serverAddress);

  // if we changed server and were connected
  if (mqttClient.connected()) mqttClient.disconnect();

  // Set server
  mqttClient.setServer(serverAddress, 1883);
  mqttClient.setCallback(mqttCallback);
  
  // Look if connection is successfull and return if not
  if (!mqttClient.connect(config.name)) {
    logger.log("Cannot connect to mqtt");
    return;
  } 

  // Subscribe to all the topics
  mqttConnected = true;

  // Build publish topics
  sprintf(&mqttTopicPubSwitch[0], "%s%c", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR);

  response = "";
  response += mqttTopicPubSwitch;
  response += "+";
  logger.log("Subscribing to: %s", response.c_str());
  mqttClient.subscribe(response.c_str());
  mqttClient.loop();

  // // Subscribe topics
  // response = "";
  // response += mqttTopicPubSwitch;
  // response += MQTT_TOPIC_SWITCH;
  // mqttClient.subscribe(response.c_str());
  // logger.log("Subscribing to: %s", response.c_str());
  // // Subscribe to sample
  // response = "";
  // response += mqttTopicPubSwitch;
  // response += MQTT_TOPIC_SAMPLE;
  // mqttClient.subscribe(response.c_str());
  // logger.log("Subscribing to: %s", response.c_str());

  sprintf(&mqttTopicPubSwitch[0], "%s%c%s%c", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR, config.name, MQTT_TOPIC_SEPARATOR);

  response = "";
  response += mqttTopicPubSwitch;
  response += "+";
  logger.log("Subscribing to: %s", response.c_str());
  mqttClient.subscribe(response.c_str());
  mqttClient.loop();
  // // Subscribe topics
  // response = "";
  // response += mqttTopicPubSwitch;
  // response += MQTT_TOPIC_SWITCH;
  // mqttClient.subscribe(response.c_str());
  // logger.log("Subscribing to: %s", response.c_str());
  // // Subscribe to sample
  // response = "";
  // response += mqttTopicPubSwitch;
  // response += MQTT_TOPIC_SAMPLE;
  // mqttClient.subscribe(response.c_str());
  // logger.log("Subscribing to: %s", response.c_str());
  
  sprintf(&mqttTopicPubSwitch[0], "%s%c%s%c%s%c%s", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR, config.name, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_STATE, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_SWITCH);
  sprintf(&mqttTopicPubSample[0], "%s%c%s%c%s%c%s", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR, config.name, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_STATE, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_SAMPLE);
  sprintf(&mqttTopicPubInfo[0], "%s%c%s%c%s%c%s", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR, config.name, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_STATE, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_INFO);
  logger.log("MQTT connected"); 
}


// Make sure enough memory is allocated for str
void setInfoString(char * str) {
  size_t idx = 0;
  idx += sprintf(&str[idx], "\n");
  // Name and firmware
  idx += sprintf(&str[idx], "%s @ firmware: %s/%s", config.name, __DATE__, __TIME__);
  idx += sprintf(&str[idx], "\nUsing %d bytes ", ringBuffer.getSize());
  // PSRAM or not
  if (ringBuffer.inPSRAM()) {
    idx += sprintf(&str[idx], "PSRAM");
  } else {
    idx += sprintf(&str[idx], "RAM");
  }
  if (rtc.connected) {
    idx += sprintf(&str[idx], "\nRTC is connected ");
    if (rtc.lost) {
      idx += sprintf(&str[idx], "but has lost time");
    } else {
      idx += sprintf(&str[idx], "and has valid time");
    }
  } else {
    idx += sprintf(&str[idx], "\nNo RTC");
  }
  idx += sprintf(&str[idx], "\nCurrent system time: ToImplement");
  // All SSIDs
  String ssids = "";
  for (size_t i = 0; i < config.numAPs; i++) {
    ssids += config.wifiSSIDs[i];
    if (i < config.numAPs-1) ssids += ", ";
  }
  idx += sprintf(&str[idx], "\nKnown Networks: [%s]", ssids.c_str());
  idx += sprintf(&str[idx], "\n");
}

void relayCB(bool value) {
  if (state != SampleState::SAMPLE) { 
    // Only if we are not sampling, we can use SPI/EEEPROM
    config.setRelayState(value); 
  }
  if (mqttClient.connected()) mqttClient.publish(mqttTopicPubSwitch, value ? MQTT_TOPIC_SWITCH_ON : MQTT_TOPIC_SWITCH_OFF);
  logger.log("Switched %s", value ? "on" : "off");
}

void sampleCB() {
  if (mqttClient.connected()) mqttClient.publish(mqttTopicPubSample, state==SampleState::SAMPLE ? MQTT_TOPIC_SWITCH_ON : MQTT_TOPIC_SWITCH_OFF);
}