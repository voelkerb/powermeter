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
#include "src/mqtt/mqtt.h"
// #include "parseCommands.ino"



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

// Define it before StreamType enum redefines it
MQTT mqtt;

// Available measures are VOLTAGE+CURRENT, ACTIVE+REACTIVE Power or both
enum class Measures{VI, PQ, VIPQ, VI_RMS};
enum class StreamType{USB, TCP, UDP, MQTT};

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
  char unit[20] = {'\0'};                   // Units of current values
  size_t numValues = 24/sizeof(float);      // Number of values
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
// tcp stuff is send over this client 
Stream * sendStream = (Stream*)&Serial;
WiFiClient * sendClient;
// UDP used for streaming
WiFiUDP udpClient;

// Stream client for external stream server
WiFiClient exStreamServer;

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

// Command stuff send over what ever
char command[COMMAND_MAX_SIZE] = {'\0'};
StaticJsonDocument<2*COMMAND_MAX_SIZE> docRcv;
StaticJsonDocument<2*COMMAND_MAX_SIZE> docSend;
StaticJsonDocument<COMMAND_MAX_SIZE> docSample;
String response = "";


char mqttTopicPubSwitch[MAX_MQTT_PUB_TOPIC_SWITCH+MAX_NAME_LEN] = {'\0'};
char mqttTopicPubSample[MAX_MQTT_PUB_TOPIC_SAMPLE+MAX_NAME_LEN] = {'\0'};
char mqttTopicPubInfo[MAX_MQTT_PUB_TOPIC_INFO+MAX_NAME_LEN] = {'\0'};

void (*outputWriter)(bool) = &writeChunks;

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

  // Set mqtt and callbacks
  mqtt.init(config.mqttServer, config.name);
  mqtt.onConnect = &onMQTTConnect;
  mqtt.onDisconnect = &onMQTTDisconnect;
  mqtt.onMessage = &mqttCallback;

  setInfoString(&command[0]);
  logger.log(&command[0]);

  // Init send buffer
  sprintf((char *)&sendbuffer[0], "%s", DATA_PREFIX);

  logger.log(ALL, "Setup done");
}

/************************ Loop *************************/
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

void onIdleOrSampling() {

  // MQTT loop
  mqtt.update();

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

/****************************************************
 * Things todo regularly if we are not sampling
 ****************************************************/
void onIdle() {
  spiffsLog.flush();
  // Arduino OTA
  ArduinoOTA.handle();

  // Re-advertise MDNS service service every 30s 
  // TODO: no clue why, but does not work properly for esp32 (maybe it is the mac side)
  if ((long)(millis() - mdnsUpdate) >= 0) {
    mdnsUpdate += MDNS_UPDATE_INTERVAL;
    // On long time no update, avoid multiupdate
    if ((long)(millis() - mdnsUpdate) >= 0) mdnsUpdate = millis() + MDNS_UPDATE_INTERVAL; 
    // initMDNS();
    //MDNS.addService("_elec", "_tcp", STANDARD_TCP_STREAM_PORT);
  }
  

  if ((long)(millis() - rtcUpdate) >= 0) {
    rtcUpdate += RTC_UPDATE_INTERVAL;
    // On long time no update, avoid multiupdate
    if ((long)(millis() - rtcUpdate) >= 0) rtcUpdate = millis() + RTC_UPDATE_INTERVAL; 
    if (rtc.connected) rtc.update();
  }
    
  // Update lifeness only on idle every second
  if ((long)(millis() - lifenessUpdate) >= 0) {
    lifenessUpdate += LIFENESS_UPDATE_INTERVAL;
    // On long time no update, avoid multiupdate
    if ((long)(millis() - lifenessUpdate) >= 0) lifenessUpdate = millis() + LIFENESS_UPDATE_INTERVAL; 
    logger.log("");
  }

  if (exStreamServer.connected()) {
    // Set everything to default settings
    standardConfig();
    streamConfig.port = STANDARD_TCP_SAMPLE_PORT;
    sendClient = (WiFiClient*)&exStreamServer;
    sendStream = sendClient;
    startSampling();
  }
  // Look for people connecting over the stream server
  // If one connected there, immidiately start streaming data
  if (!streamClient.connected()) {
    streamClient = streamServer.available();
    if (streamClient.connected()) {
      standardConfig();
      streamConfig.prefix = false;
      streamConfig.port = STANDARD_TCP_STREAM_PORT;
      sendClient = (WiFiClient*)&streamClient;
      sendStream = sendClient;
      startSampling();
    }
  }
}


/****************************************************
 * Set standard configuration
 ****************************************************/
void standardConfig() {
  streamConfig.stream = StreamType::TCP;
  streamConfig.prefix = true;
  streamConfig.samplingRate = DEFAULT_SR;
  streamConfig.measures = Measures::VI;
  streamConfig.ip = streamClient.remoteIP();
  streamConfig.port = STANDARD_TCP_SAMPLE_PORT;
  outputWriter = &writeChunks;
  streamConfig.measurementBytes = 8;
  sprintf(streamConfig.unit, "V,mA");
  streamConfig.numValues = streamConfig.measurementBytes/sizeof(float);
}


/****************************************************
 * What todo only on sampling (send data, etc)
 ****************************************************/
void onSampling() {
  // ______________ Send data to sink ________________
  outputWriter(false);

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
  } else if (streamConfig.stream == StreamType::MQTT) {
    if (!mqtt.connected) {
      logger.log(ERROR, "MQTT Server disconnected while streaming");
      stopSampling();
    }
  }
}

/****************************************************
 * Time getter function can be called at any time
 * Note: static function required for time getter 
 * function. e.g. for logger class
 ****************************************************/
char * timeStr() {
  return myTime.timeStr(true);;
}

/****************************************************
 * If MQTT Server connection was successfull
 ****************************************************/
void onMQTTConnect() {
  logger.log("MQTT connected to %s", mqtt.ip);
  mqttSubscribe();
}

/****************************************************
 * If MQTT Server disconnected
 ****************************************************/
void onMQTTDisconnect() {
  logger.log("MQTT disconnected from %s", mqtt.ip);
}

/****************************************************
 * If ESP is connected to wifi successfully
 ****************************************************/
void onWifiConnect() {
  if (not Network::apMode) {
    logger.log(ALL, "Wifi Connected");
    logger.log(ALL, "IP: %s", WiFi.localIP().toString().c_str());
    // The stuff todo if we have a network connection (and hopefully internet as well)
    myTime.updateNTPTime();
    if (!mqtt.connect()) logger.log(ERROR, "Cannot connect to MQTT Server %s", mqtt.ip);
  } else {
    logger.log(ALL, "Network AP Opened");
  }

  // Reinit mdns
  initMDNS();

  // Start the TCP server
  server.begin();
  streamServer.begin();

  initStreamServer();
}

/****************************************************
 * If ESP disconnected from wifi
 ****************************************************/
void onWifiDisconnect() {
  logger.log(ERROR, "Wifi Disconnected");

  if (state != SampleState::IDLE) {
    logger.log(ERROR, "Stop sampling (Wifi disconnect)");
    stopSampling();
  }
  if (mqtt.connected) mqtt.disconnect();
}

/****************************************************
 * If a tcp client connects.
 * We store them in list and add logger
 ****************************************************/
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

/****************************************************
 * If a tcp client disconnects.
 * We must remove the logger
 ****************************************************/
void onClientDisconnect(WiFiClient &oldClient, size_t i) {
  logger.log("Client discconnected %s port %u", oldClient.remoteIP().toString().c_str(), oldClient.remotePort());
  logger.removeLogger(streamLog[i]);
  streamLog[i]->_stream = NULL;
}


/****************************************************
 * Write chunks of data for TCP and Serial
 * depending on data sink
 ****************************************************/
void writeChunks(bool tail) {
  while(ringBuffer.available() > streamConfig.chunkSize) {
    writeData(*sendStream, streamConfig.chunkSize);
  }
  if (tail) {
    writeData(*sendStream, ringBuffer.available());
  }
}

/****************************************************
 * Write data via UDP, this requires special packet
 * handling
 ****************************************************/
void writeChunksUDP(bool tail) {
  while(ringBuffer.available() > streamConfig.chunkSize) {
    udpClient.beginPacket(streamConfig.ip, streamConfig.port);
    writeData(udpClient, streamConfig.chunkSize);
    udpClient.endPacket();
  }
  if (tail) {
    udpClient.beginPacket(streamConfig.ip, streamConfig.port);
    writeData(udpClient, ringBuffer.available());
    udpClient.endPacket();
  }
}

/****************************************************
 * Write data to mqtt sink, this requires special
 * formatiing of the data
 ****************************************************/
void writeDataMQTT(bool tail) {
  if(ringBuffer.available() < streamConfig.measurementBytes) return;


  while(ringBuffer.available() >= streamConfig.measurementBytes) {
    ringBuffer.read((uint8_t*)&values[0], streamConfig.measurementBytes);
    for (int i = 0; i < streamConfig.numValues; i++) docSample["values"][i] = values[i];
    // JsonArray array = docSend["values"].to<JsonArray>();
    // for (int i = 0; i < streamConfig.numValues; i++) array.add(value[i]);
    // docSend["unit"] = streamConfig.unit;
    docSample["ts"] = myTime.timeStr();
    response = "";
    serializeJson(docSample, response);
    // logger.log(response.c_str());
    mqtt.publish(mqttTopicPubInfo, response.c_str());
    sentSamples++;
  }
}


/****************************************************
 * Write one chunk of data to sink.
 ****************************************************/
void writeData(Stream &getter, uint16_t size) {
  if (size <= 0) return;
  uint32_t start = 0;
  if (streamConfig.prefix) {
    memcpy(&sendbuffer[start], (void*)&DATA_PREFIX[0], PREFIX_SIZE);
    start += PREFIX_SIZE;
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

void test(bool tail) {
  if (ringBuffer.available() > streamConfig.samplingRate*streamConfig.measurementBytes) {
    ringBuffer.reset();
    packetNumber++;
    size_t test = sprintf((char*)&sendbuffer[0], "Test: %s - %u\r\n", timeStr(), packetNumber);
    sendStream->write((uint8_t*)&sendbuffer[0], test);
  }
}

/****************************************************
 * Sampling interrupt, must be small and quick
 ****************************************************/
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

/****************************************************
 * RTOS task for sampling
 ****************************************************/
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
    } else if (streamConfig.measures == Measures::VI_RMS) {
      stpm34.readRMSVoltageAndCurrent(1, (float*) &data[0], (float*) &data[1]);
    }

    ringBuffer.write((uint8_t*)&data[0], streamConfig.measurementBytes);

  }
  vTaskDelete( NULL );
}

/****************************************************
 * A SQWV signal from the RTC is generated, we
 * use this signal to handle second tasks and
 * on sampling make sure that we achieve
 * the appropriate <samplingrate> samples each second
 ****************************************************/
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

/****************************************************
 * Setup the OTA updates progress
 ****************************************************/
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
    Network::allowNetworkChange = false;
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
  // If realy has toggled store its new value
  config.store();
  sampleCB();
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

/****************************************************
 * Overload of start sampling to wait for voltage
 * to reach zero crossing
 ****************************************************/
void startSampling(bool waitVoltage) {
  // Reset all variables
  state = SampleState::SAMPLE;
  sampleCB();
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

/****************************************************
 * Init the timer of the esp32
 ****************************************************/
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

/****************************************************
 * Check streamserver connection, sicne connect is 
 * a blocking task, we make it nonblocking using 
 * a separate freerots task
 ****************************************************/
TaskHandle_t streamServerTaskHandle = NULL;
void checkStreamServer(void * pvParameters) {
  while (not updating) {
    if (strcmp(config.streamServer, NO_SERVER) != 0 and !exStreamServer.connected()) {
      exStreamServer.connect(config.streamServer, STANDARD_TCP_SAMPLE_PORT);
    }
    vTaskDelay(STREAM_SERVER_UPDATE_INTERVAL);
  }
  vTaskDelete(streamServerTaskHandle); // destroy this task 
}

/****************************************************
 * Init an external server to which data is streamed 
 * automatically if it is there. Enable checker 
 ****************************************************/
void initStreamServer() {
  if (strcmp(config.streamServer, NO_SERVER) != 0 and !exStreamServer.connected()) {
    logger.log("Try to connect Stream Server: %s", config.streamServer);
    exStreamServer.connect(config.streamServer, STANDARD_TCP_SAMPLE_PORT);
    // Handle reconnects
    if (streamServerTaskHandle == NULL) {
      xTaskCreate(
            checkStreamServer,   /* Function to implement the task */
            "streamServerTask", /* Name of the task */
            10000,      /* Stack size in words */
            NULL,       /* Task input parameter */
            1,          /* Priority of the task */
            &streamServerTaskHandle);  /* Task handle */
    }
  }
}

/****************************************************
 * Subscribe to the mqtt topics we want to listen
 * and build the publish topics
 ****************************************************/
void mqttSubscribe() {
  if (!mqtt.connected) {
    logger.log(ERROR, "Sth wrong with mqtt Server");
    return;
  }

  // Build publish topics
  sprintf(&mqttTopicPubSwitch[0], "%s%c", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR);

  response = mqttTopicPubSwitch;
  response += "+";
  logger.log("Subscribing to: %s", response.c_str());
  mqtt.subscribe(response.c_str());
  
  sprintf(&mqttTopicPubSwitch[0], "%s%c%s%c", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR, config.name, MQTT_TOPIC_SEPARATOR);

  response = mqttTopicPubSwitch;
  response += "+";
  logger.log("Subscribing to: %s", response.c_str());
  mqtt.subscribe(response.c_str());
  
  sprintf(&mqttTopicPubSwitch[0], "%s%c%s%c%s%c%s", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR, config.name, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_STATE, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_SWITCH);
  sprintf(&mqttTopicPubSample[0], "%s%c%s%c%s%c%s", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR, config.name, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_STATE, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_SAMPLE);
  sprintf(&mqttTopicPubInfo[0], "%s%c%s%c%s%c%s", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR, config.name, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_STATE, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_INFO);
}

/****************************************************
 * Nice formatted info str with all available infos 
 * of this device
 * NOTE: Make sure enough memory is allocated for str
 ****************************************************/
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

/****************************************************
 * Callback if relay is switched
 ****************************************************/
void relayCB(bool value) {
  if (state != SampleState::SAMPLE) { 
    // Only if we are not sampling, we can use SPI/EEEPROM
    config.setRelayState(value); 
  }
  if (mqtt.connected) mqtt.publish(mqttTopicPubSwitch, value ? MQTT_TOPIC_SWITCH_ON : MQTT_TOPIC_SWITCH_OFF);
  logger.log("Switched %s", value ? "on" : "off");
}

/****************************************************
 * Callback if sampling will be perfomed
 ****************************************************/
void sampleCB() {
  if (mqtt.connected) mqtt.publish(mqttTopicPubSample, state==SampleState::SAMPLE ? MQTT_TOPIC_SWITCH_ON : MQTT_TOPIC_SWITCH_OFF);
  // Do not allow network changes on sampling
  Network::allowNetworkChange = state==SampleState::SAMPLE ? true : false;
}