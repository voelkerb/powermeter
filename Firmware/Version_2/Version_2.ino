#include <Arduino.h>
#include <ArduinoOTA.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <time.h>
#include <math.h>
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
#include "enums.h"
#include "src/multiLogger/src/multiLogger.h"
#include "src/DS3231_RTC/src/DS3231_RTC.h"
#include "src/time/src/timeHandling.h"
#include "src/stpm3x/src/STPM3X.h"
#include "src/relay/src/relay.h"
#include "src/network/src/network.h"
#include "src/config/config.h"
#include "src/ringbuffer/src/ringbuffer.h"
#include "src/mqtt/src/mqtt.h"

// Function prototypes required for e.g. platformio
void IRAM_ATTR sample_ISR();
void IRAM_ATTR sqwvTriggered(void* instance);
void ntpSynced(unsigned int confidence);


Relay relay(RELAY_PIN_S, RELAY_PIN_R);

TaskHandle_t xHandle = NULL;

#ifdef USE_SERIAL
  // Serial logger
  StreamLogger serialLog((Stream*)&Serial, &timeStr, &LOG_PREFIX_SERIAL[0], ALL);
#endif


// SPIFFS logger
SPIFFSLogger spiffsLog(false, &LOG_FILE[0], &timeStr, &LOG_PREFIX_SERIAL[0], WARNING);

// MultiLogger logger(&streamLog, &timeStr);
// Create singleton here
MultiLogger& logger = MultiLogger::getInstance();

Configuration config;

Rtc rtc(RTC_INT);
TimeHandler myTime(config.timeServer, LOCATION_TIME_OFFSET, &rtc, &ntpSynced);

// STPM Object
STPM stpm34(STPM_RES, STPM_CS, STPM_SYN);

// counter holds # isr calls
volatile uint16_t counter = 0;

RingBuffer ringBuffer(PS_BUF_SIZE, true);

#define SEND_BUF_SIZE MAX_SEND_SIZE+16
static uint8_t sendbuffer[SEND_BUF_SIZE] = {0};
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


SampleState state = SampleState::IDLE;
SampleState next_state = SampleState::IDLE;

// Define it before StreamType enum redefines it
MQTT mqtt;

struct StreamConfig {
  bool prefix = false;                      // Send data with "data:" prefix
  bool flowControl = false;                 // ifg flow ctr is used
  Measures measures = Measures::VI;         // The measures to send
  uint8_t measurementBytes = 8;             // Number of bytes for each measurement entry
  unsigned int samplingRate = DEFAULT_SR;   // The samplingrate
  StreamType stream = StreamType::USB;      // Channel over which to send
  unsigned int countdown = 0;               // Start at specific time or immidiately
  uint32_t chunkSize = 0;                   // Chunksize of one packet sent
  IPAddress ip;                             // Ip address of data sink
  uint16_t port = STANDARD_TCP_SAMPLE_PORT; // Port of data sink
  size_t numValues = 24/sizeof(float);      // Number of values
  Timestamp startTs;
  Timestamp stopTs;
  int slots = 0;
  int slot = 0;
};
bool rts; // ready to send

int correctSamples = 0;

char measureStr[MAX_MEASURE_STR_LENGTH] = {'\0'};
char unitStr[MAX_UNIT_STR_LENGTH] = {'\0'};

StreamConfig streamConfig;


// Stuff for frequency calculation
volatile long freqCalcStart;
volatile long freqCalcNow;
volatile long freq = 0;

// TCP clients and current connected state, each client is assigned a logger
WiFiClient client[MAX_CLIENTS];
bool clientConnected[MAX_CLIENTS] = {false};
int clientConnectionLossCnt[MAX_CLIENTS] = {0};
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
bool exStreamServerNewConnection = true;

// Information about the current sampling period
unsigned long samplingCountdown = 0;
unsigned long startSamplingMillis = 0;
unsigned long samplingDuration = 0;
unsigned long sentSamples = 0;
volatile unsigned long packetNumber = 0;
volatile unsigned long totalSamples = 0;

// Some timer stuff s.t. things are updated regularly and not at full speed
uint32_t lifenessUpdate = millis();
uint32_t mdnsUpdate = millis();
uint32_t tcpUpdate = millis();
uint32_t rtcUpdate = millis();
uint32_t mqttUpdate = millis();

// HW Timer and mutex for sapmpling ISR
hw_timer_t * timer = NULL;
portMUX_TYPE counterMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE correctionMux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t stpm_mutex;
// Current CPU speed
unsigned int coreFreq = 0;

// test stuff
uint32_t testMillis = 0;
uint32_t testMillis2 = 0;
uint16_t testSamples = 0;

// Mutex for 1s RTC Interrupt
portMUX_TYPE sqwMux = portMUX_INITIALIZER_UNLOCKED;
bool firstSqwv = true;
volatile int sqwCounter = 0;
volatile int sqwCounter2 = 0;

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

char mqttTopicPubSwitch[MAX_MQTT_TOPIC_LEN] = {'\0'};
char mqttTopicPubSample[MAX_MQTT_TOPIC_LEN] = {'\0'};
char mqttTopicPubInfo[MAX_MQTT_TOPIC_LEN] = {'\0'};

void (*outputWriter)(bool) = &writeChunks;

/************************ SETUP *************************/
void setup() {
  // Set realy to true
  // bistable relay is left as is
  // relay.set(true);

  // Load config, an set relay to wished state
  config.init();  
  config.load();
  relay.set(config.getRelayState());

  #ifdef USE_SERIAL
    // Setup serial communication
    #ifdef CMD_OVER_SERIAL
      Serial.begin(SERIAL_SPEED);
    #else
      // Disable receiving, as this also disbles an interrupt which might occur
      // multiple times a second due to emi signals on the boards (due to bad shielding)
      // Serial.begin(SERIAL_SPEED, SERIAL_8N1, SERIAL_TX_ONLY, 1);
      Serial.begin(SERIAL_SPEED, SERIAL_8N1, -1, 1, false); // FTDI 12Mbaud
    #endif
  #endif

  // At first init the rtc module to get a time behavior
  bool successAll = true;
  bool success = rtc.init();
  // Init the logging modules
  logger.setTimeGetter(&timeStr);
  #ifdef USE_SERIAL
  // Add Serial logger
  logger.addLogger(&serialLog);
  #endif
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


  coreFreq = getCpuFrequencyMhz();
  logger.log(DEBUG, "%s @ firmware %s/%s", config.netConf.name, __DATE__, __TIME__);
  logger.log(DEBUG, "Core @ %u MHz", coreFreq);

  // Init the ringbuffer
  success = ringBuffer.init();
  if (!success) logger.log(ERROR, "PSRAM init failed");
  successAll &= success;

  // Init the timer used for sampling
  timer_init();

  // generate mutex to use the stpm32/34 SPI interface
  stpm_mutex = xSemaphoreCreateMutex();
  // Setup STPM 32
  success = stpm34.init();
  stpm34.setCalibration(config.calV, config.calI);
  if (!success) logger.log(ERROR, "STPM Init Failed");
  successAll &= success;
  
   // Indicate error if there is any
  digitalWrite(ERROR_LED, !successAll);

  logger.log(ALL, "Connecting WLAN");

  // Init network connection
  Network::init(&config.netConf, onWifiConnect, onWifiDisconnect, false, &logger);
  // Setup OTA updating
  setupOTA();

  // Resever enough bytes for large string
  response.reserve(2*COMMAND_MAX_SIZE);

  // Set mqtt and callbacks
  mqtt.init(config.mqttServer, config.netConf.name);
  mqtt.onConnect = &onMQTTConnect;
  mqtt.onDisconnect = &onMQTTDisconnect;
  mqtt.onMessage = &mqttCallback;


  // Init send buffer
  snprintf((char *)&sendbuffer[0], SEND_BUF_SIZE, "%s", DATA_PREFIX);

  // print some info about this powermeter
  printInfoString();
  logger.log(ALL, "Setup done");
}

/************************ Loop *************************/
void loop() {
  // Arduino OTA
  ArduinoOTA.handle();
  // We don't do anything else while updating
  if (updating) return;

  // Stuff done on idle
  if (state == SampleState::IDLE) {
    onIdle();
  // Stuff on sampling
  } else if (state == SampleState::SAMPLE) {
    onSampling();
  }
  // Stuff on both
  onIdleOrSampling();

  // If we only have 200 ms before sampling should start, wait actively
  if (next_state != SampleState::IDLE) {
    if (streamConfig.countdown != 0 and (streamConfig.countdown - millis()) < 200) {
      // Disable any wifi sleep mode again
      if (not Network::ethernet) esp_wifi_set_ps(WIFI_PS_NONE);
      state = next_state;
      logger.log(DEBUG, "StartSampling @ around %s", myTime.timeStr());
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
  #ifdef CMD_OVER_SERIAL
  if (Serial.available()) {
    handleEvent(&Serial);
  }
  #endif

  // Handle tcp requests
  for (size_t i = 0; i < MAX_CLIENTS; i++) {
    if (client[i].available() > 0) {
      handleEvent(&client[i]);
    }
  }

  // Handle tcp clients connections
  if (millis() - tcpUpdate > TCP_UPDATE_INTERVAL) {
    tcpUpdate = millis();
  // if ((long)(millis() - tcpUpdate) >= 0) {
    // tcpUpdate += TCP_UPDATE_INTERVAL;
    // Handle disconnect
    for (size_t i = 0; i < MAX_CLIENTS; i++) {
      if (clientConnected[i]) {
        if (!client[i].connected()) {
          logger.log(WARNING, "client %s not connected for %is", client[i].remoteIP().toString().c_str(), clientConnectionLossCnt[i]);
          // client[i].connected() may fire even if connected, therefore we use this dirty hack
          // Only if multiple times disconnect seen, actually disconnect it.
          // NOTE: What happens on client write in this case?
          if (clientConnectionLossCnt[i] >= MAX_CNT_BEFORE_DISCONNECT) {
            onClientDisconnect(client[i], i);
            clientConnected[i] = false;
          }
          clientConnectionLossCnt[i]++;
        } else {
          clientConnectionLossCnt[i] = 0;
        }
      }
    }

    // Handle connects
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
  // If we logged during sampling, we flush now
  spiffsLog.flush();

  // Update stuff over mqtt
  if (mqtt.connected()) {
    if (millis() - mqttUpdate > MQTT_UPDATE_INTERVAL) {
      mqttUpdate = millis();
    // if ((long)(millis() - mqttUpdate) >= 0) {
    //   mqttUpdate += MQTT_UPDATE_INTERVAL;
    //   // On long time no update, avoid multiupdate
    //   if ((long)(millis() - mqttUpdate) >= 0) mqttUpdate = millis() + MQTT_UPDATE_INTERVAL; 
      sendStatusMQTT();
    }
  }

  // Re-advertise MDNS service service every 30s 
  // TODO: no clue why, but does not work properly for esp32 (maybe it is the mac side)
  // if ((long)(millis() - mdnsUpdate) >= 0) {
  //   mdnsUpdate += MDNS_UPDATE_INTERVAL;
  //   // On long time no update, avoid multiupdate
  //   if ((long)(millis() - mdnsUpdate) >= 0) mdnsUpdate = millis() + MDNS_UPDATE_INTERVAL; 
  //     // initMDNS();
  //     // Not required, only for esp8266
  //   //MDNS.addService("_elec", "_tcp", STANDARD_TCP_STREAM_PORT);
  // }
  
  // update RTC regularly
  if (millis() - rtcUpdate > RTC_UPDATE_INTERVAL) {
    rtcUpdate = millis();
  // if ((long)(millis() - rtcUpdate) >= 0) {
  //   rtcUpdate += RTC_UPDATE_INTERVAL;
  //   // On long time no update, avoid multiupdate
  //   if ((long)(millis() - rtcUpdate) >= 0) rtcUpdate = millis() + RTC_UPDATE_INTERVAL; 
    if (rtc.connected) rtc.update();
  }
    
  // Update lifeness only on idle every second
  if (millis() - lifenessUpdate > LIFENESS_UPDATE_INTERVAL) {
    lifenessUpdate = millis();
  // if ((long)(millis() - lifenessUpdate) >= 0) {
  //   lifenessUpdate += LIFENESS_UPDATE_INTERVAL;
  //   // On long time no update, avoid multiupdate
  //   if ((long)(millis() - lifenessUpdate) >= 0) lifenessUpdate = millis() + LIFENESS_UPDATE_INTERVAL; 
    logger.log("");
  }
  
  /*
   * NOTE: The streamserver has to send start signal by itself now,
   * Lets see if this is a better idea
   * If you want to reenable automatic sending, uncomment it
   * exStreamServer is now more an external server where we announce that we are now ready
   * to send data
   */
  // Check external stream server connection
  // if (exStreamServer.connected() and exStreamServerNewConnection) {
  //   logger.log("Stream Server connected start sampling");
  //   // If streamserver sends stop, we need a way to prevent a new start of sampling
  //   // This is how we achieve it.
  //   exStreamServerNewConnection = false;
  //   // Set everything to default settings
  //   standardConfig();
  //   streamConfig.port = STANDARD_TCP_STREAM_PORT;
  //   sendClient = (WiFiClient*)&exStreamServer;
  //   sendStream = sendClient;
  //   startSampling();
  //   // Must be done after start st startTS is in data
  //   sendStreamInfo(sendClient);
  // }

  // Look for people connecting over the streaxm server
  // If one connected there, immidiately start streaming data
  if (!streamClient.connected()) {
    streamClient = streamServer.available();
    if (streamClient.connected()) {
      logger.log("Stream Client connected start sampling");
      standardConfig();
      // No prefix, just raw data
      streamConfig.prefix = false;
      streamConfig.port = STANDARD_TCP_STREAM_PORT;
      sendClient = (WiFiClient*)&streamClient;
      sendStream = sendClient;
      startSampling();
    }
  }
}

/****************************************************
 * Send Status info over mqtt 
 ****************************************************/
void sendStatusMQTT() {
  JsonObject obj = docSend.to<JsonObject>();
  obj.clear();
  float value = 0.0;
  // Lock stpm before using it
  xSemaphoreTake(stpm_mutex, portMAX_DELAY);

  if (!relay.state) {
    docSend["power"] = 0.0;
    docSend["current"] = 0.0;
    docSend["inUse"] =  false;
  } else {
    value = round2<float>(stpm34.readActivePower(1));
    if (value < 0) value = 0.0;
    docSend["power"] = value;
    docSend["inUse"] = value > 2.0 ? true : false;
    // We want current in A
    value = round2<float>(stpm34.readRMSCurrent(1)/1000.0);
    docSend["current"] = value;
  }
  // unit is watt hours and we want kwh
  value = round2<float>(stpm34.readActiveEnergy(1)/1000.0);
  docSend["energy"] = value;
  // TODO: Something is still wrong with voltage calculation
  value = round2<float>(stpm34.readRMSVoltage(1)+230.0);

  // Unlock stpm 
  xSemaphoreGive(stpm_mutex);

  docSend["volt"] = value;
  docSend["ts"] = myTime.timestampStr(true);
  response = "";
  serializeJson(docSend, response);
  // logger.log("MQTT msg: %s", response.c_str());
  mqtt.publish(mqttTopicPubSample, response.c_str());
}


/****************************************************
 * Send Stream info to 
 ****************************************************/
void sendDeviceInfo(Stream * sender) {
  JsonObject obj = docSend.to<JsonObject>();
  obj.clear();
  docSend["cmd"] = "info";
  docSend["type"] = F("powermeter");
  docSend["version"] = VERSION;
  String compiled = __DATE__;
  compiled += " ";
  compiled += __TIME__;
  docSend["compiled"] = compiled;
  docSend["sys_time"] = myTime.timeStr();
  docSend["name"] = config.netConf.name;
  docSend["ip"] = Network::localIP().toString();
  docSend["mqtt_server"] = config.mqttServer;
  docSend["stream_server"] = config.streamServer;
  docSend["time_server"] = config.timeServer;
  docSend["sampling_rate"] = streamConfig.samplingRate;
  docSend["buffer_size"] = ringBuffer.getSize();
  docSend["psram"] = ringBuffer.inPSRAM();
  docSend["rtc"] = rtc.connected;
  docSend["calV"] = config.calV;
  docSend["calI"] = config.calI;
  docSend["state"] = state != SampleState::IDLE ? "busy" : "idle";
  docSend["relay"] = relay.state;
  JsonArray array = docSend.createNestedArray("calibration");
  array.add(config.calV);
  array.add(config.calI);

  String ssids = "[";
  for (size_t i = 0; i < config.netConf.numAPs; i++) {
    ssids += config.netConf.SSIDs[i];
    if (i < config.netConf.numAPs-1) ssids += ", ";
  }
  ssids += "]";
  docSend["ssids"] = ssids;
  if (not Network::ethernet) {
    docSend["ssid"] = WiFi.SSID();
    docSend["rssi"] = WiFi.RSSI();
    docSend["bssid"] = Network::getBSSID();
  }
  response = "";
  serializeJson(docSend, response);
  response = LOG_PREFIX + response;
  sender->println(response);
}

/****************************************************
 * Send Stream info to 
 ****************************************************/
void sendStreamInfo(Stream * sender) {
  JsonObject obj = docSend.to<JsonObject>();
  obj.clear();
  docSend["name"] = config.netConf.name;
  docSend["measures"] = measuresToStr(streamConfig.measures);
  docSend["chunksize"] = streamConfig.chunkSize;
  docSend["samplingrate"] = streamConfig.samplingRate;
  docSend["measurements"] = streamConfig.numValues;
  docSend["prefix"] = streamConfig.prefix;
  docSend["cmd"] = CMD_SAMPLE;
  docSend["unit"] = unitToStr(streamConfig.measures);
  docSend["startTs"] = myTime.timestampStr(streamConfig.startTs);
  response = "";
  serializeJson(docSend, response);
  response = LOG_PREFIX + response;
  sender->println(response);
}

/****************************************************
 * return unit as str
 ****************************************************/
char * unitToStr(Measures measures) {
  switch (measures) {
    case Measures::VI:
    case Measures::VI_RMS:
      snprintf(unitStr, MAX_UNIT_STR_LENGTH, UNIT_VI);
      break;
    case Measures::PQ:
      snprintf(unitStr, MAX_UNIT_STR_LENGTH, UNIT_PQ);
      break;
    case Measures::VIPQ:
      snprintf(unitStr, MAX_UNIT_STR_LENGTH, UNIT_VIPQ);
      break;
    default:
      snprintf(unitStr, MAX_UNIT_STR_LENGTH, "unknown");
      break;
  }
  return &unitStr[0];
}

/****************************************************
 * Convert measure to str
 ****************************************************/
char * measuresToStr(Measures measures) {
  switch (measures) {
    case Measures::VI:
      snprintf(measureStr, MAX_MEASURE_STR_LENGTH, MEASURE_VI);
      break;
    case Measures::PQ:
      snprintf(measureStr, MAX_MEASURE_STR_LENGTH, MEASURE_PQ);
      break;
    case Measures::VI_RMS:
      snprintf(measureStr, MAX_MEASURE_STR_LENGTH, MEASURE_VI_RMS);
      break;
    case Measures::VIPQ:
      snprintf(measureStr, MAX_MEASURE_STR_LENGTH, MEASURE_VIPQ);
      break;
    default:
      snprintf(measureStr, MAX_MEASURE_STR_LENGTH, "unknown");
      break;
  }
  return &measureStr[0];
}


/****************************************************
 * Set standard configuration
 ****************************************************/
void standardConfig() {
  streamConfig.slot = 0;
  streamConfig.slots = 0;
  streamConfig.stream = StreamType::TCP;
  streamConfig.prefix = true;
  streamConfig.flowControl = false;
  streamConfig.samplingRate = DEFAULT_SR;
  streamConfig.measures = Measures::VI;
  streamConfig.ip = streamClient.remoteIP();
  streamConfig.port = STANDARD_TCP_SAMPLE_PORT;
  outputWriter = &writeChunks;
  streamConfig.measurementBytes = 8;
  streamConfig.numValues = streamConfig.measurementBytes/sizeof(float);
  calcChunkSize();
  rts = true;
}


uint32_t sent = 0;
/****************************************************
 * What todo only on sampling (send data, etc)
 ****************************************************/
void onSampling() {
  // TODO: Chaos aufräumen
  // Simple seconds send slot calculation
  if (streamConfig.slots != 0) {
    Timestamp now = myTime.timestamp();
    while (now.seconds%streamConfig.slots == streamConfig.slot) {
      if (ringBuffer.available() > streamConfig.chunkSize) {
        writeData(*sendStream, streamConfig.chunkSize);
        sent += streamConfig.chunkSize;
      } else {
        break;
      }
      now = myTime.timestamp();
    }
    if (now.seconds%streamConfig.slots != streamConfig.slot and sent != 0) {
      if (ringBuffer.available() > streamConfig.chunkSize) {
        sent = ringBuffer.available()/streamConfig.measurementBytes;
        logger.log(WARNING, "not finisched %u missing", sent);
      } else {
        sent = sent/streamConfig.measurementBytes;
        logger.log("sent %u samples", sent);
      }
      sent = 0;
    }
  }
  //  if (streamConfig.slots != 0) {
  //   Timestamp now = myTime.timestamp();
  //   if (now.seconds%streamConfig.slots == streamConfig.slot) {
  //     rts = true;
  //   } else {
  //     rts = false;
  //   }
  // }

  // Send data to sink
  if (outputWriter and rts) outputWriter(false);

  // For error check during sampling
  if (sqwCounter) {
    portENTER_CRITICAL(&sqwMux);
    sqwCounter--;
    portEXIT_CRITICAL(&sqwMux);
    if (!firstSqwv and testSamples != streamConfig.samplingRate) {
      response = "MISSED ";
      response += streamConfig.samplingRate - testSamples;
      response += " SAMPLES";
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
    // testMillis = testMillis2;
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
    if (!mqtt.connected()) {
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
  return myTime.timeStr(true);
}

/****************************************************
 * If MQTT Server connection was successfull
 ****************************************************/
void onMQTTConnect() {
  logger.log("MQTT connected to %s", mqtt.ip);
  mqttSubscribe();
  // Should also publish current state
  if (mqtt.connected()) {
    mqtt.publish(mqttTopicPubSwitch, (bool)relay.state ? TRUE_STRING : FALSE_STRING, true);
  }
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
    logger.log(ALL, "Wifi Connected %s", Network::getBSSID());
    logger.log(ALL, "IP: %s", Network::localIP().toString().c_str());
    // Reinit mdns
    initMDNS();
    // The stuff todo if we have a network connection (and hopefully internet as well)
    // myTime.updateNTPTime(true); TODO: This breaks everything with the first 5 powermeters 8-13 WTF
    myTime.updateNTPTime();
    mqttUpdate = millis() + MQTT_UPDATE_INTERVAL;
    if (!mqtt.connect()) logger.log(ERROR, "Cannot connect to MQTT Server %s", mqtt.ip);
  } else {
    logger.log(ALL, "Network AP Opened");
  }

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
  if (mqtt.connected()) mqtt.disconnect();
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
      client[i].setNoDelay(true);
      client[i].setTimeout(10);
      clientConnectionLossCnt[i] = 0;
      // Set connected flag
      clientConnected[i] = true;
      streamLog[i]->_type = INFO; // This might be later reset
      streamLog[i]->_stream = (Stream*)&client[i];
      logger.addLogger(streamLog[i]);
      #ifdef SEND_INFO_ON_CLIENT_CONNECT
      sendDeviceInfo((Stream*)&client[i]);
      #endif
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
  logger.log("Client disconnected %s port %u", oldClient.remoteIP().toString().c_str(), oldClient.remotePort());
  logger.removeLogger(streamLog[i]);
  streamLog[i]->_stream = NULL;
  //  Reset new connection for streamserver
  // if ((WiFiClient*)&oldClient == (WiFiClient*)&exStreamServer) {
  //   exStreamServerNewConnection = true;
  // }
}


/****************************************************
 * Write chunks of data for TCP and Serial
 * depending on data sink
 ****************************************************/
void writeChunks(bool tail) {
  // uint32_t start = millis();
  while(ringBuffer.available() > streamConfig.chunkSize) {
    // start = millis();
    writeData(*sendStream, streamConfig.chunkSize);
    // Serial.printf("%ums\n", millis()-start);
  }
  if (tail) {
    while(ringBuffer.available() > streamConfig.chunkSize) writeData(*sendStream, streamConfig.chunkSize);
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
    while(ringBuffer.available() > streamConfig.chunkSize) {
      udpClient.beginPacket(streamConfig.ip, streamConfig.port);
      writeData(udpClient, streamConfig.chunkSize);
      udpClient.endPacket();
    }
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

  // May be the most inefficient way to do it 
  // TODO: Improve this

  while(ringBuffer.available() >= streamConfig.measurementBytes) {

    JsonObject objSample = docSample.to<JsonObject>();
    objSample.clear();
    objSample["unit"] = unitStr;
    objSample["ts"] = myTime.timestampStr();
    ringBuffer.read((uint8_t*)&values[0], streamConfig.measurementBytes);
    // JsonArray array = docSend["values"].to<JsonArray>();
    // for (int i = 0; i < streamConfig.numValues; i++) objSample["values"][i] = values[i];
    JsonArray array = docSend.createNestedArray("values");
    for (int i = 0; i < streamConfig.numValues; i++) array.add(values[i]);
    response = "";
    serializeJson(docSample, response);
    logger.log(response.c_str());
    mqtt.publish(mqttTopicPubInfo, response.c_str());
    sentSamples++;
  }
}


/****************************************************
 * Write one chunk of data to sink.
 ****************************************************/
void writeData(Stream &getter, uint16_t size) {
  // long startDone = micros();
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
  // long prefixDone = micros();

  ringBuffer.read(&sendbuffer[start], size);
  // long readDone = micros();
  // Everything is sent at once (hopefully)
  uint32_t sent = getter.write((uint8_t*)&sendbuffer[0], size+start);
  // long sendDone = micros();
  if (sent > start) sentSamples += (sent-start)/streamConfig.measurementBytes;
  // long allDone = micros();
}

void test(bool tail) {
  if (ringBuffer.available() > streamConfig.samplingRate*streamConfig.measurementBytes) {
    ringBuffer.reset();
    packetNumber++;
    size_t test = snprintf((char*)&sendbuffer[0], SEND_BUF_SIZE, "Test: %s - %lu\r\n", timeStr(), packetNumber);
    sendStream->write((uint8_t*)&sendbuffer[0], test);
  }
}

/****************************************************
 * Sampling interrupt, must be small and quick
 ****************************************************/
static SemaphoreHandle_t timer_sem;
volatile uint32_t mytime = micros();
xQueueHandle xQueue = xQueueCreate(1000, sizeof(bool));
bool volatile IRAM_timeout = false;
void IRAM_ATTR sample_ISR2() {
  if (counter < streamConfig.samplingRate) {
    // Vaiables for frequency count
    portENTER_CRITICAL_ISR(&counterMux);
    counter++;
    portEXIT_CRITICAL(&counterMux);

    BaseType_t xHigherPriorityTaskWoken;
    bool success = false;
    BaseType_t xStatus = xQueueSendToBackFromISR( xQueue, &success, &xHigherPriorityTaskWoken );
    
    // BaseType_t xStatus = xQueueSendToBack( xQueue, &data, 10 );
    // check whether sending is ok or not
    if( xStatus == pdPASS ) {
    } else {
      IRAM_timeout = true;
    }
  }
  xTaskResumeFromISR( xHandle );
}

void IRAM_ATTR sample_ISR() {
  static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (counter < streamConfig.samplingRate) {
    
    xSemaphoreGiveFromISR(timer_sem, &xHigherPriorityTaskWoken);
    if ( xHigherPriorityTaskWoken) {
      portYIELD_FROM_ISR(); // this wakes up sample_timer_task immediately
    }
  }
}

/****************************************************
 * RTOS task for handling a new sample
 ****************************************************/
bool QueueTimeout = false;
void sample_timer_task2( void * parameter) {
  vTaskSuspend( NULL );  // ISR wakes up the task

  float data[4] = { 0.0 };

  int test = 0;
  bool success = false;
  while(state != SampleState::IDLE){

    BaseType_t xStatus = xQueueReceive( xQueue, &success, 0);

    if(xStatus == pdPASS) {

      if (counter >= streamConfig.samplingRate) {
        freqCalcNow = micros();
        freq = freqCalcNow-freqCalcStart;
        freqCalcStart = freqCalcNow;
        // if (rtc.connected) timerAlarmDisable(timer);
        if(IRAM_timeout) {
          logger.log(ERROR, "Lost interrupt!");
          IRAM_timeout = false;
        }

        if(QueueTimeout) {
          logger.log(ERROR, "Queue timeout!");
          QueueTimeout = false;
        }
      }

      
      int samplesToTake = 1;
      // If we are in the middle of a second sqwv
      if (totalSamples % streamConfig.samplingRate == 1) {
        // samplesCorrect > 0 means we have too less samples
        // samplesCorrect < 0 means we have too much samples
        // If we have to coorect samples upwards
        if (correctSamples > 0) {
          samplesToTake++;
          portENTER_CRITICAL_ISR(&correctionMux);
          correctSamples--;
          portEXIT_CRITICAL_ISR(&correctionMux);
        } else if (correctSamples < 0) {
          samplesToTake--;
          portENTER_CRITICAL_ISR(&correctionMux);
          correctSamples++;
          portEXIT_CRITICAL_ISR(&correctionMux);
        }
      } 

      // Lock stpm before using it
      // xSemaphoreTake(stpm_mutex, portMAX_DELAY);
      // With correction, this should normally be 1
      for (int i = 0; i < samplesToTake; i++) {
        totalSamples++;

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

        bool success = ringBuffer.write((uint8_t*)&data[0], streamConfig.measurementBytes);
        if (!success) {
          ringBuffer.reset();
          uint32_t lostSamples = ringBuffer.getSize()/streamConfig.measurementBytes;
          logger.log(ERROR, "Overflow during ringBuffer write, lost %lu samples", lostSamples);
          // Maybe for long term recordning, we restart here as data is corrupt no matter what we do now...
          ESP.restart();
        }
      }
      // Unlock stpm 
      // xSemaphoreGive(stpm_mutex);

    } else {
      QueueTimeout = true;
    }

    if(!uxQueueMessagesWaiting(xQueue)) {
      vTaskSuspend( NULL );  // release computing time, ISR wakes up the task.
    }
  }
  vTaskDelete( NULL );
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

    // Vaiables for frequency count
    portENTER_CRITICAL_ISR(&counterMux);
    counter++;
    portEXIT_CRITICAL(&counterMux);
    

    if (counter >= streamConfig.samplingRate) {
      freqCalcNow = micros();
      freq = freqCalcNow-freqCalcStart;
      freqCalcStart = freqCalcNow;
      // if (rtc.connected) timerAlarmDisable(timer);
    }

    int samplesToTake = 1;
    // If we are in the middle of a second sqwv
    if (totalSamples % streamConfig.samplingRate == 1) {
      // samplesCorrect > 0 means we have too less samples
      // samplesCorrect < 0 means we have too much samples
      // If we have to coorect samples upwards
      if (correctSamples > 0) {
        samplesToTake++;
        portENTER_CRITICAL_ISR(&correctionMux);
        correctSamples--;
        portEXIT_CRITICAL_ISR(&correctionMux);
      } else if (correctSamples < 0) {
        samplesToTake--;
        portENTER_CRITICAL_ISR(&correctionMux);
        correctSamples++;
        portEXIT_CRITICAL_ISR(&correctionMux);
      }
    } 
    // Lock stpm before using it
    // xSemaphoreTake(stpm_mutex, portMAX_DELAY);
    // With correction, this should normally be 1
    for (int i = 0; i < samplesToTake; i++) {
      totalSamples++;

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

      bool success = ringBuffer.write((uint8_t*)&data[0], streamConfig.measurementBytes);
      if (!success) {
        ringBuffer.reset();
        uint32_t lostSamples = ringBuffer.getSize()/streamConfig.measurementBytes;
        logger.log(ERROR, "Overflow during ringBuffer write, lost %lu samples", lostSamples);
        // Maybe for long term recordning, we restart here as data is corrupt no matter what we do now...
        ESP.restart();
      }
    }
    // Unlock stpm 
    // xSemaphoreGive(stpm_mutex);

  }
  vTaskDelete( NULL );
}





/****************************************************
 * Setting up the SQWV Interrupt must be done on 
 * core 0 because internally it starts stops the other
 * interrupt, this must be done on same core
 ****************************************************/
const gpio_num_t rtc_int_pin = (gpio_num_t)RTC_INT;

void setupSqwvInterrupt() {
  rtc.enableRTCSqwv(1);
  gpio_install_isr_service(3);

  // Configure interrupt for low level
  gpio_config_t config = {
		.pin_bit_mask = 1ull << rtc_int_pin,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE
	};
	ESP_ERROR_CHECK(gpio_config(&config));
  esp_err_t err;
  
  err = gpio_set_intr_type(rtc_int_pin, GPIO_INTR_POSEDGE);
  if (err != ESP_OK) logger.log(ERROR, "set intr type sqwv failed with error 0x%x", err);

  err = gpio_isr_handler_add(rtc_int_pin, &sqwvTriggered, (void *) nullptr);
  if (err != ESP_OK) logger.log(ERROR, "sqwv isr failed with error 0x%x", err);
}


/****************************************************
 * Disable the SQWV Interrupt
 ****************************************************/
void disableSqwvInterrupt() {
  gpio_isr_handler_remove(rtc_int_pin);
  gpio_uninstall_isr_service();
}



/****************************************************
 * A SQWV signal from the RTC is generated, we
 * use this signal to handle second tasks and
 * on sampling make sure that we achieve
 * the appropriate <samplingrate> samples each second
 ****************************************************/
volatile bool sqwvIsHigh = true;
long lastSqwvTime = millis();
void IRAM_ATTR sqwvTriggered(void * instance) {
  if (sqwvIsHigh) {
    gpio_set_intr_type(rtc_int_pin, GPIO_INTR_LOW_LEVEL);
		sqwvIsHigh = false;
  
    // This is the low triggered interrupt we are talking about
    // long now = millis();
    // portENTER_CRITICAL_ISR(&criticalMsgMux);
    // criticalMsgAvailable = true;
    // testMillis2 = now-lastSqwvTime;
    // portEXIT_CRITICAL_ISR(&criticalMsgMux);
    // lastSqwvTime = now;

    // Disable timer if not already done in ISR
    timerAlarmDisable(timer);
    timerWrite(timer, 0);
    // Reset counter if not already done in ISR
    portENTER_CRITICAL_ISR(&counterMux);
    testSamples = counter;
    counter = 0;
    portEXIT_CRITICAL(&counterMux);
    
    // timerAlarmWrite(timer, TIMER_CYCLES_FAST, true);
    // We take one sample and enable the timing interrupt afterwards
    // this should allow the ISR e.g. at 4kHz to take 3999 samples during the next second
    // timerWrite(timer, 0);
    
    // indicate to main that a second has passed and store time 
    portENTER_CRITICAL_ISR(&sqwMux);
    sqwCounter++;
    sqwCounter2++;
    portEXIT_CRITICAL(&sqwMux);

    sample_ISR();
    // timerWrite(timer, TIMER_CYCLES_FAST-1);
    // Enable timer
    timerAlarmEnable(timer);
    // testMillis2 = millis();
  } else {
		gpio_set_intr_type(rtc_int_pin, GPIO_INTR_HIGH_LEVEL);
		sqwvIsHigh = true;
	}
}


/****************************************************
 * Stop Sampling will go into ide state, stop the
 * interrupt and if any stream client is connected,
 * will send EOF to the client
 ****************************************************/
void stopSampling() {
  if (rtc.connected) {
    // rtc.disableInterrupt();
    disableSqwvInterrupt();
  }
  state = SampleState::IDLE;
  next_state = SampleState::IDLE;
  // Stop the timer interrupt
  turnInterrupt(false);
  samplingDuration = millis() - startSamplingMillis;
  streamConfig.stopTs = myTime.timestamp();
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

  // xTaskCreatePinnedToCore(  sample_timer_task,     /* Task function. */
  //     "Consumer",       /* String with name of task. */
  //     4096,            /* Stack size in words. */
  //     NULL,             /* Parameter passed as input of the task */
  //     10,                /* Priority of the task. */
  //     &xHandle,            /* Task handle. */
  //     1);

  counter = 0;
  sentSamples = 0;
  totalSamples = 0;
  correctSamples = 0;
  // TIMER_CYCLES_FAST = (1000000) / streamConfig.samplingRate; // Cycles between HW timer inerrupts
  //             240Mhz      240       e.g. 4000Hz     
      
  TIMER_CYCLES_FAST = (1000000) / streamConfig.samplingRate; // Cycles between HW timer inerrupts
  calcChunkSize();
  ringBuffer.reset();
  samplingDuration = 0;
  packetNumber = 0;
  // This will reset the sqwv pin
  firstSqwv = true;
  sqwCounter = 0;
  sqwCounter2 = 0;
    
  if (rtc.connected) {
    // rtc.enableInterrupt(1, sqwvTriggered); // TODO
    setupSqwvInterrupt();
  }
  
  // If we should wait for voltage to make positive zerocrossing
  if (waitVoltage) {
    while(stpm34.readVoltage(1) > 0) {yield();}
    while(stpm34.readVoltage(1) < 0) {yield();}
  }
  freqCalcNow = micros();
  freqCalcStart = micros();
  startSamplingMillis = millis();
  streamConfig.startTs = myTime.timestamp();
  turnInterrupt(true);
}

void turnInterrupt(bool on) {
  if (on) {
    xTaskCreatePinnedToCore(  sample_timer_task,     /* Task function. */
        "Consumer",       /* String with name of task. */
        10000,            /* Stack size in words. */
        NULL,             /* Parameter passed as input of the task */
        32,                /* Priority of the task. */ // Highest priority possible?
        &xHandle,            /* Task handle. */
        1);
    xTaskCreatePinnedToCore(  startTimerInterrupt,     /* Task function. */
        "startInterrupt",       /* String with name of task. */
        4096,            /* Stack size in words. */
        NULL,             /* Parameter passed as input of the task */
        2,                /* Priority of the task. */
        NULL,            /* Task handle. */
        1);
  } else {
    xTaskCreatePinnedToCore(  stopTimerInterrupt,     /* Task function. */
        "stopInterrupt",       /* String with name of task. */
        4096,            /* Stack size in words. */
        NULL,             /* Parameter passed as input of the task */
        2,                /* Priority of the task. */
        NULL,            /* Task handle. */
        1);
  }
}

void startTimerInterrupt( void * param ) {
  cli();//stop interrupts
  timer_init();
  // The timer runs at 80 MHZ, independent of cpu clk
  timerAlarmWrite(timer, TIMER_CYCLES_FAST, true);
  timerWrite(timer, 0);
  // Take first sample here
  sample_ISR();
  // Alternative to take first sample
  // timerWrite(timer, TIMER_CYCLES_FAST-1);
  timerAlarmEnable(timer);
  sei();
  vTaskDelete( NULL );
}

void stopTimerInterrupt( void * param ) {
  cli();//stop interrupts
  if (timer != NULL) timerAlarmDisable(timer);
  timerEnd(timer);
  timer = NULL;
  sei();
  vTaskDelete( NULL );
}

/****************************************************
 * Detach or attach sampling interupt
 ****************************************************/
void turnInterrupt2(bool on) {
  cli();//stop interrupts
  if (on) {
    timer_init();
    // The timer runs at 80 MHZ, independent of cpu clk
    timerAlarmWrite(timer, TIMER_CYCLES_FAST, true);
    timerWrite(timer, 0);
    // Take first sample here
    sample_ISR();
    // Alternative to take first sample
    // timerWrite(timer, TIMER_CYCLES_FAST-1);
    timerAlarmEnable(timer);
  } else {
    if (timer != NULL) timerAlarmDisable(timer);
    timerEnd(timer);
    timer = NULL;
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
  // timer = timerBegin(1, 80, true);
  // timer = timerBegin(1, 8, true);
  timerAttachInterrupt(timer, &sample_ISR, true);
}

/****************************************************
 * Init the MDNs name from eeprom, only the number ist
 * stored in the eeprom, construct using prefix.
 ****************************************************/
void initMDNS() {
  char * name = config.netConf.name;
  if (strlen(name) == 0) {
    logger.log(ERROR, "Sth wrong with mdns");
    strcpy(name,"powermeterX");
  }
  // Setting up MDNs with the given Name
  logger.log("MDNS Name: %s", name);
  if (!MDNS.begin(String(name).c_str())) {             // Start the mDNS responder for esp8266.local
    logger.log(ERROR, "Setting up MDNS responder!");
  }
  MDNS.addService("_elec", "_tcp", STANDARD_TCP_STREAM_PORT);
}

/****************************************************
 * Check streamserver connection, since connect is 
 * a blocking task, we make it nonblocking using 
 * a separate freerots task
 ****************************************************/
TaskHandle_t streamServerTaskHandle = NULL;
void checkStreamServer(void * pvParameters) {
  bool first = true;
  vTaskDelay(STREAM_SERVER_UPDATE_INTERVAL/10);
  while (not updating) {
    if (!exStreamServer.connected()                         // If not already connected
        and state == SampleState::IDLE                      // and in idle mode
        and Network::connected and not Network::apMode      // Network is STA mode
        and strcmp(config.streamServer, NO_SERVER) != 0) {  // and server is set
      if (exStreamServer.connect(config.streamServer, STANDARD_TCP_STREAM_PORT)) {
        logger.log("Connected to StreamServer: %s", config.streamServer);
        onClientConnect(exStreamServer);
      } else {
        if (first) {
          logger.log(WARNING, "Cannot connect to StreamServer: %s", config.streamServer);
          first = false;
        }
      }
    } else {
      
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
    // logger.log("Try to connect Stream Server: %s", config.streamServer);
    // exStreamServer.connect(config.streamServer, STANDARD_TCP_STREAM_PORT);
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
  if (!mqtt.connected()) {
    logger.log(ERROR, "Should subscribe but is not connected to mqtt Server");
    return;
  }

  // Build publish topics
  snprintf(&mqttTopicPubSwitch[0], MAX_MQTT_TOPIC_LEN, "%s%c", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR);

  response = mqttTopicPubSwitch;
  response += "+";
  logger.log("Subscribing to: %s", response.c_str());
  mqtt.subscribe(response.c_str());
  
  snprintf(&mqttTopicPubSwitch[0], MAX_MQTT_TOPIC_LEN, "%s%c%s%c", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR, config.netConf.name, MQTT_TOPIC_SEPARATOR);

  response = mqttTopicPubSwitch;
  response += "+";
  logger.log("Subscribing to: %s", response.c_str());
  mqtt.subscribe(response.c_str());
  
  snprintf(&mqttTopicPubSwitch[0], MAX_MQTT_TOPIC_LEN, "%s%c%s%c%s%c%s", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR, config.netConf.name, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_STATE, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_SWITCH);
  snprintf(&mqttTopicPubSample[0], MAX_MQTT_TOPIC_LEN, "%s%c%s%c%s%c%s", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR, config.netConf.name, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_STATE, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_SAMPLE);
  snprintf(&mqttTopicPubInfo[0],   MAX_MQTT_TOPIC_LEN, "%s%c%s%c%s%c%s", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR, config.netConf.name, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_STATE, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_INFO);
}

/****************************************************
 * Nice formatted info str with all available infos 
 * of this device
 * NOTE: Make sure enough memory is allocated for str
 ****************************************************/
void printInfoString() {
  // Name and firmware
  logger.log("%s @ firmware: %s/%s", config.netConf.name, __DATE__, __TIME__);

  logger.log("Using %d bytes %s", ringBuffer.getSize(), ringBuffer.inPSRAM()?"PSRAM":"RAM");
 
  if (rtc.connected) {
    logger.log("RTC is connected %s", rtc.lost?"but has lost time":"and has valid time");
  } else {
    logger.log("No RTC");
  }
  logger.append("Known Networks: [");
  for (size_t i = 0; i < config.netConf.numAPs; i++) {
    logger.append("%s%s", config.netConf.SSIDs[i], i < config.netConf.numAPs-1?", ":"");
  }
  logger.flushAppended();
}

/****************************************************
 * Callback if relay is switched
 ****************************************************/
void relayCB(bool value) {
  if (state != SampleState::SAMPLE) { 
    // Only if we are not sampling, we can use SPI/EEEPROM
    config.setRelayState(value); 
  }
  if (mqtt.connected()) {
    mqtt.publish(mqttTopicPubSwitch, value ? TRUE_STRING : FALSE_STRING, true);
  }
  logger.log("Switched %s", value ? "on" : "off");
}

/****************************************************
 * Callback if sampling will be perfomed
 ****************************************************/
void sampleCB() {
  if (mqtt.connected()) {
    mqtt.publish(mqttTopicPubSample, state==SampleState::SAMPLE ? MQTT_TOPIC_SWITCH_ON : MQTT_TOPIC_SWITCH_OFF);
  }
  // Do not allow network changes on sampling
  Network::allowNetworkChange = state==SampleState::SAMPLE ? false : true;
}



/****************************************************
 * Display sampling information to logger
 ****************************************************/
int samplingCheck() {
  if (state != SampleState::IDLE) {
    unsigned long _totalSamples = totalSamples;
    Timestamp now = myTime.timestamp();
    uint32_t durationMs = (now.seconds - streamConfig.startTs.seconds)*1000;
    durationMs += (int(now.milliSeconds) - int(streamConfig.startTs.milliSeconds));
    float avgRate = _totalSamples/(durationMs/1000.0);
    uint32_t _goalSamples = float(durationMs/1000.0)*streamConfig.samplingRate;
    int samplesOff = int(_goalSamples - _totalSamples);
    float secondsOff = float(samplesOff)/float(streamConfig.samplingRate);
    logger.log(INFO, "After NTP Sync: avg rate %.3f Hz", avgRate);
    LogType type = LogType::INFO;
    if (abs(secondsOff) > 0.02) type = LogType::WARNING;
    logger.log(type, "Should be: %lu, is %lu samples", _goalSamples, _totalSamples);
    logger.log(type, "Offset of %.3f s, SecondInterrupts %i, SamplingDur %.3f", secondsOff, sqwCounter2, float(durationMs/1000.0));
    logger.log(INFO, "From %s", myTime.timestampStr(streamConfig.startTs));
    logger.log(INFO, "To %s",  myTime.timestampStr(now));

    return samplesOff;
  }
  return -1;
}

void correctSampling(int samplesToCorrect) {
  int samplesCorrect = samplesToCorrect;
  // Cap it so that signal will not be distored too much
  if (samplesCorrect > MAX_CORRECT_SAMPLES) samplesCorrect = MAX_CORRECT_SAMPLES;
  else if (samplesCorrect < -MAX_CORRECT_SAMPLES) samplesCorrect = -MAX_CORRECT_SAMPLES;
  // samplesCorrect > 0 means we have too less samples
  // samplesCorrect < 0 means we have too much samples
  portENTER_CRITICAL(&correctionMux);
  correctSamples += samplesCorrect;
  portEXIT_CRITICAL(&correctionMux);
  logger.log(WARNING, "Correcting: %i samples", samplesCorrect);
}

/****************************************************
 * Callback when NTP syncs happened
 ****************************************************/
void ntpSynced(unsigned int confidence) {
  logger.log(INFO, "NTP synced with conf: %u", confidence);
  // Confidence is time in ms for ntp round trip -> the smaller the better
  if (state != SampleState::IDLE) {
    int samplesOff = samplingCheck();
    float secondsOff = float(samplesOff)/float(streamConfig.samplingRate);

    #ifdef NTP_CORRECT_SAMPLINGRATE
    if (confidence < NTP_CONFIDENCE_FOR_CORRECTION) {
      if (abs(secondsOff) >= CORRECT_SAMPLING_THRESHOLD) {
        correctSampling(samplesOff);
      }
    }
    #endif
  }
}


/****************************************************
 * Setup the OTA updates progress
 ****************************************************/
// To display only full percent updates
unsigned int oldPercent = 0;
void setupOTA() {
  // Same name as mdns
  ArduinoOTA.setHostname(config.netConf.name);
  ArduinoOTA.setPassword("energy"); 
  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash(2"21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    updating = true;
    Network::allowNetworkChange = false;
    logger.log("Start updating");

    if (state != SampleState::IDLE) {
      logger.log(WARNING, "Stop Sampling, Update Requested");
      stopSampling();

      // // Write remaining chunks with tailr
      // if (outputWriter != NULL) outputWriter(true);
      // docSend["msg"] = F("Received stop command");
      // docSend["sample_duration"] = samplingDuration;
      // docSend["samples"] = totalSamples;
      // docSend["sent_samples"] = sentSamples;
      // docSend["start_ts"] = myTime.timestampStr(streamConfig.startTs);
      // docSend["stop_ts"] = myTime.timestampStr(streamConfig.stopTs);
      // docSend["ip"] = WiFi.localIP().toString();
      // docSend["avg_rate"] = totalSamples/(samplingDuration/1000.0);
      // docSend["cmd"] = CMD_STOP;

      // serializeJson(docSend, response);
      // response = LOG_PREFIX + response;
      // if (sendStream) sendStream->println(response);
    }

    // Disconnecting all connected clients
    for (size_t i = 0; i < MAX_CLIENTS; i++) {
      if (clientConnected[i]) {
        onClientDisconnect(client[i], i);
        clientConnected[i] = false;
      }
    }
    if (exStreamServer.connected()) exStreamServer.stop();
    if (streamClient.connected()) streamClient.stop();
    
    // Stopping all other tcp stuff
    streamServer.stop();
    streamServer.close();
    server.stop();
    server.close();
    mqtt.disconnect();
  });

  ArduinoOTA.onEnd([]() {
    #ifdef USE_SERIAL
    logger.log("End");
    #endif
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    #ifdef USE_SERIAL
    unsigned int percent = (progress / (total / 100));
    if (percent != oldPercent) {
      Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
      oldPercent = percent;
    }
    #endif
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    #ifdef USE_SERIAL
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
    #endif
    // No matter what happended, simply restart
    ESP.restart();
  });
  // Enable OTA
  ArduinoOTA.begin();
}