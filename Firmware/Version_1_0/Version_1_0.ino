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

#include "constDefine.h"
#include "STPM.h"
#include "relay.h"
#include "rtc.h"
#include "config.h"
#include "ringbuffer.h"
#include "network.h"
#include "timeHandling.h"
#include "logger.h"


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
TaskHandle_t xHandleProducer = NULL;

xQueueHandle xQueue;



// Serial logger
StreamLogger serialLog((Stream*)&Serial, &timeStr, &LOG_PREFIX_SERIAL[0], ALL);
// TCP logger
StreamLogger streamLog((Stream*)NULL, &timeStr, &LOG_PREFIX[0], INFO);
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


enum StreamType{USB, TCP, UDP, TCP_RAW};

// Internal state machine for sampling
enum SampleState{STATE_IDLE, STATE_SAMPLE};

SampleState state = STATE_IDLE;
SampleState next_state = STATE_IDLE;

// Available measures are VOLTAGE+CURRENT, ACTIVE+REACTIVE Power or both
enum Measures{STATE_VI, STATE_PQ, STATE_VIPQ};

struct StreamConfig {
  bool prefix = false;                      // Send data with "data:" prefix
  Measures measures = STATE_VI;             // The measures to send
  uint8_t measurementBytes = 8;             // Number of bytes for each measurement entry
  unsigned int samplingRate = DEFAULT_SR;   // The samplingrate
  StreamType stream = USB;                  // Channel over which to send
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

bool tcpConnected = false;
WiFiClient tcpClient;
WiFiClient streamClient;
// UDP used for streaming
WiFiUDP udpClient;

char command[COMMAND_MAX_SIZE] = {'\0'};
StaticJsonDocument<2*COMMAND_MAX_SIZE> docRcv;
StaticJsonDocument<2*COMMAND_MAX_SIZE> docSend;
String response = "";

unsigned long samplingCountdown = 0;
unsigned long startSamplingMillis = 0;
unsigned long samplingDuration = 0;
unsigned long sentSamples = 0;
volatile unsigned long packetNumber = 0;
volatile unsigned long totalSamples = 0;

long lifenessUpdate = millis();
long mdnsUpdate = millis();
long wifiUpdate = millis();

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


#ifdef ARDUINO_ARCH_ESP32
#include "esp32-hal-log.h"
#endif


// How can we change those parameters in Arduino, possible?
// CONFIG_TCP_MSS=1436
// CONFIG_TCP_MSL=60000
// CONFIG_TCP_SND_BUF_DEFAULT=5744
// CONFIG_TCP_WND_DEFAULT=5744

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

  if (!success) logger.log(ERROR, "Cannot init RTC");
  successAll &= success;
  relay.set(true);
  // We do not need bluetooth, so disable it
  esp_bt_controller_disable();
  pinMode(ERROR_LED, OUTPUT);

  // Indicate lifeness / reset
  digitalWrite(ERROR_LED, HIGH);
  delay(200);
  digitalWrite(ERROR_LED, LOW);
  delay(200);
  digitalWrite(ERROR_LED, HIGH);

  config.load();

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
  wifiUpdate = millis();
}

// the loop routine runs over and over again forever:
void loop() {

  // Stuff done on idle
  if (state == STATE_IDLE) {
    onIdle();
  // Stuff on sampling
  } else if (state == STATE_SAMPLE) {
    onSampling();
  }

  // If we only have 200 ms before sampling should start, wait actively
  if (next_state != STATE_IDLE) {
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

  // Handle serial requests
  if (Serial.available()) {
    handleEvent(Serial);
  }
  // Handle tcp requests
  if (tcpClient.available() > 0) {
    handleEvent(tcpClient);
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

  // udpNtp.begin(localNTPPort);
  // getTimeNTP();

  // Reset lifeness and MDNS update
  lifenessUpdate = millis();
  mdnsUpdate = millis();
}

void onWifiDisconnect() {
  logger.log(ERROR, "Wifi Disconnected");

  if (state != STATE_IDLE) {
    logger.log(ERROR, "Stop sampling (Wifi disconnect)");
    stopSampling();
  }
}

void onIdle() {
  spiffsLog.flush();
  // Arduino OTA
  ArduinoOTA.handle();

  // Re-advertise MDNS service service every 30s 
  // TODO: no clue why, but does not work properly for esp32 (maybe it is the mac side)
  if ((long)(millis() - mdnsUpdate) >= 0) {
    // initMDNS();
    //MDNS.addService("_elec", "_tcp", STANDARD_TCP_STREAM_PORT);
    mdnsUpdate += 30000;
  }

  // Update lifeness only on idle every second
  if ((long)(millis() - lifenessUpdate) >= 0) {
    // if (rtc.connected) rtc.update();
    lifenessUpdate += 1000;
    logger.log("");
    // logger.log(ERROR, "test");
    // logger.log(WARNING, "test");
    // logger.log(DEBUG, "test");
    // logger.log(ALL, "test");
  }

  // Handle tcp client connections
  if (!tcpClient.connected()) {
    if (tcpConnected) {
      onClientDisconnect(tcpClient);
      tcpConnected = false;
    }
    // Look for people connecting over the server
    tcpClient = server.available();
    if (tcpClient.connected()) {
      // tcpClient.setNoDelay(true);
      tcpConnected = true;
      // Set new udp ip
      streamConfig.ip = tcpClient.remoteIP();
      streamConfig.port = tcpClient.remotePort();
      onClientConnect(tcpClient);
    }
  }

  // Look for people connecting over the stream server
  // If one connected there, immidiately start streaming data
  if (!streamClient.connected()) {
    streamClient = streamServer.available();
    if (streamClient.connected()) {
      // Set everything to default settings
      streamConfig.stream = TCP_RAW;
      streamConfig.prefix = false;
      streamConfig.samplingRate = DEFAULT_SR;
      streamConfig.measures = STATE_VI;
      streamConfig.ip = streamClient.remoteIP();
      streamConfig.port = streamClient.remoteIP();
      startSampling();
    }
  }
}

void onClientConnect(WiFiClient &client) {
  logger.log("Client with IP %s connected on port %u", client.remoteIP().toString().c_str(), client.remotePort());
  streamLog._stream = (Stream*)&client;
  logger.addLogger(&streamLog);
}

void onClientDisconnect(WiFiClient &client) {
  logger.log("Client discconnected", client.remoteIP().toString().c_str());
  logger.removeLogger(&streamLog);
  streamLog._stream = NULL;
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
  if (streamConfig.stream == USB) {
  } else if (streamConfig.stream == TCP) {
    if (!tcpClient.connected()) {
      logger.log(ERROR, "TCP disconnected while streaming");
      stopSampling();
    }
  // Disconnect of UDP means disconnecting from tcp port
  } else if (streamConfig.stream == UDP) {
    if (!tcpClient.connected()) {
      logger.log(ERROR, "TCP/UDP disconnected while streaming");
      stopSampling();
    }
  // Disconnect of raw stream means stop
  } else if (streamConfig.stream == TCP_RAW) {
    // Check for intended connection loss
    if (!streamClient.connected()) {
      logger.log(INFO, "TCP Stream disconnected");
      stopSampling();
    }
  }
}

// Depending on data sink, send data
void writeChunks(bool tail) {
  while(ringBuffer.available() > streamConfig.chunkSize) {
    if (streamConfig.stream == UDP) {
      udpClient.beginPacket(streamConfig.ip, streamConfig.port);
      writeData(udpClient, streamConfig.chunkSize);
      udpClient.endPacket();
    } else if (streamConfig.stream == TCP) {
      writeData(tcpClient, streamConfig.chunkSize);
    } else if (streamConfig.stream == TCP_RAW) {
      writeData(streamClient, streamConfig.chunkSize);
    } else if (streamConfig.stream == USB) {
      writeData(Serial, streamConfig.chunkSize);
    }
  }
  if (tail) {
    if (streamConfig.stream == UDP) {
      udpClient.beginPacket(streamConfig.ip, streamConfig.port);
      writeData(udpClient, ringBuffer.available());
      udpClient.endPacket();
    } else if (streamConfig.stream == TCP) {
      writeData(tcpClient, ringBuffer.available());
    } else if (streamConfig.stream == TCP_RAW) {
      writeData(streamClient, ringBuffer.available());
    } else if (streamConfig.stream == USB) {
      writeData(Serial, ringBuffer.available());
    }
  }
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

  while (state == STATE_SAMPLE) {
    xSemaphoreTake(timer_sem, portMAX_DELAY);
    test++;
    if (test%100 == 0) {
      Serial.println(micros()-mytime);
    }
    
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

    if (streamConfig.measures == STATE_VI) {
      stpm34.readVoltAndCurr((float*) &data[0]);
      // stpm34.readVoltageAndCurrent(1, (float*) &values[0], (float*) &values[1]);
    } else if (streamConfig.measures == STATE_PQ) {
      stpm34.readPower(1, (float*) &data[0], (float*) &data[2], (float*) &data[1], (float*) &data[2]);
    } else if (streamConfig.measures == STATE_VIPQ) {
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
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    logger.log("Start updating");
    // free(buffer);
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

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(docRcv, command);
  // Test if parsing succeeds.
  if (error) {
    // Remove all unallowed json characters to prevent error 
    uint32_t len = strlen(command);
    if (len > 10) len = 10;
    for (int i = 0; i < len; i++) {
      if (command[i] == '\r' || command[i] == '\n' || command[i] == '"' || command[i] == '}' || command[i] == '{') command[i] = '_';
    }
    logger.log(ERROR, "deserializeJson() failed: %.10s", &command[0]);
    return;
  }
  //docSend.clear();
  JsonObject obj = docSend.to<JsonObject>();
  obj.clear();
  response = "";
  handleJSON(getter);
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

void handleJSON(Stream &getter) {
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
      streamConfig.measures = STATE_VI;
      streamConfig.measurementBytes = 8;
    } else if (strcmp(measuresC, "v,i") == 0) {
      streamConfig.measures = STATE_VI;
      streamConfig.measurementBytes = 8;
    } else if (strcmp(measuresC, "p,q") == 0) {
      streamConfig.measures = STATE_PQ;
      streamConfig.measurementBytes = 8;
    } else if (strcmp(measuresC, "v,i,p,q") == 0) {
      streamConfig.measures = STATE_VIPQ;
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
      streamConfig.stream = USB;
    // e.g. {"cmd":{"name":"sample", "payload":{"type":"TCP", "rate":4000}}}
    } else if (strcmp(typeC, "TCP") == 0) {
      streamConfig.stream = TCP;
      streamConfig.port = STANDARD_TCP_SAMPLE_PORT;
    // e.g. {"cmd":{"name":"sample", "payload":{"type":"UDP", "rate":4000}}}
    } else if (strcmp(typeC, "UDP") == 0) {
      streamConfig.stream = UDP;
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
      docSend["port"] = streamConfig.port;
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

    next_state = STATE_SAMPLE;

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
    ESP.restart();
  }

  /*********************** INFO COMMAND ****************************/
  // e.g. {"cmd":{"name":"info"}}
  else if (strcmp(cmd, CMD_INFO) == 0) {
    docSend["cmd"] = "info";
    docSend["msg"] = F("WIFI powermeter");
    docSend["version"] = VERSION;
    String compiled = __DATE__;
    compiled += " ";
    compiled += __TIME__;
    docSend["compiled"] = compiled;
    docSend["sys_time"] = myTime.timeStr();
    docSend["name"] = config.name;
    docSend["ip"] = WiFi.localIP().toString();
    docSend["sampling_rate"] = streamConfig.samplingRate;
    docSend["buffer_size"] = ringBuffer.getSize();
    docSend["psram"] = ringBuffer.inPSRAM();
    docSend["rtc"] = rtc.connected;
    String ssids = "[";
    for (int i = 0; i < config.numAPs; i++) {
      ssids += config.wifiSSIDs[i];
      if (i < config.numAPs-1) ssids += ", ";
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
    for (int i = 0; i < config.numAPs; i++) {
      ssids += config.wifiSSIDs[i];
      ssids += ", ";
    }
    ssids += "]";
    docSend["ssids"] = ssids;
  }

  /*********************** DEl WIFI COMMAND ****************************/
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
    docSend["error"] = false;
    spiffsLog.clear();
  }

  /*********************** Get Log COMMAND ****************************/
  // e.g. {"cmd":{"name":"getLog"}}
  else if (strcmp(cmd, CMD_GET_LOG) == 0) {
    spiffsLog.flush();
    docSend["error"] = false;
    bool hasRow = spiffsLog.nextRow(&command[0]);
    getter.printf("%s{\"cmd\":\"log\",\"msg\":\"", &LOG_PREFIX[0]);
    getter.printf("*** LOGFile *** //n");
    while(hasRow) {
      getter.printf("%s//n", &command[0]);
      hasRow = spiffsLog.nextRow(&command[0]);
    }
    getter.println("*** LOGFile *** \"}");
    
    /*
    getter.printf("%s *** LOGFile *** //n", &LOG_PREFIX[0]);
    while(hasRow) {
      getter.printf("%s%s\n", &LOG_PREFIX[0], &command[0]);
      hasRow = spiffsLog.nextRow(&command[0]);
    }
    getter.printf("%s *** LOGFile *** \n", &LOG_PREFIX[0]);
    */
  }
}


/****************************************************
 * Stop Sampling will go into ide state, stop the
 * interrupt and if any stream client is connected,
 * will send EOF to the client
 ****************************************************/
void stopSampling() {
  if (rtc.connected) rtc.disableInterrupt();
  state = STATE_IDLE;
  next_state = STATE_IDLE;
  // Stop the timer interrupt
  turnInterrupt(false);
  samplingDuration = millis() - startSamplingMillis;
  if (streamClient && streamClient.connected()) streamClient.stop();
  // Reset all variables
  streamConfig.countdown = 0;
  lifenessUpdate = millis();
  mdnsUpdate = millis();
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
  if (streamConfig.stream == UDP) {
    chunkSize = min((int)chunkSize, 512);
  } else if (streamConfig.stream == USB) {
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
  state = STATE_SAMPLE;
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

// Make sure enough memory is allocated for str
void setInfoString(char * str) {
  int idx = 0;
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
  for (int i = 0; i < config.numAPs; i++) {
    ssids += config.wifiSSIDs[i];
    if (i < config.numAPs-1) ssids += ", ";
  }
  idx += sprintf(&str[idx], "\nKnown Networks: [%s]", ssids.c_str());
  idx += sprintf(&str[idx], "\n");
}
