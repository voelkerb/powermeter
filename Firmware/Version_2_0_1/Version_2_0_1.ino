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

#include "constDefine.h"
#include "enums.h"
#include "logger.h"
#include "STPM.h"
#include "relay.h"
#include "network.h"
#include "rtc.h"
#include "config.h"
#include "timeHandling.h"
#include "ringbuffer.h"

// Function prototypes required for e.g. platformio
void IRAM_ATTR sample_ISR();
void IRAM_ATTR sqwvTriggered();

TaskHandle_t xHandle = NULL;

// Create singleton here
MultiLogger& logger = MultiLogger::getInstance();

Configuration config;

Relay relay(RELAY_PIN_S, RELAY_PIN_R);

Rtc rtc(RTC_INT);
TimeHandler myTime(config.timeServer, LOCATION_TIME_OFFSET, &rtc);

// STPM Object
STPM stpm34(STPM_RES, STPM_CS, STPM_SYN);

// counter holds # isr calls
volatile uint16_t counter = 0;
// Last micros() count of isr call
volatile long nowTs = 0;
volatile long lastTs = 0;

RingBuffer ringBuffer(PS_BUF_SIZE, true);

#define SEND_BUF_SIZE MAX_SEND_SIZE+16
static uint8_t sendbuffer[SEND_BUF_SIZE] = {0};
// Buffer read/write position
uint32_t psdReadPtr = 0;
uint32_t psdWritePtr = 0;
volatile uint32_t writePtr = 0;

// Open two TCP ports, one for commands and sampling and one for a raw data stream
WiFiServer server(STANDARD_TCP_SAMPLE_PORT);

// TIMER stuff
// Calculate the number of cycles we have to wait
volatile uint32_t TIMER_CYCLES_FAST = (1000000) / DEFAULT_SR; // Cycles between HW timer inerrupts

SampleState state = SampleState::IDLE;
SampleState next_state = SampleState::IDLE;

struct StreamConfig {
  Measures measures = Measures::VI;         // The measures to send
  uint8_t measurementBytes = 8;             // Number of bytes for each measurement entry
  unsigned int samplingRate = DEFAULT_SR;   // The samplingrate
  unsigned int countdown = 0;               // Start at specific time or immidiately
  uint32_t chunkSize = 0;                   // Chunksize of one packet sent
  IPAddress ip;                             // Ip address of data sink
  StreamType stream = StreamType::USB;      // Channel over which to send
  uint16_t port = STANDARD_TCP_SAMPLE_PORT; // Port of data sink
  size_t numValues = 24/sizeof(float);      // Number of values
  Timestamp startTs;
  Timestamp stopTs;
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
bool exStreamServerNewConnection = true;

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
portMUX_TYPE counterMux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t stpm_mutex;

// test stuff
long testMillis = 0;
long testMillis2 = 0;
uint16_t testSamples = 0;

// Mutex for 1s RTC Interrupt
portMUX_TYPE sqwMux = portMUX_INITIALIZER_UNLOCKED;
bool firstSqwv = true;
volatile int sqwCounter = 0;

volatile float values[4] = {0};

// OTA Update in progress
bool updating = false;

// Command stuff send over what ever
char command[COMMAND_MAX_SIZE] = {'\0'};
StaticJsonDocument<2*COMMAND_MAX_SIZE> docRcv;
StaticJsonDocument<2*COMMAND_MAX_SIZE> docSend;
String response = "";

void (*outputWriter)(bool) = &writeChunks;

/************************ SETUP *************************/
void setup() {
  // Set realy to true
  relay.set(true);

  // Load config, an set relay to wished state
  config.init();  
  config.load();
  relay.set(config.getRelayState());

  // At first init the rtc module to get a time behavior
  bool successAll = true;
  bool success = rtc.init();
  // Init the logging modules
  logger.setTimeGetter(&timeStr);
  // Init all loggers
  logger.init();
  // init the stream logger array
  for (size_t i = 0; i < MAX_CLIENTS; i++) {
    StreamLogger * theStreamLog = new StreamLogger(NULL, &timeStr, &LOG_PREFIX[0], INFO);
    streamLog[i] = theStreamLog;
  }

  if (!success) logger.log(ERROR, "Cannot init RTC");
  successAll &= success;
  

  // We do not need bluetooth, so disable it
  esp_bt_controller_disable();

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
  if (!success) logger.log(ERROR, "STPM Init Failed");
  successAll &= success;

  logger.log(ALL, "Connecting WLAN");

  // Init network connection
  Network::init(&config, onWifiConnect, onWifiDisconnect);
  // Setup OTA updating
  setupOTA();

  // Resever enough bytes for large string
  response.reserve(2*COMMAND_MAX_SIZE);

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

  // Watchdog
  yield();
}

void onIdleOrSampling() {

  // Handle tcp requests
  for (size_t i = 0; i < MAX_CLIENTS; i++) {
    if (client[i].available() > 0) {
      handleEvent(&client[i]);
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
  docSend["name"] = config.name;
  docSend["ip"] = Network::localIP().toString();
  docSend["mqtt_server"] = config.mqttServer;
  docSend["stream_server"] = config.streamServer;
  docSend["time_server"] = config.timeServer;
  docSend["sampling_rate"] = streamConfig.samplingRate;
  docSend["buffer_size"] = ringBuffer.getSize();
  docSend["psram"] = ringBuffer.inPSRAM();
  docSend["rtc"] = rtc.connected;
  docSend["state"] = state != SampleState::IDLE ? "busy" : "idle";
  docSend["relay"] = relay.state;
  String ssids = "[";
  for (size_t i = 0; i < config.numAPs; i++) {
    ssids += config.wifiSSIDs[i];
    if (i < config.numAPs-1) ssids += ", ";
  }
  ssids += "]";
  docSend["ssids"] = ssids;
  docSend["ssid"] = WiFi.SSID();
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
  docSend["name"] = config.name;
  docSend["chunksize"] = streamConfig.chunkSize;
  docSend["samplingrate"] = streamConfig.samplingRate;
  docSend["measurements"] = streamConfig.numValues;
  docSend["cmd"] = CMD_SAMPLE;
  docSend["startTs"] = myTime.timestampStr(streamConfig.startTs);
  response = "";
  serializeJson(docSend, response);
  response = LOG_PREFIX + response;
  sender->println(response);
}

/****************************************************
 * Set standard configuration
 ****************************************************/
void standardConfig() {
  streamConfig.stream = StreamType::TCP;
  streamConfig.samplingRate = DEFAULT_SR;
  streamConfig.measures = Measures::VI;
  streamConfig.ip = streamClient.remoteIP();
  streamConfig.port = STANDARD_TCP_SAMPLE_PORT;
  outputWriter = &writeChunks;
  streamConfig.measurementBytes = 8;
  streamConfig.numValues = streamConfig.measurementBytes/sizeof(float);
  calcChunkSize();
}


uint32_t sent = 0;
/****************************************************
 * What todo only on sampling (send data, etc)
 ****************************************************/
void onSampling() {

  // Send data to sink
  if (outputWriter) outputWriter(false);

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
  if (streamConfig.stream == StreamType::TCP || streamConfig.stream == StreamType::UDP) {
    if (!sendClient->connected()) {
      logger.log(ERROR, "TCP/UDP disconnected while streaming");
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
 * If ESP is connected to wifi successfully
 ****************************************************/
void onWifiConnect() {
  if (not Network::apMode) {
    logger.log(ALL, "Wifi Connected");
    logger.log(ALL, "IP: %s", Network::localIP().toString().c_str());
    // Reinit mdns
    initMDNS();
    // The stuff todo if we have a network connection (and hopefully internet as well)
    myTime.updateNTPTime(true);
  } else {
    logger.log(ALL, "Network AP Opened");
  }

  // Start the TCP server
  server.begin();
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
    udpClient.beginPacket(streamConfig.ip, streamConfig.port);
    writeData(udpClient, ringBuffer.available());
    udpClient.endPacket();
  }
}



/****************************************************
 * Write one chunk of data to sink.
 ****************************************************/
void writeData(Stream &getter, uint16_t size) {
  // long startDone = micros();
  if (size <= 0) return;
  uint32_t start = 0;
  memcpy(&sendbuffer[start], (void*)&DATA_PREFIX[0], PREFIX_SIZE);
  start += PREFIX_SIZE;
  memcpy(&sendbuffer[start], (void*)&size, sizeof(uint16_t));
  start += sizeof(uint16_t);
  memcpy(&sendbuffer[start], (void*)&packetNumber, sizeof(uint32_t));
  start += sizeof(uint32_t);
  packetNumber += 1;

  ringBuffer.read(&sendbuffer[start], size);
  // Everything is sent at once (hopefully)
  uint32_t sent = getter.write((uint8_t*)&sendbuffer[0], size+start);
  // long sendDone = micros();
  if (sent > start) sentSamples += (sent-start)/streamConfig.measurementBytes;
}


/****************************************************
 * Sampling interrupt, must be small and quick
 ****************************************************/
static SemaphoreHandle_t timer_sem;
volatile uint32_t mytime = micros();

void IRAM_ATTR sample_ISR() {
  static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // Vaiables for frequency count
  portENTER_CRITICAL_ISR(&counterMux);
  counter++;
  portEXIT_CRITICAL(&counterMux);

  //Serial.println("TIMERISR");
  // mytime = micros();
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


  while (state == SampleState::SAMPLE) {
    xSemaphoreTake(timer_sem, portMAX_DELAY);

    if (counter >= streamConfig.samplingRate) {
      freqCalcNow = micros();
      freq = freqCalcNow-freqCalcStart;
      freqCalcStart = freqCalcNow;
      if (rtc.connected) timerAlarmDisable(timer);
    }

    totalSamples++;
    stpm34.readVoltAndCurr((float*) &data[0]);

    bool success = ringBuffer.write((uint8_t*)&data[0], streamConfig.measurementBytes);
    if (!success) {
      ringBuffer.reset();
      uint32_t lostSamples = ringBuffer.getSize()/streamConfig.measurementBytes;
      logger.log(ERROR, "Overflow during ringBuffer write, lost %lu samples", lostSamples);
    }
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
  sample_ISR();
  // Enable timer
  timerAlarmEnable(timer);
  // indicate to main that a second has passed and store time 
  portENTER_CRITICAL_ISR(&sqwMux);
  sqwCounter++;
  portEXIT_CRITICAL(&sqwMux);
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
    updating = true;
    Network::allowNetworkChange = false;
    logger.log("Start updating");

    if (state != SampleState::IDLE) {
      logger.log(WARNING, "Stop Sampling, Update Requested");
      stopSampling();
    }

    // Disconnecting all connected clients
    for (size_t i = 0; i < MAX_CLIENTS; i++) {
      if (clientConnected[i]) {
        onClientDisconnect(client[i], i);
        clientConnected[i] = false;
      }
    }
    if (exStreamServer.connected()) exStreamServer.stop();
    
    server.stop();
    server.close();
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("End");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    unsigned int percent = (progress / (total / 100));
    if (percent != oldPercent) {
      Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
      oldPercent = percent;
    }
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR)Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
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
  streamConfig.stopTs = myTime.timestamp();
  // Reset all variables
  streamConfig.countdown = 0;
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

/****************************************************
 * Overload of start sampling to wait for voltage
 * to reach zero crossing
 ****************************************************/
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
  if (rtc.connected) rtc.enableInterrupt(1, sqwvTriggered);
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

/****************************************************
 * Detach or attach sampling interupt
 ****************************************************/
void turnInterrupt(bool on) {
  cli();//stop interrupts
  if (on) {
    timer_init();
    // The timer runs at 80 MHZ, independent of cpu clk
    timerAlarmWrite(timer, TIMER_CYCLES_FAST, true);
    timerWrite(timer, 0);
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
  char * name = config.name;
  if (strlen(name) == 0) {
    logger.log(ERROR, "Sth wrong with mdns");
    strcpy(name,"powermeterX");
  }
  // Setting up MDNs with the given Name
  logger.log("MDNS Name: %s", name);
  if (!MDNS.begin(name)) {             // Start the mDNS responder for esp8266.local
    logger.log(ERROR, "Setting up MDNS responder!");
  }
}

/****************************************************
 * Check streamserver connection, sicne connect is 
 * a blocking task, we make it nonblocking using 
 * a separate freerots task
 ****************************************************/
TaskHandle_t streamServerTaskHandle = NULL;
void checkStreamServer(void * pvParameters) {
  bool first = true;
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