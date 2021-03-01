extern "C" {
  #include "user_interface.h"
}
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <time.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <TimeLib.h>
#include <ESP8266mDNS.h>

// Arduino Updater
#include <ArduinoOTA.h>

#include <PubSubClient.h>

#include "constDefine.h"
#include "src/logger/logger.h"
#include "src/stpm/STPM.h"
#include "src/relay/relay.h"
#include "src/network/network.h"
#include "src/config/config.h"
#include "src/time/timeHandling.h"
#include "src/ringbuffer/ringbuffer.h"



// Serial logger
StreamLogger serialLog((Stream*)&Serial, &timeStr, &LOG_PREFIX_SERIAL[0], ALL);
// SPIFFS logger
SPIFFSLogger spiffsLog(false, &LOG_FILE[0], &timeStr, &LOG_PREFIX_SERIAL[0], WARNING);

// MultiLogger logger(&streamLog, &timeStr);
// Create singleton here
MultiLogger& logger = MultiLogger::getInstance();

Configuration config;

Relay relay(RELAY_PIN_S);

TimeHandler myTime(ntpServerName, LOCATION_TIME_OFFSET);

// STPM Object
STPM stpm34(STPM_RES, STPM_CS, STPM_SYN);

// counter holds # isr calls
volatile uint16_t counter = 0;
// Last micros() count of isr call
volatile long nowTs = 0;
volatile long lastTs = 0;

RingBuffer ringBuffer(BUF_SIZE);

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
enum SampleState{STATE_IDLE, STATE_SAMPLE};

SampleState state = STATE_IDLE;
SampleState next_state = STATE_IDLE;

// Available measures are VOLTAGE+CURRENT, ACTIVE+REACTIVE Power or both
enum Measures{STATE_VI, STATE_PQ, STATE_VIPQ};
enum StreamType{USB, TCP, UDP, TCP_RAW, MQTT};

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

// TCP clients and current connected state, each client is assigned a logger
WiFiClient client[MAX_CLIENTS];
bool clientConnected[MAX_CLIENTS] = {false};
StreamLogger * streamLog[MAX_CLIENTS];

// Strean client is for ffmpeg direct streaming
WiFiClient streamClient;
// tcp stuff is send over this client 
Stream * newGetter;
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
long mqttUpdate = millis();

// Current CPU speed
unsigned int coreFreq = 0;

// test stuff
long testMillis = 0;
long testMillis2 = 0;
uint16_t testSamples = 0;

volatile float data[4] = { 0.0 };

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

// Required for flag in wifi connect callback function
bool netConnected = false;

/************************ SETUP *************************/
void setup() {
  // Setup serial communication
  Serial.begin(SERIAL_SPEED);

  bool successAll = true;
  bool success = successAll;
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

  successAll &= success;
  relay.set(true);
  relay.setCallback(relayCB);
  
  config.init();
  config.load();

  coreFreq = ESP.getCpuFreqMHz();
  logger.log(DEBUG, "%s @ firmware %s/%s", config.name, __DATE__, __TIME__);
  logger.log(DEBUG, "Core @ %u MHz", coreFreq);

  success = ringBuffer.init();
  if (!success) logger.log(ERROR, "RAM init failed");
  successAll &= success;


  // Setup STPM 32
  success = stpm34.init();
  if (!success) logger.log(ERROR, "STPM Init Failed");
  successAll &= success;

  
   // Indicate error if there is any
  // digitalWrite(ERROR_LED, !successAll);

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
  mqttUpdate = millis();
}

/************************ Loop *************************/
void loop() {
  if (updating) return;

  if (netConnected) {
    onNetworkConnect();
    netConnected = false;
  }

  // Stuff done on idle
  if (state == STATE_IDLE) {
    onIdle();
  // Stuff on sampling
  } else if (state == STATE_SAMPLE) {
    onSampling();
  }

  onIdleOrSampling();

  // If we only have 200 ms before sampling should start, wait actively
  if (next_state != STATE_IDLE) {
    if (streamConfig.countdown != 0 and (streamConfig.countdown - millis()) < 200) {
      // Disable any wifi sleep mode
      wifi_set_sleep_type(NONE_SLEEP_T);
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

/****************************************************
 * What todo independent of sampling state
 ****************************************************/
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

    
  // Update lifeness only on idle every second
  if ((long)(millis() - lifenessUpdate) >= 0) {
    lifenessUpdate += LIFENESS_UPDATE_INTERVAL;
    logger.log("");
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
      streamConfig.port = STANDARD_TCP_SAMPLE_PORT;
      sendClient = &streamClient;
      startSampling();
    }
  }
}

/****************************************************
 * What todo only on sampling (send data, etc)
 ****************************************************/
void onSampling() {
  // ______________ Send data to sink ________________
  writeChunks(false);


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
    if (!sendClient->connected()) {
      logger.log(ERROR, "TCP disconnected while streaming");
      stopSampling();
    }
  // Disconnect of UDP means disconnecting from tcp port
  } else if (streamConfig.stream == UDP) {
    if (!sendClient->connected()) {
      logger.log(ERROR, "TCP/UDP disconnected while streaming");
      stopSampling();
    }
  // Disconnect of raw stream means stop
  } else if (streamConfig.stream == TCP_RAW) {
    // Check for intended connection loss
    if (!sendClient->connected()) {
      logger.log(INFO, "TCP Stream disconnected");
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
 * If ESP is connected to wifi successfully
 * NOTE: Must be small callback function, 
 * nothing else is allowed here, otherwise esp will crash
 ****************************************************/
void onWifiConnect() {
  netConnected = true;
}

/****************************************************
 * If ESP is connected to Network successfully
 ****************************************************/
void onNetworkConnect() {
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
  mqttUpdate = millis();
}

/****************************************************
 * If ESP disconnected from wifi
 ****************************************************/
void onWifiDisconnect() {
  logger.log(ERROR, "Wifi Disconnected");

  if (state != STATE_IDLE) {
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
 * Write all remaining chunks of data over channel
 * depending on data sink
 ****************************************************/
void writeChunks(bool tail) {
  while(ringBuffer.available() > streamConfig.chunkSize) {
    if (streamConfig.stream == UDP) {
      udpClient.beginPacket(streamConfig.ip, streamConfig.port);
      writeData(udpClient, streamConfig.chunkSize);
      udpClient.endPacket();
    } else if (streamConfig.stream == TCP) {
      writeData(*sendClient, streamConfig.chunkSize);
    } else if (streamConfig.stream == TCP_RAW) {
      writeData(*sendClient, streamConfig.chunkSize);
    } else if (streamConfig.stream == USB) {
      writeData(Serial, streamConfig.chunkSize);
    } else if (streamConfig.stream == MQTT) {
      writeDataMQTT(streamConfig.chunkSize);
    }
  }
  if (tail) {
    if (streamConfig.stream == UDP) {
      udpClient.beginPacket(streamConfig.ip, streamConfig.port);
      writeData(udpClient, ringBuffer.available());
      udpClient.endPacket();
    } else if (streamConfig.stream == TCP) {
      writeData(*sendClient, ringBuffer.available());
    } else if (streamConfig.stream == TCP_RAW) {
      writeData(*sendClient, ringBuffer.available());
    } else if (streamConfig.stream == USB) {
      writeData(Serial, ringBuffer.available());
    }
  }
}

/****************************************************
 * Write data to mqtt sink, this requires special
 * formatiing of the data
 ****************************************************/
void writeDataMQTT(uint16_t size) {
  if (size <= 0) return;
  size_t i = 0;
  while (i < size) { 
    i++;
  }
  
}

/****************************************************
 * Write one chunk of data to sink.
 ****************************************************/
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

/****************************************************
 * ISR for sampling
 ****************************************************/
volatile uint32_t isrTime = 0;
void ICACHE_RAM_ATTR sample_ISR(){
  cli();//stop interrupts
  // Vaiables for frequency count
  counter++;
  totalSamples++;
  if (counter >= streamConfig.samplingRate) {
    freqCalcNow = micros();
    freq = freqCalcNow-freqCalcStart;
    freqCalcStart = freqCalcNow;
    counter = 0;
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

  // Calculate next time to execute and compare to current time
  timer_next = timer_next + TIMER_CYCLES_FAST;
  timer_now = ESP.getCycleCount();
  // Check If next is not in the past and handle overflow
  // Cant we make this better with absolut values?
  if (timer_next > timer_now || (timer_now <= (uint32_t)4294967296 && timer_next <= TIMER_CYCLES_FAST)) {
    timer0_write(timer_next);
  // If the ISR took to long, we indicate the error and start the ISR again immidiately
  } else {
    Serial.println(F("Info:Timer error"));
    timer0_write(ESP.getCycleCount() + 1000);
  }
  sei();
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

/****************************************************
 * Overload of start sampling to wait for voltage
 * to reach zero crossing
 ****************************************************/
void startSampling(bool waitVoltage) {
  // Reset all variables
  state = STATE_SAMPLE;
  

  counter = 0;
  sentSamples = 0;
  totalSamples = 0;
  TIMER_CYCLES_FAST = (1000000) / streamConfig.samplingRate; // Cycles between HW timer inerrupts
  calcChunkSize();
  ringBuffer.reset();
  samplingDuration = 0;
  packetNumber = 0;
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
 * Detach or attach intterupt
 ****************************************************/
void turnInterrupt(bool on) {
  cli();//stop interrupts
  if (on) {
    timer0_attachInterrupt(sample_ISR);
    timer_next = ESP.getCycleCount() + TIMER_CYCLES_FAST;
    freqCalcStart = micros();
    timer0_write(timer_next);
  } else {
    timer0_detachInterrupt();
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

  sprintf(&mqttTopicPubSwitch[0], "%s%c%s%c", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR, config.name, MQTT_TOPIC_SEPARATOR);

  response = "";
  response += mqttTopicPubSwitch;
  response += "+";
  logger.log("Subscribing to: %s", response.c_str());
  mqttClient.subscribe(response.c_str());
  mqttClient.loop();
  
  sprintf(&mqttTopicPubSwitch[0], "%s%c%s%c%s%c%s", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR, config.name, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_STATE, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_SWITCH);
  sprintf(&mqttTopicPubSample[0], "%s%c%s%c%s%c%s", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR, config.name, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_STATE, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_SAMPLE);
  sprintf(&mqttTopicPubInfo[0], "%s%c%s%c%s%c%s", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR, config.name, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_STATE, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_INFO);
  logger.log("MQTT connected"); 
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
  idx += sprintf(&str[idx], "RAM");
  idx += sprintf(&str[idx], "\nNo RTC");
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
  if (mqttClient.connected()) mqttClient.publish(mqttTopicPubSwitch, value ? MQTT_TOPIC_SWITCH_ON : MQTT_TOPIC_SWITCH_OFF);
  logger.log("Switched %s", value ? "on" : "off");
}

/****************************************************
 * Callback if sampling will be perfomed
 ****************************************************/
void sampleCB() {
  if (mqttClient.connected()) mqttClient.publish(mqttTopicPubSample, state==STATE_SAMPLE ? MQTT_TOPIC_SWITCH_ON : MQTT_TOPIC_SWITCH_OFF);
}
