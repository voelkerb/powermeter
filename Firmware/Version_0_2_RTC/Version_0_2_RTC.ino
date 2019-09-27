#include <SPI.h>
#include "STPM.h"
#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiAP.h>
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
#include "ringbuffer.h"
#include "network.h"
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>

// Arduino Updater
#include <ArduinoOTA.h>






// Serial Speed and DEBUG option
#define SERIAL_SPEED 2000000
#define DEBUG
#define SENT_LIFENESS_TO_CLIENTS


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
#define MAX_SEND_SIZE 512 // 1024
// Chunk sizes of data to send
const int PS_BUF_SIZE = 3*1024*1024 + 512*1024;

RingBuffer ringBuffer(PS_BUF_SIZE, true);

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
#define DEFAULT_SR 4000
// Calculate the number of cycles we have to wait
volatile uint32_t TIMER_CYCLES_FAST = (1000000) / DEFAULT_SR; // Cycles between HW timer inerrupts
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


enum StreamType{USB, TCP, UDP, TCP_RAW};

// Internal state machine for sampling
enum SampleState{STATE_IDLE, STATE_SAMPLE};

SampleState state = STATE_IDLE;
SampleState next_state = STATE_IDLE;

// Available measures are VOLTAGE+CURRENT, ACTIVE+REACTIVE Power or both
enum Measures{STATE_VI, STATE_PQ, STATE_VIPQ};

typedef struct StreamConfig {
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


WiFiClient tcpClient;
WiFiClient streamClient;
// UDP used for streaming
WiFiUDP udpClient;

#define COMMAND_MAX_SIZE 300
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
long wifiUpdate = millis();

bool apMode = false;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
unsigned int coreFreq = 0;

TaskHandle_t TaskCore2;
#include "esp32-hal-cpu.h"


extern "C" {
  #include <esp_spiram.h>
  #include <esp_himem.h>
}

TaskHandle_t TaskLoop2;

long testMillis = 0;
long testMillis2 = 0;
uint16_t testSamples = 0;
volatile bool sqwFlag = false;

bool firstSqwv = true;

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

  // if it was enabled, save FPU registers
  if(cp_state) {
    xthal_save_cp0(cp0_regs);
  // Else enable it
  } else {
    xthal_set_cpenable(1);
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
    stpm34.readVoltageAndCurrent(1, (float*) &values[0], (float*) &values[1]);
  } else if (streamConfig.measures == STATE_PQ) {
    stpm34.readPower(1, (float*) &values[0], (float*) &values[2], (float*) &values[1], (float*) &values[2]);
  } else if (streamConfig.measures == STATE_VIPQ) {
    stpm34.readAll(1, (float*) &values[0], (float*) &values[1], (float*) &values[2], (float*) &values[3]);
  }

  ringBuffer.write((uint8_t*)&values[0], streamConfig.measurementBytes);

  if(cp_state) {
     // Restore FPU registers
     xthal_restore_cp0(cp0_regs);
   } else {
     // turn it back off
     xthal_set_cpenable(0);
   }
   portEXIT_CRITICAL_ISR(&timerMux);
}

// Checks if motion was detected, sets LED HIGH and starts a timer
void IRAM_ATTR sqwvTriggered() {
  // Disable timer if not already done in ISR
  timerAlarmDisable(timer);
  // Reset counter if not already done in ISR
  testSamples = counter;
  portENTER_CRITICAL_ISR(&timerMux);
  counter = 0;
  portEXIT_CRITICAL_ISR(&timerMux);
  //
  // timerAlarmWrite(timer, TIMER_CYCLES_FAST, true);
  // If we set the timer to 0, the last sample might be missed,
  // if the sqwv is triggered before the last sample is sampled
  // If we set it to half, we have the largest margin
  timerWrite(timer, 0);
  sample_ISR();
  // Enable timer
  timerAlarmEnable(timer);
  // indicate to main
  sqwFlag = true;
  testMillis2 = millis();
}

/************************ SETUP *************************/
void setup() {
  // We do not need bluetooth, so disable it
  esp_bt_controller_disable();

   // Setup serial communication
  Serial.begin(SERIAL_SPEED);

  relay.set(true);

  config.load();

  Serial.print("Info:");
  Serial.print(config.name);
  Serial.print(" @ firmware: ");
  Serial.print(__DATE__);
  Serial.print("/");
  Serial.println(__TIME__);

  if (ringBuffer.init()) {
    Serial.printf("Info:Allocated %u Bytes of RAM\n", PS_BUF_SIZE);
  } else {
    Serial.println("Info:ps_malloc failed");
  }


  // config.makeDefault();
  // config.store();


  timer_init();

  coreFreq = getCpuFrequencyMhz();
  #ifdef DEBUG
  Serial.print("Info:Core @ ");
  Serial.print(coreFreq);
  Serial.println("MHz");
  #endif
  // Setup STPM 32
  stpm34.init();

  rtc.init();

#ifdef DEBUG
  Serial.println("Info:Connecting WLAN ");
#endif

  Network::init(&config, onWifiConnect, onWifiDisconnect);

  setupOTA();

  /*
  bool relayState = false;
  while(true) {
    // Setup STPM 32
    stpm34.init();
    float voltage = stpm34.readVoltage(1);
    Serial.println(voltage);
    delay(1000);
    yield();
    //if (relayState) Serial.println("Relay On");
    //else Serial.println("Relay Off");
    relay.set(relayState);
    //digitalWrite(RELAY_PIN_S, relayState);
    //digitalWrite(RELAY_PIN_R, !relayState);
    relayState = ! relayState;
  }*/

  response.reserve(2*COMMAND_MAX_SIZE);


  setInfoString(&command[0]);
  Serial.println(command);

  Serial.println(F("Info:Setup done"));

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
      #ifdef DEBUG
      updateTime(false);
      char * cur = printCurrentTime();
      Serial.print(F("Info:StartSampling @ "));
      Serial.println(cur);
      #endif
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

void onWifiConnect() {
  #ifdef DEBUG
  Serial.println("Info: Main: Wifi Connected");
  Serial.print("Info: IP: ");
  Serial.println(WiFi.localIP());
  #endif

  // Reinit mdns
  initMDNS();
  // Start the TCP server
  server.begin();
  streamServer.begin();

  udpNtp.begin(localNTPPort);

  getTimeNTP();
  // Reset lifeness and MDNS update
  lifenessUpdate = millis();
  mdnsUpdate = millis();
}

void onWifiDisconnect() {
  #ifdef DEBUG
  Serial.println("Info: Main: Wifi Disconnected");
  #endif
  if (state != STATE_IDLE) {
    #ifdef DEBUG
    Serial.println("Info: Stop sampling (Wifi disconnect)");
    #endif
    stopSampling();
  }
}

void onIdle() {
  // Arduino OTA
  ArduinoOTA.handle();

  // Re-advertise MDNS service service
  if ((long)(millis() - mdnsUpdate) >= 0) {
    MDNS.addService("elec", "tcp", STANDARD_TCP_STREAM_PORT);
    mdnsUpdate += 100000;
  }

  // Update lifeness only on idle every second
  if ((long)(millis() - lifenessUpdate) >= 0) {
    if (rtc.connected) rtc.update();
    lifenessUpdate += 1000;
    #ifdef DEBUG
    char * cur = printCurrentTime();
    Serial.print(F("Info:"));
    Serial.println(cur);
    #endif
  }

  // Handle tcp client connections
  if (!tcpClient.connected()) {
    // Look for people connecting over the server
    tcpClient = server.available();
    // Set new udp ip
    streamConfig.ip = tcpClient.remoteIP();
    streamConfig.port = tcpClient.remotePort();
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

void onSampling() {

  if (sqwFlag) {
    sqwFlag = false;
    if (!firstSqwv and testSamples != 0) {
      response = "Info:******** MISSED ";
      response += streamConfig.samplingRate - testSamples;
      response += " SAMPLES ********";
      Serial.println(response);
      if (tcpClient.connected() and streamConfig.prefix and streamConfig.stream != UDP) {
        tcpClient.println(response);
      }
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
    if (streamConfig.prefix and streamConfig.stream != UDP and tcpClient.connected()) {
      tcpClient.println(response);
    }
    #endif
  }

  // Send data to sink
  writeChunks(false);

  // ______________ Handle Disconnect ________________

  // NOTE: Cannot detect disconnect for USB
  if (streamConfig.stream == USB) {
  } else if (streamConfig.stream == TCP) {
    if (!tcpClient.connected()) {
      #ifdef DEBUG
      Serial.println(F("Info:TCP disconencted"));
      #endif
      stopSampling();
    }
  // Disconnect of UDP means disconnecting from tcp port
  } else if (streamConfig.stream == UDP) {
    if (!tcpClient.connected()) {
      #ifdef DEBUG
      Serial.println(F("Info:TCP/UDP disconnected"));
      #endif
      stopSampling();
    }
  // Disconnect of raw stream means stop
  } else if (streamConfig.stream == TCP_RAW) {
    // Check for intended connection loss
    if (!streamClient.connected()) {
      #ifdef DEBUG
      Serial.println(F("Info:TCP Stream disconnected"));
      #endif
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

char data_id[5] = {'D','a','t','a',':'};
void writeData(Stream &getter, uint16_t size) {
  if (size <= 0) return;
  uint32_t start = 0;
  long ttime = millis();
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



void setupOTA() {
  ArduinoOTA.setPassword("energy");
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
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
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
      response += printTime(ts,0);
      updateTime(true);
      uint32_t delta = ts - currentSeconds;
      uint32_t nowMs = millis();
      delta *= 1000;
      delta -= currentMilliseconds;
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
    // Write remaining chunks with tail
    writeChunks(true);
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
    docSend["name"] = config.name;
    docSend["sampling_rate"] = streamConfig.samplingRate;
    /*
    docSend["buffer_size"] = ringBuffer.getSize();
    docSend["psram"] = ringBuffer.inPSRAM();
    docSend["rtc"] = rtc.connected;
    docSend["system_time"] = printCurrentTime();*/
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
 * Stop Sampling will go into ide state, stop the
 * interrupt and if any stream client is connected,
 * will send EOF to the client
 ****************************************************/
void stopSampling() {
  // Stop the 2nd loop (RAM -> PSRAM) if it is running
  if (TaskLoop2 != NULL) vTaskDelete(TaskLoop2);
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
  // createPSRAM_TASK();
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
  // This will reset the sqwv pin
  firstSqwv = true;
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
    Serial.print(F("Info:Sth wrong with mdns"));
    strcpy(name,"powerMeterX");
  }
  // Setting up MDNs with the given Name
  Serial.print(F("Info:MDNS Name: ")); Serial.println(name);
  if (!MDNS.begin(String(name).c_str())) {             // Start the mDNS responder for esp8266.local
    Serial.println(F("Info:Error setting up MDNS responder!"));
  }
  MDNS.addService("elec", "tcp", STANDARD_TCP_STREAM_PORT);
}

TaskHandle_t ntpTaskHandle = NULL;
unsigned long lastTry = millis();
void updateTimeInBG( void * pvParameters ) {
  int32_t delta = millis()-ntpValidMillis;
  getTimeNTP();
  delta = millis()-ntpValidMillis;

  if (ntpEpochSeconds != 0) {
    currentMilliseconds = ntpMilliseconds + delta%1000;
    currentSeconds = ntpEpochSeconds + delta/1000 + currentMilliseconds/1000;
    currentMilliseconds = currentMilliseconds%1000;
    if (rtc.connected and rtc.lost) {
      unsigned long slocal = currentSeconds + LOCATION_OFFSET;
      DateTime ntpNow(year(slocal), month(slocal), day(slocal), hour(slocal), minute(slocal), second(slocal));
      rtc.setTime(ntpNow);
    }
  }
  ntpTaskHandle = NULL;
  vTaskDelete( NULL );
}

void updateTime(bool ntp) {
  int32_t delta = millis()-ntpValidMillis;
  if (ntp) {
    getTimeNTP();
    lastTry = millis();
    delta = millis()-ntpValidMillis;
  }
  if (((delta > 60000) && (millis()-lastTry > 10000))) {
    lastTry = millis();
    if (ntpTaskHandle == NULL) {
      // let it check on second core (not in loop)
      xTaskCreatePinnedToCore(
                        updateTimeInBG,   /* Function to implement the task */
                        "timeUpdateTask", /* Name of the task */
                        10000,      /* Stack size in words */
                        NULL,       /* Task input parameter */
                        0,          /* Priority of the task */
                        &ntpTaskHandle,       /* Task handle. */
                        0);  /* Core where the task should run */
    } else {
      Serial.println("Info: Task Handle not none");
    }
  }
  if (ntpEpochSeconds == 0) return;
  currentMilliseconds = ntpMilliseconds + delta%1000;
  currentSeconds = ntpEpochSeconds + delta/1000 + currentMilliseconds/1000;
  currentMilliseconds = currentMilliseconds%1000;
}

char timeString[32];
char * printCurrentTime() {
  updateTime(false);
  return printTime(currentSeconds, currentMilliseconds);
}

char * printDuration(unsigned long ms) {
  sprintf(timeString, "%02d:%02d:%02d.%03d", (int)hour(ms/1000), (int)minute(ms/1000), (int)second(ms/1000), (int)ms%1000);
  return timeString;
}

char * printTime(unsigned long s, unsigned long ms) {
  unsigned long slocal = s + LOCATION_OFFSET;
  // Serial.print("Date: ");
  // Serial.print()%4d-%02d-%02d %02d:%02d:%02d\n", year(t_unix_date1), month(t_unix_date1), day(t_unix_date1), hour(t_unix_date1), minute(t_unix_date1), second(t_unix_date1));
  sprintf(timeString, "%4d-%02d-%02d %02d:%02d:%02d.%03d", (int)year(slocal), (int)month(slocal), (int)day(slocal), (int)hour(slocal), (int)minute(slocal), (int)second(slocal), (int)ms);
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

// Make sure enough memory is allocated for str
void setInfoString(char * str) {
  int idx = 0;
  idx += sprintf(&str[idx], "\n");
  // Name and firmware
  idx += sprintf(&str[idx], "Info: %s @ firmware: %s/%s", config.name, __DATE__, __TIME__);
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
