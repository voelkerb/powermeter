extern "C" {
  #include "user_interface.h"
}
#include <SPI.h>
#include "STPM.h"
#include <ESP8266WiFi.h>
// #include <esp_wifi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <time.h>
#include <ESP8266mDNS.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <TimeLib.h>
#include <DNSServer.h>
#include "FS.h"

const byte DNS_PORT = 53;
DNSServer dnsServer;

// Serial Speed and DEBUG option
#define SERIAL_SPEED 2000000
#define DEBUG

#define VERSION "0.2_x"

#define MAX_MDNS_LEN 16
#define MDNS_START_ADDRESS 0
// Pins for STPM34 SPI Connection
const int STPM_CS = 15;
const int STPM_SYN = 4;
// Reset pin of STPM
const int STPM_RES = 5;

// Pins for 230V Relay
const int RELAY_PIN = 2;

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
#define BUF_SIZE (4096*10)
uint32_t chunkSize = 256;
// Circular chunked buffer
static uint8_t buffer[BUF_SIZE] = {0};
// Buffer read/write position
uint16_t readPtr = 0;
volatile uint16_t writePtr = 0;

// WLAN configuration
const char* ssid =  "esewifi";
const char* password =  "silkykayak943";

// Open two TCP ports, one for commands and sampling and one for a raw data stream
WiFiServer server(54321);
WiFiServer streamServer(54322);

// TIMER stuff
#define CLOCK 160 // clock frequency in MHz.
#define DEFAUL_SR 4000
unsigned int samplingRate = DEFAUL_SR; // Standard read frequency is 4kHz
// Calculate the number of cycles we have to wait
volatile uint32_t TIMER_CYCLES_FAST = (CLOCK * 1000000) / samplingRate; // Cycles between HW timer inerrupts
volatile uint32_t timer_next;
volatile uint32_t timer_now;

// Sample state machine
#define CMD_SAMPLE "sample"
#define CMD_SWITCH "switch"
#define CMD_STOP "stop"
#define CMD_RESTART "restart"
#define CMD_INFO "info"
#define CMD_MDNS "mdns"
#define CMD_NTP "ntp"

#define STATE_IDLE -1
#define STATE_SAMPLE_USB 0
#define STATE_SAMPLE_TCP 1
#define STATE_STREAM_TCP 2
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

#define COMMAND_MAX_SIZE 128
char command[COMMAND_MAX_SIZE] = {'\0'};
StaticJsonDocument<COMMAND_MAX_SIZE> doc;
String response = "";

char mdnsName[MAX_MDNS_LEN] = {'\0'};


#define LOCATION_OFFSET (2*60*60)
IPAddress timeServerIP; // time.nist.gov NTP server address
// Use local tcp server with hopefully a small ping
const char* ntpServerName = "0.de.pool.ntp.org";
unsigned int localNTPPort = 2390; // local port to listen for UDP packets
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes
byte ntpBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing
// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;
unsigned long ntpValidMillis = 0;
unsigned long ntpEpochSeconds = 0;
unsigned long ntpMilliseconds = 0;

unsigned long currentSeconds = 0;
unsigned long currentMilliseconds = 0;

unsigned long samplingCountdown = 0;
unsigned long startSamplingMillis = 0;
unsigned long samplingDuration = 0;
unsigned long sentSamples = 0;
volatile unsigned long totalSamples = 0;


typedef void (*GetterTypeFunc)(uint8_t channel, float *voltage, float *current, float* active, float* reactive);
GetterTypeFunc getterFunc = NULL;

void setup() {
  // Set Relay pins as output which should be switched on by default
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);

   // Setup serial communication
  Serial.begin(SERIAL_SPEED);

  // Setup STPM 32
  stpm34.init();
  timer0_isr_init();

  // Load the MDNS name from eeprom
  EEPROM.begin(2*MAX_MDNS_LEN);

#ifdef DEBUG
  Serial.printf("Info:Connecting WLAN to %s \r\n", ssid);
#endif
  WiFi.mode(WIFI_STA);
  // esp_wifi_set_ps(WIFI_PS_NONE);
  wifi_set_sleep_type(NONE_SLEEP_T);
  char * name = getMDNS();
  WiFi.hostname(name);
  WiFi.begin(ssid, password);
  long start = millis();
  bool connected = true;
  while (WiFi.status() != WL_CONNECTED) {
    yield();
    // After trying to connect for 5s continue without wifi
    if (millis() - start > 8000) {
      connected = false;
      WiFi.forceSleepBegin();
      Serial.println(F("Info:Connection Failed! Continuing without wifi..."));
      break;
    }
  }
#ifdef DEBUG
  if (connected) {
    Serial.println(F("Info:Connected to WLAN"));
    Serial.print(F("Info:Connected to ")); Serial.println(ssid);
    Serial.print(F("Info:IP Address: ")); Serial.println(WiFi.localIP());
  }
#endif

  initMDNS();
  // Start the TCP server
  server.begin();
  streamServer.begin();

  client.setDefaultNoDelay(false);

  udp.begin(localNTPPort);

  getTimeNTP();
  // modify TTL associated  with the domain name (in seconds)
  // default is 60 seconds
  dnsServer.setTTL(300);
  // set which return code will be used for all other domains (e.g. sending
  // ServerFailure instead of NonExistentDomain will reduce number of queries
  // sent by clients)
  // default is DNSReplyCode::NonExistentDomain
  dnsServer.setErrorReplyCode(DNSReplyCode::ServerFailure);

  // start DNS server for a specific domain name
  String dns = String(name);
  dns += ".local";
  dnsServer.start(DNS_PORT, dns, WiFi.localIP());

  response.reserve(128);
  WiFi.setOutputPower(20.5);

  Serial.println(F("Info:Setup done"));
}


uint16_t getReadWriteDistance() {
  if (writePtr < readPtr) return writePtr + (BUF_SIZE - readPtr);
  else return writePtr - readPtr;
}

long mdnsUpdate = millis();
// the loop routine runs over and over again forever:
void loop() {
  dnsServer.processNextRequest();
  // If we only have 100 ms before sampling should start, wait actively
  if (samplingCountdown != 0 and (samplingCountdown - millis()) < 2000) {
    WiFi.setOutputPower(20.5);
    // Ramp up TCP Speed
    if (next_state == STATE_SAMPLE_TCP) {
      response = "Info:KeepAlive";
      // client.setNoDelay(true);
      while (((long)(samplingCountdown) - millis()) > 100) {
        Serial.println(response);
        client.println(response);
        delay(2);
        yield();
      }
      // client.setNoDelay(false);
    }
    int32_t mydelta = samplingCountdown - millis();
    if (mydelta > 0) delay(mydelta-1);
    #ifdef DEBUG
    state = next_state;
    updateTime(false);
    char * cur = printCurrentTime();
    Serial.print(F("Info:StartSampling @ "));
    Serial.println(cur);
    #endif
    startSampling();
    samplingCountdown = 0;
  }
  if (state == STATE_IDLE and millis() - mdnsUpdate >= 1000) {
    // MDNS.update();
    mdnsUpdate = millis();
    #ifdef DEBUG
    char * cur = printCurrentTime();
    Serial.print(F("Info:"));
    Serial.println(cur);
    #endif
  }
  MDNS.update();
  // Output frequency
  if (freq != 0) {
    long fr = freq;
    freq = 0;
    float frequency = (float)samplingRate/(float)((fr)/1000000.0);
    response = F("Info:");
    response += frequency;
    response += F("Hz");
    char * cur = printCurrentTime();
    response += F("\r\nInfo:");
    response += cur;
    #ifdef DEBUG
    Serial.println(response);
    #endif
    if (client.connected()) {
      client.println(response);
    }
  }

  // Handle serial events
  if (Serial.available()) {
    handleEvent(Serial);
  }


  // Handle client connections
  if (!client.connected()) {
    // Look for people connecting over the server
    client = server.available();
    // We want that because we don't want each buffer part to be send immidiately
    // client.setNoDelay(false);
    // Each buffer should be written directly
    client.setNoDelay(false);
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

void writeChunks(Stream &getter, bool tail) {
  while(getReadWriteDistance() > chunkSize) writeChunk(getter, chunkSize);
  if (tail) writeChunk(getter, getReadWriteDistance());
}

void writeChunks(Stream &getter, bool tail, bool prefix) {
  while(getReadWriteDistance() > chunkSize) writeChunk(getter, chunkSize, prefix);
  if (tail) writeChunk(getter, getReadWriteDistance(), prefix);
}

void writeChunk(Stream &getter, uint16_t size) {
  writeChunk(getter, size, true);
}

uint8_t size_prefix[sizeof(uint16_t)];
void writeChunk(Stream &getter, uint16_t size, bool prefix) {
  if (size <= 0) return;
  uint16_t end = (readPtr+size)%BUF_SIZE;
  if (prefix) {
    getter.print(F("Data:"));
    memcpy(&size_prefix[0], (void*)&size, sizeof(uint16_t));
    getter.write(size_prefix, sizeof(uint16_t));
  }
  if (end > readPtr) {
    getter.write((uint8_t*)&buffer[readPtr], size);
  } else {
    getter.write((uint8_t*)&buffer[readPtr], BUF_SIZE - readPtr);
    getter.write((uint8_t*)&buffer[0], end);
  }
  sentSamples += size/MEASURMENT_BYTES;
  readPtr = end;
  getter.flush();
}

float values[4] = {0};

/****************************************************
 * ISR for sampling
 ****************************************************/
void ICACHE_RAM_ATTR sample_ISR(){
  cli();//stop interrupts
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
  memcpy(&buffer[writePtr], (void*)&values[0], MEASURMENT_BYTES);
  if (writePtr<readPtr and ((writePtr+MEASURMENT_BYTES) % BUF_SIZE) >= readPtr) {
    Serial.println(F("Info:BufferOvf"));
  }
  writePtr = (writePtr+MEASURMENT_BYTES) % BUF_SIZE;


  // Calculate next time to execute and compare to current time
  timer_next = timer_next + TIMER_CYCLES_FAST;
  timer_now = ESP.getCycleCount();
  // Check If next is not in the past and handle overflow
  // Cant we make this better with absolut values?
  if (timer_next > timer_now || (timer_now <= 4294967296 && timer_next <= TIMER_CYCLES_FAST)) {
    timer0_write(timer_next);
  // If the ISR took to long, we indicate the error and start the ISR again immidiately
  } else {
    Serial.println(F("Info:Timer error"));
    timer0_write(ESP.getCycleCount() + 1000);
  }
  sei();
}


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
  DeserializationError error = deserializeJson(doc, command);
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
  response = F("Info:");
  handleJSON();

  getter.println(response);
  #ifdef DEBUG
  if (&getter != &Serial) Serial.println(response);
  #endif
}

void handleJSON() {
  // All commands look like the following:
  // {"cmd":{"name":"commandName", "payload":{<possible data>}}}
  // e.g. mdns

  const char* cmd = doc[F("cmd")]["name"];
  JsonObject root = doc.as<JsonObject>();
  if (cmd == nullptr) {
    response += F("JSON does not match format, syntax: {\"cmd\":{\"name\":<commandName>, \"[payload\":{<possible data>}]}}");
    return;
  }

  /*********************** SAMPLING COMMAND ****************************/
  // e.g. {"cmd":{"name":"sample", "payload":{"type":"Serial", "rate":4000}}}
  if(strcmp(cmd, CMD_SAMPLE) == 0) {
    // For sampling we need type payload and rate payload
    const char* typeC = root["cmd"]["payload"]["type"];
    const char* measuresC = root["cmd"]["payload"]["measures"];
    int rate = doc["cmd"]["payload"]["rate"].as<int>();
    unsigned long ts = doc["cmd"]["payload"]["time"].as<unsigned long>();

    if (typeC == nullptr or rate == 0) {
      response += F("Not a valid \"sample\" command");
      if (typeC == nullptr) response += F(", \"type\" missing");
      if (rate == 0) response += F(", \"rate\" missing");
      return;
    }
    if (rate > 8000 || rate <= 0) {
      response += F("SamplingRate could not be set to ");
      response += rate;
      return;
    }
    if (strcmp(measuresC, "v,i") == 0) {
      measures = STATE_VI;
      MEASURMENT_BYTES = 8;
    } else if (strcmp(measuresC, "p,q") == 0) {
      measures = STATE_PQ;
      MEASURMENT_BYTES = 8;
    } else if (strcmp(measuresC, "v,i,p,q") == 0) {
      measures = STATE_VIPQ;
      MEASURMENT_BYTES = 16;
    } else {
      response += F("Unsupported measures: ");
      response += measuresC;
      return;
    }
    if (strcmp(typeC, "Serial") == 0) {
      next_state = STATE_SAMPLE_USB;
    } else if (strcmp(typeC, "TCP") == 0) {
      next_state = STATE_SAMPLE_TCP;
    } else if (strcmp(typeC, "FFMPEG") == 0) {
      bool success = tryConnectStream();
      if (success) {
        next_state = STATE_STREAM_TCP;
        response += F("Connected to TCP stream");
      } else {
        response += F("Could not connect to TCP stream");
        return;
      }
    } else {
      response += F("Unsupported sampling type: ");
      response += typeC;
      return;
    }
    // Set global sampling variable
    samplingRate = rate;

    TIMER_CYCLES_FAST = (CLOCK * 1000000) / samplingRate; // Cycles between HW timer inerrupts
    chunkSize = min(max(MEASURMENT_BYTES, int(float(0.1*(float)(samplingRate*MEASURMENT_BYTES)))), BUF_SIZE);

    response += F("SamplingRate set to ");
    response += samplingRate;
    response += F("Hz\nInfo:BufferFlushSize: ");
    response += chunkSize;
    response += F("\nInfo:TIMER_CYCLES_FAST: ");
    response += TIMER_CYCLES_FAST;
    response += F("\nInfo:Data pushed over ");
    response += typeC;


    if (ts != 0) {
      response += F("\nInfo:Start Sampling at: ");
      response += printTime(ts,0);
      updateTime(true);
      uint32_t delta = ts - currentSeconds;
      uint32_t nowMs = millis();
      delta *= 1000;
      delta -= currentMilliseconds;
      if (delta > 20000 or delta < 500) {
        response += F("\nInfo:Cannot start sampling in: ");
        response += delta;
        response += F("ms");
        samplingCountdown = 0;
      } else {
        response += F("\nInfo:Start sampling in: ");
        response += delta;
        response += F("ms");
        samplingCountdown = nowMs + delta;
      }
      return;
    }
    state = next_state;
    startSampling();
  }

  /*********************** SWITCHING COMMAND ****************************/
  else if (strcmp(cmd, CMD_SWITCH) == 0) {
    // For switching we need value payload
    const char* payloadValue = doc["cmd"]["payload"]["value"];
    if (payloadValue == nullptr) {
      response += F("Not a valid \"switch\" command");
      return;
    }
    bool value = doc["cmd"]["payload"]["value"].as<bool>();
    response += F("Switching: ");
    response += value ? F("On") : F("Off");
    switchRelay(RELAY_PIN, true);
  }

  /*********************** STOP COMMAND ****************************/
  else if (strcmp(cmd, CMD_STOP) == 0) {
    // State is reset in stopSampling
    next_state = state;
    stopSampling();
    // Send remaining
    if (next_state == STATE_SAMPLE_USB) writeChunks(Serial, true);
    else if (next_state == STATE_SAMPLE_TCP) writeChunks(client, true);
    response += F("Sampled for ");
    response += printDuration(samplingDuration);
    response += F("\nInfo:Samples ");
    response += totalSamples;
    response += F("\nInfo:Sent Samples ");
    response += sentSamples;
    response += F("\nInfo:Avg SR: ");
    response += totalSamples/(samplingDuration/1000.0);
    response += F("Hz");
    response += F("\nInfo:stop");
  }

  /*********************** RESTART COMMAND ****************************/
  else if (strcmp(cmd, CMD_RESTART) == 0) {
    stopSampling();
    turnInterrupt(false);
    ESP.restart();
  }

  /*********************** INFO COMMAND ****************************/
  else if (strcmp(cmd, CMD_INFO) == 0) {
    response += F("WIFI powermeter. Version: ");
    response += VERSION;
    response += F("\nInfo:Buffer Size:");
    response += BUF_SIZE;
  }

  /*********************** MDNS COMMAND ****************************/
  // e.g. {"cmd":{"name":"mdns", "payload":{"name":"newName"}}}
  else if (strcmp(cmd, CMD_MDNS) == 0) {
    const char* newName = doc["cmd"]["payload"]["name"];
    if (newName == nullptr) {
      response += F("MDNS name required in payload with key name");
      return;
    }
    if (strlen(newName) < MAX_MDNS_LEN) {
      writeMDNS((char * )newName);
    } else {
      Serial.println(F("Not allowed"));
      response += F("MDNS name too long, only string of size ");
      response += MAX_MDNS_LEN;
      response += F(" allowed");
      return;
    }
    char * name = getMDNS();
    response = F("Info:Set MDNS name to: ");
    response += name;
  }

  /*********************** MDNS COMMAND ****************************/
  // e.g. {"cmd":{"name":"ntp"}}
  else if (strcmp(cmd, CMD_NTP) == 0) {
    if (getTimeNTP()) {
      response += F("Time synced");
    } else {
      response += F("Error syncing time");
    }
    char * timeStr = printTime(ntpEpochSeconds, ntpMilliseconds);
    Serial.println(timeStr);
    delay(1000);
    timeStr = printCurrentTime();
    Serial.println(timeStr);
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
  state = STATE_IDLE;
  turnInterrupt(false);
  samplingDuration = millis() - startSamplingMillis;
  if (streamClient && streamClient.connected()) streamClient.stop();
}

/****************************************************
 * Start Sampling requires to start the interrupt and
 * to calculate the chunkSize size depending on
 * the samplingrate currently set. Furthermore,
 * all buffer indices are reset to the default values.
 ****************************************************/
void startSampling() {
  switchRelay(RELAY_PIN, true);
  counter = 0;
  sentSamples = 0;
  totalSamples = 0;
  TIMER_CYCLES_FAST = (CLOCK * 1000000) / samplingRate; // Cycles between HW timer inerrupts
  chunkSize = min(max(MEASURMENT_BYTES, int(float(0.1*(float)(samplingRate*MEASURMENT_BYTES)))), BUF_SIZE/2);
  writePtr = 0;
  readPtr = 0;
  startSamplingMillis = millis();
  samplingDuration = 0;
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
 * Switch relay to open or closed
 ****************************************************/
inline void switchRelay(int number, bool value) {
  digitalWrite(number, value);
}


/****************************************************
 * Init the MDNs name from eeprom, only the number ist
 * stored in the eeprom, construct using prefix.
 ****************************************************/
void initMDNS() {
  char * name = getMDNS();
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

char * getMDNS() {
  uint16_t address = MDNS_START_ADDRESS;
  uint8_t chars = 0;
  EEPROM.get(address, chars);
  address += sizeof(chars);
  if (chars < MAX_MDNS_LEN) {
    EEPROM.get(address, mdnsName);
  }
  return mdnsName;
}

void writeMDNS(char * newName) {
  uint16_t address = MDNS_START_ADDRESS;
  uint8_t chars = strlen(newName);
  EEPROM.put(address, chars);
  address += sizeof(chars);
  for (uint8_t i = 0; i < chars; i++) EEPROM.put(address+i, newName[i]);
  EEPROM.put(address+chars, '\0');
  EEPROM.commit();
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
  udp.beginPacket(timeServerIP, 123); //NTP requests are to port 123
  udp.write(ntpBuffer, NTP_PACKET_SIZE);
  udp.endPacket();// should flush
  // Wait for packet to arrive with timeout of 2 seconds
  int cb = udp.parsePacket();
  while(!cb) {
    if (start + 2000 < millis()) break;
    cb = udp.parsePacket();
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
  udp.read(ntpBuffer, NTP_PACKET_SIZE);
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
