#include <SPI.h>
#include "STPM_mod.h"
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <time.h>

// Serial Speed and DEBUG option
#define SERIAL_SPEED 115200
#define DEBUG
//#define SERIAL_OUTPUT
#define VERSION 0.8

// Commands
#define CMD_SWITCH_ON 's'
#define CMD_SWITCH_OFF 'o'
#define CMD_SWITCH_FOR 'f'
#define CMD_START_STREAM_AT 't'
#define CMD_START_STREAM 'm'
#define CMD_STOP_STREAM 'n'
#define CMD_START_SAMPLE 'a'
#define CMD_STOP_SAMPLE 'p'
#define CMD_RESET '?'
#define CMD_RESTART 'r'

// Pins for STPM34 SPI Connection
const int STPM_CS = 15;
const int STPM_SYN = 4;
// Reset pin of STPM
const int STPM_RES = 5;

// Pins for 230V Relay
const int RELAY_PIN = 2;

time_t currentTime;
time_t currentTimeMs;
time_t sampleTime;
time_t sampleTimeMs;

// STPM Object
STPM stpm34(STPM_RES, STPM_CS, STPM_SYN);

// Buffering stuff
#define MEASURMENT_BYTES 16 //(16+2)
volatile float values[5] = {0};
char buffer[MEASURMENT_BYTES] = {0};
char dummy[MEASURMENT_BYTES] = "Alle meine En\n\r";
#define NUM_BUF 5
#define BUF_SIZE MEASURMENT_BYTES*128

char buffer2[NUM_BUF][BUF_SIZE] = {};
volatile int whichBufferRead = -1;
volatile uint8_t whichBufferWrite = 0;
volatile uint16_t inBuffer = 0;

// WLAN configuration
const char* ssid =  "esewifi";
const char* password =  "silkykayak943";
WiFiServer server(54321);
WiFiServer streamServer(54322);

// TIMER stuff
#define CLOCK 160 // clock frequency in MHz.
#define READ_FREQ 4000 // Sample frequency in Hz Controlled by the HW timer
const int TIMER_CYCLES_FAST = (CLOCK * 1000000) / READ_FREQ; // Cycles between HW timer inerrupts
volatile unsigned long next;
volatile unsigned long now;

// Sample for duration stuff
unsigned long sampleCntr = 0;
unsigned long sampleCnt = 0;
unsigned long preSampleCnt = 0;
unsigned long postSampleCnt = 0;


// For sample state machine
#define STATE_IDLE -1
#define STATE_SAMPLE 0
#define STATE_PRE 1
#define STATE_ON 2
#define STATE_POST 3
#define STATE_STREAM 4
volatile int state = STATE_IDLE;
volatile unsigned long i = 0;
volatile unsigned long j = 0;
volatile unsigned long k = 0;
// Stuff for frequency calculation
volatile long freqCalcStart;
volatile long freqCalcNow;
volatile long freq = 0;

IPAddress timeServerIP; // time.nist.gov NTP server address
const char* ntpServerName = "0.de.pool.ntp.org";
unsigned int localNTPPort = 2390; // local port to listen for UDP packets
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes
byte ntpBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing
// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

void setup() {
   // Setup serial communication
  Serial.begin(SERIAL_SPEED);
  #ifdef DEBUG
    delay(300);
    Serial.print(F("Info:WIFI powermeter starting up. Version: "));

    Serial.println(VERSION);
    Serial.print("BUF_SIZE:");
    Serial.println(BUF_SIZE);
  #endif

  // Set Relay pins as output which should be switched off by default
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  //buffer[16] = '\r';
  //buffer[17] = '\n';
  // Clear the buffer at beginning
  for (int i = 0; i < NUM_BUF; i++) {
    for (int j = 0; j < BUF_SIZE; j+=2) {
      buffer2[i][j] = '\r';
      buffer2[i][j+1] = '\n';
    }
  }

  // Setup STPM 32
  #ifdef DEBUG
    Serial.println(F("Info:Connecting to STPM"));
  #endif
  stpm34.init();
  #ifdef DEBUG

    stpm34.readVoltageAndCurrent(1, (float*) &values[0], (float*) &values[1]);
    Serial.print(F("Info:Voltage: "));
    Serial.println(values[0]);
    Serial.print(F("Info:Current: "));
    Serial.println(values[1]);
    Serial.println(F("Info:Connected to STPM"));
  #endif

  timer0_isr_init();

  // Setup WLAN
  #ifdef DEBUG
    Serial.printf("Info:Connecting WLAN to %s ", ssid);
  #endif
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    #ifdef DEBUG
      delay(500);
      Serial.print(F("."));
    #endif
  }
  #ifdef DEBUG
    Serial.println(F(" connected to WLAN"));
    Serial.print("Info:Connected to "); Serial.println(ssid);
    Serial.print("Info:IP Address: "); Serial.println(WiFi.localIP());
  #endif
  // Start the TCP server
  server.begin();
  streamServer.begin();

  udp.begin(localNTPPort);
  // configTime(0, 0, "0.de.pool.ntp.org");
  // configTime(2 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  Serial.println(F("Info:Setup done"));
}

WiFiClient client;
WiFiClient streamClient;

// the loop routine runs over and over again forever:
void loop() {

  // Output frequency
  if (freq != 0) {
    noInterrupts();
    long fr = freq;
    freq = 0;
    interrupts();
    float frequency = (float)READ_FREQ/(float)((fr)/1000000.0);
    String response = "Info:";
    response += frequency;
    response += "Hz";
    #ifdef DEBUG
    Serial.println(response);
    #endif
    if (client.connected()) {
      client.println(response);
    }
  }
  // Handle serial events
  if (Serial.available()) {
    serialEvent();
  }


  if (!client.connected()) {
    // Look for people connecting over the server
    client = server.available();
    // We want that because we don't want each buffer part to be send immidiately
    client.setNoDelay(false);
  } else {

    // read data from the connected client
    if (client.available() > 0) {
      tcpEvent();

    }
    // if we are sampling, then pass the data over the to the client
    if (state == STATE_SAMPLE) {
      noInterrupts();
      int whichBuffer = whichBufferRead;
      whichBufferRead = -1;
      interrupts();
      if (whichBuffer != -1) {
        for (int i = 0; i < BUF_SIZE; i += MEASURMENT_BYTES) {
          client.write(&buffer2[whichBuffer][i], MEASURMENT_BYTES);
          client.write("\r\n", 2);
        }
      }
    // If we are streaming then pass data over connected users of the streaming server
    } else if (state == STATE_STREAM) {
      if (streamClient.connected()) {
        noInterrupts();
        int whichBuffer = whichBufferRead;
        whichBufferRead = -1;
        interrupts();
        if (whichBuffer != -1) {
          // Leave out \r\n over this connection
          streamClient.write(&buffer2[whichBuffer][0], BUF_SIZE);
        }
      }
    }
  }


  if (!streamClient.connected()) {
    // Look for people connecting over the streaming server
    streamClient = streamServer.available();
    // TODO: Is this required?
    streamClient.setNoDelay(true);
  }

  yield();
}


//timer interrupt 4kHz
void ICACHE_RAM_ATTR sample_ISR(){
  //next = ESP.getCycleCount() + TIMER_CYCLES_FAST - 135; // 150 is ca. the time it takes to go inside the service routine
  next = next + TIMER_CYCLES_FAST; // 150 is ca. the time it takes to go inside the service routine
  //ESP.wdtFeed();
  // Output second tick
  i++;
  if (i == READ_FREQ) {
    freqCalcNow = micros();
    freq = freqCalcNow-freqCalcStart;
    freqCalcStart = freqCalcNow;
    i = 0;
  }

  stpm34.readVoltageAndCurrent(1, (float*) &values[0], (float*) &values[1]);
  values[2] = stpm34.readActivePower(1);
  values[3] = stpm34.readReactivePower(1);
  memcpy(&buffer2[whichBufferWrite][inBuffer], (void*)&values[0], MEASURMENT_BYTES);
  inBuffer += MEASURMENT_BYTES;
  if (inBuffer > BUF_SIZE-MEASURMENT_BYTES) {
    inBuffer = 0;
    whichBufferRead = whichBufferWrite;
    whichBufferWrite = (whichBufferWrite + 1) % NUM_BUF;
  }

  #ifdef SERIAL_OUTPUT
  Serial.print(values[0]);
  Serial.print(", ");
  Serial.print(values[1]);
  Serial.println("");
  #endif
  now = ESP.getCycleCount();
  if (next > now) {
    timer0_write(next);
  } else {
    timer0_write(now + 1000);
  }
}


void serialEvent() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == ' ' || c == '\n' || c == '\r') return;
  String response = handleCommand(c, Serial);
  Serial.println(response);
}

void tcpEvent() {
  if (!client.available()) return;
  char c = client.read();
  if (c == ' ' || c == '\n' || c == '\r') return;
  Serial.println(c);
  String response = handleCommand(c, client);
  Serial.println(response);
  client.println(response);
  //client.write(buffer, 18);
}


// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address) {
  Serial.println("Sending NTP packet...");
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
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(ntpBuffer, NTP_PACKET_SIZE);
  udp.endPacket();// should flush
}

bool getTime() {
  WiFi.hostByName(ntpServerName, timeServerIP);
  sendNTPpacket(timeServerIP);
  long start = millis();
  // Wait for packet to arrive
  delay(1000);
  int cb = udp.parsePacket();
  if (!cb) {
    Serial.println("No NTP response yet");
    return false;
  } else {
    Serial.print("received NTP packet, length=");
    Serial.println(cb);
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
    currentTimeMs = mssec;
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - 2208988800UL;
    // Add the time we waited
    long deviation = millis() - start;
    currentTime = epoch + deviation/1000;
    currentTimeMs = mssec + deviation%1000;
    return true;
  }
  // Serial.print("Waiting for time");
  // currentTime = 0;
  // while (!currentTime) {
  //   currentTime = time(nullptr);
  //   Serial.print(".");
  //   delay(100);
  // }
  // Serial.println("");
  // Serial.println(ctime(&currentTime));
}

String handleCommand(char c, Stream &getter) {
  String response = "";
  switch (c) {
    case CMD_RESET:
      turnInterrupt(false);
      switchRelay(RELAY_PIN, false);
      response = "Info:Setup done";
      break;
    case CMD_RESTART:
      // This will only work after at least one manual reset of the module
      turnInterrupt(false);
      ESP.restart();
      break;
    case CMD_SWITCH_ON:
      switchRelay(RELAY_PIN, true);
      break;
    case CMD_SWITCH_OFF:
      switchRelay(RELAY_PIN, false);
      break;
    case CMD_START_STREAM:
      state = STATE_STREAM;
      turnInterrupt(true);
      break;
    case CMD_START_STREAM_AT: {
      getTime();
      long start = millis();

      Serial.print("Current time: ");
      Serial.print(currentTime);
      Serial.print(".");
      Serial.println(currentTimeMs);
      Serial.println(ctime(&currentTime));
      sampleTime = parseTime(getter);
      sampleTimeMs = parseTime(getter);
      while (getter.available()) getter.read();
      while (sampleTimeMs > 1000) sampleTimeMs /= 10;

      Serial.print("Start to sample at: ");
      Serial.print(sampleTime);
      Serial.print(".");
      Serial.println(sampleTimeMs);
      Serial.println(ctime(&sampleTime));

      time_t delta = sampleTime-currentTime;

      int deltaMs = sampleTimeMs-currentTimeMs;

      time_t deltaInMs = delta*1000 + deltaMs;

      Serial.print("We have to wait: ");
      Serial.print(deltaInMs);
      Serial.println(" ms");
      if (deltaInMs > 10000) {
        Serial.println("Sampling has to start after at least 10s");
      } else if (deltaInMs < 0) {
        Serial.println("Time is in the past");
      } else if (millis() - start > deltaInMs) {
        Serial.println("Give us at least 2s to prepare");
      } else {
        long ctdTimer = millis();
        long ctd = (deltaInMs - (millis() - start)) / 1000;
        while(millis() - start < (long)deltaInMs) {
          if ((millis() - ctdTimer) > 1000) {
            Serial.println(ctd);
            getter.println(ctd);
            ctd -= 1;
            ctdTimer = millis();
          }
          yield();
        }
        Serial.println("Start Sampling");
        state = STATE_STREAM;
        turnInterrupt(true);
      }
      break;
    }
    case CMD_STOP_STREAM:
      turnInterrupt(false);
      state = STATE_IDLE;
      break;
    case CMD_START_SAMPLE:
      state = STATE_SAMPLE;
      turnInterrupt(true);
      break;
    case CMD_STOP_SAMPLE:
      turnInterrupt(false);
      state = STATE_IDLE;
      break;
    case CMD_SWITCH_FOR:
      delay(10);
      turnInterrupt(false);
      state = STATE_PRE;
      preSampleCnt = parseValue(getter);
      sampleCnt = parseValue(getter);
      postSampleCnt = parseValue(getter);
      response = "Info:Switching for: ";
      response += preSampleCnt;
      response += ", ";
      response += sampleCnt;
      response += ", ";
      response += postSampleCnt;
      response += "";
      turnInterrupt(true);
      break;
  }
  return response;
}

void turnInterrupt(bool on) {
  cli();//stop interrupts
  if (on) {
    timer0_attachInterrupt(sample_ISR);
    next = ESP.getCycleCount() + TIMER_CYCLES_FAST;
    timer0_write(next);
  } else {
    timer0_detachInterrupt();
  }
  sei();
}

time_t parseTime(Stream &getter) {
  time_t value = 0;
  long timeOut = millis();
  while(millis() - timeOut < 50) {
    if (getter.available()) {
      int i = getter.read() - '0';
      if (i < 0 || i > 9) break;
      else {
        value = value*10 + i;
      }
      timeOut = millis();
    }
  }
  return value;
}



/****************************************************
 * Parse a long value given any stream object.
 * Checks stream.available and returns after timeout
 ****************************************************/
long parseValue(Stream &getter) {
  long value = 0;
  long timeOut = millis();
  while(millis() - timeOut < 50) {
    if (getter.available()) {
      int i = getter.read() - '0';
      if (i < 0 || i > 9) break;
      else {
        value = value*10 + i;
      }
      timeOut = millis();
    }
  }
  return value;
}


inline void switchRelay(int number, bool value) {
  digitalWrite(number, value);
}
