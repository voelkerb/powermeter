#include <SPI.h>
#include "STPM_mod.h"
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <time.h>
#include <ESP8266mDNS.h>
#include <EEPROM.h>

// Serial Speed and DEBUG option
#define SERIAL_SPEED 2000000
#define DEBUG
//#define SERIAL_OUTPUT
#define VERSION 0.9


#define MDNS_PREFIX "powermeter"
char MDNSName[4] = "X\x0";

// Commands
#define CMD_SWITCH_ON 's'
#define CMD_SWITCH_OFF 'o'
#define CMD_SWITCH_FOR 'f'
#define CMD_SET_SAMPLINGRATE 'q'
#define CMD_START_STREAM_AT 't'
#define CMD_START_STREAM 'm'
#define CMD_START_SAMPLE 'a'
#define CMD_STOP_SAMPLE 'p'
#define CMD_RESET '?'
#define CMD_RESTART 'r'
#define CMD_MDNS 'n'

// Pins for STPM34 SPI Connection
const int STPM_CS = 15;
const int STPM_SYN = 4;
// Reset pin of STPM
const int STPM_RES = 5;

// Pins for 230V Relay
const int RELAY_PIN = 2;

// Variables for NTP timed streaming
time_t currentTime;
time_t currentTimeMs;
time_t sampleTime;
time_t sampleTimeMs;

// STPM Object
STPM stpm34(STPM_RES, STPM_CS, STPM_SYN);

// Buffering stuff
// Number of bytes for one measurement
#define MEASURMENT_BYTES 16 //(16+2)
volatile float values[5] = {0};
#define NUM_BUF 10
// Chunk sizes of data to send
#define BUF_SIZE MEASURMENT_BYTES*256
// Circular chunked buffer
char buffer[NUM_BUF][BUF_SIZE] = {};
// Which buffer to read
volatile int whichBufferRead = 0;
// Which buffer to write
volatile uint8_t whichBufferWrite = 0;
// The position inside a bufferchunk to write to
volatile uint16_t inBuffer = 0;
// For e.g. 10Hz. Reading we want the chunk size to be smaller
// s.t. we do not have to wait e.g. 25 seconds to become data
// Calculation is down in startSampling function
volatile int bufferFlushSize = BUF_SIZE;

// WLAN configuration
const char* ssid =  "esewifi";
const char* password =  "silkykayak943";
// Open two TCP ports, one for commands and sampling and one for a raw data stream
WiFiServer server(54321);
WiFiServer streamServer(54322);

// TIMER stuff
#define CLOCK 160 // clock frequency in MHz.
unsigned int samplingRate = 4000; // Standard read frequency is 4kHz
// Calculate the number of cycles we have to wait
volatile uint32_t TIMER_CYCLES_FAST = (CLOCK * 1000000) / samplingRate; // Cycles between HW timer inerrupts
volatile uint32_t next;
volatile uint32_t now;

// Sample for duration stuff
unsigned long sampleCntr = 0;
unsigned long sampleCnt = 0;
unsigned long preSampleCnt = 0;
unsigned long postSampleCnt = 0;

// Sample state machine
#define STATE_IDLE -1
#define STATE_SAMPLE 0
#define STATE_PRE 1
#define STATE_ON 2
#define STATE_POST 3
#define STATE_STREAM 4
#define STATE_SAMPLE_USB 5
volatile int state = STATE_IDLE;

volatile unsigned long i = 0;
volatile unsigned long j = 0;
volatile unsigned long k = 0;
// Stuff for frequency calculation
volatile long freqCalcStart;
volatile long freqCalcNow;
volatile long freq = 0;

IPAddress timeServerIP; // time.nist.gov NTP server address
const char* lineSeperator = "\r\n";
// Use local tcp server with hopefully a small ping
const char* ntpServerName = "0.de.pool.ntp.org";
unsigned int localNTPPort = 2390; // local port to listen for UDP packets
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes
byte ntpBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing
// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

WiFiClient client;
WiFiClient streamClient;

void setup() {

   // Setup serial communication
  Serial.begin(SERIAL_SPEED);

#ifdef DEBUG
  delay(300);
  Serial.print(F("Info:WIFI powermeter starting up. Version: "));
  Serial.println(VERSION);
  Serial.print("Info:Buffer Size:");
  Serial.println(BUF_SIZE);
#endif

  // Set Relay pins as output which should be switched on by default
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);

  // Clear the buffer at beginning
  for (int i = 0; i < NUM_BUF; i++) {
    for (int j = 0; j < BUF_SIZE; j+=2) {
      buffer[i][j] = '\r';
      buffer[i][j+1] = '\n';
    }
  }

#ifdef DEBUG
  Serial.println(F("Info:Connecting to STPM"));
#endif
  // Setup STPM 32
  stpm34.init();
#ifdef DEBUG
  Serial.println(F("Info:Connected to STPM"));
#endif

  timer0_isr_init();

#ifdef DEBUG
  Serial.printf("Info:Connecting WLAN to %s \r\n", ssid);
#endif
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    yield();
    // After trying to connect for 5s continue without wifi
    if (millis() - start > 8000) {
      WiFi.forceSleepBegin();
      Serial.println(F("Info:Connection Failed! Continuing without wifi..."));
      break;
    }
  }
#ifdef DEBUG
  Serial.println(F("Info:Connected to WLAN"));
  Serial.print(F("Info:Connected to ")); Serial.println(ssid);
  Serial.print(F("Info:IP Address: ")); Serial.println(WiFi.localIP());
#endif

  // Load the MDNS name from eeprom
  EEPROM.begin(16);

  initMDNS();
  // Start the TCP server
  server.begin();
  streamServer.begin();

  udp.begin(localNTPPort);
  // configTime(0, 0, "0.de.pool.ntp.org");
  // configTime(2 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  Serial.println(F("Info:Setup done"));
}



// the loop routine runs over and over again forever:
void loop() {

  // Output frequency
  if (freq != 0) {
    noInterrupts();
    long fr = freq;
    freq = 0;
    interrupts();
    float frequency = (float)samplingRate/(float)((fr)/1000000.0);
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

  // Handle client connections
  if (!client.connected()) {
    // Look for people connecting over the server
    client = server.available();
    // We want that because we don't want each buffer part to be send immidiately
    client.setNoDelay(false);
  } else {
    // Handle client events
    if (client.available() > 0) {
      tcpEvent();
    }
  }

  // if we are sampling, then pass the data over to the client
  if (state == STATE_SAMPLE && client.connected()) {
    // TODO: This is not nice and might cause a crash since the buffer is shared
    while (whichBufferRead != whichBufferWrite) {
      for (int i = 0; i < bufferFlushSize; i += MEASURMENT_BYTES) {
        client.write((uint8_t*)&buffer[whichBufferRead][i], MEASURMENT_BYTES);
        client.write("\r\n", 2);
      }
      whichBufferRead = (whichBufferRead + 1) % NUM_BUF;
    }
  // If we are streaming then pass data over connected users of the streaming server
  } else if (state == STATE_STREAM) {
    if (streamClient.connected()) {
      // TODO: This is not nice and might cause a crash since the buffer is shared
      while (whichBufferRead != whichBufferWrite) {
        // Leave out \r\n over this connection
        streamClient.write(&buffer[whichBufferRead][0], bufferFlushSize);
        whichBufferRead = (whichBufferRead + 1) % NUM_BUF;
      }
    // Client is not connected yet, rush and try to connect
    } else {
      tryConnectStream();
    }
    // NOTE: Try to calculate buffer distance
    // whichBufferRead = 16;
    // if (whichBufferRead != whichBufferWrite) {
    //   Serial.print(whichBufferRead);
    //   Serial.print(" : ");
    //   Serial.println(whichBufferWrite);
    //   Serial.print("Distance: ");
    //   Serial.println((whichBufferWrite + NUM_BUF - whichBufferRead)%NUM_BUF);
    // }
  // USB Sampling happens here
  } else if (state == STATE_SAMPLE_USB) {
    while (whichBufferRead != whichBufferWrite) {
      for (int i = 0; i < bufferFlushSize; i += MEASURMENT_BYTES) {
        Serial.write((uint8_t*)&buffer[whichBufferRead][i], MEASURMENT_BYTES);
        Serial.print("\r\n");
      }
      whichBufferRead = (whichBufferRead + 1) % NUM_BUF;
    }
  }

  yield();
}


/****************************************************
 * Init the MDNs name from eeprom, only the number ist
 * stored in the eeprom, construct using prefix.
 ****************************************************/
void initMDNS() {
  EEPROM.get(0, MDNSName);
  // Setting up MDNs with the given Name
  String MDNSstr = MDNS_PREFIX; MDNSstr += MDNSName;
  Serial.print(F("Info:MDNS Name: ")); Serial.println(MDNSstr);
  if (!MDNS.begin(MDNSstr.c_str())) {             // Start the mDNS responder for esp8266.local
    Serial.println(F("Info:Error setting up MDNS responder!"));
  }
}

/****************************************************
 * Try to connect the streaming tcp client.
 ****************************************************/
void tryConnectStream() {
  if (!streamClient.connected()) {
    // Look for people connecting over the streaming server
    streamClient = streamServer.available();
    if (streamClient && streamClient.connected()) {
      Serial.println("Info:Stream Connected");
      // Write null byte here
      streamClient.setNoDelay(true);
      delay(100);
      uint8_t zeros[MEASURMENT_BYTES] = {0};
      for (int i = 0; i < 1000; i++) streamClient.write(&zeros[0], MEASURMENT_BYTES);
      Serial.println("Info:Data Written");
      // TODO: Is this required?
    }
  }
}

/****************************************************
 * ISR for sampling
 ****************************************************/
void ICACHE_RAM_ATTR sample_ISR(){
  // Vaiables for frequency count
  i++;
  if (i == samplingRate) {
    freqCalcNow = micros();
    freq = freqCalcNow-freqCalcStart;
    freqCalcStart = freqCalcNow;
    i = 0;
  }
  // stpm34.readVoltageAndCurrent(1, (float*) &values[0], (float*) &values[1]);
  // values[2] = stpm34.readActivePower(1);
  // values[3] = stpm34.readReactivePower(1);
  stpm34.readAll(1, (float*) &values[0], (float*) &values[1], (float*) &values[2], (float*) &values[3]);
  memcpy(&buffer[whichBufferWrite][inBuffer], (void*)&values[0], MEASURMENT_BYTES);
  inBuffer += MEASURMENT_BYTES;
  // Handle end of chunk size
  if (inBuffer > bufferFlushSize-MEASURMENT_BYTES) {
    inBuffer = 0;
    whichBufferWrite = (whichBufferWrite + 1) % NUM_BUF;
    // If the next buffer has not sent to the client, indicate that data is missing
    if (whichBufferWrite == whichBufferRead) Serial.println("Info:Buffer ERROR, Head meets Tail...");
  }

  // Calculate next time to execute and compare to current time
  next = next + TIMER_CYCLES_FAST;
  now = ESP.getCycleCount();
  // Check If next is not in the past and handle overflow
  // Cant we make this better with absolut values?
  if (next > now || (now <= 4294967296 && next <= TIMER_CYCLES_FAST)) {
    timer0_write(next);
  // If the ISR took to long, we indicate the error and start the ISR again immidiately
  } else {
    Serial.println("Info:Timer error");
    timer0_write(ESP.getCycleCount() + 1000);
  }
}

/****************************************************
 * A serial event occured
 ****************************************************/
void serialEvent() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == ' ' || c == '\n' || c == '\r') return;
  String response = handleCommand(c, Serial);
  if (response.length() > 5) Serial.println(response);
}

/****************************************************
 * A TCP event occured
 ****************************************************/
void tcpEvent() {
  if (!client.available()) return;
  char c = client.read();
  if (c == ' ' || c == '\n' || c == '\r') return;
  Serial.print("Info:");
  Serial.println(c);
  String response = handleCommand(c, client);
  if (response.length() > 5) {
    Serial.println(response);
    client.println(response);
  }
}


/****************************************************
 * send an NTP request to the time server at the given
 * address
 ****************************************************/
unsigned long sendNTPpacket(IPAddress& address) {
  Serial.println("Info:Sending NTP packet...");
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

/****************************************************
 * Get the current time using ntp synchronisation.
 * Will return true on success and false on fail.
 * TODO: What about the time it takes for the packet
 * to arrive?
 ****************************************************/
bool getTime() {
  WiFi.hostByName(ntpServerName, timeServerIP);
  sendNTPpacket(timeServerIP);
  long start = millis();
  // Wait for packet to arrive with timeout of 2 seconds
  int cb = udp.parsePacket();
  while(!cb) {
    if (start + 2000 < millis()) break;
    cb = udp.parsePacket();
    delay(10);
  }
  if (!cb) {
    Serial.println("Info:No NTP response yet");
    return false;
  } else {
    Serial.print("Info:Received NTP packet, length=");
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
}

/****************************************************
 * Handle a given command from a given stream.
 ****************************************************/
String handleCommand(char c, Stream &getter) {
  String response = "";
  switch (c) {
    case CMD_RESET:
      turnInterrupt(false);
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
    case CMD_SET_SAMPLINGRATE: {
      int rate = parseValue(getter);
      if (rate <= 8000 && rate > 0) samplingRate = rate;
      else {
        response += "Info:samplingRate could not be set to ";
        response += rate;
        break;
      }
      Serial.print("Info:SamplingRate set to ");
      Serial.println(samplingRate);
      Serial.print("Info:BufferFlushSize: ");
      Serial.println(bufferFlushSize);
      break;
    }
    case CMD_START_STREAM: {
      state = STATE_STREAM;
      tryConnectStream();
      startSampling();
      break;
    }
    case CMD_START_STREAM_AT: {
      bool success = getTime();
      long start = millis();
      Serial.print("Info:Current time: ");
      Serial.print(currentTime);
      Serial.print(".");
      Serial.println(currentTimeMs);
      Serial.print("Info:");
      Serial.println(ctime(&currentTime));
      sampleTime = parseTime(getter);
      sampleTimeMs = parseTime(getter);
      while (getter.available()) getter.read();
      // If we have no succes, we stop
      if (!success) {
        response += "Info:Could not connect to NTP Server";
        break;
      }
      // Sample time should be in milliseconds
      while (sampleTimeMs > 1000) sampleTimeMs /= 10;

      Serial.print("Info:Start to sample at: ");
      Serial.print(sampleTime);
      Serial.print(".");
      Serial.println(sampleTimeMs);
      Serial.print("Info:");
      Serial.println(ctime(&sampleTime));

      time_t delta = sampleTime-currentTime;
      int deltaMs = sampleTimeMs-currentTimeMs;
      time_t deltaInMs = delta*1000 + deltaMs;

      Serial.print("Info:We have to wait: ");
      Serial.print(deltaInMs);
      Serial.println(" ms");
      if (deltaInMs > 10000) {
        response += "Info:Sampling has to start after at least 10s\r\n";
      } else if (deltaInMs < 0) {
        response += "Info:Time is in the past\r\n";
      } else if (millis() - start > deltaInMs) {
        response += "Info:Give us at least 2s to prepare\r\n";
      } else {
        response += "Info:Will start to sample at: ";
        response += ctime(&sampleTime);
        response += "\r\n";
        long ctdTimer = millis();
        long ctd = (deltaInMs - (millis() - start)) / 1000;
        // While waiting for the countdown to finish, connect to the stream client and show lifeness
        while(millis() - start < (long)deltaInMs) {
          if ((millis() - ctdTimer) > 1000) {
            Serial.print("Info:");
            Serial.println(ctd);
            getter.print("Info:");
            getter.println(ctd);
            ctd -= 1;
            ctdTimer = millis();
          }
          tryConnectStream();
          yield();
        }
        Serial.println("Info:Start Sampling");
        state = STATE_STREAM;
        startSampling();
      }
      break;
    }
    case CMD_START_SAMPLE:
      state = STATE_SAMPLE;
      startSampling();
      // Look if data is requested over serial or over tcp
      if (&getter == &Serial) state = STATE_SAMPLE_USB;
      break;
    case CMD_STOP_SAMPLE:
      stopSampling();
      break;
    case CMD_MDNS: {
      int value = parseValue(getter);
      // To initially store the mdsn name into eeprom
      sprintf (MDNSName, "%i", value);
      EEPROM.put(0, MDNSName);
      EEPROM.commit();
      // while(true){}
      EEPROM.get(0, MDNSName);

      response = "Info:Set MDNS name to: ";
      response += MDNS_PREFIX;
      response += MDNSName;
      break;
    }
    case CMD_SWITCH_FOR:
      delay(10);
      state = STATE_PRE;
      preSampleCnt = parseValue(getter);
      sampleCnt = parseValue(getter);
      postSampleCnt = parseValue(getter);
      response += "Info:Switching for: ";
      response += preSampleCnt;
      response += ", ";
      response += sampleCnt;
      response += ", ";
      response += postSampleCnt;
      response += "";
      startSampling();
      break;
  }
  return response;
}

/****************************************************
 * Stop Sampling will go into ide state, stop the
 * interrupt and if any stream client is connected,
 * will send EOF to the client
 ****************************************************/
void stopSampling() {
  state = STATE_IDLE;
  turnInterrupt(false);
  if (streamClient && streamClient.connected()) streamClient.stop();
}

/****************************************************
 * Start Sampling requires to start the interrupt and
 * to calculate the bufferflush size depending on
 * the samplingrate currently set. Furthermore,
 * all buffer indices are reset to the default values.
 ****************************************************/
void startSampling() {
  TIMER_CYCLES_FAST = (CLOCK * 1000000) / samplingRate; // Cycles between HW timer inerrupts
  bufferFlushSize = min(max(MEASURMENT_BYTES, int(0.1*samplingRate*MEASURMENT_BYTES)), BUF_SIZE);
  whichBufferRead = 0;
  whichBufferWrite = 0;
  inBuffer = 0;
  turnInterrupt(true);
}

/****************************************************
 * Detach or attach intterupt
 ****************************************************/
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

/****************************************************
 * Parse a time_t value given any stream object.
 * Checks stream.available and returns after timeout
 ****************************************************/
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

/****************************************************
 * Switch relay to open or closed
 ****************************************************/
inline void switchRelay(int number, bool value) {
  digitalWrite(number, value);
}
