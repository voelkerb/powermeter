#include <SPI.h>
#include "STPM_mod.h"
#include <ESP8266WiFi.h>
#include <WiFiClient.h>

// Serial Speed and DEBUG option
#define SERIAL_SPEED 115200
//#define DEBUG
//#define SERIAL_OUTPUT
#define VERSION 0.8

// Commands
#define CMD_SWITCH_ON 's'
#define CMD_SWITCH_OFF 'o'
#define CMD_SWITCH_FOR 'f'
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

// STPM Object
STPM stpm34(STPM_RES, STPM_CS, STPM_SYN);

// Buffering stuff
#define MEASURMENT_BYTES 18 //(16+2)
volatile float values[5] = {0};
char buffer[MEASURMENT_BYTES] = {0};
char dummy[MEASURMENT_BYTES] = "Alle meine Entc\n\r";
#define NUM_BUF 5
#define BUF_SIZE 2048

char buffer2[NUM_BUF][BUF_SIZE] = {};
volatile int whichBufferRead = -1;
volatile uint8_t whichBufferWrite = 0;
volatile uint16_t inBuffer = 0;

// WLAN configuration
const char* ssid =  "esewifi";
const char* password =  "silkykayak943";
WiFiServer server(54321);

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
volatile int state = STATE_IDLE;
volatile unsigned long i = 0;
volatile unsigned long j = 0;
volatile unsigned long k = 0;
// Stuff for frequency calculation
volatile long freqCalcStart;
volatile long freqCalcNow;
volatile long freq = 0;

void setup() {
   // Setup serial communication
  #ifdef DEBUG
    Serial.begin(SERIAL_SPEED);
    delay(300);
    Serial.print(F("Info:WIFI powermeter starting up. Version: "));

    Serial.println(VERSION);
    Serial.print("BUF_SIZE:");
    Serial.println(BUF_SIZE);
  #endif

  // Set Relay pins as output which should be switched off by default
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  buffer[16] = '\r';
  buffer[17] = '\n';
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

  Serial.println(F("Info:Setup done"));
}

WiFiClient client;

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
  if (Serial.available()) {
    serialEvent();
  }
  if (!client.connected()) {
    // if (state == STATE_SAMPLE) {
    //   // We lost connection to client, stop sampling
    //   turnInterrupt(false);
    //   state = STATE_IDLE;
    // }
    // try to connect to a new client
    client = server.available();
  } else {
    // read data from the connected client
    if (client.available() > 0) {
      tcpEvent();
    }
    noInterrupts();
    int whichBuffer = whichBufferRead;
    whichBufferRead = -1;
    interrupts();
    if (whichBuffer != -1) {
      //client.print("Heurika:");
      //client.println(whichBufferRead);
      client.write(&buffer2[whichBuffer][0], BUF_SIZE);
    }
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
  /*
  // In
  if (state == STATE_IDLE) {
    return;
  } else if (state == STATE_SAMPLE) {
  // Wait for preduration, then switch relay
  } else if (state == STATE_PRE) {
    j++;
    if (j == preSampleCnt) {
      j = 0;
      state = STATE_ON;
      switchRelay(RELAY_PIN, true);
    }
  } else if (state == STATE_ON) {
    j++;
    if (j == sampleCnt) {
      j = 0;
      state = STATE_POST;
      switchRelay(RELAY_PIN, false);
    }
  } else if (state == STATE_POST) {
    j++;
    if (j == postSampleCnt) {
      j = 0;
      state = STATE_IDLE;
    }
  }
  // This is way too slow...
  */
  stpm34.readVoltageAndCurrent(1, (float*) &values[0], (float*) &values[1]);
  values[2] = stpm34.readActivePower(1);
  values[3] = stpm34.readReactivePower(1);
  memcpy(&buffer[0], (void*)&values[0], 16);
  memcpy(&buffer2[whichBufferWrite][inBuffer], &buffer[0], MEASURMENT_BYTES);
  //memcpy(&buffer2[whichBufferWrite][inBuffer], &dummy[0], 18);
  inBuffer += MEASURMENT_BYTES;
  if (inBuffer > BUF_SIZE-MEASURMENT_BYTES) {
    inBuffer = 0;
    whichBufferRead = whichBufferWrite;
    whichBufferWrite = (whichBufferWrite + 1) % NUM_BUF;
  }

  #ifdef SERIAL_OUTPUT
  Serial.write(buffer, 18);
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
  String response = handleCommand(c);
  Serial.println(response);
}

void tcpEvent() {
  if (!client.available()) return;
  char c = client.read();
  if (c == ' ' || c == '\n' || c == '\r') return;
  Serial.println(c);
  String response = handleCommand(c);
  Serial.println(response);
  client.println(response);
  //client.write(buffer, 18);
}

String handleCommand(char c) {
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
      preSampleCnt = parseDuration();
      sampleCnt = parseDuration();
      postSampleCnt = parseDuration();
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

long parseDuration() {
  long duration = 0;
  while(true) {
    int i = Serial.read() - '0';
    if (i < 0 || i > 9) break;
    else {
      duration *= 10;
      duration += i;
    }
  }
  return duration;
}

inline void switchRelay(int number, bool value) {
  digitalWrite(number, value);
}
