// Private keys can be defined in this file
// Create it and insert e.g. LORA or WIFI keys
#if __has_include("privateConfig.h")
#include "privateConfig.h"
#endif


// Does the meter have a LoRaWAN module connected to the expansion header?
// #define LORA_WAN

// Does the meter have a SENSOR board connected to the expansion header?
#define SENSOR_BOARD


// Version string
#define VERSION "2.3"

// #define DEBUG_DEEP
#define SENT_LIFENESS_TO_CLIENTS
// #define REPORT_VALUES_ON_LIFENESS

#define SEND_INFO_ON_CLIENT_CONNECT

// Serial Speed and DEBUG option
// #define SERIAL_SPEED 9600
#define SERIAL_SPEED 38400 // For use with sensor board and LoRaWAN

#define USE_SERIAL
// #define SERIAL_LOGGER
// #define CMD_OVER_SERIAL

// Check for invalid configurations
#if defined(SENSOR_BOARD) 
#if !defined(USE_SERIAL)
#error CONFIG ERROR: cannot use sensor board without serial (USE_SERIAL defined) 
#endif
#if defined(LORA_WAN)
#error CONFIG ERROR: cannot use SENSOR BOARD and LORA WAN module
#endif
#if defined(SERIAL_LOGGER)
#error CONFIG ERROR: cannot use SENSOR BOARD and serial logger
#endif
#if SERIAL_SPEED != 38400
#error CONFIG ERROR: Sensor board uses 38400 baud
#endif
#endif

#if defined(LORA_WAN)
#if !defined(USE_SERIAL)
#error CONFIG ERROR: cannot use LORA WAN module without serial (USE_SERIAL defined) 
#endif
#if defined(SERIAL_LOGGER)
#error CONFIG ERROR: cannot use LORA WAN module and serial logger
#endif
#if SERIAL_SPEED != 38400
#error CONFIG ERROR: LoRaWAN Module uses 38400 baud (9600 factory default)
#endif
#endif

#if defined(SERIAL_LOGGER) && !defined(USE_SERIAL)
#error CONFIG ERROR: cannot use seriallog without serial (USE_SERIAL defined) 
#endif

// Default values
#define STANDARD_UDP_PORT 54323
#define STANDARD_TCP_SAMPLE_PORT 54321
#define STANDARD_TCP_STREAM_PORT 54322
#define DEFAULT_SR 4000

#define NTP_CORRECT_SAMPLINGRATE
// Confidence threshold in ms
#define NTP_CONFIDENCE_FOR_CORRECTION 25
// Correction threshold in s
#define CORRECT_SAMPLING_THRESHOLD 0.005
#define MAX_CORRECT_SAMPLES 5

// Location time difference between us (Freiburg, Germany) and NTP time 
#define LOCATION_TIME_OFFSET 3600//7200 // 2 hours or (2*60*60)

#define MDNS_UPDATE_INTERVAL 30000
#define TCP_UPDATE_INTERVAL 1000
#define LIFENESS_UPDATE_INTERVAL 1000
#define RTC_UPDATE_INTERVAL 30000
#define STREAM_SERVER_UPDATE_INTERVAL 30000
#define MQTT_UPDATE_INTERVAL 5000
#define ENERGY_UPDATE_INTERVAL 19000
#define LORA_UPDATE_INTERVAL 20000
#define SENSOR_UPDATE_INTERVALL 5000

// We allow a max of 3 tcp clients for performance reasons
#define MAX_CLIENTS 3

#define DEF_LOG_FILE "/log.txt"

#define DEF_LOG_PREFIX_SERIAL ""
#define DEF_LOG_PREFIX "Info:"

#define DEF_DATA_PREFIX "Data:"
#define PREFIX_SIZE strlen(DATA_PREFIX)

// const char * LOG_FILE = "/log.txt";

// const char LOG_PREFIX_SERIAL[] = "";
// const char LOG_PREFIX[] = "Info:";

// const char DATA_PREFIX[] = "Data:";
// const size_t PREFIX_SIZE = strlen(DATA_PREFIX); 

// Pin definitions
// Red error led for first flash
#define ERROR_LED 13

// Pins for STPM34 SPI Connection
#define STPM_CS 5
#define STPM_SYN 14
// Reset pin of STPM
#define STPM_RES 12

// Pins for 230V Relay
#define RELAY_PIN_S 26
#define RELAY_PIN_R 27

// Pins for RTC conenction
#define RTC_INT 25
#define RTC_32k 35
#define RTC_RST 32


// Buffering stuff
// Should be power of 2
// #define MAX_SEND_SIZE 2048
#define MAX_SEND_SIZE 1024
// PSRAM Buffer
#define PS_BUF_SIZE 3*1024*1024 + 512*1024
#define COMMAND_MAX_SIZE 500


#define MAX_UNIT_STR_LENGTH 20
#define MAX_MEASURE_STR_LENGTH 20

#define MEASURE_VI "v,i"
#define MEASURE_PQ "p,q"
#define MEASURE_VI_RMS "v,i_RMS"
#define MEASURE_VIPQ "v,i,p,q"


#define UNIT_VI "V,mA"
#define UNIT_PQ "W,VAR"
#define UNIT_VIPQ "V,mA,W,VAR"


// Communication commands
#define CMD_SAMPLE "sample"
#define CMD_REQ_SAMPLES "reqSamples"
#define CMD_FLOW "cts"
#define CMD_SWITCH "switch"
#define CMD_STOP "stop"
#define CMD_RESTART "restart"
#define CMD_DAILY_RESTART "dailyRestart"
#define CMD_FACTORY_RESET "factoryReset"
#define CMD_BASIC_RESET "basicReset"
#define CMD_INFO "info"
#define CMD_MDNS "mdns"
#define CMD_NTP "ntp"
#define CMD_ADD_WIFI "addWifi"
#define CMD_REMOVE_WIFI "delWifi"
#define CMD_RESET_ENERGY "resetEnergy"
#define CMD_CLEAR_LOG "clearLog"
#define CMD_GET_LOG "getLog"
#define CMD_MQTT_SERVER "mqttServer"
#define CMD_STREAM_SERVER "streamServer"
#define CMD_TIME_SERVER "timeServer"
#define CMD_LOG_LEVEL "log"
#define CMD_CALIBRATION "calibration"
#define CMD_LORA "lora"

#ifdef SENSOR_BOARD
// Sensor Board commands
#define CMD_GET_PIR "getPIR"
#define CMD_GET_TEMP "getTemp"
#define CMD_GET_HUM "getHum"
#define CMD_GET_LIGHT "getLight"
#define CMD_GET_SENSORS "getSensors"
#define CMD_SET_LED "setLED"
#define CMD_SENSOR_BOARD_INFO "sensorBoardInfo"
#define CMD_CAL_TEMP "calibrateTemp"
#define CMD_CAL_HUM "calibrateHum"
#define CMD_CAL_LIGHT "calibrateLight"
#define CMD_PWR_IND "powerIndication"

// Sensor Board standard values
#define STD_MIN_LED_WATT 2  // in W
#define STD_MAX_LED_WATT 200// in W
#define TEMP_HISTERESIS 0.1 // in °C
#define HUM_HISTERESIS 0.5  // in %
#define LIGHT_HISTERESIS 5  // in lux
#endif


#define LOG_LEVEL_ALL "all"
#define LOG_LEVEL_DEBUG "debug"
#define LOG_LEVEL_INFO "info"
#define LOG_LEVEL_WARNING "warning"
#define LOG_LEVEL_ERROR "error"


#define MQTT_TOPIC_BASE "powermeter"
#define MQTT_TOPIC_SEPARATOR '/'
#define MQTT_TOPIC_SWITCH "switch"
#define MQTT_TOPIC_SWITCH_TS "switchTS"
#define MQTT_TOPIC_SWITCH_ON "on"
#define MQTT_TOPIC_SWITCH_OFF "off"
#define MQTT_TOPIC_SWITCH_HANDLED "-"
#define MQTT_TOPIC_SAMPLE "sample"
#define MQTT_TOPIC_SAMPLING "sampling"
#define MQTT_TOPIC_STATE "state"
#define MQTT_TOPIC_CMD "cmd"
#define MQTT_TOPIC_INFO "info"

#ifdef SENSOR_BOARD
#define MQTT_TOPIC_SENSOR "sensors"
#endif

#define TRUE_STRING "true"
#define FALSE_STRING "false"

#define MAX_MQTT_TOPIC_LEN sizeof(MQTT_TOPIC_BASE) + sizeof(MQTT_TOPIC_STATE) + sizeof(MQTT_TOPIC_SWITCH) + 4*sizeof(MQTT_TOPIC_SEPARATOR) + 2+MAX_NAME_LEN


// LORA_CONFIG
#define LORA_MSG_SIZE 3
#define LORA_PREFIX 0xab

// #define the following in your privateConfig.h
#ifndef APP_KEY
#define APP_KEY {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL}
#endif
#ifndef DEV_EUI
#define DEV_EUI {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL}
#endif
#ifndef APP_EUI
#define APP_EUI {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL}
#endif
#ifndef LORA_PORT
#define LORA_PORT 8
#endif

