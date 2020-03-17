

// Serial Speed and DEBUG option
// #define SERIAL_SPEED 9600
#define SERIAL_SPEED 2000000
// #define DEBUG_DEEP
#define SENT_LIFENESS_TO_CLIENTS


#define VERSION "2.1"

// Default values
#define STANDARD_UDP_PORT 54323
#define STANDARD_TCP_SAMPLE_PORT 54321
#define STANDARD_TCP_STREAM_PORT 54322
#define DEFAULT_SR 4000


// Location time difference between us (Freiburg, Germany) and NTP time 
#define LOCATION_TIME_OFFSET 3600//7200 // 2 hours or (2*60*60)

#define MDNS_UPDATE_INTERVAL 30000
#define TCP_UPDATE_INTERVAL 100
#define LIFENESS_UPDATE_INTERVAL 1000
#define RTC_UPDATE_INTERVAL 30000
#define STREAM_SERVER_UPDATE_INTERVAL 30000
#define MQTT_UPDATE_INTERVAL 5000

// We allow a max of 3 tcp clients for performance reasons
#define MAX_CLIENTS 3

const char * LOG_FILE = "/log.txt";

const char LOG_PREFIX_SERIAL[] = "";
const char LOG_PREFIX[] = "Info:";

const char DATA_PREFIX[] = "Data:";
const size_t PREFIX_SIZE = strlen(DATA_PREFIX); 

// Pin definitions
// Red error led for first flash
const int ERROR_LED = 13;

// Pins for STPM34 SPI Connection
const int STPM_CS = 5;
const int STPM_SYN = 14;
// Reset pin of STPM
const int STPM_RES = 12;

// Pins for 230V Relay
const int RELAY_PIN_S = 26;
const int RELAY_PIN_R = 27;

// Pins for RTC conenction
const int RTC_INT = 25;
const int RTC_32k = 35;
const int RTC_RST = 32;


// Buffering stuff
#define MAX_SEND_SIZE 512 // 1024
// PSRAM Buffer
const int PS_BUF_SIZE = 3*1024*1024 + 512*1024;
#define COMMAND_MAX_SIZE 500


#define MEASURE_VI "v,i"
#define MEASURE_PQ "p,q"
#define MEASURE_VI_RMS "v,i_RMS"
#define MEASURE_VIPQ "v,i,p,q"


#define UNIT_VI "V,mA"
#define UNIT_PQ "W,VAR"
#define UNIT_VIPQ "V,mA,W,VAR"


// Communication commands
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
#define CMD_CLEAR_LOG "clearLog"
#define CMD_GET_LOG "getLog"
#define CMD_MQTT_SERVER "mqttServer"
#define CMD_STREAM_SERVER "streamServer"
#define CMD_TIME_SERVER "timeServer"


#define MQTT_TOPIC_BASE "powermeter"
#define MQTT_TOPIC_SEPARATOR '/'
#define MQTT_TOPIC_SWITCH "switch"
#define MQTT_TOPIC_SWITCH_ON "on"
#define MQTT_TOPIC_SWITCH_OFF "off"
#define MQTT_TOPIC_SAMPLE "sample"
#define MQTT_TOPIC_SAMPLING "sampling"
#define MQTT_TOPIC_STATE "state"
#define MQTT_TOPIC_CMD "cmd"
#define MQTT_TOPIC_INFO "info"

#define TRUE_STRING "true"
#define FALSE_STRING "false"

const int MAX_MQTT_PUB_TOPIC_SWITCH = sizeof(MQTT_TOPIC_BASE) + sizeof(MQTT_TOPIC_STATE) + sizeof(MQTT_TOPIC_SWITCH) + 4*sizeof(MQTT_TOPIC_SEPARATOR) + 2;
const int MAX_MQTT_PUB_TOPIC_SAMPLE = sizeof(MQTT_TOPIC_BASE) + sizeof(MQTT_TOPIC_STATE) + sizeof(MQTT_TOPIC_SAMPLE) + 4*sizeof(MQTT_TOPIC_SEPARATOR) + 2;
const int MAX_MQTT_PUB_TOPIC_INFO = sizeof(MQTT_TOPIC_BASE) + sizeof(MQTT_TOPIC_STATE) + sizeof(MQTT_TOPIC_INFO) + 4*sizeof(MQTT_TOPIC_SEPARATOR) + 2;


template < typename TOut, typename TIn >
TOut round2( TIn value ) {
   return static_cast<TOut>((int)(value * 100 + 0.5) / 100.0);
}