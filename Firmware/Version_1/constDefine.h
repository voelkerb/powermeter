

// Serial Speed and DEBUG option
// #define SERIAL_SPEED 9600
#define SERIAL_SPEED 115200
// #define DEBUG_DEEP
#define SENT_LIFENESS_TO_CLIENTS


#define VERSION "1.2"

// Default values
#define STANDARD_UDP_PORT 54323
#define STANDARD_TCP_SAMPLE_PORT 54321
#define STANDARD_TCP_STREAM_PORT 54322
#define DEFAULT_SR 4000


// Location time difference between us (Freiburg, Germany) and NTP time 
#define LOCATION_TIME_OFFSET 3600//7200 // 2 hours or (2*60*60)

#define MQTT_UPDATE_INTERVAL 10000
#define MDNS_UPDATE_INTERVAL 30000
#define TCP_UPDATE_INTERVAL 100
#define LIFENESS_UPDATE_INTERVAL 1000

// We allow a max of 3 tcp clients for performance reasons
#define MAX_CLIENTS 3

const char * LOG_FILE = "/log.txt";

const char* ntpServerName = "time.uni-freiburg.de";//"0.de.pool.ntp.org";

const char LOG_PREFIX_SERIAL[] = "";
const char LOG_PREFIX[] = "Info:";

// Pin definitions
// Red error led for first flash
const int ERROR_LED = 0;
// const int ERROR_LED = LED_BUILTIN;

// Pins for STPM34 SPI Connection
const int STPM_CS = 15;
const int STPM_SYN = 4;
// Reset pin of STPM
const int STPM_RES = 5;

// Pin for 230V Relay
const int RELAY_PIN_S = 2;


// Buffering stuff
#define MAX_SEND_SIZE 512 // 1024
// PSRAM Buffer
const int BUF_SIZE = (10*1024);
#define COMMAND_MAX_SIZE 500

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

const int MAX_MQTT_PUB_TOPIC_SWITCH = sizeof(MQTT_TOPIC_BASE) + sizeof(MQTT_TOPIC_STATE) + sizeof(MQTT_TOPIC_SWITCH) + 4*sizeof(MQTT_TOPIC_SEPARATOR) + 2;
const int MAX_MQTT_PUB_TOPIC_SAMPLE = sizeof(MQTT_TOPIC_BASE) + sizeof(MQTT_TOPIC_STATE) + sizeof(MQTT_TOPIC_SAMPLE) + 4*sizeof(MQTT_TOPIC_SEPARATOR) + 2;
const int MAX_MQTT_PUB_TOPIC_INFO = sizeof(MQTT_TOPIC_BASE) + sizeof(MQTT_TOPIC_STATE) + sizeof(MQTT_TOPIC_INFO) + 4*sizeof(MQTT_TOPIC_SEPARATOR) + 2;