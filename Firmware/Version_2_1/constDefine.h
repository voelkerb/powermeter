

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

// We allow a max of 3 tcp clients for performance reasons
#define MAX_CLIENTS 3

const char * LOG_FILE = "/log.txt";

const char* ntpServerName = "time.uni-freiburg.de";//"0.de.pool.ntp.org";

const char LOG_PREFIX_SERIAL[] = "";
const char LOG_PREFIX[] = "Info:";

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
#define COMMAND_MAX_SIZE 400

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