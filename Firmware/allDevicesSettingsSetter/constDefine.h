

// Serial Speed and DEBUG option
// #define SERIAL_SPEED 9600
#define SERIAL_SPEED 2000000


#define VERSION "1.0"
const char * DEVICE_NAME = "PowerMeterWifiSetter";
// Default values
#define STANDARD_TCP_PORT 54321

const char * POWERMETER_IP = "192.168.4.1";
const char * POWERMETER_MDNS = "powermeter";

const int POWERMETER_MDNS_LENGHT = strlen(POWERMETER_MDNS);

// Location time difference between us (Freiburg, Germany) and NTP time 
#define LOCATION_TIME_OFFSET 3600//7200 // 2 hours or (2*60*60)

#define WIFI_CHECK_PERIODE_MS 5000
#define MDNS_UPDATE_INTERVAL 30000
#define TCP_UPDATE_INTERVAL 100
#define LIFENESS_UPDATE_INTERVAL 1000
#define POWERMETER_CONNECT_UPDATE_INTERVAL 5000

const char LOG_PREFIX_SERIAL[] = "";
const char LOG_PREFIX[] = "Info:";



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
#define CMD_STREAM_SERVER "streamServer"
#define CMD_TIME_SERVER "timeServer"

