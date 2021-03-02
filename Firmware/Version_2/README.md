[powermeter]: (https://github.com/voelkerb/powermeter)

# [Firmware Version 2]

The [powermeter] has to be connected to an outlet for power supply. \
At first boot, the [powermeter] will open a WiFi-Network called "powermeterX".\
It is not password protected, so you can simply connect to it. 
Open a TCP connection to IP ```powermeterX.local``` (or ```192.168.4.1```) at port ```54321```.
This will show you basic information of the [powermeter] as a JSON encoded messages.
```bash
Info:{"cmd":"info","type":"powermeter","version":"2.2","compiled":"Mar_2_2021_09:57:58","sys_time":"03/02/2021 10:37:07.662","name":"powermeterX","ip":"192.168.4.1","mqtt_server":"-","stream_server":"-","time_server":"time.google.com","sampling_rate":4000,"buffer_size":3670016,"psram":true,"rtc":false,"calV":1,"calI":1,"state":"idle","relay":1,"calibration":[1,1],"ssids":"[energywifi]","ssid":"-","rssi":-71,"bssid":"-"}
```
You will further notice that a lifeness log messages is sent each second.

**Each commands sent to the [powermeter] over USB or a TCP channel requires JSON encoding and is answered by a JSON encoded answer message preceded by a the text "Info:".**
We will further see how to use each command.

## Info
```{"cmd":"info"}```\
The info command will print information about the [powermeter]. As this is the most often required command, it is automatically sent to each new connected TCP client. 

## Log Level
```{"cmd":"log","level":<logLevel>}```\
Choose between the following log levels: ```all, debug, info, warning, error```\
Each new connection is initialized with log level ```info```.
```bash
Info:{"error":false,"msg":"Log Level set to: warning"}
```

## Name
```{"cmd":"mdns","payload":{"name":"<newName>"}}```\
This command should be used to give the [powermeter] a new unique name.
It will be used to announce presence over MDNS and if no known network is found, an AP is opened using this name.

```bash
Info:[I]03/02 10:42:24: MDNS Name: powermeterX
Info:{"error":false,"msg":"Set MDNS name to: powermeterX","mdns_name":"powermeterX"}
```

## Relay
```{"cmd":"switch","payload":{"value":"<false or true>"}}```\
Switches the relay (and thus the connected appliance) _on_ or _off_.
```bash
Info:{"error":false,"msg":"Switching: On"}
```

## Restart
```{"cmd":"restart"}```\
Restarts the [powermeter]. The TCP connection will be lost.

## Reset
```{"cmd":"factoryReset"}```\
Resets everything to defaults. The default values are:
- name: _powermeterX_
- wifi aps: _energywifi_ -> (pwd: _silkykayak943_)
- timeserver: _time.google.com_
- streamserver: -
- mqtt broker: -
- _calV_ = _calI_ = 1.0

If you want to reset everything except the name, use: ```{"cmd":"basicReset"}```

## Log
```{"cmd":"getLog"}```\
The [powermeter] has some decent logging capabilities. Despite logging over a connected TCP connection and Serial. _Warning_ and _Error_ messages are logged to internal flash memory.
The _getLog_ command will dump the all logs in the flash and contains two JSON response messages.
```bash
Info:{"cmd":"log","msg":"*** LOGFile *** [E]03/02 11:26:20: Cannot connect to MQTT Server -//n*** LOGFile *** "}
Info:{"error":false}
```
All newlines are replaces by "_//n_".\
To clear all log messages, use ```{"cmd":"clearLog"}```.

## Set time server
```{"cmd":"ntp"}```\
Will perform an NTP time synchronization.
```
Info:[I]03/02 11:28:14: NTP synced with conf: 11
Info:{"msg":"Time synced","error":false,"current_time":"03/02/2021 11:27:40.182"}
```
To set a new NTP timeserver, use the command: ```{"cmd":"timeServer", "payload":{"server":"<serverAddress>"}}```. The _serverAddress_ can be also be its DNS name e.g. time.google.com
```bash
Info:[I]03/02 11:28:14: NTP synced with conf: 11
Info:{"error":false,"msg":"Set TimeServer address to: time.google.com","time_server":"time.google.com"}
```

## WiFi
Add wifi AP: ```{"cmd":"addWifi", "payload":{"ssid":"<ssidName>","pwd":"<pwdName>"}}```
```bash
Info:{"error":false,"ssid":"Test","pwd":"TestPwd","msg":"New Ap, SSID: Test, PW: TestPwd","ssids":"[energywifi, Test]"}
```
Remove a wifi AP: ```{"cmd":"delWifi", "payload":{"ssid":"<ssidName>"}}```
```bash
Info:{"error":false,"msg":"Removed SSID: Test","ssids":"[energywifi]"}
```

## MQTT
Once connected to an MQTT server, the powermeter will publish the current power consumption each 5 seconds and each state change of the relay. 
To set the MQTT server, use the command: ```{"cmd":"mqttServer", "payload":{"server":"<ServerAddress>"}}```
Currently, only the MQTT standard server port ```1883``` is supported. 
```bash
Info:[I]03/02 11:38:42: MQTT connected to 192.168.0.13
Info:[I]03/02 11:38:42: Subscribing to: powermeter/+
Info:[I]03/02 11:38:42: Subscribing to: powermeter/powermeterX/+
Info:{"error":false,"msg":"Set MQTTServer address to: 192.168.0.13","mqtt_server":"192.168.0.13"}
```
### Receiving MQTT messages
The relay state is send as a retained message on topic ```powermeter/<name>/state/switch```
```bash
powermeter/powermeterX/state/switch true
```
Each 5 seconds, information about the power consumption is sent.
It contains the _active power_ in _Watt_, the _RMS voltage_ in _V_,  the _RMS current in _A_, the current timestamp, and the accumulated energy since last restart. Furthermore, if more than _2 Watt_ is drawn, ```inUse``` is ```true```.
```bash
powermeter/powermeter15/state/sample {"power":1116.89,"inUse":true,"current":4.82,"energy":0,"volt":236.56,"ts":"1614681575"}
powermeter/powermeter28/state/sample {"power":0.01,"inUse":false,"current":0,"energy":0,"volt":230.85,"ts":"1614681576"}
powermeter/powermeter27/state/sample {"power":0,"current":0,"inUse":false,"energy":0,"volt":231.28,"ts":"1614681576"}
```

### Sending MQTT messages
Mqtt can also be used to send any command. Special topics are used to switch the relay or to get basic electricity related measurements, but any of the available commands can be sent.
* Switching the relay using topic ```powermeter/<name>/switch```:
  ```bash
  mosquitto_pub -h 192.168.0.13 -t 'powermeter/powermeter27/switch' -m true
  ```
* Sample a single value using topic ```powermeter/<name>/sample``` and message one of ```v,i,p,q,s```. On no message, the active power is sent.
  ```bash
  powermeter/powermeter27/sample p
  ````
  Response:
  ```bash
  powermeter/powermeter27/state/sample {"value":1.003773,"unit":"W","ts":"03/02/2021 11:59:59.034"}
  ```
* Send any command using topic ```powermeter/<name>/cmd```
  ```bash
  mosquitto_pub -h 192.168.0.13 -t 'powermeter/powermeter27/cmd' -m '{"cmd":"switch","payload":{"value":"false"}}'
  ````
* If you have multiple powermeters and want to send the message to all at the same time, you can simple ditch the specific powermeter name and all will answer. 
  ```bash
  powermeter/sample p
  powermeter/powermeter24/state/sample {"value":1.487539,"unit":"W","ts":"03/02/2021 12:08:28.208"}
  powermeter/powermeter28/state/sample {"value":0.039796,"unit":"W","ts":"03/02/2021 12:08:28.210"}
  powermeter/powermeter20/state/sample {"value":193.0246,"unit":"W","ts":"03/02/2021 12:08:28.208"}
  powermeter/powermeter26/state/sample {"value":15.28467,"unit":"W","ts":"03/02/2021 12:08:28.209"}
  ...
  ```

## Getting Data
Finally, to get some high frequency data beyond whats possible using [#mqtt] out of the powermeters, you have multiple possibilities. 

### Using a stream server
Use the command 
After setting a stream server address for each powermeter, 
** Set the Stream Server. The device will automatically connect to this TCP server on port ```54322```
** command: _{"cmd":"streamServer", "payload":{"server":"<ServerAddress>"}}_
* *MQTT*:
** Set address of an mqtt broker. Device will connect and regularly send relay and sampling state changes as well as power drawn if not in sampling mode
** command: _{"cmd":"timeServer", "payload":{"server":"<ServerAddress>"}}_
* *Sampling*:
** Start sampling:
*** command: _{"cmd":"sample", "payload":{"type":"<type>", "measures":"<measures>", "rate":<rate>, "prefix":<prefix>, "port":<port>,"time":<time>,"flowCtr":<flowCtr>,"slot":[<slot>,<slots>],"ntpConf":<ntpConf>}}_
*** format: The raw data returned will be encoded as 32 Bit MSB first float values.
*** Some of the values are optional like rate, prefix, port, measures, time, flowCtr, slot, ntpConf
*** <type> + optional: <port>:
**** "Serial": sampled data is sent over USB Serial connection
**** "TCP": sampled data is sent over TCP connection (the one you opened to it). 
**** "UDP": sampled data is sent over a UDP connection. You need to specify the UDP Port with <port>.
**** "MQTT": sampled data is sent over MQTT (broker must be set). 
**** "FFMPEG": sampled data is sent over TCP at port 54322. No prefix is supported and no line ending. Just the raw data. Can be used directly with ffmpeg streaming.
*** <rate>: 
**** Integer value between 1 and 32000. It however has to be an integer divider of 32000 otherwise it will not work (e.g. 16k, 8k, 4k, 2k, 1k, etc. )
**** default: 8000
**** Note: TODO: currently, only max 8000 is working due to high cpu pressure
*** <measures> (optional):
**** "v,i_L1" or "v,i": will return v: voltage and i: current of L1.
**** "v,i_L2" or "v,i": will return v: voltage and i: current of L2.
**** "v,i_L3" or "v,i": will return v: voltage and i: current of L3.
**** "p,q": will return p: active and q: reactive power of all channels ordered p_L1,p_L2,p_L3,q_L1,q_L2,q_L3.
***** Note: max samplingrate is 50 Hz
**** "v,i_RMS": will return v: voltage and i: current RMS values in order v,v,v,i,i,i.
***** Note: max samplingrate is 50 Hz
**** default =  "v,i,v,i,v,i"
*** <prefix> (optional):
**** "true" or "false": if true, each chunk of measurement will be sent with the prefix "Data:"<chunk_length><packet_num>
**** <chunk_length>: 2 bytes length of chunk, format: _<H_
**** <packet_num>: running packet number as 4 bytes integer, format: _<I_
**** Default: true
*** <ntpConf> (optional):
**** integer: interpreted as milliseconds. The ntp request before starting the streaming needs to be confident within this threshold value. 
**** Note: NTP requests are send over UDP to an NTP server. The time it takes to get the answer needs to be considered as well for millisecond resolution. As the request has to sent to the server and transported back, the half time of the request is added to the received time. The request is thus only confident to this timespan. In the worst case - if a response took 10ms - it could be 0ms for sending to the server and 10ms for getting the response. This would mean, the time is off the actual time about 5ms - which is our confidence level. As most sampling is stored relative (start time + sampling rate), getting the start time as exact as possible is crucial. 
**** Default: no NTP request is sent
*** <flowCtr> (optional):
**** "true" or "false": if true, sink must request x amount of samples with cts cmd. See below for more detail 
**** Default: false
*** <time> (optional):
**** unix timestamp at which sampling should be started. The timestamp must be in the future more than 500ms but is not allowed to be further in time as 20seconds. 
**** Note: This can be used to start sampling with multiple devices at an exact point in time which is the same for all devices.** [<slot>,<slots>] (optional):
**** <slot> integer, slot number data is sent
**** <slots> integer, total number of slots
**** The idea is that only one device sends data at the same time in a network with multiple device _d_i_. Each device will only send data if the following condition is true: now.seconds%slots == slot
**** Example: 3 device d_i with configs: d_0 = [0,3], d_1 = [0,1], d_2 = [2,3]. All devices send data each 3 seconds. e.g. d_0 at second 0, 3, 6, ... 
**** Note: This only works, if all data sampled can be send out in this second. If you e.g. have 10 device, one device has to send 10s of data every 10s within 1s. The idea behind is that wifi/tcp send collisions are avoided.
**** Default: false
** Stop sampling/streaming etc: 
*** command: _{"cmd":"stop"}_
** Request a given amount of samples: 
*** command: _{"cmd":"reqSamples","samples":<numSamples>}_
*** _<numSamples>_: long, must be between 10 and 2000
*** Note: The device must be sampling and during sampling flow control must have been set!
** Flow control during sampling: 
*** command: _{"cmd":"cts","value":<value>}_
*** <value> can be true or false. True to let device stream data, false to stop.
*** Note: does not have to be paired with flowControl sampling
* *Misc*:
** Test if device is still alive:
*** command: _?_
*** Note: will return "Info: Setup done" and will stop sampling! (Just for backward compatibility)
** Keepalive:
*** command: _!_
*** Note: is simply ignored but can be sent at any time


Each command sent to the [powermeter] is answered by a JSON encoded message. 
The Get the IP address of the device either by looking at it's boot up message or by looking in the DHCP list of your router. It is always a good idea to assign a fixed IP to the device.
From Version 0.9 on, mDNS support allows to fresolve the device's IP address using the mDNS name. See boot up message of device for the name or explore the network with a corresponding app e.g. bonjour browser. 
Commands can be either send over USB or over a TCP socket connection. Therefore connect to the socket <IP address> or <mDNS Name>.local on port 54321.
An additional raw stream can be retrieved using port 54322. 



## Compile from source using Arduino or PlatformIO

* In order to upload the firmware, you must install the latest version of the Arduino ESP32 environment. See [official instructions](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md):
  * Enter _https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json_ into _Arduino > Settings > Additional Board Manager URLs_ field.
  * Open Boards Manager from _Tools > Board_ menu and install _esp32 platform_
* Select _ESP32 Dev kit_ with _PSRAM enabled_ from _Tools > Board_ 

    <img src="/docu/figures/FirmwareSelectPort.png" width="500">
    
* Open the latest firmware "Version_X_X".
* Press _Compile_

## Upload using an FTDI

* Connect the [powermeter] to an FTDI according to the following wiring: 

  <img src="/docu/figures/Connections.png" width="500">

* Select no more than 2Mbauds
* Press _Compile and Upload_

## Upload using Arduino-OTA

* Select the [powermeter] you want to upload to from the avaialble network ports
* If the corresponding [powermeter] is not shown, make sure you are in the same network or try to reset the [powermeter] (unplug it from the socket and plug it back in).

    <img src="/docu/figures/NetworkPort.png" width="500">

* Upload using password "energy"

    <img src="/docu/figures/Password.png" width="500">

## Upload using custom Uploader

  <img src="/docu/figures/upload.gif">

* Compile the firmware
* Copy the path of the compiled binary _elf_ or _bin_ (the _bin_ will be used either way)

    <img src="/docu/figures/CopyBin.png">

* Use the upload script
  ```bash
  python3 upload.py powermeter <pathToElfOrBin> 
  ```
  Select one or mulitple [powermeter] from the provided list
  ```
  Available Devices:
  #  powermeter:              Device:                  IP:                      
  0  powermeter0              powermeter0              192.168.0.145            
  1  powermeter15             powermeter15             192.168.0.113            
  2  powermeter20             powermeter20             192.168.0.111            
  3  powermeter21             powermeter21             192.168.0.118            
  4  powermeter24             powermeter24             192.168.0.115            
  5  powermeter26             powermeter26             192.168.0.119            
  6  powermeter27             powermeter27             192.168.0.138            
  7  powermeter28             powermeter28             192.168.0.114            
  Press ENTER to continue with all devices.
  Deselect specific devices e.g.: -2,-4,-7
  Select specific devices e.g.: 1,3,5,6
  Search again a/A
  Press r/R to reset
  e/E to cancel Or press CTR-C to exit program
  ```
  Press enter and watch the magic




## Supported Hardware
This firmware version supports Hardware Revision 2.


Requirements:
- [RTC](https://github.com/voelkerb/ESP.DS3231_RTC/)
- [multiLogger](http://github.com/voelkerb/ESP.multiLogger) (-> see ```advanced.ino```)

```C++
#include "timeHandling.h"

// CB for successful ntp syncs
void ntpSynced(unsigned int confidence);

TimeHandler myTime(config.timeServer, LOCATION_TIME_OFFSET, NULL, &ntpSynced);

Timestamp then;

void setup() {
  Serial.begin(9600);
  // Connect to WiFi
  ...
  // Initial NTP update after connecting to WiFi
  myTime.updateNTPTime();
  then = myTime.timestamp();
}

void loop() {
  Timestamp now = myTime.timestamp();
  if ((now.seconds-then.seconds) >= 10) {
    Serial.println("Another 10s passed");
    Serial.print("Current time: ");
    Serial.print(myTime.timeStr());
    then = now;
  }
  ...
}

/****************************************************
 * Callback when NTP syncs happened and
 * it's estimated confidence in ms
 ****************************************************/
void ntpSynced(unsigned int confidence) {
  Serial.print("NTP synced with conf: ");
  Serial.println(confidence);
}

```



This class can also be used with the an RTC.
The current time will then be gathered from the rtc object. After each successfull NTP request, the RTC time is updated. 

```C++
#include "timeHandling.h"
#include "rtc.h"
...
Rtc rtc(RTC_INT, SDA_PIN, SCL_PIN);
TimeHandler myTime(config.timeServer, LOCATION_TIME_OFFSET, &rtc, &ntpSynced);
...

```

You can also use it to get log time strings for a [multiLogger](https://github.com/voelkerb/ESP.multiLogger/) instance. See advanced example:
 ```advanced.ino```.

```bash
 - [I]02/06 08:28:16: Connecting to WiFi..
 - [I]02/06 08:28:16: Connected to the WiFi network: ******** with IP: *********
 - [D]02/06 08:28:16: Sending NTP packet...
 - [I]02/25 12:34:04: NTP Time: 02/25/2021 12:34:04.597, td 17
 - [D]02/25 12:34:04: Success NTP Time
 - [D]02/25 12:34:04: NTP synced with conf: 17
 - [I]02/25 12:34:04: Current time: 02/25/2021 12:34:04.728
 - [I]02/25 12:34:14: Another 10s passed
 - [I]02/25 12:34:14: Current time: 02/25/2021 12:34:14.000
 - [I]02/25 12:34:24: Another 10s passed
 - [I]02/25 12:34:24: Current time: 02/25/2021 12:34:24.000
```
