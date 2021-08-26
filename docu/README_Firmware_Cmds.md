[PowerMeter]: (https://github.com/voelkerb/PowerMeter)

# How to interface with a PowerMeter

## The Basics
The [PowerMeter] has to be connected to an outlet for power supply. \
At first boot, the [PowerMeter] will open a WiFi-Network called "powermeterX".\
It is not password protected, so you can simply connect to it. 
Open a TCP connection to IP ```powermeterX.local``` (or ```192.168.4.1```) at port ```54321```.
This will show you basic information of the [PowerMeter] as a JSON encoded messages.
```bash
Info:{"cmd":"info","type":"PowerMeter","version":"2.2","compiled":"Mar_2_2021_09:57:58","sys_time":"03/02/2021 10:37:07.662","name":"powermeterX","ip":"192.168.4.1","mqtt_server":"-","stream_server":"-","time_server":"time.google.com","sampling_rate":4000,"buffer_size":3670016,"psram":true,"rtc":false,"calV":1,"calI":1,"state":"idle","relay":1,"calibration":[1,1],"ssids":"[energywifi]","ssid":"-","rssi":-71,"bssid":"-"}
```
You will further notice that a lifeness log messages is sent each second.
We will further see how to use commands to interface with a PowerMeter.

**Each command sent to the [PowerMeter] over USB or a TCP channel requires JSON encoding as ```{"cmd":<cmdName>,[optional]}``` and is answered by a JSON encoded answer message preceded by a the text "Info:". If an error occurred while processing the commmand, the error key is set true in the returned JSON.**

## Overview
| CMD                                            | Description                                  |        
| ---------------------------------------------- |----------------------------------------------|
| ["info"](#info)                                | Get basic information                        |
| ["switch"](#relay)                             | Switch relay _on_ or _off_                   |
| ["mdns"](#name)                                | Set new mDNS and general [PowerMeter] name   |
| ["addWifi"](#wifi)                             | Add WiFi AP                                  |
| ["delWifi"](#wifi)                             | Remove WiFi AP                               |
| ["calibration"](#calibration)                  | Set calibration parameter                    |
| ["restart"](#restart)                          | Restart [PowerMeter]                         |
| ["dailyRestart"](#restart)                     | Schedule reset every day                     |
| ["factoryReset"](#reset)                       | Reset to standard values                     |
| ["basicReset"](#reset)                         | Reset everything but the name                |
| ["resetEnergy"](#reset-energy)                 | Reset accumulated energy                     |
| ["sample"](#using-a-sampling-command)          | Start receiving high frequency data          |
| ["reqSamples"](#using-a-sampling-command)      | Get certain amount of high frequency samples |
| ["cts"](#pause-streaming)                      | Allow/forbid to stream samples               |
| ["stop"](#stop-streaming)                      | Stop receiving data                          |
| ["log"](#log-level)                            | Change log level                             |
| ["clearLog"](#log)                             | Clear logs                                   |
| ["getLog"](#log)                               | Returns all warning and error log messages   |
| ["mqttServer"](#mqtt)                          | Set MQTT server                              |
| ["streamServer"](#using-a-stream-server)       | Set stream server                            |
| ["ntp"](#time-synchronization)                 | Trigger NTP sync                             |
| ["timeServer"](#time-synchronization)          | Set time server for NTP                      |
| ["lora"](#lora)                                | Interface with the LoRaWAN module            |
| ["getSensors"](#getting-sensor-values)         | Get sensor reading of all sensors            |
| ["getHum"](#getting-sensor-values)             | Get humidity level                           |
| ["getTemp"](#getting-sensor-values)            | Get temperature                              |
| ["getLight"](#getting-sensor-values)           | Get light intensity                          |
| ["getPIR"](#getting-sensor-values)             | Get binary movement detection                |
| ["setLED"](#changing-the-leds)                 | Set the LEDs to show a specific pattern      |

## Info
```{"cmd":"info"}```\
The info command will print information about the [PowerMeter]. As this is the most often required command, it is automatically sent to each new connected TCP client. 

## Log Level
```{"cmd":"log","level":<logLevel>}```\
Choose between the following log levels: ```all, debug, info, warning, error```\
Each new connection is initialized with log level ```info```.
```bash
Info:{"error":false,"msg":"Log Level set to: warning"}
```

## Name
```{"cmd":"mdns","payload":{"name":"<newName>"}}```\
This command should be used to give the [PowerMeter] a new unique name.
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
Restarts the [PowerMeter]. The TCP connection will be lost.

You can also trigger the module to automatically reset at a particular time (```<hour>:<minute>:00```) each day for maintenance reason. Use ```<hour> = <minute> = -1``` to disable.\
```{"cmd":"restartDaily","hour":<hour>,"minute":<minute>}```

## Reset
```{"cmd":"factoryReset"}```\
Resets everything to defaults. The default values are:
- name: _powermeterX_
- wifi aps: <StandardSSID> -> (pwd: <StandardPWD>)
- timeserver: _time.google.com_
- streamserver: -
- mqtt broker: -
- _calV_ = _calI_ = 1.0
- _energy_ = 0
- _resetHour_ = _resetMinute_ = -1

If you want to reset everything except the name, use: ```{"cmd":"basicReset"}```

## Log
```{"cmd":"getLog"}```\
The [PowerMeter] has some decent logging capabilities. Despite logging over a connected TCP connection and Serial. _Warning_ and _Error_ messages are logged to internal flash memory.
The _getLog_ command will dump the all logs in the flash and contains two JSON response messages.
```bash
Info:{"cmd":"log","msg":"*** LOGFile *** [E]03/02 11:26:20: Cannot connect to MQTT Server -//n*** LOGFile *** "}
Info:{"error":false}
```
All newlines are replaces by "_//n_".\
To clear all log messages, use ```{"cmd":"clearLog"}```.

## Calibration
```{"cmd":"calibration","calV":<calV>,"calI":<calI>}```\
Calibrate the [PowerMeter].
```<calV>``` and ```<calI>``` are floating point values with the default value 1.0.
Every measured voltage and current value is multiplied by the calibration factor. Power and Energy are multiplied by ```<calV>```*```<calI>```.

## Reset Energy
```{"cmd":"resetEnergy"}```\
Simply resets the accumulated energy stored in flash. 

## Time synchronization
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
Once connected to an MQTT server, the PowerMeter will publish the current power consumption each 5 seconds and each state change of the relay. 
To set the MQTT server, use the command: ```{"cmd":"mqttServer", "payload":{"server":"<ServerAddress>"}}```
Currently, only the MQTT standard server port ```1883``` is supported. 
```bash
Info:[I]03/02 11:38:42: MQTT connected to 192.168.0.13
Info:[I]03/02 11:38:42: Subscribing to: powermeter/+
Info:[I]03/02 11:38:42: Subscribing to: powermeter/powermeterX/+
Info:{"error":false,"msg":"Set MQTTServer address to: 192.168.0.13","mqtt_server":"192.168.0.13"}
```

NOTE: In sampling mode, MQTT is disabled for performance reason.


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
* Sample a single value using topic ```powermeter/<name>/sample``` and message one of ```v,i,p,q,s,e``` (_RMS voltage_,_RMS Current_, _active_, _reactive_, _apparent power_ or _active energy_). On no, or any other message, the _active power_ is sent.
  ```bash
  powermeter/powermeter27/sample p
  ````
  The response contains a JSON dictionary with the key _value_, _unit_ and _ts_, example:
  ```bash
  powermeter/powermeter27/state/sample {"value":1.003773,"unit":"W","ts":"03/02/2021 11:59:59.034"}
  ```
* You can also send any command, which can be sent over a bare TCP connection using topic ```powermeter/<name>/cmd```
  ```bash
  mosquitto_pub -h 192.168.0.13 -t 'powermeter/powermeter27/cmd' -m '{"cmd":"switch","payload":{"value":"false"}}'
  ````
* If you have multiple PowerMeters and want to send the message to all at the same time (e.g. to change some global settings such as the MQTT broker), you can simple ditch the specific PowerMeter name and all will answer. 
  ```bash
  powermeter/sample p
  powermeter/powermeter24/state/sample {"value":1.487539,"unit":"W","ts":"03/02/2021 12:08:28.208"}
  powermeter/powermeter28/state/sample {"value":0.039796,"unit":"W","ts":"03/02/2021 12:08:28.210"}
  powermeter/powermeter20/state/sample {"value":193.0246,"unit":"W","ts":"03/02/2021 12:08:28.208"}
  powermeter/powermeter26/state/sample {"value":15.28467,"unit":"W","ts":"03/02/2021 12:08:28.209"}
  ...
  ```

## SensorBoard
If a [SensorBoard](https://github.com/voelkerb/powermeter.sensorboard/) is connected to the expansion header and the Firmware is configured to make use of it (see [compile info](https://github.com/voelkerb/powermeter/blob/master/docu/README_Firmware_2_compile.md)), additional environmental sensor data is available.

### Getting sensor values

* ```{"cmd":"getPIR"}```\
Returns if motion is detected near the [PowerMeter].

* ```{"cmd":"getHum"}```\
Returns the current humidity level in %.

* ```{"cmd":"getTemp"}```\
Returns the current temperature in degrees Celsius.

* ```{"cmd":"getLight"}```\
Returns the current light intensity in Lux.

* ```{"cmd":"getSensors"}```\
Returns all sensor values.

### Changing the LEDs

```bash
{"cmd":"setLED","brightness":<brightness>,"pattern":<patter>,"duration":<duration>,"fgColor":[<red>,<green>,<blue>],"bgColor":[<red>,<green>,<blue>]}
```
* ```<brightness>``` (optional): ```0.0-100.0```
  * gets permanently stored as the new maximum brightness value.
* ```<pattern>``` (optional): ```0-5```; the specific LED pattern to show.
  * ```0```: _static_ pattern, shows a static color
  * ```1```: _blink_ pattern, blink between foreground and background color. The blink interval is 1s.
  * ```2```: _round_ pattern, one LED (fgColor) goes around. Other LEDs in bgColor. The time to move to the next LED is 200ms.
  * ```3```: _glow_ pattern, glow (fade) from fg to bg color and back. Glowing takes around 1.5s.
  * ```4```: _power_ pattern, show the power consumption mapped to a color (<3W = Black, 3-500W linear mapped between green and red, >500W full red). The LED color is updated depending on the current power each 5s.
* ```<duration>``` (optional): ```-1-2^32```
  * duration in milliseconds, the new patter is shown. Afterwards, the last pattern (with a duration of -1) is shown.
  * ```-1``` indicates that the pattern should be shown for infinite length.
* ```<fgColor>``` (optional):
  * foreground color of the pattern.
  * array with three values from ```0-255``` for each color: _red_, _green_, _blue_
  * default: [64,64,64]
* ```<bgColor>``` (optional):
  * background color of the pattern.
  * array with three values from ```0-255``` for each color: _red_, _green_, _blue_
  * default: [0,0,0]

## LoRaWAN
If a supported module is connected to the expansion header and the Firmware is configured to make use of it (see [compile info](https://github.com/voelkerb/powermeter/blob/master/docu/README_Firmware_2_compile.md)), data is also sent via LoRaWAN.

Credentials for OTA activation must be set at compile time. We may add this to the configuration later on.

### Communicate with the module

* ```{"cmd":"lora", "msg":"info"}```\
Send this command to get info about the currently set DEV_EUI and APP_EUI and the connection status. Sample output:
  ```bash
  Info:[I]08/03 18:02:03: LoRa-TX: AT+ID
  Info:[I]08/03 18:02:03: LoRa-RX: +ID: DevAddr, <XX:XX:XX:XX>
  Info:[I]08/03 18:02:03: LoRa-RX: +ID: DevEui, <XX:XX:XX:XX:XX:XX:XX:XX>
  Info:[I]08/03 18:02:03: LoRa-RX: +ID: AppEui, <XX:XX:XX:XX:XX:XX:XX:XX>
  Info:{"error":false,"connected":true,"joined":true}
  ```
* ```{"cmd":"lora", "msg":"join"}```\
Try to join the network.
Sample output:
  ```bash
  Info:[I]08/03 18:04:17: LoRa-TX: AT+MODE=LWOTAA
  Info:[I]08/03 18:04:17: LoRa-RX: +MODE: LWOTAA
  Info:[I]08/03 18:04:17: LoRa-TX: AT+DR=EU868
  Info:[I]08/03 18:04:17: LoRa-RX: +DR: EU868
  Info:[I]08/03 18:04:17: LoRa-TX: AT+CH=NUM,0-2
  Info:[I]08/03 18:04:17: LoRa-RX: +CH: NUM, 0-2
  Info:[I]08/03 18:04:17: LoRa-TX: AT+KEY=APPKEY,"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"
  Info:[I]08/03 18:04:17: LoRa-RX: +KEY: APPKEY XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
  Info:[I]08/03 18:04:17: LoRa-TX: AT+CLASS=A
  Info:[I]08/03 18:04:17: LoRa-RX: +CLASS: A
  Info:[I]08/03 18:04:17: LoRa-TX: AT+JOIN
  Info:[I]08/03 18:04:17: LoRa-RX: +JOIN: Start
  Info:[I]08/03 18:04:17: LoRa-RX: +JOIN: NORMAL
  Info:[I]08/03 18:04:26: LoRa-RX: +JOIN: Network joined
  Info:[I]08/03 18:04:26: LoRa-RX: +JOIN: NetID 000013 DevAddr <XX:XX:XX:XX>
  Info:[I]08/03 18:04:26: LoRa-RX: +JOIN: Done
  ```
* ```{"cmd":"lora", "msg":"AT"}```\
Sample output:
  ```bash
  Info:[I]08/03 18:04:11: LoRa-TX: AT
  Info:{"error":false,"connected":true,"joined":true}
  Info:[I]08/03 18:04:11: LoRa-RX: +AT: OK
  ```
Anything else is regarded as an AT command. See the modules documentation for more detail: [Grove-E5](https://files.seeedstudio.com/products/317990687/res/LoRa-E5%20AT%20Command%20Specification_V1.0%20.pdf)


### UpLink Messages

If the module is connected, the code will automatically try to join the LoRaWAN network each ```LORA_UPDATE_INTERVAL''' seconds. If connected, active and reactive power, RMS voltage and current and electrical energy is sent with the relay state and the system time as raw bytes. 
All values are send as little-endian in the following order:

* active power as 4 byte float value
* reactive power as 4 byte float value
* RMS current as 4 byte float value
* RMS voltage as 4 byte float value
* electrical energy as 4 byte float value
* unix timestamp as 4 byte unsigned int
* relay state as 1 byte unsigned int

These can be decoded in the TTN network application layer e.g. using the following uplink formatter:

```JavaScript
function Decoder(bytes, port) {
  var decoded = {};
  if (port == 8) {  // Adjust the port depending on your config
    var values = ["active","reactive","irms","vrms","energy","ts","relay"];
    var valueBytes = [4, 4, 4, 4, 4, 4, 1];
    var i = 0;
    for (let k = 0; k < values.length; k++) {
      if (i >= bytes.length) { break; }
      var buf = new ArrayBuffer(valueBytes[k]);  // Create a buffer
      var view = new DataView(buf);  // Create a data view of it
      // Set individual bytes LE
      for (let j=0; j < valueBytes[k]; j++) { view.setUint8(valueBytes[k]-1-j, bytes[i+j]); }
      var num = 0;
      if (values[k] == "ts") { num = view.getUint32(0); }
      else if (values[k] == "relay") { num = view.getUint8(0); } 
      else { num = view.getFloat32(0); }
      decoded[values[k]] = num;
      i += valueBytes[k];
    }
  }
  return decoded;
}
```


### DownLink Messages

Down link messages must be sent on the same port according to the following format:
```<prefix><relayState><reset>```.

* ```<prefix>```: Should match the config and exists to check data integrity.
* ```<relayState>```: 1 byte unsigned int. ```0``` to switch off, ```1``` to switch on, and ```2``` to toggle.
* ```<reset>```: 1 byte unsigned int. ```0``` to do nothing, ```1``` to restart the PowerMeter.
If you have configured TTN to be accessible via an MQTT server, you can toggle the relay using:

```
mosquitto_pub -h <ttnLocation>.cloud.thethings.network -d -t '<ttnVerssion>/<yourApplicationID>@ttn/devices/<yourDeviceID>/down/push' -u <yourMqttUser> -P <yourMqttPWD> -m '{"downlinks":[{"f_port": 8,"frm_payload":"AB0200","priority": "NORMAL"}]}' -d
```
The message must be a JSON dictionary in the shown format. The DownLink message is set via the key ```frm_payload``` and must be base64 encoded. 


## Getting High Frequency Data
Finally, to get some high frequency data out of the PowerMeters beyond whats possible using [mqtt](#mqtt), you have multiple possibilities. 

### Using FFmpeg
Using ffmpeg is by far the most simple way to store high frequency data. Thus, the sampling rate remains at 4kHz and only current and voltage measurements are streamed.
On your host computer, simply install [ffmpeg](https://ffmpeg.org) and run the following:
```bash
ffmpeg -nostdin -hide_banner -fflags +nobuffer+flush_packets -f f32le -ar 4000.0 -guess_layout_max 0 -ac 2.0 -flush_packets 1 -thread_queue_size 512 -analyzeduration 0 -i tcp://<PowerMeterIP>:54322 -c:a wavpack -frame_size 4000 -metadata:s:a:0 CHANNELS=2 -metadata:s:a:0 CHANNEL_TAGS="v,i" -metadata:s:a:0 title="<PowerMeterName>" -map 0 -y output.mkv 
```
You can directly use the MDNS name for ```<PowerMeterIP>```. Some of the settings in call such as the metadata are of course optional. 

### Using a stream server
At your future data sink (which simply might be your computer), host a TCP server at port ```54322```.
Find you IP address and set it as the target stream server for each PowerMeter using the command ```{"cmd":"streamServer", "payload":{"server":"<YourIp>"}}```.
The [PowerMeter] will check if the stream server is available each 30s and automatically connects to it.
For the data format, see [Data Format](#data-format).

### Using a sampling command
This is the most complicated command of the PowerMeter. It has multiple and potentially optional parameter which will be explained in the following. 
```bash
{"cmd":"sample", "payload":{"type":"<type>", "measures":"<measures>", "rate":<rate>, "prefix":<prefix>, "port":<port>,"time":<time>,"flowCtr":<flowCtr>,"slot":[<slot>,<slots>],"ntpConf":<ntpConf>}}
```
Example:
The response for the command ```{"cmd":"stop","payload":{"type":"TCP","rate":1}}``` is:
```bash
Info:{"error":false,"measures":"v,i","chunksize":8,"samplingrate":1,"conn_type":"TCP","measurements":2,"prefix":true,"flowCtr":false,"cmd":"sample","unit":"V,mA","startTs":"1614697441.119"}
```

Parameters:
* ```<type>``` + optional: ```<port>```: The Type of the data stream.
  * "Serial": sampled data is sent over USB Serial connection
  * "TCP": sampled data is sent over TCP connection (the one you opened to it). 
  * "UDP": sampled data is sent over a UDP connection. You need to specify the UDP Port with ```<port>```.
  * "MQTT": sampled data is sent over MQTT (broker must be set, see [MQTT](#mqtt)). 
  * "FFMPEG": connect with an ffmpeg call to the stream server on port ```54322```. No prefix is supported and no line ending, just the raw data. Can be used directly with ffmpeg streaming. Allows to change settings while using [ffmpeg streaming](#using-ffmpeg).

* ```<rate>```: The goal sampling rate of the data.
  * Integer value between 1 and 8000 (default: 4000)

* ```<measures>``` (optional):
  * "v,i": will return _voltage_ and _current_
  * "p,q": will return _active_ and _reactive power_
  * "v,i_RMS": will return _RMS voltage_ and _RMS current_
  * "v,i,p,q": will return _voltage_, _current_, _active_, and _reactive power_
  * Default: "v,i"

* ```<prefix>``` (optional):
  * "true" or "false": if _true_, each chunk of measurement will be sent with the prefix ```"Data:"<chunk_length><packet_num><data>```
  * ```<chunk_length>```: 2 bytes stating the length the amount of bytes in ```<data>```, format: _```<H```_
  * ```<packet_num>```: running packet number as 4 bytes integer, format: _```<I```_
  * Default: _true_

* ```<ntpConf>``` (optional):
  * Integer, interpreted as milliseconds. The NTP request before starting the sampling needs to be confident within this threshold value. 
  * NOTE: NTP requests are send over UDP to the specified NTP server. The time it takes to get the answer needs to be considered as well for millisecond resolution. As the request has to be sent to the server and from the server back to the [PowerMeter], half of the time the request took is added to the received NTP time. The request is thus only confident up to the time it took to receive the response, as in the worst case - if a response took 10ms - it could be 0ms for sending to the server and 10ms for getting the response. This would mean, the time is off the actual time about 5ms - which is our confidence level. As most sampling is stored relative (start time + sampling rate), getting the start time as exact as possible is crucial. 
  * Default: no NTP request is sent

* ```<flowCtr>``` (optional):
  * "true" or "false": if _true_, the sink has to request _x_ amount of samples with the ```reqSamples``` command or actively enable sending with a [cts](#pause-streaming) command.
  * Default: _false_
  * Request ```<numSamples>``` using the command: ```{"cmd":"reqSamples","samples":<numSamples>}```
    * ```<numSamples>```: long, must be between 10 and 2000
    * NOTE: In order for the command to work, the [PowerMeter] must be sampling and during the sampling command ```flowCtr``` must have been set to _true_!

* ```<time>``` (optional):
  * Unix timestamp at which sampling should be started. The timestamp must be in the future more than 500ms but is not allowed to be further in time then 20s. 
  * NOTE: This can be used to start sampling with multiple devices at an exact point in time. Sampling further starts at a positive voltage zero crossing. Therewith, PowerMeters at the same phase are synchronized within 1/f<sub>L</sub> with f<sub>L</sub> being the grid line frequency.  
  * Default: Sampling is started immediately 

* ```[<slot>,<slots>]``` (optional):
  * ```<slot>``` integer, the slot number in which data is sent
  * ```<slots>``` integer, the total number of slots
  * The idea is that only one device sends data at the same time in a network with multiple device ```d_i```. Each device will only send data if the following condition is true: ```now.seconds%slots == slot```
  * Example: 3 devices _d<sub>i</sub>_ with configs: _d<sub>0</sub> = [0,3]_, _d<sub>1</sub> = [1,3]_, _d<sub>2</sub> = [2,3]_. All devices send data each 3 seconds. e.g. _d<sub>0</sub>_ at second _0,3,6,..._ 
  * NOTE: This only works, if all data sampled can be send out in this second. If you e.g. have 10 devices, one device has to send 10s of data every 10s within just 1s. If the PowerMeter is not able to sent all data within this second, buffer overflows will occur. However, it avoids wifi/tcp collisions caused by multiple PowerMeters.
  * Default: _false_

### Pause streaming
You can use flow control during sampling with the command: ```{"cmd":"cts","value":<value>}```
* ```<value>``` can be _true_ or _false_. _True_ to let device stream data, _false_ to pause.
* NOTE: does not necessarily have to be paired with sampling in which ```flowCtr``` was set to _true_.

### Stop streaming 
```{"cmd":"stop"}```\
Send this command to stop sampling or streaming. As a response, you get information about the sampling process.
```bash
Info:{"msg":"Received stop command","sample_duration":16157,"samples":16,"sent_samples":16,"start_ts":"1614697441.119","stop_ts":"1614697457.276","ip":"192.168.0.138","avg_rate":0.955618,"cmd":"stop"}
```

## Misc
### Availability
You can send ```?``` without a JSON encoding to test if the device is still reachable. It will return ```Info: Setup done``` and will stop sampling! (Just for backward compatibility)

### Keepalive
As a keepalive command, you can send ```!```. This simply gets ignored but might prevent the TCP connection from being closed automatically if nothing is sent.

## Data Format
The data is returned encoded as 32 Bit MSB first float values.
If not explicitly disabled using ```"prefix":"false"```, data is preceded by a prefix.\
```"Data:"<chunk_length><packet_num><data>```
  * ```<chunk_length>```: 2 bytes describing the length of this data chunk, format: _```<H```_
  * ```<packet_num>```: running packet number as 4 bytes integer, format: _```<I```_
If disable or the stream type is set to FFMPEG, just the ```<data>``` is returned.
