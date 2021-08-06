[PowerMeter]: (https://github.com/voelkerb/PowerMeter)

# How to compile Firmware Version 2

## Preprocessor Settings

Before you want to compile the code, make sure the preprocessor settings are as you want them to be.
Take a look at [constDefine.h](https://github.com/voelkerb/PowerMeter/blob/master/Firmware/Version_2/constDefine.h).
The PowerMeter supports multiple addons such as LoRa or an additional sensor board. Depending on your configuration, the preprocessor settings must be set correctly. 

### Standard Settings:
You can set some standard WiFi configurations which will also be kept on a configuration reset. Simply add you WiFi credentials to a file named ```privateConfig.h``` under _src/config/_  
```c++
#define NUM_STANDARD_WIFIS 2

const char * MY_STANDARD_WIFIS[] = {
     /*SSID*/ "mySecretAP1", /*PWD*/ "mySecret1",
     /*SSID*/ "mySecretAP2", /*PWD*/ "mySecret2",
};
```

### General Settings:

* ```SENT_LIFENESS_TO_CLIENTS```: Uncomment to send lifeness messages with the current system time each second to connected TCP clients or over serial.
* ```REPORT_ENERGY_ON_LIFENESS```: Uncomment to send energy info as lifeness messages.
* ```LIFENESS_UPDATE_INTERVAL```: Interval for sending lifeness messages.
* ```SEND_INFO_ON_CLIENT_CONNECT```: Uncomment to send system info JSON dictionary automatically on TCP client connect.
* ```USE_SERIAL```: Comment to disable serial, uncomment if you want to use serial.
* ```SERIAL_SPEED```: Baudrate of any serial connection. The serial connectors are accessible via the extension header.
* ```SERIAL_LOGGER```: Uncomment if you want log messages to be sent via serial.
* ```LOG_PREFIX_SERIAL```: Prefix for logs over serial connection.
* ```CMD_OVER_SERIAL```: Uncomment if you want commands to be processed via serial.
* ```PS_BUF_SIZE```: Amount of PSRAM for data buffering. Arduino does only support up to 3.5MB.
* ```DEFAULT_SR```: Default samplingrate.
* ```LOCATION_TIME_OFFSET```: Location time offset from UTC. 
* ```MDNS_UPDATE_INTERVAL```: Interval for MDNS updates.
* ```TCP_UPDATE_INTERVAL```: Interval for adding TCP clients.
* ```RTC_UPDATE_INTERVAL```: Interval for updating the RTC's time.
* ```ENERGY_UPDATE_INTERVAL```: Interval energy is stored to flash. If no energy is reported, this is also the interval it is updated. As energy register overflows around each 19Wh, keep it smaller than ```t < 19Wh*3600s/MAX_POWER```. With a worst case ```MAX_POWER``` of ```3600W``` this results in around ```19s```. 
* ```STREAM_SERVER_UPDATE_INTERVAL```: Interval for checking if an external TCP server is ready to receive data.
* ```MQTT_UPDATE_INTERVAL```: Interval to send energy info over MQTT.
* ```LOG_PREFIX```: Prefix for logs via TCP connection.
* ```DATA_PREFIX```: Prefix for data via TCP connection.

### Connection Settings:

* ```STANDARD_UDP_PORT```: Standard port for UDP connections.
* ```STANDARD_TCP_SAMPLE_PORT```: Standard port for TCP connections.
* ```STANDARD_TCP_STREAM_PORT```: Standard port for TCP streaming e.g. via ffmpeg.
* ```MAX_CLIENTS```: Allow at max this many TCP clients.
* ```MAX_SEND_SIZE```: Maximum chunk size for data. Chunksize is set dynamically depending on the samplingrate.  

### NTP Sampling Correction

* ```NTP_CORRECT_SAMPLINGRATE```: If NTP synchronizations should be used to correct the sampling rate by means of repeating or leaving out samples.
* ```NTP_CONFIDENCE_FOR_CORRECTION```: Required confidence for NTP sampling correction in milliseconds.
* ```CORRECT_SAMPLING_THRESHOLD```: Drift in seconds when sampling correction should be applied.
* ```MAX_CORRECT_SAMPLES```: Maximum number of samples to correct for one sync.

### LoRaWAN Settings:

* ```LORA_WAN```: Comment to disable, uncomment to enable LoRaWAN connection via an extension board with AT command firmware (e.g. [Grove LoRa E5](https://wiki.seeedstudio.com/Grove_LoRa_E5_New_Version/)). Therewith you can e.g. connect to [The Things Network](https://www.thethingsnetwork.org). Lora Credentials (such as ```APP_KEY```, ```APP_EUI``` and ```DEV_EUI``` - in case of LoRaWAN OTA Authentification) should be stored in an external ```privateConfig.h``` file in the base folder of the specific firmware (see [constDefine.h](https://github.com/voelkerb/PowerMeter/blob/master/Firmware/Version_2/constDefine.h) for more details).
* ```APP_KEY```: Application key of OTA activation for TTN network.
* ```DEV_EUI```: Device EUI for TTN network.
* ```APP_EUI```: Application EUI of OTA activation for TTN network.
* ```LORA_PORT```: Port over which data is sent and received.



## Compile from source using Arduino or PlatformIO

* In order to upload the firmware, you must install the latest version of the Arduino ESP32 environment. See [official instructions](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md):
  * Enter _https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json_ into _Arduino > Settings > Additional Board Manager URLs_ field.
  * Open Boards Manager from _Tools > Board_ menu and install _esp32 platform_
* Select _ESP32 Dev kit_ with _240MHz_ and _PSRAM enabled_ from _Tools > Board_ 

    <img src="/docu/figures/FirmwareSelectPort.png" width="500">
    
* Open the latest firmware "Version_X_X".
* Press _Compile_

## Upload using an FTDI

* Connect the [PowerMeter] to an FTDI according to the following wiring: 

  <img src="/docu/figures/Connections.png" width="600">

* Select no more than 2Mbauds
* Press _Compile and Upload_

## Upload using Arduino-OTA

* Select the [PowerMeter] you want to upload to from the available network ports
* If the corresponding [PowerMeter] is not shown, make sure you are in the same network or try to reset the [PowerMeter] (unplug it from the socket and plug it back in).

    <img src="/docu/figures/NetworkPort.png" width="400">

* Upload using password "energy"

    <img src="/docu/figures/Password.png" width="300">

## Upload using custom Uploader

  <img src="/docu/figures/upload.gif">

* Compile the firmware
* Copy the path of the compiled binary _elf_ or _bin_ (the _bin_ will be used either way)

    <img src="/docu/figures/CopyBin.png">

* Use the upload script
  ```bash
  python3 upload.py PowerMeter <pathToElfOrBin> 
  ```
  Select one or mulitple [PowerMeter] from the provided list
  ```
  Available Devices:
  #  PowerMeter:              Device:                  IP:                      
  0  PowerMeter0              PowerMeter0              192.168.0.145            
  1  PowerMeter15             PowerMeter15             192.168.0.113            
  2  PowerMeter20             PowerMeter20             192.168.0.111            
  3  PowerMeter21             PowerMeter21             192.168.0.118            
  4  PowerMeter24             PowerMeter24             192.168.0.115            
  5  PowerMeter26             PowerMeter26             192.168.0.119            
  6  PowerMeter27             PowerMeter27             192.168.0.138            
  7  PowerMeter28             PowerMeter28             192.168.0.114            
  Press ENTER to continue with all devices.
  Deselect specific devices e.g.: -2,-4,-7
  Select specific devices e.g.: 1,3,5,6
  Search again a/A
  Press r/R to reset
  e/E to cancel Or press CTR-C to exit program
  ```
  Press enter and watch the magic
