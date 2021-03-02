[powermeter]: (https://github.com/voelkerb/powermeter)

# Firmware Version 2

## Compile from Source using Arduino or PlatformIO

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

* Compile the firmware
* Copy the path of the compiled binary _elf_ or _bin_ (the _bin_ will be used either way)

    <img src="/docu/figures/CopyBin.png" width="500">

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
  Press enter and watch the magic appear

  <img src="/docu/figures/upload.gif">


 Upload the new firmware.
Connect to the device via a USB Serial or TCP connection to a PC. Look for the "COM" port (Windows) or "/dev/ttyXXX" port (Unix) and set the correct baudrate which depends on the current setting in the Firmware (default: 2000000).
The device has to be connected to an outlet for power supply. The USB connection is protected using optocoupler up to 3kV.
Get the IP address of the device by looking at it's boot up message or by looking in the DHCP list of your router. It is always a good idea to assign a fixed IP to the device.
From Version 0.9 on, mDNS support allows to fresolve the device's IP address using the mDNS name. See boot up message of device for the name or explore the network with a corresponding app e.g. bonjour browser. 
Commands can be either send over USB or over a TCP socket connection. Therefore connect to the socket <IP address> or <mDNS Name>.local on port 54321.
An additional raw stream can be retrieved using port 54322. 


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
