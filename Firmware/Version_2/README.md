[powermeter]: (https://github.com/voelkerb/powermeter)

# Firmware Version 2

## Upload using Arduino or PlatformIO

* In order to upload the firmware, you must install the latest version of the Arduino ESP32 environment. See [official instructions](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md):
  * Enter _https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json_ into _Arduino > Settings > Additional Board Manager URLs_ field.
  * Open Boards Manager from _Tools > Board_ menu and install _esp32 platform_
* Select _ESP32 Dev kit_ with _PSRAM enabled_ from _Tools > Board_ 
* Open the latest firmware "Version_X_X".

1. Connect the [powermeter] using an FTDI: 

    ![Firmware Select port](/docu/figures/FirmwareSelectPort.png)
    <img src="/docu/figures/FirmwareSelectPort.png" width="148">

2. Select the network port. 

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
