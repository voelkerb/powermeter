[PowerMeter]: (https://github.com/voelkerb/PowerMeter)

# How to compile firmware version 1

## Compile from source using Arduino or PlatformIO

* In order to upload the firmware, you must install the latest version of the Arduino ESP8266 environment. See [official instructions](https://github.com/esp8266/Arduino):
  * Enter _http://arduino.esp8266.com/stable/package_esp8266com_index.json_ into _Arduino > Settings > Additional Board Manager URLs_ field.
  * Open Boards Manager from _Tools > Board_ menu and install _esp8266 platform_
  * Select _Node MCU 1.0_ from _Tools > Board_ 
  * Select a reasonable fast _upload speed_ and select _160 MHz_ as the _CPU Frequency_
    
* Open the latest firmware "Version_X_X".
* Press _Compile_

## Upload using USB

* Plug the [PowerMeter] into a socket, as it is requires for power supply 
* Connect the [PowerMeter] over micro USB to you PC. The USB connection is galvanically isolated - so do not panick.
* Select no more than 2Mbauds
* Press _Compile and Upload_

## Upload using Arduino-OTA

* Select the [PowerMeter] you want to upload to from the avaialble network ports
* If the corresponding [PowerMeter] is not shown, make sure you are in the same network or try to reset the [PowerMeter] (unplug it from the socket and plug it back in).

    <img src="/docu/figures/NetworkPort.png" width="400">

* Upload using password "energy"

    <img src="/docu/figures/Password.png" width="200">

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


