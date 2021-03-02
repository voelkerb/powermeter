[powermeter]: (https://github.com/voelkerb/powermeter)

# How to compile Firmware Version 2

## Compile from source using Arduino or PlatformIO

* In order to upload the firmware, you must install the latest version of the Arduino ESP32 environment. See [official instructions](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md):
  * Enter _https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json_ into _Arduino > Settings > Additional Board Manager URLs_ field.
  * Open Boards Manager from _Tools > Board_ menu and install _esp32 platform_
* Select _ESP32 Dev kit_ with _240MHz_ and _PSRAM enabled_ from _Tools > Board_ 

    <img src="/docu/figures/FirmwareSelectPort.png" width="500">
    
* Open the latest firmware "Version_X_X".
* Press _Compile_

## Upload using an FTDI

* Connect the [powermeter] to an FTDI according to the following wiring: 

  <img src="/docu/figures/Connections.png" width="600">

* Select no more than 2Mbauds
* Press _Compile and Upload_

## Upload using Arduino-OTA

* Select the [powermeter] you want to upload to from the avaialble network ports
* If the corresponding [powermeter] is not shown, make sure you are in the same network or try to reset the [powermeter] (unplug it from the socket and plug it back in).

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
