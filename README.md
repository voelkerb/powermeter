# PowerMeter

The PowerMeter is a WiFi equipped electricity meter which can also switch the connected appliance _on_ and _of_. 
You can record high-frequency voltage and current waveforms or embedd it into your smart home system using e.g. _MQTT_.
Further modular add-ons allow to use other wireless connectivity such as LoRaWAN or to embedd other sensors into the housing. 

<p align="center">
<img src="/docu/figures/Powermeter.jpg" width="200px">
</p>

### Clone this repo with
```bash
git clone --recurse-submodules git@github.com:voelkerb/powermeter.git
```

If you are interested in gathering the whole-house energy consumption, see the [SmartMeter](https://github.com/voelkerb/smartmeter) instead.

## Hardware Version _1_ vs _2_
The development started with hardware [Version 1.0](/Schematic/Version_1). It consists of an ESP8266 microcontroller and a dedicated electricity monitoring chip. As the main idea behind this project was to record voltage and current waveforms at higher frequencies, several drawbacks for this use case were fixed with Hardware [Version 2.0](/Schematic/Version_2). It features a faster microcontroller with 8MB internal storage for data buffering, an RTC for precise sampling and time keeping and a full 16A relay so that high-power appliances like dish washers can be switched _on_ and _off_. The USB connection present in [Version 1.0](/Schematic/Version_1) has been removed, but can still be added with an additional extension board.

## Why to use?
It's open source, easy to use, easy to build and offers lots of flexibility. 

The hardware is...
* tested to work reliable with 230V.
* flexible enough to be integrated into existing projects.
* extendable via a UART interface.
* rather cheap (all components sum up to around 30€).

The firmware supports...
* existing 2.4GHz WPA2 networks or can create its own network.
* to receive commands via USB, TCP or MQTT.
* to store the configuration in the internal non volatile memory.
* to provide data at sampling rates from 1/5Hz all the way up to 8kHz.
* to calibrate the measurements.
* integration into most existing smart home systems e.g. HomeKit (via homebridge and MQTT).
* FFMpeg streaming. This allows to store high frequency data directly into files (e.g. MKV).


## How to build one?
You want to build your own PowerMeter? As the PCBs are provided in this repo, you can simply make your own.

Depending on your requirements, you might either build [Version 1.0](/Schematic/Version_1) or [Version 2.0](/Schematic/Version_2). 
* [Version 1.0](/Schematic/Version_1) features an ESP8266 microcontroller and a 10A solid state relay.
    * Benefits: 
        * The parts are cheaper (around 30€).
        * It comes with a USB interface for uploading the firmware or streaming data
    * Drawbacks:
        * Non-modular.
        * The 10A relay can only switch connected appliances with a maximum power consumption of <2300 Watt.
* [Version 2.0](/Schematic/Version_2) features an ESP32 microcontroller and a 16A bistable relay.
    * Benefits: 
        * You can switch appliances with up to 3600Watt. 
        * You have multi-core microcontroller that also supports Bluetooth.
        * 8MB of internal storage allows to buffer data. If you stream high frequency data at e.g. 2kHz, this can hold up to 500s of data on network dropouts.
        * An RTC keeps track of time allowing you to synchronize sampling rates during high frequency sampling.
        * Extendable by stacking other modules on an extension header. 
    * Drawbacks:
        * A little bit more expensive (parts around 35€).

Steps to build your own PowerMeter:
1. Make the PCB ([Version 1.0](/Schematic/Version_1) or [Version 2.0](/Schematic/Version_2)). I recommend to use [JLC](https://jlcpcb.com) as it is super cheap and the quality is still decent.
2. Buy the parts listed under [BOM](/BOM).
3. Solder everything together. We used a small reflow oven, but it could also be done using a fine soldering iron. 
4. Buy a matching housing. We highly recommend to use [this one](https://www.conrad.de/de/p/bopla-eletec-se-432-de-cee-stecker-gehaeuse-120-x-65-x-50-abs-polycarbonat-lichtgrau-graphitgrau-1-st-522228.html) as the PCB was specially designed for it, the mounting holes match, and it is safe to use with 230V. We also experimented with custom 3D prints (see [CAD](/CAD)) but it's at your own risk to do so. 
5. Wire the PowerMeters using the following diagram. 

<img src="/docu/figures/socket.png">

6. Upload the [firmware](/Firmware) according to the instructions for your version. 
7. Interface with the PowerMeter accordingly as stated [here](/docu/README_Firmware_Cmds.md) 

# Use cases
## Smart homes
There are plenty of use cases for a smart plug inside a smart home.
* Switch appliances _on_ and _off_ using a voice assistant or your smartphone
* Include it in home automation software to switch appliances based on an automation. There are lots of possibilities here, e.g.: 
    * Switch appliances _on_ and _off_ by time of day.
    * Switch appliances _off_ automatically at standby consumption.
    * Trigger an alert if the consumption exceeds a limit.
    
      <p align="center">
      <img src="/docu/figures/officeSpeaker.jpeg" width="200px">
      </p>
* Monitor the power consumption of an appliance over day. See this faulty fridge as an example.
         
<p align="center">
<img src="/docu/figures/fridge.png" width="600px">
</p>

## For research purpose
* Analyze high frequency voltage and current waveforms.
         
   <p align="center">
   <img src="/docu/figures/fridgeUI.png" width="600px">
   </p>
* Record high frequency electricity datasets (see e.g. the [FIRED](https://github.com/voelkerb/FIRED_dataset_helper) dataset which can be used for Non-Intrusive Load Monitoring).


# License:
## Firmware:
Copyright (c) 2019 Benjamin Völker. All rights reserved.
This work is licensed under the terms of the [CC 4.0 licence](https://creativecommons.org/licenses/by/4.0/).

## Hardware:
Copyright (c) 2019 Benjamin Völker. All rights reserved.
This work is licensed under the terms of the [TAPR Open Hardware License](https://web.tapr.org/TAPR_Open_Hardware_License_v1.0.txt).

# Reference

Please cite our publications if you compare to or use this system:
* Benjamin Völker, Philipp M. Scholl, and Bernd Becker. 2019. Semi-Automatic Generation and Labeling of Training Data for Non-intrusive Load Monitoring. In Proceedings of the Tenth ACM International Conference on Future Energy Systems (e-Energy '19). Association for Computing Machinery, New York, NY, USA, 17–23. DOI:https://doi.org/10.1145/3307772.3328295
 
* Benjamin Völker, Marc Pfeifer, Philipp M. Scholl, and Bernd Becker. 2020. FIRED: A Fully-labeled hIgh-fRequency Electricity Disaggregation Dataset. In Proceedings of the 7th ACM International Conference on Systems for Energy-Efficient Buildings, Cities, and Transportation (BuildSys '20). Association for Computing Machinery, New York, NY, USA, 294–297. DOI:https://doi.org/10.1145/3408308.3427623

* Völker, B.; Pfeifer, M.; Scholl, P.M.; Becker, B. A Framework to Generate and Label Datasets for Non-Intrusive Load Monitoring. Energies 2021, 14, 75. https://doi.org/10.3390/en14010075


Kudos also go to Pascal Verboket & Valentin Czisch for developing the first draft schematics prior to [Version 1.0](/Schematic/Version_1).
