# PowerMeter

The PowerMeter is a WiFi-enabled electricity meter that can also control the connected appliance’s power status.
It enables the recording of high-frequency voltage and current waveforms or integration into smart home systems via protocols such as _MQTT_.
Additional modular components facilitate the integration of other wireless connectivity options, including LoRaWAN, and the embedding of additional sensors within the housing. 

<p align="center">
<img src="/docu/figures/Powermeter.jpg" width="200px">
</p>

### Clone this repo with
```bash
git clone --recurse-submodules git@github.com:voelkerb/powermeter.git
```

If you are interested in gathering the entire house energy consumption, please refer to the [SmartMeter](https://github.com/voelkerb/smartmeter) instead.

## Hardware Version _1_ vs _2_
The development commenced with hardware [version 1.0](Schematic/Version_1). It comprises an ESP8266 microcontroller and a dedicated electricity monitoring chip. As the primary objective of this project was to record voltage and current waveforms at elevated frequencies, several limitations associated with this application were addressed in hardware [version 2.0](Schematic/Version_2). This version features a more powerful microcontroller with 8MB of internal storage for data buffering, an RTC for precise sampling and timekeeping, and a full 16A relay, enabling the control of high-power appliances such as dishwashers. The USB connection present in hardware [version 1.0](Schematic/Version_1) has been eliminated, but could be reintroduced with an additional extension board.

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
Are you interested in constructing your own PowerMeter? Since the PCBs are provided in this repository, you can simply assemble your own.

Depending on your specific requirements, you may choose to build either [Version 1.0](/Schematic/Version_1) or [Version 2.0](/Schematic/Version_2).
* [Version 1.0](/Schematic/Version_1) incorporates an ESP8266 microcontroller and a 10A solid-state relay.
    * Advantages:
        * The components are more cost-effective (approximately 30€).
        * It includes a USB interface for uploading the firmware or streaming data.
    * Disadvantages:
        * It lacks modularity.
        * The 10A relay can only switch connected appliances with a maximum power consumption of less than 2300 Watt.
* [Version 2.0](/Schematic/Version_2) features an ESP32 microcontroller and a 16A bistable relay.
    * Advantages:
        * You can switch appliances with a maximum power consumption of up to 3600 Watt.
        * It possesses a multi-core microcontroller that also supports Bluetooth.
        * 8MB of internal storage enables data buffering. If you stream high-frequency data at, for instance, 2kHz, this can accommodate up to 500s of data during network interruptions.
        * An RTC maintains timekeeping, allowing you to synchronize sampling rates during high-frequency sampling.
        * It can be extended by stacking additional modules on an extension header.
    * Disadvantages:
        * The cost of the components is slightly higher (approximately 35€).

Steps to build your own PowerMeter:

1. Create the PCB ([version 1.0](/Schematic/Version_1)) or [version 2.0](/Schematic/Version_2))). We recommend using [JLC PCB](https://jlcpcb.com) because it’s very affordable and the quality is still good.
2. Purchase the components listed in the [BOM](/BOM) (Bill of Materials).
3. Solder everything together. We used a small reflow oven, but a fine soldering iron could also be used.
4. Buy a matching housing. We highly recommend using [this one](https://www.conrad.de/de/p/bopla-eletec-se-432-de-cee-stecker-gehaeuse-120-x-65-x-50-abs-polycarbonat-lichtgrau-graphitgrau-1-st-522228.html) because the PCB was specially designed for it, the mounting holes match, and it’s safe to use with 230V. We also tried custom 3D prints (see [CAD](/CAD)), but it’s at your own risk.

5. Wire the PowerMeters using the provided diagram. 

<img src="/docu/figures/socket.png">

6. Upload the [firmware](/Firmware) according to the instructions for your version. 
7. Interface with the PowerMeter according to the protocol documented [here](/docu/README_Firmware_Cmds.md) 

# Use cases
## Smart homes
There are plenty of use cases for a smart plug inside a smart home.
* Switch appliances _on_ and _off_ using a voice assistant or your smartphone
* Include it in home automation software to switch appliances based on automations, e.g.: 
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
