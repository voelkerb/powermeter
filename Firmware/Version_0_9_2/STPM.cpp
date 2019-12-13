/***************************************************
 Library for getting energy-data out of a STPM3X.
 See example file to get an idea of how to use this
 class.

 Feel free to use the code as it is.

 Benjamin Völker, voelkerb@me.com
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

#include "STPM.h"
#include <SPI.h>

//SPISettings spiSettings(7900000, MSBFIRST, SPI_MODE3);

DSP_CR100bits_t DSP_CR100bits;
DSP_CR101bits_t DSP_CR101bits;
DSP_CR200bits_t DSP_CR200bits;
DSP_CR201bits_t DSP_CR201bits;
DSP_CR400bits_t DSP_CR400bits;
US1_REG100bits_t US1_REG100bits;

DFE_CR101bits_t DFE_CR101bits;
DFE_CR201bits_t DFE_CR201bits;
DSP_CR301bits_t DSP_CR301bits;
DSP_CR500bits_t DSP_CR500bits;



STPM::STPM(int resetPin, int csPin, int synPin) {
  RESET_PIN = resetPin;
  CS_PIN = csPin;
  SYN_PIN = synPin;
  _autoLatch = true;
  _crcEnabled = true;
}

STPM::STPM(int resetPin, int csPin) {
  RESET_PIN = resetPin;
  CS_PIN = csPin;
  SYN_PIN = -1;
  _autoLatch = true;
  _crcEnabled = true;
}

void STPM::init() {
  pinMode(CS_PIN, OUTPUT);
  if (SYN_PIN != -1) pinMode(SYN_PIN, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(CS_PIN, LOW);
  digitalWrite(RESET_PIN, LOW);
  delay(35);
  digitalWrite(RESET_PIN, HIGH);
  delay(35);
  digitalWrite(CS_PIN, HIGH);
  // Init sequence by togling 3 times syn pin
  if (SYN_PIN != -1) {
    for (int i = 0; i < 3; i++) {
      digitalWrite(SYN_PIN, LOW);
      delay(2);
      digitalWrite(SYN_PIN, HIGH);
      delay(2);
    }
  }
  digitalWrite(CS_PIN, LOW);
  delay(5);
  digitalWrite(CS_PIN, HIGH);
  SPI.begin();
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE2));
  Init_STPM34();
  #ifdef DEBUG_DEEP
	Serial.println(F("End Init"));
  #endif
}

void STPM::Init_STPM34() {
  #ifdef DEBUG_DEEP
	Serial.println(F("Start Init_STPM34"));
  #endif
  u8 readAdd, writeAdd, dataLSB, dataMSB;

  //set Voltage Reference
  #ifdef DEBUG
  Serial.println(F("Info:Set DSP Control Register 1 LSW"));
  #endif
  DSP_CR100bits.ENVREF1 = 1;       //enable internal Vref1 bit for CH0;
  DSP_CR100bits.TC1 = 0x02;         //set temperature compensation for CH0; Vref1=1.18v default
  readAdd = 0x00;
  writeAdd = 0x00;
  dataLSB = DSP_CR100bits.LSB;
  dataMSB = DSP_CR100bits.MSB;
  sendFrameCRC(0x00, 0x00, dataLSB, dataMSB); //write to  CR1 register

  #ifdef DEBUG
  Serial.println(F("Info:Set DSP Control Register 1 MSW"));
  #endif
  DSP_CR101bits.BHPFV1 = 0;//1;//0;        //HPF enable voltage;DC cancellation
  DSP_CR101bits.BHPFC1 = 0;//1;//0;        //HPF enable current;DC cancellation
  DSP_CR101bits.BLPFV1 = 1;        //LPF wideband voltage;set up fundamental mode
  DSP_CR101bits.BLPFC1 = 1;        //LPF wideband current;set up fundamental mode
  DSP_CR101bits.LPW1 = 0x04;        //LED Division factor
  readAdd = 0x00;
  writeAdd = 0x01;
  dataLSB = DSP_CR101bits.LSB;
  dataMSB = DSP_CR101bits.MSB;
  sendFrameCRC(readAdd, writeAdd, dataLSB, dataMSB); //write to  CR1 register

  #ifdef DEBUG
  Serial.println(F("Info:Set DSP Control Register 2 LSW"));
  #endif
  DSP_CR200bits.ENVREF2 = 1;       //enable internal Vref1 bit for CH1;
  DSP_CR200bits.TC2 = 0x02;        //set temperature compensation for CH1;  Vref2=1.18v default
  readAdd = 0x01;
  writeAdd = 0x02;
  dataLSB = DSP_CR200bits.LSB;
  dataMSB = DSP_CR200bits.MSB;
  sendFrameCRC(readAdd, writeAdd, dataLSB, dataMSB); //write to  CR2 register

  #ifdef DEBUG
  Serial.println(F("Info:Set DSP Control Register 2 MSW"));
  #endif
  DSP_CR201bits.BHPFV2 = 0;//1;//0;        //HPF enable voltage;DC cancellation
  DSP_CR201bits.BHPFC2 = 0;//1;//0;        //HPF enable current;DC cancellation
  DSP_CR201bits.BLPFV2 = 1;        //LPF bypassed -  wideband voltage;set up fundamental mode
  DSP_CR201bits.BLPFC2 = 1;        //LPF bypassed -  wideband current;set up fundamental mode
  DSP_CR201bits.LPW2 = 0x04;        //LED Division factor
  readAdd = 0x02;
  writeAdd = 0x03;
  dataLSB = DSP_CR201bits.LSB;
  dataMSB = DSP_CR201bits.MSB;
  sendFrameCRC(readAdd, writeAdd, dataLSB, dataMSB);

  // Set current gain: 0x00 = 2, 0x01 = 4, 0x02 = 8, 0x03 = 16
  setCurrentGain(1, sixteenX);
  setCurrentGain(2, sixteenX);


  #ifdef DEBUG
  Serial.println(F("Info:GAIN (Bit 26/27):"));
  // Read Gain
  readFrame(0x18, readBuffer);
  printRegister(readBuffer, "GainC1:");
  readFrame(0x1A, readBuffer);
  printRegister(readBuffer, "GainC2:");

  Serial.println(F("Info:LPW: (Bit 24-27)"));
  readFrame(0x0, readBuffer);
  printRegister(readBuffer, "LPW1:");
  readFrame(0x01, readBuffer);
  printRegister(readBuffer, "LPW2:");
  #endif

  autoLatch(false);
  CRC(false);

  #ifdef DEBUG_DEEP
	Serial.println(F("End Init_STPM34"));
  #endif
}

void STPM::CRC(bool enabled) {
  if (_crcEnabled == enabled) return;
  // Disable CRC
  if (!enabled) {
    #ifdef DEBUG
    Serial.println(F("Info:Disable CRC"));
    #endif
    US1_REG100bits.CRC_EN=0;         //disable CRC polynomial
    sendFrameCRC(0x24,0x24,US1_REG100bits.LSB,US1_REG100bits.MSB);
  // Enable CRC
  } else {
    #ifdef DEBUG
    Serial.println(F("Info:Enable CRC"));
    #endif
    US1_REG100bits.CRC_EN=1;         //disable CRC polynomial
    sendFrame(0x24,0x24,US1_REG100bits.LSB,US1_REG100bits.MSB);
  }
  _crcEnabled = enabled;
}


void STPM::autoLatch(bool enabled) {
  #ifdef DEBUG
  Serial.println(F("Info:Set DSP Control Register 3 LSW"));
  if (enabled) {
    Serial.println(F("Info:Automatic latching"));
  } else {
    Serial.println(F("Info:Manual latching"));
  }
  #endif
  _autoLatch = enabled;
  if (_autoLatch) {
    DSP_CR301bits.SWAuto_Latch = 1;      // Automatic measurement register latch at 7.8125 kHz
    DSP_CR301bits.SW_Latch1 = 0;
    DSP_CR301bits.SW_Latch2 = 0;
  } else {
    DSP_CR301bits.SWAuto_Latch = 0;      // Automatic measurement register latch at 7.8125 kHz
    DSP_CR301bits.SW_Latch1 = 1;
    DSP_CR301bits.SW_Latch2 = 1;
  }
  if (_crcEnabled) sendFrameCRC(0x05, 0x05, DSP_CR301bits.LSB, DSP_CR301bits.MSB);
  else sendFrame(0x05, 0x05, DSP_CR301bits.LSB, DSP_CR301bits.MSB);
}


void STPM::readAll(uint8_t channel, float *voltage, float *current, float* active, float* reactive) {
  if (!_autoLatch) latch();

  address = PH1_Active_Power_Address;
  if (channel == 2) address = PH2_Active_Power_Address;
  sendFrame(address, 0xff, 0xff, 0xff);
  //SPI.beginTransaction(spiSettings);
  digitalWrite(CS_PIN, LOW);
  //delayMicroseconds(1000);
  readBuffer[0] = SPI.transfer(0xff);
  readBuffer[1] = SPI.transfer(0xff);
  readBuffer[2] = SPI.transfer(0xff);
  readBuffer[3] = SPI.transfer(0xff);
  *active = calcPower(buffer0to28(readBuffer));
  readBuffer[0] = SPI.transfer(0xff);
  readBuffer[1] = SPI.transfer(0xff);
  readBuffer[2] = SPI.transfer(0xff);
  readBuffer[3] = SPI.transfer(0xff);
  // We skip fundamental power here
  readBuffer[0] = SPI.transfer(0xff);
  readBuffer[1] = SPI.transfer(0xff);
  readBuffer[2] = SPI.transfer(0xff);
  readBuffer[3] = SPI.transfer(0xff);
  *reactive = calcPower(buffer0to28(readBuffer));
  digitalWrite(CS_PIN, HIGH);
  // *apparent = calcPower(buffer0to28(readBuffer));
  address = V1_Data_Address;
  if (channel == 2) address = V2_Data_Address;
  sendFrame(address, 0xff, 0xff, 0xff);
  //SPI.beginTransaction(spiSettings);
  digitalWrite(CS_PIN, LOW);
  //delayMicroseconds(1000);
  readBuffer[0] = SPI.transfer(0xff);
  readBuffer[1] = SPI.transfer(0xff);
  readBuffer[2] = SPI.transfer(0xff);
  readBuffer[3] = SPI.transfer(0xff);
  *voltage = calcVolt(buffer0to32(readBuffer));
  readBuffer[0] = SPI.transfer(0xff);
  readBuffer[1] = SPI.transfer(0xff);
  readBuffer[2] = SPI.transfer(0xff);
  readBuffer[3] = SPI.transfer(0xff);
  digitalWrite(CS_PIN, HIGH);
  *current = calcCurrent(buffer0to32(readBuffer));
}

void STPM::setCurrentGain(uint8_t channel, Gain gain) {
  // Set current gain: 0x00 = 2, 0x01 = 4, 0x02 = 8, 0x03 = 16
  if (channel != 1 && channel != 2) {
    #ifdef DEBUG
    Serial.print(F("Info:setCurrentGain: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return;
  }
  u8 readAdd, writeAdd, dataLSB, dataMSB;


  if (channel == 1) {
    // Read Gain
    readFrame(0x18, readBuffer);
    //set GAIN1, in DFE_CR1 register
    readAdd = 0x00;
    writeAdd = 0x19;
    DFE_CR101bits.LSB = readBuffer[2];
    DFE_CR101bits.MSB = readBuffer[3];
    DFE_CR101bits.GAIN1 = gain;
    dataLSB = DFE_CR101bits.LSB;
    dataMSB = DFE_CR101bits.MSB;    //set current gain for Chn1
  } else {
    //set GAIN2 in DFE_CR2 register
    readAdd = 0x00;
    writeAdd = 0x1B;
    DFE_CR201bits.LSB = readBuffer[0];
    DFE_CR201bits.MSB = readBuffer[1];
    DFE_CR201bits.GAIN1 = gain;
    dataLSB = DFE_CR201bits.LSB;
    dataMSB = DFE_CR201bits.MSB;
  }
  sendFrameCRC(readAdd, writeAdd, dataLSB, dataMSB);
}


void STPM::readPeriods(float* ch1, float* ch2) {
  if (!_autoLatch) latch();
  u8 address = Period_Address;
  readFrame(address, readBuffer);
  *ch1 = calcPeriod(buffer0to11(readBuffer));
  *ch2 = calcPeriod(buffer16to27(readBuffer));
}


float STPM::readTotalActiveEnergy() {
  if (!_autoLatch) latch();
  u8 address = Tot_Active_Energy_Address;
  readFrame(address, readBuffer);
  return (calcEnergy(buffer0to32(readBuffer)));
}

float STPM::readTotalFundamentalEnergy() {
  if (!_autoLatch) latch();
  u8 address = Tot_Fundamental_Energy_Address;
  readFrame(address, readBuffer);
  return (calcEnergy(buffer0to32(readBuffer)));
}

float STPM::readTotalReactiveEnergy() {
  if (!_autoLatch) latch();
  u8 address = Tot_Reactive_Energy_Address;
  readFrame(address, readBuffer);
  return (calcEnergy(buffer0to32(readBuffer)));
}

float STPM::readTotalApparentEnergy() {
  if (!_autoLatch) latch();
  u8 address = Tot_Apparent_Energy_Address;
  readFrame(address, readBuffer);
  return (calcEnergy(buffer0to32(readBuffer)));
}

float STPM::readActiveEnergy(uint8_t channel) {
  if (!_autoLatch) latch();
  if (channel != 1 && channel != 2) {
    #ifdef DEBUG
    Serial.print(F("Info:readActiveEnergy: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  u8 address = PH1_Active_Energy_Address;
  if (channel == 2) address = PH2_Active_Energy_Address;
  readFrame(address, readBuffer);
  return (calcEnergy(buffer0to32(readBuffer)));
}

float STPM::readFundamentalEnergy(uint8_t channel) {
  if (!_autoLatch) latch();
  if (channel != 1 && channel != 2) {
    #ifdef DEBUG
    Serial.print(F("Info:readFundamentalEnergy: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  u8 address = PH1_Fundamental_Energy_Address;
  if (channel == 2) address = PH2_Fundamental_Energy_Address;
  readFrame(address, readBuffer);
  return (calcEnergy(buffer0to32(readBuffer)));
}

float STPM::readReactiveEnergy(uint8_t channel) {
  if (!_autoLatch) latch();
  if (channel != 1 && channel != 2) {
    #ifdef DEBUG
    Serial.print(F("Info:readReactiveEnergy: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  u8 address = PH1_Reactive_Energy_Address;
  if (channel == 2) address = PH2_Reactive_Energy_Address;
  readFrame(address, readBuffer);
  return (calcEnergy(buffer0to32(readBuffer)));
}

float STPM::readApparentEnergy(uint8_t channel) {
  if (!_autoLatch) latch();
  if (channel != 1 && channel != 2) {
    #ifdef DEBUG
    Serial.print(F("Info:readApparentEnergy: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  u8 address = PH1_Apparent_Energy_Address;
  if (channel == 2) address = PH2_Apparent_RMS_Energy_Address;
  readFrame(address, readBuffer);
  return (calcEnergy(buffer0to32(readBuffer)));
}

void STPM::readPower(uint8_t channel, float* active, float* fundamental, float* reactive, float* apparent) {
  if (!_autoLatch) latch();
  if (channel != 1 && channel != 2) {
    #ifdef DEBUG
    Serial.print(F("Info:readPower: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return;
  }
  address = PH1_Active_Power_Address;
  if (channel == 2) address = PH2_Active_Power_Address;
  sendFrame(address, 0xff, 0xff, 0xff);
  //SPI.beginTransaction(spiSettings);
  digitalWrite(CS_PIN, LOW);
  //delayMicroseconds(1000);
  readBuffer[0] = SPI.transfer(0xff);
  readBuffer[1] = SPI.transfer(0xff);
  readBuffer[2] = SPI.transfer(0xff);
  readBuffer[3] = SPI.transfer(0xff);
  *active = calcPower(buffer0to28(readBuffer));
  readBuffer[0] = SPI.transfer(0xff);
  readBuffer[1] = SPI.transfer(0xff);
  readBuffer[2] = SPI.transfer(0xff);
  readBuffer[3] = SPI.transfer(0xff);
  *fundamental = calcPower(buffer0to28(readBuffer));
  readBuffer[0] = SPI.transfer(0xff);
  readBuffer[1] = SPI.transfer(0xff);
  readBuffer[2] = SPI.transfer(0xff);
  readBuffer[3] = SPI.transfer(0xff);
  *reactive = calcPower(buffer0to28(readBuffer));
  readBuffer[0] = SPI.transfer(0xff);
  readBuffer[1] = SPI.transfer(0xff);
  readBuffer[2] = SPI.transfer(0xff);
  readBuffer[3] = SPI.transfer(0xff);
  digitalWrite(CS_PIN, HIGH);
  *apparent = calcPower(buffer0to28(readBuffer));
}

float STPM::readActivePower(uint8_t channel) {
  if (!_autoLatch) latch();
  if (channel != 1 && channel != 2) {
    #ifdef DEBUG
    Serial.print(F("Info:readActivePower: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  u8 address = PH1_Active_Power_Address;
  if (channel == 2) address = PH2_Active_Power_Address;
  readFrame(address, readBuffer);
  return calcPower(buffer0to28(readBuffer));
}

float STPM::readFundamentalPower(uint8_t channel) {
  if (!_autoLatch) latch();
  if (channel != 1 && channel != 2) {
    #ifdef DEBUG
    Serial.print(F("Info:readFundamentalPower: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  u8 address = PH1_Fundamental_Power_Address;
  if (channel == 2) address = PH2_Fundamental_Power_Address;
  readFrame(address, readBuffer);
  return (calcPower(buffer0to28(readBuffer)));
}

float STPM::readReactivePower(uint8_t channel) {
  if (!_autoLatch) latch();
  if (channel != 1 && channel != 2) {
    #ifdef DEBUG
    Serial.print(F("Info:readReactivePower: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  u8 address = PH1_Reactive_Power_Address;
  if (channel == 2) address = PH2_Reactive_Power_Address;
  readFrame(address, readBuffer);
  return (calcPower(buffer0to28(readBuffer)));
}

float STPM::readApparentRMSPower(uint8_t channel) {
  if (!_autoLatch) latch();
  if (channel != 1 && channel != 2) {
    #ifdef DEBUG
    Serial.print(F("Info:readApparentRMSPower: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  u8 address = PH1_Apparent_RMS_Power_Address;
  if (channel == 2) address = PH2_Apparent_RMS_Power_Address;
  readFrame(address, readBuffer);
  return (calcPower(buffer0to28(readBuffer)));
}

float STPM::readApparentVectorialPower(uint8_t channel) {
  if (!_autoLatch) latch();
  if (channel != 1 && channel != 2) {
    #ifdef DEBUG
    Serial.print(F("Info:readApparentVectorialPower: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  u8 address = PH1_Apparent_Vectorial_Power_Address;
  if (channel == 2) address = PH2_Apparent_Vectorial_Power_Address;
  readFrame(address, readBuffer);
  return (calcPower(buffer0to28(readBuffer)));
}

float STPM::readMomentaryActivePower(uint8_t channel) {
  if (!_autoLatch) latch();
  if (channel != 1 && channel != 2) {
    #ifdef DEBUG
    Serial.print(F("Info:readMomentaryActivePower: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  u8 address = PH1_Momentary_Active_Power_Address;
  if (channel == 2) address = PH2_Momentary_Active_Power_Address;
  readFrame(address, readBuffer);
  return (calcPower(buffer0to28(readBuffer)));
}

float STPM::readMomentaryFundamentalPower(uint8_t channel) {
  if (!_autoLatch) latch();
  if (channel != 1 && channel != 2) {
    #ifdef DEBUG
    Serial.print(F("Info:readMomentaryFundamentalPower: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  u8 address = PH1_Momentary_Fundamental_Power_Address;
  if (channel == 2) address = PH2_Momentary_Fundamental_Power_Address;
  readFrame(address, readBuffer);
  return (calcPower(buffer0to28(readBuffer)));
}

// Overvoltage/overcurrent SWELL and undervoltage SAG
void STPM::readCurrentPhaseAndSwellTime(uint8_t channel, float* phase, float* swell) {
  if (!_autoLatch) latch();
  if (channel != 1 && channel != 2) {
    #ifdef DEBUG
    Serial.print(F("Info:readCurrentPhaseAndSwellTime: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    *phase = -1;
    *swell = -1;
    return;
  }
  u8 address = C1PHA_SWC1_TIME_Address;
  if (channel == 2) address = C2PHA_SWC2_TIME_Address;
  readFrame(address, readBuffer);
  *swell = (buffer0to14(readBuffer));
  *phase = (buffer16to27(readBuffer));
}

// Overvoltage/overcurrent SWELL and undervoltage SAG
void STPM::readVoltageSagAndSwellTime(uint8_t channel, float* sag, float* swell) {
  if (!_autoLatch) latch();
  if (channel != 1 && channel != 2) {
    #ifdef DEBUG
    Serial.print(F("Info:readSagAndSwellTime: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    *sag = -1;
    *swell = -1;
    return;
  }
  u8 address = SAG1_SWV1_TIME_Address;
  if (channel == 2) address = SAG2_SWV2_TIME_Address;
  readFrame(address, readBuffer);
  *swell = (buffer0to14(readBuffer));
  *sag = (buffer16to30(readBuffer));
}

void STPM::readVoltageAndCurrent(uint8_t channel, float *voltage, float *current) {
  if (!_autoLatch) latch();

  if (channel != 1 && channel != 2) {
    #ifdef DEBUG
    Serial.print(F("Info:readVoltage: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return;
  }
  address = V1_Data_Address;
  if (channel == 2) address = V2_Data_Address;
  sendFrame(address, 0xff, 0xff, 0xff);
  //SPI.beginTransaction(spiSettings);
  digitalWrite(CS_PIN, LOW);
  //delayMicroseconds(1000);
  readBuffer[0] = SPI.transfer(0xff);
  readBuffer[1] = SPI.transfer(0xff);
  readBuffer[2] = SPI.transfer(0xff);
  readBuffer[3] = SPI.transfer(0xff);
  *voltage = calcVolt(buffer0to32(readBuffer));
  readBuffer[0] = SPI.transfer(0xff);
  readBuffer[1] = SPI.transfer(0xff);
  readBuffer[2] = SPI.transfer(0xff);
  readBuffer[3] = SPI.transfer(0xff);
  digitalWrite(CS_PIN, HIGH);
  *current = calcCurrent(buffer0to32(readBuffer));
}

float STPM::readVoltage(uint8_t channel) {
  if (!_autoLatch) latch();
  if (channel != 1 && channel != 2) {
    #ifdef DEBUG
    Serial.print(F("Info:readVoltage: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  u8 address = V1_Data_Address;
  if (channel == 2) address = V2_Data_Address;
  readFrame(address, readBuffer);
  return (calcVolt((int32_t)buffer0to32(readBuffer)));
}


float STPM::readCurrent(uint8_t channel) {
  if (!_autoLatch) latch();
  if (channel != 1 && channel != 2) {
    #ifdef DEBUG
    Serial.print(F("Info:readCurrent: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  u8 address = C1_Data_Address;
  if (channel == 2) address = C2_Data_Address;
  readFrame(address, readBuffer);
  return (calcCurrent((int32_t)buffer0to32(readBuffer)));
}


float STPM::readFundamentalVoltage(uint8_t channel) {
  if (!_autoLatch) latch();
  if (channel != 1 && channel != 2) {
    #ifdef DEBUG
    Serial.print(F("Info:readFundamentalVoltage: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  u8 address = V1_Fund_Address;
  if (channel == 2) address = V2_Fund_Address;
  readFrame(address, readBuffer);
  return (calcVolt(buffer0to32(readBuffer)));
}

void STPM::readRMSVoltageAndCurrent(uint8_t channel, float* voltage, float* current) {
  if (!_autoLatch) latch();
  if (channel != 1 && channel != 2) {
    #ifdef DEBUG
    Serial.print(F("Info:readRMSVoltage: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    *voltage = -1;
    *current = -1;
    return;
  }
  u8 address = 0x48;
  if (channel == 2) address = 0x4A;
  readFrame(address, readBuffer);
  *voltage = calcVolt((int16_t)buffer0to14(readBuffer));
  *current = calcCurrent((int16_t)buffer15to32(readBuffer));
}


/*
* Latch a new measurement in the registers. CS_PIN should be high and automatic
* measurement off.
*/
void STPM::latchReg() {
	latch();
}

/*
* Latch a new measurement in the registers. CS_PIN should be high and automatic
* measurement off. As inline function.
*/
inline void STPM::latch() {
#ifdef SPI_LATCH
  DSP_CR301bits.SW_Latch1 = 1;      // Register latch
  DSP_CR301bits.SW_Latch2 = 1;      // Register latch
  sendFrame(0x05, 0x05, DSP_CR301bits.LSB, DSP_CR301bits.MSB);
#else
  digitalWrite(SYN_PIN, LOW);
  delayMicroseconds(4);
  digitalWrite(SYN_PIN, HIGH);
  delayMicroseconds(4);
#endif
}

/* Conversion of the register values into voltage, current, power, etc
*  see: STPM3x design and calibration guideline for customers v3.1.xlsm
*/

// In ms
inline float STPM::calcPeriod (int16_t value) {
  return (float)value * 8.0/1000.0;
}
/* Returns power in W (Power register LSB)
*  LSBP = (1+R1/R2) * Vref^2 / (Ks * kint * Av * Ai * calV  * calI * 2^28)
*  R1 810 KOhm, R2 470 Ohm, Vref = 1180 mV, Ks = 3 mOhm, Kint = 1,
*  Ac = 2, Ai = 16, calV = 0.875, calI = 0.875
*/
inline float STPM::calcPower (int32_t value) {
  return value * 0.0001217; //0.000152; old value from Benny.
}
/* Returns Energy in Ws (Energy register LSB)
*  LSBE = (1+R1/R2) * Vref^2 / (Ks * kint * Av * Ai * calV  * calI * 2^17 * Fs)
*  R1 810KOhm, R2 470 Ohm, Vref = 1180 mV, Ks = 3 mOhm, Kint = 1,
*  Ac = 2, Ai = 16, calV = 0.875, calI = 0.875, Fs = 7812,5Hz
*/
inline float STPM::calcEnergy (int32_t value) {
  return (float)value * 0.0000319; // maybe this value is 10x to small? test it.
  //0.00040; old value from Benny
}

/* Retruns current in mA (LSB current)
*  LSBC = Vref   /  (calI* Ai * 2^23 * ks * kint)
*  Vref = 1180 mV, Ks = 3 mOhm, calI = 0.875, Kint = 1, Ai = 16
*/
inline float STPM::calcCurrent (int32_t value) {
  return (float)value * 0.003349213; //1000; // 0.0041875 old value from Benny
}

/* Retruns current in mA (LSB current)
*  LSBC = Vref   /  (calI* Ai * 2^17 * ks * kint)
*  Vref = 1180 mV, Ks = 3 mOhm, calI = 0.875, Kint = 1, Ai = 16
*/
inline float STPM::calcCurrent (int16_t value) {
  return (float)value * 0.2143; //1000; // 0.26794 old value from Benny
}

/* This value is for R1 810 KOhm R2 470 Ohm
*  Instantaneous LSB voltage  = Vref*(1+R1/R2) / (calV * Av * 2^23)
*  with calV = 0.875 and Av = 2
*  retuns Voltage in V
*/
inline float STPM::calcVolt (int32_t value) {
  return ((float)value/*-14837*/)*0.000138681; ///7186.4; (from Benny function unknown)
}

/* This value is for R1 810 KOhm R2 470 Ohm
*  LSB voltage= Vref*(1+R1/R2) / (calV * Av * 2^15)
*  with Vref = 1180 mV, calV = 0.875 and Av = 2
*  retuns Voltage in V
*/
inline float STPM::calcVolt (int16_t value) {
  return ((float)value/*-56*/)* 0.0354840440; // old value from Benny  0.03550231588
}

inline int32_t STPM::buffer0to32(uint8_t *buffer) {
  return (((buffer[3] << 24) | (buffer[2] << 16)) | (buffer[1] << 8)) | buffer[0];
  // int32_t value = buffer[3];
  // value = (value << 8) + buffer[2];
  // value = (value << 8) + buffer[1];
  // value = (value << 8) + buffer[0];
  // return value;
}

inline int32_t STPM::buffer15to32(uint8_t *buffer) {
  return (((buffer[3] << 16) | (buffer[2] << 8)) | buffer[1]) >> 7;
  // int32_t value = buffer[3];
  // value = (value << 8) + buffer[2];
  // value = ((value << 8) + buffer[1]) >> 7;
  // return value;
}


inline int16_t STPM::buffer16to30(uint8_t *buffer) {
  return (buffer[3]&0x7f << 8) | buffer[2];
  // int16_t value = buffer[3]&0x7f;//7F
  // value = (value << 8) + buffer[2];
  // return value;
}

inline int32_t STPM::buffer0to28(uint8_t *buffer) {
  return (((buffer[3] << 24) | (buffer[2] << 16)) | (buffer[1] << 8)) | buffer[0];
  // int32_t value = buffer[3];
  // value = (value << 8) + buffer[2];
  // value = (value << 8) + buffer[1];
  // value = (value << 8) + buffer[0];
  // return value;
}

inline int16_t STPM::buffer0to14(uint8_t *buffer) {
  return (buffer[1]&0x7f << 8) | buffer[0];
  // int16_t value = buffer[1]&0x7f;//7F
  // value = (value << 8) + buffer[0];
  // return value;
}

inline int16_t STPM::buffer0to11(uint8_t *buffer) {
  return (buffer[1]&0x0f << 8) | buffer[0];
  // int16_t value = buffer[1]&0x0f;//7F
  // value = (value << 8) + buffer[0];
  // return value;
}

inline int16_t STPM::buffer16to27(uint8_t *buffer) {
  return (buffer[3]&0x0f << 8) | buffer[2];
  // int16_t value = buffer[3]&0x0f;//7F
  // value = (value << 8) + buffer[2];
  // return value;
}

inline void STPM::readFrame(uint8_t address, uint8_t *buffer) {
  sendFrame(address, 0xff, 0xff, 0xff);
  //SPI.beginTransaction(spiSettings);
  digitalWrite(CS_PIN, LOW);
  //delayMicroseconds(1000);
  for (uint8_t i = 0; i < 4; i++) {
    buffer[i] = SPI.transfer(0xff);
  }
  digitalWrite(CS_PIN, HIGH);
  //SPI.endTransaction();
  //delayMicroseconds(500);
}


inline void STPM::sendFrame(uint8_t readAdd, uint8_t writeAdd, uint8_t dataLSB, uint8_t dataMSB) {
  digitalWrite(CS_PIN, LOW);
  //delayMicroseconds(500);
  SPI.transfer(readAdd);
  SPI.transfer(writeAdd);
  SPI.transfer(dataLSB);
  SPI.transfer(dataMSB);
  digitalWrite(CS_PIN, HIGH);
  //delayMicroseconds(500);
}


void STPM::sendFrameCRC(uint8_t readAdd, uint8_t writeAdd, uint8_t dataLSB, uint8_t dataMSB) {
  u8 frame[STPM3x_FRAME_LEN];
  frame[0] = readAdd;
  frame[1] = writeAdd;
  frame[2] = dataLSB;
  frame[3] = dataMSB;
  frame[4] = CalcCRC8(frame);
#ifdef DEBUG_DEEP
  Serial.print("Info:Sending:");
  printFrame(frame, STPM3x_FRAME_LEN);
#endif
  u8 shit[STPM3x_FRAME_LEN] = {0x00, 0x00, 0x00, 0x00, 0x00};
  //SPI.beginTransaction(spiSettings);
  digitalWrite(CS_PIN, LOW);
  //delayMicroseconds(50);
  shit[0] = SPI.transfer(frame[0]);
  shit[1] = SPI.transfer(frame[1]);
  shit[2] = SPI.transfer(frame[2]);
  shit[3] = SPI.transfer(frame[3]);
  shit[4] = SPI.transfer(frame[4]);
  digitalWrite(CS_PIN, HIGH);
  //SPI.endTransaction();
  //delayMicroseconds(5);
  #ifdef DEBUG_DEEP
  Serial.print("Info:Shit:\t");
  printFrame(shit, STPM3x_FRAME_LEN);
  #endif
}
void STPM::printFrame(u8 *frame, uint8_t length) {
  char buffer[4];
  Serial.print(F("Info:"));
  for (uint8_t i = 0; i < length; i++) {
    sprintf(buffer, "%02x", frame[i]);
    Serial.print(F("|"));
    Serial.print(buffer);
  }
  Serial.println(F("|"));
}

void STPM::printRegister(u8 *frame, const char* regName) {
  Serial.print(F("Info:"));
  Serial.println(regName);
  Serial.print(F("Info:"));
  Serial.print(F("|"));
  for (int8_t i = 31; i >= 0; i--) {
    Serial.print(i);
    Serial.print(F("|"));
    if (i <= 10) Serial.print(" ");
  }
  Serial.println("");
  Serial.print(F("Info:"));
  Serial.print(F("| "));
  for (int8_t i = 3; i >= 0; i--) {
    Serial.print((frame[i] & 0x80) > 0);Serial.print(F("| "));
    Serial.print((frame[i] & 0x40) > 0);Serial.print(F("| "));
    Serial.print((frame[i] & 0x20) > 0);Serial.print(F("| "));
    Serial.print((frame[i] & 0x10) > 0);Serial.print(F("| "));
    Serial.print((frame[i] & 0x08) > 0);Serial.print(F("| "));
    Serial.print((frame[i] & 0x04) > 0);Serial.print(F("| "));
    Serial.print((frame[i] & 0x02) > 0);Serial.print(F("| "));
    Serial.print((frame[i] & 0x01) > 0);Serial.print(F("| "));
  }
  Serial.println("");
}

u8 STPM::CalcCRC8(u8 *pBuf) {
  u8 i;
  CRC_u8Checksum = 0x00;
  for (i = 0; i < STPM3x_FRAME_LEN - 1; i++) {
    Crc8Calc(pBuf[i]);
  }
  return CRC_u8Checksum;
}

void STPM::Crc8Calc(u8 u8Data) {
  u8 loc_u8Idx;
  u8 loc_u8Temp;
  loc_u8Idx = 0;
  while (loc_u8Idx < 8) {
    loc_u8Temp = u8Data ^ CRC_u8Checksum;
    CRC_u8Checksum <<= 1;
    if (loc_u8Temp & 0x80) {
      CRC_u8Checksum ^= CRC_8;
    }
    u8Data <<= 1;
    loc_u8Idx++;
  }
}
