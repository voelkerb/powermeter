#include <SPI.h>
#include "STPM.h"

#define SERIAL_SPEED 115200
#define DEBUG

// Pins for STPM34 SPI Connection
// const int STPM_CS = 8;
// const int STPM_SYN = 9;
// // Reset pin of STPM
// const int STPM_RES = 7;
const int STPM_CS = 15;
const int STPM_SYN = 4;
// Reset pin of STPM
const int STPM_RES = 5;

// STPM Object
STPM stpm34(STPM_RES, STPM_CS, STPM_SYN);

void setup() {
  // Begin Serial Communication
  Serial.begin(SERIAL_SPEED);

  // Init STPM device (set pins, configure etc)
  stpm34.init();

  delay(2000);

  Serial.println("Setup done");
}

// the loop routine runs over and over again forever:
void loop() {
  float values[2] = {0,0};
  stpm34.readRMSVoltageAndCurrent(1, &values[0], &values[1]);
  float periods[2] = {0,0};
  stpm34.readPeriods(&periods[0], &periods[1]);
  float activeEnergy = stpm34.readActiveEnergy(1);
  float fEnergy = stpm34.readFundamentalEnergy(1);
  float reEnergy = stpm34.readReactiveEnergy(1);
  float aEnergy = stpm34.readApparentEnergy(1);
  float activePower = stpm34.readActivePower(1);
  float fPower = stpm34.readFundamentalPower(1);
  float rPower = stpm34.readReactivePower(1);
  float aPower = stpm34.readApparentRMSPower(1);
  float avPower = stpm34.readApparentVectorialPower(1);
  float maPower = stpm34.readMomentaryActivePower(1);
  float mfPower = stpm34.readMomentaryFundamentalPower(1);
  float v = stpm34.readVoltage(1);
  float i = stpm34.readCurrent(1);
  float fv = stpm34.readFundamentalVoltage(1);
  Serial.print("Vrms: ");
  Serial.print(values[0]);
  Serial.print("V\tCrms: ");
  Serial.print(values[1]);
  Serial.print("mA\t");
  Serial.print(" v: ");
  Serial.print(v);
  Serial.print("\ti: ");
  Serial.print(i);
  Serial.print(" \tperiod0: ");
  Serial.print(periods[0]);
  Serial.print("\tperiod1: ");
  Serial.print(periods[1]);
  Serial.print("\taE: ");
  Serial.print(activeEnergy);
  Serial.print("\taP: ");
  Serial.print(activePower);
  Serial.println("");
  delay(100);
}
