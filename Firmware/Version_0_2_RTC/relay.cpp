/***************************************************
 Library for switching a bistable relais using set
 and reset pins

 Feel free to use the code as it is.

 Benjamin Völker, voelkerb@me.com
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

#include "Relay.h"


Relay::Relay(int pinSet, int pinReset) {
  SET_PIN = pinSet;
  RES_PIN = pinReset;
  state = -1;
  pinMode(SET_PIN, OUTPUT);
  pinMode(RES_PIN, OUTPUT);
}

void Relay::set(bool value) {
  if (state != -1 and (bool)state == value) return;
  digitalWrite(SET_PIN, value);
  digitalWrite(RES_PIN, !value);
  delay(HOLD_TIME_MS);
  // Relais is bistable, so disable pins to avoid constant power drawn by transistor
  digitalWrite(SET_PIN, LOW);
  digitalWrite(RES_PIN, LOW);
  state = (int)value;
}
