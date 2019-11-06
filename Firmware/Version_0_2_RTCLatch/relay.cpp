/***************************************************
 Library for network stuff, connection, AP and so on.

 Feel free to use the code as it is.

 Benjamin Völker, voelkerb@me.com
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

#include "relay.h"


Relay::Relay(int pinSet, int pinReset) {
  _SET_PIN = pinSet;
  _RES_PIN = pinReset;
  state = -1;
  pinMode(_SET_PIN, OUTPUT);
  pinMode(_RES_PIN, OUTPUT);
}

void Relay::set(bool value) {
  if (state != -1 and (bool)state == value) return;
  digitalWrite(_SET_PIN, value);
  digitalWrite(_RES_PIN, !value);
  delay(HOLD_TIME_MS);
  // Relais is bistable, so disable pins to avoid constant power drawn by transistor
  digitalWrite(_SET_PIN, LOW);
  digitalWrite(_RES_PIN, LOW);
  state = (int)value;
}
