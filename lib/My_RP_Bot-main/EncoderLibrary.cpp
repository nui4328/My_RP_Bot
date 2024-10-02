#include "EncoderLibrary.h"

// Static instance initialization
EncoderLibrary* EncoderLibrary::_instance = nullptr;

EncoderLibrary::EncoderLibrary(int pinA1, int pinB1, int pinA2, int pinB2)
  : _encoderPinA1(pinA1), _encoderPinB1(pinB1), _encoderPinA2(pinA2), _encoderPinB2(pinB2),
    encoderPoss1(0), encoderPoss2(0) {
  _instance = this; // Assign static instance to current object
}

void EncoderLibrary::setupEncoder() {
  pinMode(_encoderPinA1, INPUT_PULLUP);
  pinMode(_encoderPinB1, INPUT_PULLUP);
  pinMode(_encoderPinA2, INPUT_PULLUP);
  pinMode(_encoderPinB2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(_encoderPinA1), encoderInterrupt1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(_encoderPinA2), encoderInterrupt2, CHANGE);
}

int EncoderLibrary::Poss_L() {
  return encoderPoss1;
}

int EncoderLibrary::Poss_R() {
  return encoderPoss2;
}

void EncoderLibrary::resetEncoders() {
  encoderPoss1 = 0;
  encoderPoss2 = 0;
}

// Interrupt Service Routine for Encoder 1
void EncoderLibrary::encoderInterrupt1() {
  int encoderPinA1Val = digitalRead(_instance->_encoderPinA1);
  if (encoderPinA1Val == HIGH) {
    if (digitalRead(_instance->_encoderPinB1) == LOW) {
      _instance->encoderPoss1--;
    } else {
      _instance->encoderPoss1++;
    }
  }
}

// Interrupt Service Routine for Encoder 2
void EncoderLibrary::encoderInterrupt2() {
  int encoderPinA2Val = digitalRead(_instance->_encoderPinA2);
  if (encoderPinA2Val == HIGH) {
    if (digitalRead(_instance->_encoderPinB2) == LOW) {
      _instance->encoderPoss2--;
    } else {
      _instance->encoderPoss2++;
    }
  }
}
