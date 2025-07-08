#include "EncoderAdvanced.h"

EncoderAdvanced* EncoderAdvanced::_instance = nullptr;

EncoderAdvanced::EncoderAdvanced(int pinA1, int pinB1, int pinA2, int pinB2,
                                 int pinA3, int pinB3, int pinA4, int pinB4)
  : _pinA1(pinA1), _pinB1(pinB1), _pinA2(pinA2), _pinB2(pinB2),
    _pinA3(pinA3), _pinB3(pinB3), _pinA4(pinA4), _pinB4(pinB4),
    pos1A(0), pos1B(0), pos2A(0), pos2B(0), pos3A(0), pos3B(0), pos4A(0), pos4B(0) {
  _instance = this;
}

void EncoderAdvanced::setupEncoders() {
  pinMode(_pinA1, INPUT_PULLUP); pinMode(_pinB1, INPUT_PULLUP);
  pinMode(_pinA2, INPUT_PULLUP); pinMode(_pinB2, INPUT_PULLUP);
  pinMode(_pinA3, INPUT_PULLUP); pinMode(_pinB3, INPUT_PULLUP);
  pinMode(_pinA4, INPUT_PULLUP); pinMode(_pinB4, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(_pinA1), interruptWrapper1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(_pinA2), interruptWrapper2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(_pinA3), interruptWrapper3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(_pinA4), interruptWrapper4, CHANGE);
}

int EncoderAdvanced::Poss_1A() { return pos1A; }
int EncoderAdvanced::Poss_1B() { return pos1B; }
int EncoderAdvanced::Poss_2A() { return pos2A; }
int EncoderAdvanced::Poss_2B() { return pos2B; }
int EncoderAdvanced::Poss_3A() { return pos3A; }
int EncoderAdvanced::Poss_3B() { return pos3B; }
int EncoderAdvanced::Poss_4A() { return pos4A; }
int EncoderAdvanced::Poss_4B() { return pos4B; }

void EncoderAdvanced::resetEncoders() {
  pos1A = pos1B = pos2A = pos2B = pos3A = pos3B = pos4A = pos4B = 0;
}

void EncoderAdvanced::encoderInterrupt1() {
  int a = digitalRead(_pinA1);
  int b = digitalRead(_pinB1);
  if (a == HIGH) pos1A += (b == LOW) ? -1 : 1;
  else pos1B += (b == LOW) ? 1 : -1;
}

void EncoderAdvanced::encoderInterrupt2() {
  int a = digitalRead(_pinA2);
  int b = digitalRead(_pinB2);
  if (a == HIGH) pos2A += (b == LOW) ? -1 : 1;
  else pos2B += (b == LOW) ? 1 : -1;
}

void EncoderAdvanced::encoderInterrupt3() {
  int a = digitalRead(_pinA3);
  int b = digitalRead(_pinB3);
  if (a == HIGH) pos3A += (b == LOW) ? -1 : 1;
  else pos3B += (b == LOW) ? 1 : -1;
}

void EncoderAdvanced::encoderInterrupt4() {
  int a = digitalRead(_pinA4);
  int b = digitalRead(_pinB4);
  if (a == HIGH) pos4A += (b == LOW) ? -1 : 1;
  else pos4B += (b == LOW) ? 1 : -1;
}

void EncoderAdvanced::interruptWrapper1() { if (_instance) _instance->encoderInterrupt1(); }
void EncoderAdvanced::interruptWrapper2() { if (_instance) _instance->encoderInterrupt2(); }
void EncoderAdvanced::interruptWrapper3() { if (_instance) _instance->encoderInterrupt3(); }
void EncoderAdvanced::interruptWrapper4() { if (_instance) _instance->encoderInterrupt4(); }
