#ifndef ENCODER_ADVANCED_H
#define ENCODER_ADVANCED_H

#include "Arduino.h"

class EncoderAdvanced {
  public:
    EncoderAdvanced(int pinA1, int pinB1, int pinA2, int pinB2,
                    int pinA3, int pinB3, int pinA4, int pinB4);

    void setupEncoders();

    int Poss_1A(); int Poss_1B();
    int Poss_2A(); int Poss_2B();
    int Poss_3A(); int Poss_3B();
    int Poss_4A(); int Poss_4B();

    void resetEncoders();

    void encoderInterrupt1();
    void encoderInterrupt2();
    void encoderInterrupt3();
    void encoderInterrupt4();

    static void interruptWrapper1();
    static void interruptWrapper2();
    static void interruptWrapper3();
    static void interruptWrapper4();

  private:
    int _pinA1, _pinB1;
    int _pinA2, _pinB2;
    int _pinA3, _pinB3;
    int _pinA4, _pinB4;

    volatile int pos1A, pos1B;
    volatile int pos2A, pos2B;
    volatile int pos3A, pos3B;
    volatile int pos4A, pos4B;

    static EncoderAdvanced* _instance;
};

#endif
