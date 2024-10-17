#ifndef EncoderLibrary_h
#define EncoderLibrary_h

#include "Arduino.h"

// ประกาศตัวแปรสำหรับ encoder positions
volatile int encoderPoss1 = 0;
volatile int encoderPoss2 = 0;

volatile int encoderPinA1Last = LOW;
volatile int encoderPinA2Last = LOW;

class EncoderLibrary {
  public:
    // Constructor
    EncoderLibrary(int pinA1, int pinB1, int pinA2, int pinB2)
      : _encoderPinA1(pinA1), _encoderPinB1(pinB1), _encoderPinA2(pinA2), _encoderPinB2(pinB2) {}

    // Method to setup encoder
    void setup_encoder() {
        pinMode(_encoderPinA1, INPUT_PULLUP); 
        pinMode(_encoderPinB1, INPUT_PULLUP);
        pinMode(_encoderPinA2, INPUT_PULLUP); 
        pinMode(_encoderPinB2, INPUT_PULLUP);

        attachInterrupt(digitalPinToInterrupt(_encoderPinA1), encoderInterrupt1, CHANGE);
        attachInterrupt(digitalPinToInterrupt(_encoderPinA2), encoderInterrupt2, CHANGE);
    }

    // ฟังก์ชันสำหรับส่งคืนค่า encoder position
    int getEncoder1Position() {
        return encoderPoss1;
    }

    int getEncoder2Position() {
        return encoderPoss2;
    }

  private:
    int _encoderPinA1, _encoderPinB1;
    int _encoderPinA2, _encoderPinB2;

    // ฟังก์ชันขัดจังหวะ (interrupt service routine) สำหรับ Encoder 1
    static void encoderInterrupt1() {
        int encoderPinA1Val = digitalRead(_instance->_encoderPinA1);  
        if ((encoderPinA1Last == LOW) && (encoderPinA1Val == HIGH)) {
            if (digitalRead(_instance->_encoderPinB1) == LOW) {
                encoderPoss1--;
            } else {
                encoderPoss1++;
            }
        } 
        encoderPinA1Last = encoderPinA1Val;  
    }

    // ฟังก์ชันขัดจังหวะ (interrupt service routine) สำหรับ Encoder 2
    static void encoderInterrupt2() {
        int encoderPinA2Val = digitalRead(_instance->_encoderPinA2);  
        if ((encoderPinA2Last == LOW) && (encoderPinA2Val == HIGH)) {
            if (digitalRead(_instance->_encoderPinB2) == LOW) {
                encoderPoss2--;
            } else {
                encoderPoss2++;
            }
        } 
        encoderPinA2Last = encoderPinA2Val;  
    }

    static EncoderLibrary* _instance; // Static instance สำหรับการเรียกใช้ใน ISR
};

// Static instance initialization
EncoderLibrary* EncoderLibrary::_instance = nullptr;

#endif
