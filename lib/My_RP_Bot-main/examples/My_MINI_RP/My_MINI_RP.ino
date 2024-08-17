#include <my_MINI_RP.h>
#include <my_TCS34725.h>
#define EEPROM_ADDR 0x50
#include <VL6180X.h>
VL6180X dis;
//------------------------->>
// Encoder 1
const int encoderPinA1 = 6; 
const int encoderPinB1 = 7; 

// Encoder 2
const int encoderPinA2 = 15; 
const int encoderPinB2 = 20; 

// Variables to track positions for both encoders
volatile int encoderPoss1 = 0;  // Position for Encoder 1
volatile int encoderPoss2 = 0;  // Position for Encoder 2

int encoderPinA1Last = LOW;  
int encoderPinA2Last = LOW;
//------------------------->>
int servo_down = 80;      //-------------------->> ตั้งค่า มื่อจับลงขนานกับพื้น
int servo_up = servo_down + 85;       //-------------------->> ตั้งค่า มื่อจับยกขึ้นตั้งฉากกับพื้น
int servoL_open = 168;      //-------------------->> ตั้งค่า กางฝ่ามือด้านซ้าย
int servoR_open = 19;  //-------------------->> ตั้งค่า กางฝ่ามือด้านขวา

int servo_close = 130; 


void setup() 
  {    
      _seting();         //-------------------->>   ฟังก์ชัน ตั้งค่าต่าง ๆ ใน tab seting
      setup_encoder();
       
       sw();             //-------------------->> ฟังก์ชันรอกดปุ่ม
       
      
      //////////////////------------------------>>>>>  Run funtion
     fline(50,50,0.8,0,'n','l',80, "a2", 30);
  

     
      //////////////////------------------------>>>>>  

  }

void loop() 
  {
    Motor(15, 15);
    Serial.print("Encoder_L: ");
    Serial.print(encoderPoss1);
    Serial.print("     ");
    Serial.print("Encoder_R: ");
    Serial.println(encoderPoss2);

    delay(100);  // Delay for better readability in serial out
  }
