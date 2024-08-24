
#include <my2wd_encoder.h>
#include <Wire.h>
//#include <my_TCS34725.h>  

#include <VL6180X.h>
#include <my_mpu6050.h> 
#include <EEPROM.h>
#define EEPROM_ADDR 0x50
#include <TCA9548A.h>  // Library สำหรับ TCA9548A

VL6180X sensor1;  // เซ็นเซอร์ VL6180X ตัวที่ 1
VL6180X sensor2;  // เซ็นเซอร์ VL6180X ตัวที่ 2
VL6180X sensor3;  // เซ็นเซอร์ VL6180X ตัวที่ 3


TCA9548A I2CMux;  // ตัวคูณ 
uint8_t range1, range2, range3;
//------------------------->>
// กำหนดพินอินพุตตัวเข้ารหัส
const int pinA = 11;
const int pinB = 10;

// กำหนดตัวแปรเพื่อติดตามตำแหน่งและทิศทางของตัวเข้ารหัส
volatile int encoderPos = 0;
volatile int encoderPos_r = 0;
int encoderDir = 1;
int prevAVal;
//------------------------->>

int setsensor_front[] = {3, 4}; 
int setsensor_center[] = {1, 6}; 
int setsensor_behind[] = {0, 7}; 

float lastError;

int min_26, max_26, min_27, max_27;
void setup()
  {    
     Serial.begin(9600);
     sensor_set();              // ค่าเริ่มต้น eeprom, bit_analogRead=>12   
     setup_mpu();
     setup_encoder();

      
     calibration_Yak();   
     mydisplay_background(black);
     mydisplay("MY-MAKERS", 20, 30, 2, white);
     
     read_epp_sensor();
     sw_start();
     
//\\------------------------------------------->>>>>  เขียนโค๊ดที่นี้      
   //Motor(30,30);
   
  ////////////////////////////////////////////
/*
   cal_dis_L();
   
   while(digitalRead(9)==1){}
   bz(200);
   
   cal_dis_R();
   read_epp_sensor();
   while(digitalRead(9)==1){}
   bz(200);
  */
  /////////////////////////////////////////// 
   fline(20, 20, 0.5);
   fline(20, 20, 0.5);
   fline(20, 20, 0.5);
   fline(20, 20, 0.5);
   fline(20, 20, 0.5);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   fline(20, 20, 1.2);
   
 
//\\------------------------------------------->>>>>   เขียนโค๊ดที่นี้ 
     
      
  }

void loop() 
  {  
    Serial.println(analogRead(28)); 
    /*
    rangs();
    Serial.print(analogRead(26)); 
    Serial.print("   ");
    Serial.print(analogRead(27)); 
    Serial.print("   "); 
    Serial.print(range3); 
    Serial.print("   "); 
    Serial.println("   ");
    */ 
    /*
    for(int i=1; i<4; i++)
      {
        Serial.print(rangs(i)); 
        Serial.print("   "); 
      }
    Serial.println(""); 
   */
    //Serial.println(encoderPos); 
    //delay(10);

/*
    float color[3] = {my_tcs('r'), my_tcs('g'), my_tcs('b')};
    for(int i = 0; i<3; i++)
      {
        Serial.print(color[i]);Serial.print("  ");
      }
     Serial.println(" ");
*/

  }
