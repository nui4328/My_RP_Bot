
#include <my2wd_encoder.h>
#include <Wire.h>
#include <my_mpu6050.h> 
#include <EEPROM.h>
#define EEPROM_ADDR 0x50

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

bool lines;
bool lines_fw;
bool ch_lr = false;
bool ch_set_fb = false;
bool ch_bw = false;
bool line_l = true;
bool line_r = true;

int motor_slow = 15;
int fw_to_rotate = 360;

void setup()
  {    
     Serial.begin(9600);
     sensor_set();              // ค่าเริ่มต้น eeprom, bit_analogRead=>12   
     setup_encoder();

     mydisplay_background(black);
     mydisplay("MY-MAKERS", 20, 30, 2, white);
     
     
     sw_start();
     
//\\------------------------------------------->>>>>  เขียนโค๊ดที่นี้
    
     fw(50, 50, 3350, "non-line");
     rotate_left(40, 1000);
     set_b(3);
     fw(50, 50, 4000, "line");
     delay(300);
     encoderPos = 0;
     bw(50, 50, 1100, "non-line");
     rotate_left(40, 1100);
     fw(50, 50, 3300, "non-line");
     rotate_right(40, 800);
     fw(50, 50, 100, "line");
     set_f(3);
     bw(50, 50, 2600, "line");

     //set_f(5);

   
//\\------------------------------------------->>>>>   เขียนโค๊ดที่นี้ 
     
      
  }

void loop() 
  {  
    Serial.println(analogRead(28)); 
   

  }
