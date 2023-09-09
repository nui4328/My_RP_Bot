#include <my_rp_2wd.h>
#include <Wire.h>
#include <my_TCS34725.h>  
#include <my_mpu6050.h> 
#include <EEPROM.h>
#define EEPROM_ADDR 0x50
  
//------------------------->>
// กำหนดพินอินพุตตัวเข้ารหัส
const int pinA = 0;
const int pinB = 1;

// กำหนดตัวแปรเพื่อติดตามตำแหน่งและทิศทางของตัวเข้ารหัส
volatile int encoderPos = 0;
volatile int encoderPos_r = 0;
int encoderDir = 1;
int prevAVal;
//------------------------->>

int sensor_maxs[] = {936, 942, 825, 755, 941, 822}; 
int sensor_mins[] = {219, 219, 175, 156, 239, 144}; 
 

unsigned long lasts_time = millis();

int f_deg_r = 89;
int f_deg_l = -89;
int b_deg_r = 89;
int b_deg_l = -89;

int tl = 500;   //------> เซตการหมุนตัว
int tr = 500;

void setup()
  {    
     Serial.begin(9600);
     sensor_set();              // ค่าเริ่มต้น eeprom, bit_analogRead=>12   
     setup_mpu();
     setup_encoder();
     calibration_Yak();   
     mydisplay_background(black);
     mydisplay("MY-MAKERS", 20, 30, 2, white);

     servo(23,130);
     
     sw_start();   
     
     //turn_left();sw();
     //turn_right();sw();
//\\------------------------------------------->>>>>  เขียนโค๊ดที่นี้  
        
    Motor(50,50);
   /*
    fw(100, 100,0, 'r', 1);
    set_bl_b(2);
    fw(100, 100,0, 'r', 1);
    set_bl_b(1);
    fw(90, 90,0, 'r', 1);
    set_bl_b(1);
    fw(90, 90,0, 'l', 1);
    set_bl_b(2);
    
    fw(100, 100,0, 's', 1);
    open_servo();
    open_servo();
    open_servo();
    set_bl(2);
    bw(90, 90,350, 'l', 1);
    set_bl_b(2);
    fw(90, 90,0, 'r', 1);
    set_bl_b(2);

    fw(90, 90,0, 's', 1);
    open_servo();
    open_servo();
    open_servo();
    
    bw(90, 90,0, 'l', 1);
    set_bl(1);
    bw(90, 90,0, 'r', 2);
    bw(90, 90,0, 'r', 1);
    set_bl(1);
    bw(90, 90,0, 'l', 1);
    set_bl(1);
    bw(90, 90,0, 'l', 1);
    set_bl(2);

    bw(100, 100,0, 'l', 1);
    set_bl(2);
    bw(100, 100,0, 's', 1);
 */
    
 

//\\------------------------------------------->>>>>   เขียนโค๊ดที่นี้ 
     
      
  }

void loop() 
  {  


    for(int i = 0; i<7; i++)
      {
        Serial.print(md_sensors(i));Serial.print("  ");
      }
     Serial.println(" ");


  }
