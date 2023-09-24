#include <my_rp_4wd.h>
#include <Wire.h>
#include <EEPROM.h>
#define EEPROM_ADDR 0x50
  
unsigned long lasts_time=millis();
//------------------------->>

int sensor_maxs[] = {936, 942, 925, 955, 941, 922}; 
int sensor_mins[] = {189, 189, 175, 186, 189, 184}; 

void setup()
  {    
     Serial.begin(9600);
     sensor_set();              // ค่าเริ่มต้น eeprom, bit_analogRead=>12   
    
     mydisplay_background(black);
     mydisplay("MY-MAKERS", 20, 30, 2, white);
     pinMode(27,INPUT_PULLUP);

     servo(22, 90);
     servo(23, 90);
     //servo(28, 20);
     sw_start();    
         //cal_censor(0, 0);sw();
         //test_line_r();
         
//\\------------------------------------------->>>>>  เขียนโค๊ดที่นี้ 


   
//\\------------------------------------------->>>>>   เขียนโค๊ดที่นี้ 
     
      
  }

void loop() 
  {  
    
  }
