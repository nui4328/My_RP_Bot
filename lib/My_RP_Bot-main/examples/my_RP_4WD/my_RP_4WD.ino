#include <my_rp_4wd.h>
#include <Wire.h>
#include <EEPROM.h>
#define EEPROM_ADDR 0x50
  
unsigned long lasts_time=millis();
//------------------------->>

void setup()
  {    
     Serial.begin(9600);
     sensor_set();              // ค่าเริ่มต้น eeprom, bit_analogRead=>12   
    
     mydisplay_background(black);
     mydisplay("MY-MAKERS", 20, 30, 2, white);
     pinMode(27,INPUT_PULLUP);

     servo(22, 90);
     sw_start();    
         //cal_censor(0, 0);sw();
         
//\\------------------------------------------->>>>>  เขียนโค๊ดที่นี้ 


   
//\\------------------------------------------->>>>>   เขียนโค๊ดที่นี้ 
     
      
  }

void loop() 
  {  
    
  }
