#include <my_rp_4wd.h>
#include <Wire.h>
#include <EEPROM.h>
#define EEPROM_ADDR 0x50
  
void setup()
  {    
     Serial.begin(9600);
     sensor_set();              // ค่าเริ่มต้น eeprom, bit_analogRead=>12   
    
     mydisplay_background(black);
     mydisplay("MY-MAKERS", 20, 30, 2, white);

     servo(22, 60);
     sw();    
         
//\\------------------------------------------->>>>>  เขียนโค๊ดที่นี้ 

//\\------------------------------------------->>>>>   เขียนโค๊ดที่นี้ 
     
      
  }

void loop() 
  { 
  }
