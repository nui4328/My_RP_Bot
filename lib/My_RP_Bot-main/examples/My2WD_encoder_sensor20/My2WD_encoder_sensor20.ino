
#include <my2wd_encoder.h>


void setup()
  {    
     Serial.begin(9600);
     sensor_set();              // ค่าเริ่มต้น eeprom, bit_analogRead=>12  
     mydisplay_background(black);
     mydisplay("MY-MAKERS", 20, 30, 2, white);
     begin_robot();
  
//\\------------------------------------------->>>>>  เขียนโค๊ดที่นี้   

   fw (60, 60, 1.4, 5.0, 30); 
  
//\\------------------------------------------->>>>>   เขียนโค๊ดที่นี้ 
     
      
  }

void loop() 
  {  
     Serial.println(mcp_20(0));delay(10);

    
      

  }
