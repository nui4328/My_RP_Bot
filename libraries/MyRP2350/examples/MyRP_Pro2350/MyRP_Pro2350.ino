
#include <my_rp2350_Pro.h>

void setup() 
  {
    setup_rp2350_pro();
     distance_scale(1.55);
    arm_up_close();  //--->> ยกแขนขึ้นหุบฝ่ามือเข้า
    sw();  //--->> คำสั่งรอกดปุ่ม


    ////------------------------------------------------------------------------------>> รันคำสั่งต่าง ๆ ที่นี่





    ////------------------------------------------------------------------------------>> จบการรันคำสั่งต่าง ๆ 

  }

void loop() 
  {
    Serial.print( analogRead(46) );   Serial.print( "   " );   Serial.println( analogRead(47) ); 
    //Serial.println(my.gyro('z'));   
  delay(10);

  }
