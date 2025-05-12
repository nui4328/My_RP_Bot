
#include <my_RPOPT_mini.h>

int servo_down = 50;      //-------------------->> ตั้งค่า มื่อจับลงขนานกับพื้น
int servo_close = 130;    //-------------------->> ตั้งค่า ฝ่ามือให้ปลายเข้ามาแตะกันพอดี

void setup() 
  {
      setup_robot();
      to_set_motor_LR(90, 90);  //------->> ตั้งค่ามอเตอร์ถ้าหุ่นยนต์วิ่งไม่ตรง
      //Add_rear_sensor(true);   //------>> ติดตั้งเซนเซอร์หลัง
      Add_rear_sensor(false);    //------>> ไม่ติดตั้งเซนเซอร์หลัง

      
      arm_open_up();
      //arm_down_close();
      sw();    //------>> คำสั่งรอกดปุ่ม start

      
      
    //--------------------------------------------->>  เอาคำสั่งต่าง ๆ มา RUN ตรงนี้
 

    //can_1();
    //can_2();
    //can_3();
    //can_4();
    //can_5();
   // can_6();


       


    

    //--------------------------------------------->>  เอาคำสั่งต่าง ๆ มา RUN ตรงนี้
    //--------------------------------------------->>
    encoder.resetEncoders();
  }
    
    
void loop() 
  {
    Serial.println(my.gyro('z')); delay(10);   
    
    /*
    Serial.print(encoder.Poss_L());
    Serial.print("  ");
    Serial.println(encoder.Poss_R());
    delay(10);
    */
  }
