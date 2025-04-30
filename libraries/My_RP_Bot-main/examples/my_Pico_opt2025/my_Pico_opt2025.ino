
#include <my_RPOPT_mini.h>


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

    fw_distance(15, 15, 0.55, 500, 10);

     // can_1();
     // can_2();
     // can_3();

    

    //--------------------------------------------->>  เอาคำสั่งต่าง ๆ มา RUN ตรงนี้
    //--------------------------------------------->>
    encoder.resetEncoders();
  }
    
    
void loop() 
  {
    /* 
    Serial.print(encoder.Poss_L());
    Serial.print("  ");
    Serial.println(encoder.Poss_R());
    */
  }
