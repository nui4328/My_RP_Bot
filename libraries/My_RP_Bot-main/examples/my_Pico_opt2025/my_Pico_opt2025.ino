
#include <my_RPOPT_mini.h>


void setup() 
  {
      setup_robot();
      to_set_motor_LR(90, 90);  //------->> ตั้งค่ามอเตอร์ถ้าหุ่นยนต์วิ่งไม่ตรง

      arm_open_up();
      //arm_down_close();
      sw();    //------>> คำสั่งรอกดปุ่ม start

      
      
    //--------------------------------------------->>  เอาคำสั่งต่าง ๆ มา RUN ตรงนี้

  

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
