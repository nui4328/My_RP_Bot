
#include <my_RPOPT_mini.h>


void setup() 
  {
      setup_robot();
      to_set_motor_LR(90, 90);  //------->> ตั้งค่ามอเตอร์ถ้าหุ่นยนต์วิ่งไม่ตรง
      //Add_rear_sensor(true);   //------>> ติดตั้งเซนเซอร์หลัง
      Add_rear_sensor(false);    //------>> ไม่ติดตั้งเซนเซอร์หลัง

      
      //arm_open_up();
      //arm_down_close();
      sw();    //------>> คำสั่งรอกดปุ่ม start

      
      
    //--------------------------------------------->>  เอาคำสั่งต่าง ๆ มา RUN ตรงนี้

for(int i=0; i<15; i++)
  {
    fw_line(30, 30, 0.25, 's', "a5", 20);
    turn_left_sensor(3, 54, "a2", 20); 

    fw_line(30, 30, 0.25, 's', "a5", 20);
    turn_right_sensor(2, 54, "a3", 20);

    fw_line(30, 30, 0.25, 's', "a5", 20);
    turn_right_sensor(2, 54, "a3", 20);
    
    fw_line(30, 30, 0.25, 's', "a0", 20);
    turn_left_sensor(2, 54, "a2", 20); 
    
    fw_line(30, 30, 0.75, 's', "a5", 20);
    turn_right_sensor(2, 34, "a3", 20);

    fw_line(30, 30, 0.35, 's', "a0", 20);
    turn_left_sensor(3, 34, "a2", 20); 
   
    fw_line(30, 30, 0.45, 'p', "a0", 0);

    fw_line(30, 30, 0.35, 's', "a0", 20);
    turn_right_sensor(5, 34, "a3", 20); 


     fw_line(50, 50, 0.35, 'p', "a0", 0);

     fw_line(30, 30, 0.35, 's', "a5", 20);
     turn_right_sensor(2, 34, "a3", 20); 

     fw_line(30, 30, 0.85, 's', "a5", 20);
     turn_right_sensor(3, 34, 60, 20); 

     fw_distance(70, 70, 0.30, 25, 0);
     fw_line(20, 20, 0.25, 's', "a5", 20);
     turn_right_wheel(5, 34, "a3", 20); 
  }
    
    
    
    
 

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
    delay(10);
    */
  }
