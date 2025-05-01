#include <my2wd_encoderOPT.h>
//---------------------------------\\


void setup()
  {
    set_pid_moveLR(1.35, 0.00015, 0.040);   //------->> ตั้งค่า pid สำหรับหมุนตัว
    set_pid_chopsticks(0.6, 0.00001, 0.15); //------->> ตั้งค่า pid สำหรับขึ้นตะเกียบ
    setup_OPT();
    pinMode(20, INPUT_PULLUP);
    

    //servo(29, 90);
    _servo(0, 90);     //----->แขนซ้ายชี้ไปข้างหน้า
    _servo(1, 90);     //----->แขนขวาชี้ไปข้างหน้า
    servo(27, servo_27_close-110);   
    servo(28, servo_28_close-110);
    delay(100); 
    servo(27, servo_27_close);   
    servo(28, servo_28_close);
    servo(29, 90);
    robot_start();
    eep_to_code();
    

    //reset_arm();
    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
    

   for(int i=0; i<5; i++)
    {
     fw(50, 50, 0.85, 90, "none_line");

     moveLR(75, 90);

     fw(60, 60, 0.35, 50, "line");
     moveLR(75, 90);

    fw(50, 50, 0.35, 25, "line");
     moveLR(75, 90);

     fw(50, 50, 0.35, 26, "none_line");
     moveLR(80, -90);
     set_b(1);
     fw(50, 50, 0.35, 25, "none_line");
     moveLR(80, -90);
     set_b(5);
     fw_bridge(50, 50, 0.55, 175, "line");
     moveLR(60, -185);
     set_b(3);

     fw_bridge(70, 70, 0.35, 170, "line");
     moveLR(75, 90);

     fw(50, 52, 0.35, 25, "line");
     moveLR(80, 90);

     fw(50, 50, 0.35, 25, "none_line");
     moveLR(80, -90);

     fw(50, 50, 0.35, 25, "line");
     moveLR(75, -90);
     set_b(1);

     fw(70, 70, 0.35, 50, "line");
     moveLR(75, -90);
     set_b(1);

     fw(70, 70, 0.35, 90, "line");

     moveLR(60, -180);
     set_b(3);
    }

   

    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
  }

void loop() {
  
   
  //Serial.print(digitalRead(20));Serial.println("  ");  //อ่านค่าลิมิตสวิทย์
 //Serial.print(analogRead(26));Serial.print("  ");      //อ่านค่าวัดระย
  Serial.println(my.gyro('z')); delay(10);              //อ่านค่า gyro
}
