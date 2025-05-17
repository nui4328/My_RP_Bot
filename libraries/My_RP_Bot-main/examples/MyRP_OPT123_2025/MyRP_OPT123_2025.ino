#include <my2wd_encoderOPT.h>
//---------------------------------\\


void setup()
  {
    Acceptable_values_moveLR(1.0,  5);     //------->> ตั้งค่า error  output ที่ยอมรับได้ในการหมุน
    set_pid_moveLR(1.50, 0.0001, 0.0450);   //------->> ตั้งค่า pid สำหรับหมุนตัว
    set_move_before_moveLR(190);   //------->> ตั้งค่า เดินหน้า ถอยหลัง ก่อนการหมุน

    setup_OPT();
    pinMode(20, INPUT_PULLUP);
    

    //servo(29, 90);
    /*
    _servo(0, 90);     //----->แขนซ้ายชี้ไปข้างหน้า
    _servo(1, 90);     //----->แขนขวาชี้ไปข้างหน้า

    servo(27, servo_27_close);   
    servo(28, servo_28_close);servo(27, servo_27_close);   
    servo(28, servo_28_close);
    delay(100); 
    servo(27, servo_27_close);   
    servo(28, servo_28_close);
    */
    servo(29, 90);
    robot_start();
    eep_to_code();
    

    //reset_arm();
    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
     st1();

     

    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
  }

void loop() {
  
   //servo(29, 0); delay(500);
   //servo(29, 90); delay(500);
   //servo(29, 180); delay(500);
   //servo(29, 90); delay(500);
  Serial.print(digitalRead(20));Serial.print("  ");  //อ่านค่าลิมิตสวิทย์
  Serial.print(analogRead(26));Serial.println("  ");      //อ่านค่าวัดระย
  //Serial.println(my.gyro('z')); delay(10);              //อ่านค่า gyro
}
