#include <my2wd_encoderOPT.h>
//---------------------------------\\

void setup()
  {
    set_pid_moveLR(1.45, 0.0001, 0.035);   //------->> ตั้งค่า pid สำหรับหมุนตัว
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
    robot_start();
    eep_to_code();
    Serial.println(my.gyro('z')); delay(10);

    //reset_arm();
    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
   

     



    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
  }

void loop() {

  //Serial.print(digitalRead(20));Serial.println("  ");  //อ่านค่าลิมิตสวิทย์
 //Serial.print(analogRead(26));Serial.print("  ");      //อ่านค่าวัดระย
  //Serial.println(my.gyro('z')); delay(10);              //อ่านค่า gyro
}
