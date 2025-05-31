#include <my2wd_encoderOPT.h>
//---------------------------------\\

int servo_28_close = 148;   // แขนขวา
int servo_27_close = 145;   // แขนซ้าย

int trim_servo_0 = 10;
int trim_servo_1 = 3;

void setup()
  {
    Acceptable_values_moveLR(1.0,  5);     //------->> ตั้งค่า error  output ที่ยอมรับได้ในการหมุน
    set_pid_moveLR(1.50, 0.00015, 0.045);   //------->> ตั้งค่า pid สำหรับหมุนตัว
    set_move_before_moveLR(190);   //------->> ตั้งค่า เดินหน้า ถอยหลัง ก่อนการหมุน

    setup_OPT();
    pinMode(20, INPUT_PULLUP);
    

    //servo(29, 90);
    
    _servo(0, 90);     //----->แขนซ้ายชี้ไปข้างหน้า
    _servo(1, 90);     //----->แขนขวาชี้ไปข้างหน้า

    servo(27, servo_27_close-100);   
    servo(28, servo_28_close-100); 
    
    servo(29, 90);
    robot_start();
    eep_to_code();
    

    //reset_arm();
    ///----------------------------------------------------->> เขียนโค๊ดที่นี้

      arm_Slide(1000);  //----->>  เลื่อนแขนขึ้น หน่วงเวลา 400
      //arm_Slide(-400);  //----->>  เลื่อนแขนลงหน่วงเวลา 400
      //arm_Slide(0);  //----->>  เลื่อนแขนลงให้สุดเพื่อเริ่มต้น




    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
  }

void loop() {

  Serial.print(digitalRead(20));Serial.println("  ");  //อ่านค่าลิมิตสวิทย์
  //Serial.print(analogRead(26));Serial.println("  ");      //อ่านค่าวัดระย
  Serial.println(my.gyro('z')); delay(10);              //อ่านค่า gyro
  //Serial.print(encoder.Poss_L());Serial.print("  ");
  //Serial.print(encoder.Poss_R());Serial.println("  ");
}
