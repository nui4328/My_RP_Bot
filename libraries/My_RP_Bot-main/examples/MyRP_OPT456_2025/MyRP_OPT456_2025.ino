#include <my2wd_encoderOPT.h>
//---------------------------------\\

int servo_28_close = 150;   // แขนขวา
int servo_27_close = 150;   // แขนซ้าย

int trim_servo_0 = -5;
int trim_servo_1 = 0;

void setup()
  {
    Acceptable_values_moveLR(1.0,  5);     //------->> ตั้งค่า error  output ที่ยอมรับได้ในการหมุน
    set_pid_moveLR(1.40, 0.00010, 0.025);   //------->> ตั้งค่า pid สำหรับหมุนตัว
    set_move_before_moveLR(190);   //------->> ตั้งค่า เดินหน้า ถอยหลัง ก่อนการหมุน

    setup_OPT();
    pinMode(20, INPUT_PULLUP);
    

    //servo(29, 90);
    
    _servo(0, 30);     //----->แขนซ้ายชี้ไปข้างหน้า
    _servo(1, 30);     //----->แขนขวาชี้ไปข้างหน้า

    servo(27, servo_27_close);   
    servo(28, servo_28_close); 
    
    //servo(29, 90);
    robot_start();
    eep_to_code();
     arm_Slide(0);
      arm_Slide(300);
    _servo(0, 180);     //----->แขนซ้ายชี้ไปข้างหน้า
    _servo(1, 180);     //----->แขนขวาชี้ไปข้างหน้า
    
    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
      
      fw(50, 50, 1.5, 30, "none_line");
      moveLR(60, -90); 
      fw(70, 70, 1.5, 60, "none_line");
      moveLR(80, 90); 

      fw(70, 70, 1.5, 60, "none_line");
      moveLR(60, -90);

      fw(70, 70, 1.5, 60, "none_line");
      moveLR(60, -90);
      set_b(1);

      fw(50, 50, 1.5, 30, "none_line"); delay(1000);
      bw(50, 50, 1.5, 30, "none_line");
      _servo(0, 30);     //----->แขนซ้ายชี้ไปข้างหน้า
      _servo(1, 30);     //----->แขนขวาชี้ไปข้างหน้า
      moveLR(60, 90);
      set_f(1);

      bw(70, 70, 1.5, 60, "line", -300);
      moveLR(60, 90);

      bw(70, 70, 1.5, 60, "none_line");
      moveLR(60, -90);

      bw(70, 70, 1.5, 60, "line");
      moveLR(60, 90);
      bw(70, 70, 1.5, 30, "line");
      

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
