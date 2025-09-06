#include <myMD_2568.h>
//---------------------------------\\


void setup()
  {
    Acceptable_values_moveLR(1.0,  3);     //------->> ตั้งค่า error  output ที่ยอมรับได้ในการหมุน
    set_pid_moveLR(1.750, 0.0001, 0.0250);   //------->> ตั้งค่า pid สำหรับหมุนตัว
    set_move_before_moveLR(190);   //------->> ตั้งค่า เดินหน้า ถอยหลัง ก่อนการหมุน

    setup_OPT();
    pinMode(20, INPUT_PULLUP);
    
    servo(26, 165);
    servo(27, 165);
    robot_start();
    eep_to_code();
    

    
    ///----------------------------------------------------->> เขียนโค๊ดที่นี้


    fw(50, 50, 1.5, 40, "none_line"); 
    moveLR(70, 90); 
    fw(40, 40, 1.5, 40, "none_line"); 
    delay(1000);
    servo26_Place();
    bw(50, 50, 1.5, 40, "none_line"); 
    moveLR(70, 90); 
    fw(40, 40, 1.5, 40, "none_line"); 
    delay(1000); servo26_Place();
 











     

    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
  }

void loop() {

  //Serial.print(digitalRead(20));Serial.println("  ");  //อ่านค่าลิมิตสวิทย์
  //Serial.print(analogRead(26));Serial.println("  ");      //อ่านค่าวัดระย
  Serial.println(my.gyro('z')); delay(10);              //อ่านค่า gyro

   //Serial.print(encoder.Poss_L());Serial.print("  ");

    //Serial.print(encoder.Poss_R());Serial.println("  ");
}
