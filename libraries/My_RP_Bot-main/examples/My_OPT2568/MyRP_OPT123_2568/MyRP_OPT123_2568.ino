#include <Wire.h>
#include <my2wd_encoderOPT.h>

#define EEPROM_ADDR 0x50
// ขา GPIO สำหรับปุ่ม
#define BUTTON_PIN 9


int last_servo_0 = 0;
int last_servo_1 = 0;

int servo_28_close = 140;   // แขนขวา หุบเข้า
int servo_27_close = 140;   // แขนซ้าย หุบเข้า

int trim_servo_0 = -2;   // ค่าชดเชยหัวไหล่เวลาประกอบแล้วหัวไหลไม่ชี้ไปข้างหน้า
int trim_servo_1 = -12;




void setup()
  {
    Acceptable_values_moveLR(1,  5);     //------->> ตั้งค่า error  output ที่ยอมรับได้ในการหมุน
    set_pid_moveLR(1.15, 0.000050, 0.022);   //------->> ตั้งค่า pid สำหรับหมุนตัว
    set_move_before_moveLR(150);   //------->> ตั้งค่า เดินหน้า ถอยหลัง ก่อนการหมุน
    dis_to_stand(220);  //--------->> เพิ่มระยะหลังจากวัดระยะของเซนเซอร์ไม่ถึงเป้าหมาย

    setup_OPT();
    pinMode(20, INPUT_PULLUP);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    //setServo29Revers("revers");         //-------->>  สำหรับตั้งค่า servoขึ้นลง 
    //setServo29Revers("none_revers");
    

    //--------------------------------------------->> ตั่งค่าเริ่มต้นของ arm
    
    _servo(0, 60);     //----->แขนซ้ายชี้ไปข้างหน้า
    _servo(1, 60);     //----->แขนขวาชี้ไปข้างหน้า
    servo(27, servo_27_close);   
    servo(28, servo_28_close);     
    
    //----------------------------------------------->>

    servo(27, servo_27_close-80);   
    servo(28, servo_28_close-80); 
    robot_start();  //-------->> คำสั่งรอกดปุ่ม
    eep_to_code();

   //  _servo(0, 150);     //----->แขนซ้ายชี้ไปข้างหลัง
   //  _servo(1, 150);     //----->แขนขวาชี้ไปข้างหลัง

    
    ///----------------------------------------------------->> เขียนโค๊ดที่นี้

     arm_Slide(0); 
     arm_Slide(500);  
     fw_distancess(10, 10, 1.5, 2700) ;


    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
  }

void loop() {
   
 // _servo(15, 120);  delay(1000);
 // _servo(15, 10);  delay(1000);
  //Serial.print(digitalRead(20));Serial.println("  ");  //อ่านค่าลิมิตสวิทย์
 Serial.print(analogRead(26));Serial.println("  "); delay(10);      //อ่านค่าวัดระย
 // Serial.println(my.gyro('z')); delay(10);              //อ่านค่า gyro

  //Serial.print(encoder.Poss_L());Serial.print("  ");
  //Serial.print(encoder.Poss_R());Serial.println("  ");
}
