#include <my2wd_encoderOPT.h>
//---------------------------------\\
unsigned long lastTimes = millis();

void setup()
  {
    set_pid_moveLR(1.40, 0.0001, 0.035);   //------->> ตั้งค่า pid สำหรับหมุนตัว
    setup_OPT();

    //servo(29, 90);
    robot_start();
    eep_to_code();
    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
   

    servo(27, servo_27_close-110);   
    servo(28, servo_28_close-110); 
    _servo(0, 90);     //----->แขนซ้ายชี้ไปข้างหน้า
    _servo(1, 90);     //----->แขนขวาชี้ไปข้างหน้า
    fw_pid(50, 50, 27, "none_line");
    moveLR(100, 90);
    fw_pid(50, 50, 92, "none_line",250);
    moveLR(80, -90);
    
    fw_pid(50, 50, 60, "line");
    moveLR(100, 90);
    fw_pid(50, 50, 55, "none_line");
    moveLR(100, 90);
     set_b(1);
    moveLR(200,90, -90);
    
    fw_pid_distance(20, 20, 2500,50);
    servo(27, servo_27_close);   
    servo(28, servo_28_close);
    arm_up(170);delay(200);
    
    bw_pid(50, 50, 10, "line");
    bw_pid(50, 50, 20, "line",-20);
    moveLR(100, 90);

    set_b(1);
    moveLR(200,100, 90);

    fw_pid(50, 50, 30, "none_line");

     moveLR(100, -90);
     fw_pid(50, 50, 62, "none_line");
     moveLR(80, 90);
     fw_pid(50, 50, 60, "none_line");

     moveLR(100, 90);

     fw_pids(40, 40, 10, "none_line",10);
    fw_pid_distance(7, 7, 2500,300);
    servo(27, servo_27_close-110);   
    servo(28, servo_28_close-110);
    /*
    servo(27, servo_27_close-110);   
    servo(28, servo_28_close-110); 
    _servo(0, 90);     //----->แขนซ้ายชี้ไปข้างหน้า
    _servo(1, 90);     //----->แขนขวาชี้ไปข้างหน้า
    fw_pid(20, 20, 30, "none_line");
    servo(27, servo_27_close);   
    servo(28, servo_28_close+10);   delay(300); 
    bw_pid(40, 40, 30, "line",50);
    moveLR(70, -90);
    fw_pids(40, 40, 25, "none_line",10);
    fw_pid_distance(7, 7, 2500,1000);delay(300);
    servo(27, servo_27_close-110);   
    servo(28, servo_28_close-110);

    bw_pid(20, 21, 10, "none_line");
    bw_pids(30, 31, 30, "line", -900);
    set_b(3);*/

/*
    moveLR(90, 90);
    set_b(2);
    moveLR(200, 90, -90);
    fw_pid(50, 50, 30, "none_line");
    moveLR(90, 90); 

    fw_pid(50, 50, 30, "none_line");
    moveLRS(90, -90); 

    fw_pid(50, 50, 30, "none_line");
    moveLRS(90, -90); 

    fw_pid(50, 50, 30, "none_line");
    moveLRS(90, -90); 

    fw_pid(50, 50, 60, "none_line");
     */

     

    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
  }

void loop() {

 Serial.print(analogRead(26));Serial.print("  ");
 Serial.println(digitalRead(ENCODER_PIN));
}
