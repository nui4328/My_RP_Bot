#include <Wire.h>
#include <my2wd_encoderOPT.h>
#include <my_TCS34725.h>
// ที่อยู่ I2C ของ CAT24C128
#define EEPROM_ADDR 0x50
// ขา GPIO สำหรับปุ่ม
#define BUTTON_PIN 9

// ที่อยู่สำหรับเก็บค่าสีใน EEPROM
#define RED_SET_ADDR 0    // เก็บชุดค่าสีแดง (R, G, B)
#define GREEN_SET_ADDR 6  // เก็บชุดค่าสีเขียว (R, G, B)
#define YELLOW_SET_ADDR 12 // เก็บชุดค่าสีเหลือง (R, G, B)

// สถานะการเลือกสี
enum ColorState { NONE, RED, GREEN, YELLOW, SEARCH };
ColorState currentState = NONE;
bool colorsStored = false; // ตัวแปรเพื่อตรวจสอบว่าเก็บครบ 3 สีแล้วหรือไม่

int last_servo_0 = 0;
int last_servo_1 = 0;

int servo_28_close = 140;   // แขนขวา หุบเข้า
int servo_27_close = 140;   // แขนซ้าย หุบเข้า

int trim_servo_0 = -2;   // ค่าชดเชยหัวไหล่เวลาประกอบแล้วหัวไหลไม่ชี้ไปข้างหน้า
int trim_servo_1 = -12;



//-------------------------------------------------------->>   แก้ไข เรียงจากซ้ายไปขวา
String color_right = "Green";
String color_left = "Red";

String stsnd_1[] = {"Red", "Yellow", "Green"};
String stsnd_2[] = {"Red", "Yellow", "Green"};
String stsnd_3[] = {"Red", "Yellow", "Green"};

int mission_right0, mission_right1, mission_right2 ;
int mission_left0, mission_left1, mission_left2;


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
    servo(27, servo_27_close-110);   
    servo(28, servo_28_close-110);     
    
    //----------------------------------------------->>


    robot_start();  //-------->> คำสั่งรอกดปุ่ม
    eep_to_code();
    //storeColors(); // เรียกฟังก์ชันเก็บค่าสี
    
    
   //  _servo(0, 150);     //----->แขนซ้ายชี้ไปข้างหลัง
   // _servo(1, 150);     //----->แขนขวาชี้ไปข้างหลัง

    
    ///----------------------------------------------------->> เขียนโค๊ดที่นี้

    
      arm_Slide(0);
      delay(1000);
      servo(27, servo_27_close);   
      servo(28, servo_28_close);
      delay(300); 
      get_color_arm();
      arm_Slide(0);
      servo(27, servo_27_close-40);   
      servo(28, servo_28_close-40); 
      delay(500);  
      servo(27, servo_27_close);   
      servo(28, servo_28_close);  
      delay(500);  
      arm_Slide(900);
      //robot_start();
      fw_distance(10, 10, 1.5, 2500) ;

      arm_right();
      bw(20, 20, 1.5, 10, "none_line"); 

      arm_left();
      
     




    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
    ///----------------------------------------------------->> เขียนโค๊ดที่นี้
  }

void loop() {
   if (colorsStored) {
    searchColor();
    delay(10); // ค้นหาสีทุก 0.5 วินาที
  }
   //Serial.print("color_right : "); Serial.print(color_right); Serial.print("     "); Serial.print("color_left : ");Serial.println(color_left);delay(10); 
  //searchColor();delay(10); 
  /*
    Serial.print(my_tcs('r'));
    Serial.print("  ");
    Serial.print(my_tcs('g'));
    Serial.print("   ");
    Serial.println(my_tcs('b'));delay(10); 
    */  
 // _servo(15, 120);  delay(1000);
 // _servo(15, 10);  delay(1000);
  //Serial.print(digitalRead(20));Serial.println("  ");  //อ่านค่าลิมิตสวิทย์
 Serial.print(analogRead(26));Serial.println("  "); delay(10);      //อ่านค่าวัดระย
 // Serial.println(my.gyro('z')); delay(10);              //อ่านค่า gyro

  //Serial.print(encoder.Poss_L());Serial.print("  ");
  //Serial.print(encoder.Poss_R());Serial.println("  ");
}
