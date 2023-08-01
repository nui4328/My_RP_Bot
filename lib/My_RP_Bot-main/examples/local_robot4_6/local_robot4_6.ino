 #include <my_rp2040.h>
 #include <my_TCS34725.h>  
 #include <Wire.h>
#include <EEPROM.h>
#define EEPROM_ADDR 0x50
  
unsigned long lasts_time_servo = millis();
unsigned long lasts_time_ch = millis();

int servo_down = 55;
int servo_up = 143;

float boxcolor_red[3]={1.88, 0.56, 0.55};
float boxcolor_green[3]={1.02, 1.03, 0.96};
float boxcolor_yello[3]={1.32, 1.13, 0.55};
float boxcolor_blue[3]={1.02, 1.03, 0.96};

String color_box;
String color_floor_L = "green";
String color_floor_MD = "yello";
String color_floor_R = "red";
int tll = 0;
int trr = 0;
int tmd = 0;

char start_robot;
int no_can;

void setup()
  {    
     Serial.begin(9600);
      sensor_set();              // ค่าเริ่มต้น eeprom, bit_analogRead=>12      
      pos_motor_cal(10, 10, 15);
      to_set_motor_LR(100, 100);          //ตั้งค่ามอเตอร์ให้หุ่นยนต์วิ่งตรง      
      to_slow_motor(25, 25);           // ตั้งค่า ความเร็วมอเตอร์เวลาเข้าแยก
      to_turn_center_l(-100, 100 );   // ตั้งค่ามอเตอร์หมุนซ้ายขวา ตอนเลี้ยวซ้ายแบบ center
      to_turn_center_r(100, -100 );   // ตั้งค่ามอเตอร์หมุนซ้ายขวา ตอนเลี้ยวขาวแบบ center
      to_turn_front_l(-20, 100);     // ตั้งค่ามอเตอร์หมุนซ้ายขวา ตอนเลี้ยวซ้ายแบบ เซนเซอร์หน้า  เดินหน้า
      to_turn_front_r(100, -20);     // ตั้งค่ามอเตอร์หมุนซ้ายขวา ตอนเลี้ยวขวาแบบ เซนเซอร์หน้า  เดินหน้า
      to_speed_turn_fl(100, 30, 35);  // ค่าพุ่งตัวออกหลังจากการเลี้ยว มีผลต่อตัวแปรสุดท้ายของคำสั่งเลี้ยว ถ้าเป็น 0
      to_speed_turn_fr(30, 100, 35);  // ค่าพุ่งตัวออกหลังจากการเลี้ยว มีผลต่อตัวแปรสุดท้ายของคำสั่งเลี้ยว ถ้าเป็น 0      
      to_brake_fc(10, 60);            // ตั้งค่าในการหยุดมอเตอร์ก่อนหมุนซ้ายขวา to_brake_fc(...f..., ....c...); 
      to_delay_f(15);                 // ระยะทางในการข้ามเส้นเวลาหมุนตัวแบบ f
    
      kp_sl(0.15, 0.15);  // kp และ ki ในฟังก์ เดินตามเส้นแบบช้า
      kd_fw(0.4);
      kd_bw(0.5);

     
    //  servo(8,servo_up );  //ลง 
    //  servo(23,120);   
      

      //servo_down_close();

      ///////////////////////////////////------->>>>  เก็บค่าสีกล่อง โดยเอากล่องมาวางหน้าเซนเซอร์แล้วกดปุ่ม sw-GP9 
/*
      bz(200);
      put_colorbox_red();
      put_colorbox_green();
      put_colorbox_yello();
    
      read_eep_box_red();     ///////////------->>>>  เก็บค่าสีกล่อง จาก eepprom ไว้ในตัวแปร  float boxcolor_red[3];
      read_eep_box_green();                                                         //  float boxcolor_blue[3];
      read_eep_box_yello();
      sw_eep();
 */ 
      ///////////////////////////////////------->>>>  เก็บค่าสีกล่อง โดยเอากล่องมาวางหน้าเซนเซอร์แล้วกดปุ่ม sw-GP9 

      //read_tcs();
      
      sw();
      
     // read_eep_box_red();     ///////////------->>>>  เก็บค่าสีกล่อง จาก eepprom ไว้ในตัวแปร  float boxcolor_red[3];
    //  read_eep_box_green();                                                         //  float boxcolor_blue[3];
     // read_eep_box_yello();                                                       //  float boxcolor_yello[3]

      
//\\------------------------------------------->>>>>  เขียนโค๊ดที่นี้      

      box_1();
      box_2();

      box_3();
      box_4();
      box_5();
      box_6();
     box_7();
      box_8();
      box_9();
      box_10();
      box_11();
      box_12();
      box_13();
      box_14();
      box_15();


//\\------------------------------------------->>>>>   เขียนโค๊ดที่นี้ 
     
      
  }

void loop() 
  {  

    //check_box();
   
 /*
    float color[3] = {my_tcs('r'), my_tcs('g'), my_tcs('b')};
    for(int i = 0; i<3; i++)
      {
        Serial.print(color[i]);Serial.print("  ");
      }
     Serial.println(" ");
 */

  }
