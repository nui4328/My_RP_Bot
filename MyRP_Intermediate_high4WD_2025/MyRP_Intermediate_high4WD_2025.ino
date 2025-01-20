
#include <my2wd_encoder.h>
#include <Wire.h>
#include <my_mpu6050.h> 
#include <my_TCS34725.h>

#define sevopico26 26
Servo servo_pico26;

// ค่าคงที่สำหรับการคำนวณ
const float wheelDiameter = 5.0;                      // เส้นผ่านศูนย์กลางล้อ (เซนติเมตร)
const float wheelCircumference = PI * wheelDiameter;  // เส้นรอบวงล้อ
const int pulsesPerRevolution = 430;                  // จำนวนพัลส์ต่อรอบ
const float pulsesPerCm = pulsesPerRevolution / wheelCircumference; // พัลส์ต่อเซนติเมตร


int red_box, green_box, blue_box, yello_box, ch_poit;
int servo27 = 90;
int servo28 = 90;
int servo20 = 90;
int num_encoder = 0;
int roll_L = 900;

String floor_R;
String floor_M;
String floor_L;

String can;

String color_ch="none";

char lr ;
bool lines;
bool lines_fw;
bool ch_lr = false;
bool ch_set_fb = false;
bool ch_bw = false;
bool line_l = true;
bool line_r = true;
bool LR_free = false;
bool ch_point;
int motor_slow = 15;
int fw_to_rotate = 350;  
int get_encode_toline;

int rotate_left = 370;  // หมุนซ้ายกลับออกมาหากไม่ตรงให้ปรับค่าตรงนี้  หมุนซ้ายมากให้ลดลง //---------------------->>>
int rotate_right = 370;

int Back_from_line = 100;       // เจอเส้นแล้วถอยหลังก่อนจะหมุ่นตัวซ้ายเพื่อเดินต่อ
int Forward_not_finding_line = 400;  // เดินหน้าต่อเมื่อไม่เจอเส้น ให้ถึงกลางบล๊อก

int start_begin = 0;
 // หมุนซ้ายกลับออกมาหลังจากวางกล่องตามสี
int speed_rotate = 30;
int _ch_lr;


int set_rotate_Right = 320; 
int Go_back_before_turning_left = 8;
int set_rotate_Left = 360;
int Go_back_before_color = 23;
int Go_back_before_yelow = 20;
int Go_back_before_sticks = 30;
int rotate_yelow = 380; 
int rotate_color = 370; 
void setup()
  {    
     Serial.begin(9600);
     servo_pico26.attach(sevopico26, 500, 2500);
        
     sensor_set();              // ค่าเริ่มต้น eeprom, bit_analogRead=>12   
     
     encoder.setupEncoder();    //-------------------->> เรียกฟังก์ชัน setupEncoder
     encoder.resetEncoders();  //--------------------->> ฟังก์ชันรอก

     mydisplay_background(black);
     mydisplay("MY-MAKERS", 20, 30, 2, white);
     read_eppcolor();
     servo(20, servo20);
     servo(28, servo28);
     servo(27, servo27);
     servo_pico26.write(20);

     begin_robot();
     
//\\------------------------------------------->>>>>  เขียนโค๊ดหุ่นยนต์ที่นี้
      LR_free = true;
      start_begin = 1;
 
      encoder.resetEncoders();
      
      //do{Motor(30, 30);} while(encoder.Poss_R() < 900);     
      
      speed_rotate = 50;           //------------>>  ตั้งความเร็วในการหมุน ซ้ายขวา
      rotate_right = 300;           //------------>>  ตั้งค่า องศา ในการหมุน ขวา 
      rotate_left = 280;           //------------>>  ตั้งค่า องศา ในการหมุน ซ้าย
      

      
      while(1)
        {
          move_fw(25, 25, 30);     //------------>>  ตั้งค่า 30, 30  คือความเร็วของมอเตอร์  ซ้าย และ ขวา  30 คือ ระยะทางเดิน ระหว่างบล๊อก 
        }

      
//\\------------------------------------------->>>>>   เขียนโค๊ดที่นี้ 
     
      
  }

void loop() 
  {  
    check_colors();
    Serial.println(check_color);
    
    /*
    for(int i=0; i<3; i++)
      {
        Serial.print(red_eep[i]);
        Serial.print(" ");
      }
     Serial.println("");
     */
    
  }
