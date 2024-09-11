
#include <my2wd_encoder.h>
#include <Wire.h>
#include <my_mpu6050.h> 
#include <my_TCS34725.h>
#include <EEPROM.h>
#define EEPROM_ADDR 0x50

//------------------------->>
my_TCS34725 tcss = my_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
// กำหนดพินอินพุตตัวเข้ารหัส


double cr[] = {444.00, 118.00, 118.00};
double cg[] = {98.00, 192.00, 151.00};
double cb[] = {90.00, 147.00, 231.00};
double cy[] = {980.00, 748.00, 293.00};
uint16_t readRED_eep[3];
uint16_t readGREEN_eep[3];
uint16_t readBLUE_eep[3];
uint16_t readYELLO_eep[3];
float red_eep[3];
float green_eep[3];
float blue_eep[3];
float yello_eep[3];

int red_box, green_box, blue_box, yello_box, ch_poit;
int servo20 = 80;
int servo28 = 85;
int num_encoder = 0;
int roll_L = 900;

/*
int cr[3] = {444, 118, 118};
int cg[3] = {98, 192, 151};
int cb[3] = {90, 147, 231};
int cy[3] = {980, 748, 293};
 */

String floor_R;
String floor_M;
String floor_L;

String can;

String color_ch="none";

const int pinA = 11;
const int pinB = 10;

// กำหนดตัวแปรเพื่อติดตามตำแหน่งและทิศทางของตัวเข้ารหัส
volatile int encoderPos = 0;
volatile int encoderPos_r = 0;
int encoderDir = 1;
int prevAVal;
//------------------------->>
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

int ofset_rotate_around = 92;  // หมุนซ้ายกลับออกมาหากไม่ตรงให้ปรับค่าตรงนี้  หมุนซ้ายมากให้ลดลง //---------------------->>>

int Back_from_line = 340;       // เจอเส้นแล้วถอยหลังก่อนจะหมุ่นตัวซ้ายเพื่อเดินต่อ
int Forward_not_finding_line = 600;  // เดินหน้าต่อเมื่อไม่เจอเส้น ให้ถึงกลางบล๊อก

int start_begin = 0;
int rotate_color = 800;  // หมุนซ้ายกลับออกมาหลังจากวางกล่องตามสี

void setup()
  {    
     Serial.begin(9600);
     sensor_set();              // ค่าเริ่มต้น eeprom, bit_analogRead=>12   
     setup_encoder();

     mydisplay_background(black);
     mydisplay("MY-MAKERS", 20, 30, 2, white);
     read_eppcolor();
     servo(20, 90);
     servo(28, 90);

     begin_robot();
    /*
     sw_start();
     get_red_to_eep();sw_start();
     get_green_to_eep();sw_start();
     get_blue_to_eep();sw_start();
     get_yello_to_eep();sw_start();
    */
//\\------------------------------------------->>>>>  เขียนโค๊ดหุ่นยนต์ที่นี้
     
     //servo_red(); servo_blue(); delay(1000);
     //servo_green(); servo_yello(); delay(1000);

      encoderPos = 0;
      do{Motor(30, 30);} while(encoderPos < 1500);
//\\------------------------------------------->>>>>   เขียนโค๊ดที่นี้ 
     
      
  }

void loop() 
  {  
    move_fw(35, 35, 1600, 70, 750);  // ระดับสูง
  }
