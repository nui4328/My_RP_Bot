#include <my_rp2040.h>


unsigned long lasts_time_servo = millis();
unsigned long lasts_time_ch = millis();
int servo_down = 45;
int servo_up = 130;

int ground_color = 0; 
int can_color = 0;  
int tll = 0;
int trr = 0;
int st_can1 = 0;
int st_can2 = 0;
int st_can3 = 0;
int st_can4 = 0;
void setup() 
  {    
      Serial.begin(9600);
      sensor_set();              // ค่าเริ่มต้น eeprom, bit_analogRead=>12      
      pos_motor_cal(10, 10, 10);
      to_set_motor_LR(100, 100);          //ตั้งค่ามอเตอร์ให้หุ่นยนต์วิ่งตรง      
      to_slow_motor(25, 25);           // ตั้งค่า ความเร็วมอเตอร์เวลาเข้าแยก
      to_turn_center_l(-100, 100 );   // ตั้งค่ามอเตอร์หมุนซ้ายขวา ตอนเลี้ยวซ้ายแบบ center
      to_turn_center_r(100, -100 );   // ตั้งค่ามอเตอร์หมุนซ้ายขวา ตอนเลี้ยวขาวแบบ center
      to_turn_front_l(-15, 100);     // ตั้งค่ามอเตอร์หมุนซ้ายขวา ตอนเลี้ยวซ้ายแบบ เซนเซอร์หน้า  เดินหน้า
      to_turn_front_r(100, -15);     // ตั้งค่ามอเตอร์หมุนซ้ายขวา ตอนเลี้ยวขวาแบบ เซนเซอร์หน้า  เดินหน้า
      to_speed_turn_fl(100, 30, 35);  // ค่าพุ่งตัวออกหลังจากการเลี้ยว มีผลต่อตัวแปรสุดท้ายของคำสั่งเลี้ยว ถ้าเป็น 0
      to_speed_turn_fr(30, 100, 35);  // ค่าพุ่งตัวออกหลังจากการเลี้ยว มีผลต่อตัวแปรสุดท้ายของคำสั่งเลี้ยว ถ้าเป็น 0      
      to_brake_fc(20, 50);            // ตั้งค่าในการหยุดมอเตอร์ก่อนหมุนซ้ายขวา to_brake_fc(...f..., ....c...); 
      to_delay_f(5);                 // ระยะทางในการข้ามเส้นเวลาหมุนตัวแบบ f
    
      kp_sl(0.15, 0.15);  // kp และ ki ในฟังก์ เดินตามเส้นแบบช้า
      kd_fw(0.4);
      kd_bw(2);

      
      //setup_tcs();
     
      servo(23,servo_down );  //ลง 
      servo(8,120);   

      sw();

        start_game();
        start_run();


  }

void loop() 
  {  
      

      

  }
