void setup_encoder(void);
void _seting()
  {
      Serial.begin(9600);  
      Wire.begin();  
      dis.init();
      dis.configureDefault();
      dis.setTimeout(500);

      to_set_Freq("Coreless_Motors");      // กำหนดความถี่ให้กับมอเตอร์เกาหลีหรือ มอเตอร์ Coreless ("Coreless_Motors")
      //to_set_Freq("DC_Motors");          // กำหนดความถี่ให้กับมอเตอร์ธรรมกา(DC_Motors)
      
      to_set_motor_LR(100, 100);          //ตั้งค่ามอเตอร์ให้หุ่นยนต์วิ่งตรง
      
      to_slow_motor(30, 30);              // ตั้งค่า ความเร็วมอเตอร์เวลาเข้าแยก
      to_turn_center_l(-100, 100 );       // ตั้งค่ามอเตอร์หมุนซ้ายขวา ตอนเลี้ยวซ้ายแบบ center
      to_turn_center_r(100, -100 );       // ตั้งค่ามอเตอร์หมุนซ้ายขวา ตอนเลี้ยวขาวแบบ center
      to_turn_front_l(-25, 100);          // ตั้งค่ามอเตอร์หมุนซ้ายขวา ตอนเลี้ยวซ้ายแบบ เซนเซอร์หน้า  เดินหน้า
      to_turn_front_r(100, -25);          // ตั้งค่ามอเตอร์หมุนซ้ายขวา ตอนเลี้ยวขวาแบบ เซนเซอร์หน้า  เดินหน้า
      to_speed_turn_fl(100, 20, 50);      // ค่าพุ่งตัวออกหลังจากการเลี้ยว มีผลต่อตัวแปรสุดท้ายของคำสั่งเลี้ยว ถ้าเป็น 0
      to_speed_turn_fr(20, 100, 50);      // ค่าพุ่งตัวออกหลังจากการเลี้ยว มีผลต่อตัวแปรสุดท้ายของคำสั่งเลี้ยว ถ้าเป็น 0
      
      to_brake_fc(20, 60);                // ตั้งค่าในการหยุดมอเตอร์ก่อนหมุนซ้ายขวา to_brake_fc(...f..., ....c...); 
      to_brake_bc(2, 40);
      to_delay_f(1);                 // ระยะทางในการข้ามเส้นเวลาหมุนตัวแบบ f
    
      kp_sl(0.25, 0.25);  // kp และ kd ในฟังก์ เดินตามเส้นแบบช้า
      kd_fw(0.35);
      kd_bw(0.65);

      pos_motor_cal(10, 10, 10);   // ตั้งค่า คาริเบลทเซนเซอร์
      sensor_set();
      //setup_tcs();
      
  }
void setup_encoder() 
  { 
    pinMode(encoderPinA1, INPUT_PULLUP); 
    pinMode(encoderPinB1, INPUT_PULLUP);

    // Setup for Encoder 2
    pinMode(encoderPinA2, INPUT_PULLUP); 
    pinMode(encoderPinB2, INPUT_PULLUP);

    Serial.begin(9600);  // Start serial communication for debugging

    // Attach interrupts for both encoders
    attachInterrupt(digitalPinToInterrupt(encoderPinA1), encoderInterrupt1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinA2), encoderInterrupt2, CHANGE);
 
  }

void encoderInterrupt1() {
    int encoderPinA1Val = digitalRead(encoderPinA1);  
    if ((encoderPinA1Last == LOW) && (encoderPinA1Val == HIGH)) {
        if (digitalRead(encoderPinB1) == LOW) {
            encoderPoss1--;
        } else {
            encoderPoss1++;
        }
    } 
    encoderPinA1Last = encoderPinA1Val;  
}

// Interrupt service routine for Encoder 2
void encoderInterrupt2() {
    int encoderPinA2Val = digitalRead(encoderPinA2);  
    if ((encoderPinA2Last == LOW) && (encoderPinA2Val == HIGH)) {
        if (digitalRead(encoderPinB2) == LOW) {
            encoderPoss2--;
        } else {
            encoderPoss2++;
        }
    } 
    encoderPinA2Last = encoderPinA2Val;  
}
