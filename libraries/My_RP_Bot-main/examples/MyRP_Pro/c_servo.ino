float PID_outputs=0,errorss=0,Is=0,Ds=0;
int servo_down = 52;      //-------------------->> ตั้งค่า มื่อจับลงขนานกับพื้น
int servoL_open = 165;      //-------------------->> ตั้งค่า กางฝ่ามือด้านซ้าย
int servoR_open = 10;  //-------------------->> ตั้งค่า กางฝ่ามือด้านขวา

int servo_close = 126;  //-------------------->> ตั้งค่า กางฝ่ามือด้านขวา
//////////////////--------------->>>  ใช้มือจับแบบ 3servo
void arm_ready()
  {
    servo(8, servo_down);
    servo(23, servoL_open - 50);
    servo(7, servoR_open + 50);
  }

void arm_down_open()
  {
    servo(8, servo_down);
    servo(23, servoL_open);
    servo(7, servoR_open);
  }
void arm_down_close()
  {
    servo(8, servo_down);
    servo(23, servoL_open - 117);
    servo(7, servoR_open + 114);
  }
void arm_up_close()
  {
    servo(8, servo_down + 60);
    servo(23, servoL_open - 117);
    servo(7, servoR_open + 114);
  }
void arm_big_box()
  {
    servo(8, servo_down);
    servo(23, servoL_open - 70);
    servo(7, servoR_open + 70);
  }

  
//////////////////--------------->>>  ใช้มือจับแบบ 2servo

void arm2_ready()
  {
    servo(8, servo_down);
    servo(23, servo_close-40);
  }

void arm2_down_open()
  {
    servo(8, servo_down);
    servo(23, servo_close-70);
  }
void arm2_down_close()
  {
    servo(8, servo_down);
    servo(23, servo_close);
  }
void arm2_up_close()
  {
    servo(8, servo_down + 60);
    servo(23, servo_close);
  }
void arm2_big_box()
  {
    servo(8, servo_down);
    servo(23, servo_close-20);
  }

void place_right_in(int sp, int deg)
  {
    int en_coder = map(deg, 0, 90, 0, 600);
    encoder.resetEncoders();   //----->> encoder = 0
    do{ Motor(sp, -1); }while( encoder.Poss_L() < en_coder );
    Motor(-sp, 1); delay(40);
    Motor(-1, 1); delay(40);
  }

void place_right_out(int sp, int deg)
  {
    int en_coder = map(deg, 0, 90, 0, 600);
    encoder.resetEncoders();   //----->> encoder = 0
    do{ Motor(-sp, 1); }while( encoder.Poss_L() > -en_coder );
    Motor(sp, -1); delay(40);
    Motor(1, -1); delay(40);
  }
void place_letf_in(int sp, int deg)
  {
    int en_coder = map(deg, 0, 90, 0, 600);
    encoder.resetEncoders();   //----->> encoder = 0
    do{ Motor(-1, sp); }while( encoder.Poss_R() < en_coder );
    Motor(1, -sp); delay(40);
    Motor(1, -1); delay(40);
  }

void place_letf_out(int sp, int deg)
  {
    int en_coder = map(deg, 0, 90, 0, 600);
    encoder.resetEncoders();   //----->> encoder = 0
    do{ Motor(1, -sp); }while( encoder.Poss_R() > -en_coder );
    Motor(-1, sp); delay(40);
    Motor(-1, 1); delay(40);
  }

  
  
