float PID_outputs=0,errorss=0,Is=0,Ds=0;
int servo_down = 52;      //-------------------->> ตั้งค่า มื่อจับลงขนานกับพื้น
int servoL_open = 167;      //-------------------->> ตั้งค่า กางฝ่ามือด้านซ้าย
int servoR_open = 10;  //-------------------->> ตั้งค่า กางฝ่ามือด้านขวา


void arm_ready()
  {
    servo(35, servo_down);
    servo(34, servoL_open - 30);
    servo(36, servoR_open + 30);
  }
void arm_ready_min()
  {
    servo(35, servo_down);
    servo(34, servoL_open - 65);
    servo(36, servoR_open + 65);
  }
void arm_ready_r()
  {
    servo(35, servo_down);
    servo(34, servoL_open - 30);
    servo(36, servoR_open + 70);
  }

void arm_ready_l()
  {
    servo(35, servo_down);
    servo(34, servoL_open - 70);
    servo(36, servoR_open + 30);
  }

void arm_open_min_up()
  {
    servo(35, servo_down);
    servo(34, servoL_open - 60);
    servo(36, servoR_open + 60);
    delay(200);
    servo(35, servo_down + 90);
    delay(200);
  }

void arm_down_open()
  {
    servo(35, servo_down);
    servo(34, servoL_open);
    servo(36, servoR_open);
  }
void arm_up_open()
  {
    servo(35, servo_down + 90);
    servo(34, servoL_open);
    servo(36, servoR_open);
  }
void arm_down_close()
  {
    servo(35, servo_down);
    servo(34, servoL_open-110);
    servo(36, servoR_open+114);
  }
void arm_big_box()
  {
    servo(35, servo_down);
    servo(34, servoL_open - 70);
    servo(36, servoR_open + 70);
  }

