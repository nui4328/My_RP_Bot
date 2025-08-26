
int servo_down = 52;      //-------------------->> ตั้งค่า มื่อจับลงขนานกับพื้น
int servoL_open = 167;      //-------------------->> ตั้งค่า กางฝ่ามือด้านซ้าย
int servoR_open = 10;  //-------------------->> ตั้งค่า กางฝ่ามือด้านขวา


void arm_ready()  //--------->> แขนลง กางฝ่ามือออกเตรียมเข้าไปคีบ
  {
    servo(35, servo_down);
    servo(34, servoL_open - 30);
    servo(36, servoR_open + 30);
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
    servo(36, servoR_open+110);
  }

void arm_up_close()
  {
    servo(35, servo_down+90);
    servo(34, servoL_open-110);
    servo(36, servoR_open+110);
  }
void arm_big_box()
  {
    servo(35, servo_down);
    servo(34, servoL_open - 70);
    servo(36, servoR_open + 70);
  }

