
int servo_down = 50;      //-------------------->> ตั้งค่า มื่อจับลงขนานกับพื้น
int servoL_open = 170;      //-------------------->> ตั้งค่า กางฝ่ามือด้านซ้าย
int servoR_open = 10;  //-------------------->> ตั้งค่า กางฝ่ามือด้านขวา


void arm_ready()  //--------->> แขนลง กางฝ่ามือออกเตรียมเข้าไปคีบ
  {
    servo(36, servo_down);
    servo(34, servoL_open - 30);
    servo(35, servoR_open + 30);
  }

void arm_down_open()
  {
    servo(36, servo_down);
    delay(300);
    servo(34, servoL_open);
    servo(35, servoR_open);
  }
void arm_up_open()
  {
    servo(36, servo_down + 90);
    servo(34, servoL_open);
    servo(35, servoR_open);
    servo(36, servo_down + 90);
  }
void arm_down_close()
  {
    servo(36, servo_down);
    delay(50);
    servo(34, servoL_open-110);
    servo(35, servoR_open+110);
  }

void arm_up_close()
  {
    servo(36, servo_down+90);
    servo(34, servoL_open-110);
    servo(35, servoR_open+110);
  }
void arm_big_box()
  {
    servo(36, servo_down);
    servo(34, servoL_open - 70);
    servo(35, servoR_open + 70);
  }

