float PID_outputs=0,errorss=0,Is=0,Ds=0;
int servo_down = 55;      //-------------------->> ตั้งค่า มื่อจับลงขนานกับพื้น
int servo_up = servo_down + 85;       //-------------------->> ตั้งค่า มื่อจับยกขึ้นตั้งฉากกับพื้น
int servoL_open = 173;      //-------------------->> ตั้งค่า กางฝ่ามือด้านซ้าย
int servoR_open = 13;  //-------------------->> ตั้งค่า กางฝ่ามือด้านขวา

int servo_close = 130; 

void arm_down_open()
  {
    servo(8, servo_down);
    servo(0, servoL_open);
    servo(1, servoR_open);
  }
