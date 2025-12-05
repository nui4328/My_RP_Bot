
void arm_ready()  //--------->> แขนลง กางฝ่ามือออกเตรียมเข้าไปคีบ
  {
    servo(1, servo_down);
    servo(10, servoL_open - 30);
    servo(0, servoR_open - 30);
  }

void arm_behind()  //--------->> แขนลง เอาฝ่ามือมาข้างหลัง
  {
    servo(1, servo_down);
    servo(10, servoL_open + 55);
    servo(0, servoR_open + 55);
  }
void arm_open_down()
  {    
    servo(10, servoL_open);
    servo(0, servoR_open);
    delay(300);
    servo(1, servo_down);    
  }
void arm_down_open()
  {
    servo(1, servo_down);
    delay(300);
    servo(10, servoL_open);
    servo(0, servoR_open);
  }
void arm_open_up()
  {    
    servo(10, servoL_open);
    servo(0, servoR_open); 
    delay(300);
    servo(1, servo_down+95);   
  }
void arm_up_open()
  {
    servo(1, servo_down+95);
    delay(300);
    servo(10, servoL_open);
    servo(0, servoR_open);
  }
void arm_down_close()
  {
    servo(1, servo_down);
    delay(100);
    servo(10, servoL_open - 115);
    servo(0, servoR_open - 111);
  }

void arm_up_close()
  {
    servo(1, servo_down+95);
    delay(100);
    servo(10, servoL_open - 91);
    servo(0, servoR_open - 91);
  }
void arm_big_box()
  {
    servo(1, servo_down);
    servo(10, servoL_open - 30);
    servo(0, servoR_open - 30);
  }

