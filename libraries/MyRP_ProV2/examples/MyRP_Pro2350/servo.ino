
void arm_ready()  //--------->> แขนลง กางฝ่ามือออกเตรียมเข้าไปคีบ
  {
    servo(36, servo_down);
    servo(34, servoL_open - 30);
    servo(35, servoR_open - 30);
  }

void arm_behind()  //--------->> แขนลง เอาฝ่ามือมาข้างหลัง
  {
    servo(36, servo_down);
    servo(34, servoL_open + 55);
    servo(35, servoR_open + 55);
  }
void arm_open_down()
  {    
    servo(34, servoL_open);
    servo(35, servoR_open);
    delay(300);
    servo(36, servo_down);    
  }
void arm_down_open()
  {
    servo(36, servo_down);
    delay(300);
    servo(34, servoL_open);
    servo(35, servoR_open);
  }
void arm_open_up()
  {    
    servo(34, servoL_open);
    servo(35, servoR_open); 
    delay(300);
    servo(36, servo_down+95);   
  }
void arm_up_open()
  {
    servo(36, servo_down+95);
    delay(300);
    servo(34, servoL_open);
    servo(35, servoR_open);
  }
void arm_down_close()
  {
    servo(36, servo_down);
    delay(100);
    servo(34, servoL_open - 91);
    servo(35, servoR_open - 91);
  }

void arm_up_close()
  {
    servo(36, servo_down+95);
    delay(100);
    servo(34, servoL_open - 91);
    servo(35, servoR_open - 91);
  }
void arm_big_box()
  {
    servo(36, servo_down);
    servo(34, servoL_open - 30);
    servo(35, servoR_open - 30);
  }

