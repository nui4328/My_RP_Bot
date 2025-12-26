
void arm_ready()  //--------->> แขนลง กางฝ่ามือออกเตรียมเข้าไปคีบ
  {
    servo(1, servo_down);
    servo(10, servoL_open - 30);
    servo(0, servoR_open - 30);
  }

void arm_open_down()   //--------->>  กางฝ่ามือออก  และเอาแขนลง
  {    
    servo(10, servoL_open);
    servo(0, servoR_open);
    delay(300);
    servo(1, servo_down);    
  }
void arm_down_open()   //--------->>  เอาแขนลง  และ กางฝ่ามือออก  
  {
    servo(1, servo_down);
    delay(300);
    servo(10, servoL_open);
    servo(0, servoR_open);
  }
void arm_open_up()    //--------->>  กางฝ่ามือออก  และยกแขนขึ้น
  {    
    servo(10, servoL_open);
    servo(0, servoR_open); 
    delay(300);
    servo(1, servo_down+95);   
  }
void arm_up_open()   //--------->>  เอาแขนขึ้น  และ กางฝ่ามือออก  
  {
    servo(1, servo_down+95);
    delay(300);
    servo(10, servoL_open);
    servo(0, servoR_open);
  }
void arm_down_close()  //--------->>  เอาแขนลง  และ หุบมือเข้า  
  {
    servo(1, servo_down);
    delay(100);
    servo(10, servoL_open - 115);
    servo(0, servoR_open - 111);
  }

void arm_up_close()   //--------->>  ยกแขนขึ้น  และ หุบมือเข้า  
  {
    servo(1, servo_down+95);
    delay(100);
    servo(10, servoL_open - 91);
    servo(0, servoR_open - 91);
  }
void arm_big_box()   //--------->>  คีบกล่องใหญ่
  {
    servo(1, servo_down);
    servo(10, servoL_open - 30);
    servo(0, servoR_open - 30);
  }

