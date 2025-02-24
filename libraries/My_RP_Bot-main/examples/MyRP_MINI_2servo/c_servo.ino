float PID_outputs=0,errorss=0,Is=0,Ds=0;
int servo_down = 50;      //-------------------->> ตั้งค่า มื่อจับลงขนานกับพื้น
int servo_close = 140;

void arm_ready()
  {
    servo(8, servo_down);
    servo(0, servo_close-50);
  }

void arm_close_up()
  {
    servo(0, servo_close);
    delay(400);
    servo(8, servo_down + 80);
    delay(400);
  }

void arm_down_open_up()
  {    
    servo(8, servo_down);
    delay(400);
    servo(0, servo_close-60);
    delay(400);
    servo(8, servo_down+80);
    delay(400);
  }

void place_right()
  {
    place_right_in(30, 45);
    delay(200);
    arm_down_open_up();
    place_right_out(30, 45);
  }

void place_letf()
  {
    place_letf_in(30, 45);
    delay(200);
    arm_down_open_up();
    place_letf_out(30, 45);
  }


  
void arm_down_open()
  {
    servo(8, servo_down);
    servo(0, servo_close - 60);
  }
void arm_down_close()
  {
    servo(8, servo_down);
    servo(0, servo_close);
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

  
  
