
int servo_down = 47;      //-------------------->> ตั้งค่า มื่อจับลงขนานกับพื้น
int servo_close = 150;    //-------------------->> ตั้งค่า ฝ่ามือให้ปลายเข้ามาแตะกันพอดี

void arm_ready()
  {
    servo(28, servo_down);
    servo(27, servo_close-100);
  }

void arm_slow_down()
  {
    for(int i=servo_down + 50; i>servo_down + 10; i--)
      {
        servo(28, i);
        delay(5);
      }    
  }
  
void arm_close_up()
  {
    servo(27, servo_close);
    delay(400);
    servo(28, servo_down + 70);
    delay(400);
  }

void arm_open_up()
  {
    servo(27, servo_close-60);
    delay(400);
    servo(28, servo_down + 70);
    delay(400);
  }
void arm_down_open_up()
  {    
    servo(28, servo_down);
    delay(400);
    servo(27, servo_close-60);
    delay(400);
    servo(28, servo_down+90);
    delay(400);
  }
