


void arm_ready()
  {
    servo(28, servo_down);
    servo(27, servo_close-80);
  }

void arm_down_close()
  {
    servo(28, servo_down);
    servo(27, servo_close);
  }
void arm_slow_down()
  {
    for(int i=servo_down + 70; i>servo_down + 20; i--)
      {
        servo(28, i);
        delay(5);
      }    
  }
  
void arm_close_up()
  {
    servo(27, servo_close);
    delay(200);
    servo(28, servo_down + 70);
    delay(50);
  }

void arm_open_up()
  {
    servo(27, servo_close-60);
    delay(200);
    servo(28, servo_down + 90);
    delay(200);
  }
void arm_down_open_up()
  {    
    servo(28, servo_down);
    delay(300);
    servo(27, servo_close-60);
    delay(300);
    servo(28, servo_down+90);
  }
