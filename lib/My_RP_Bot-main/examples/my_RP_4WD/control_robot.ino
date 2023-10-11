void fw(int sp, int _time)
  {
    Motor(sp, sp, sp, sp);
    delay(_time);
    Motor(0, 0, 0, 0);delay(100);
  }
void bw(int sp, int _time)
  {
    Motor(-sp, -sp, -sp, -sp);
    delay(_time);
    Motor(0, 0, 0, 0);delay(100);
  }
void move_R(int sp, int _time)
  {
    Motor(sp, -sp, -sp, sp);
    delay(_time);
    Motor(0, 0, 0, 0);delay(100);
  }
  
void move_L(int sp, int _time)
  {
    Motor(-sp, sp, sp, -sp);
    delay(_time);
    Motor(0, 0, 0, 0);delay(100);
  }
