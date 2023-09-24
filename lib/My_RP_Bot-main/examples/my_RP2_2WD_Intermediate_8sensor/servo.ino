void open_servo()
  {
    set_bl_box();
    for(int i=130; i>20; i--)
      {
        servo(23,i);
        delay(3);
      }
    delay(300);
    servo(23,135);
    delay(500);
  }
