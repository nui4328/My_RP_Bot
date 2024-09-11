void servo_red()
  {   
    red_box = 1;
    bz(100);
    servo(20, servo20-40);
    delay(400);
    servo(20, servo28);
     //------------->>  ตรวจสอบนับการวางกล่องสี
    if(red_box == 1 && green_box == 1 && blue_box == 1 && yello_box == 1 && ch_poit >= 2)
      {
        bz(300);bz(300);bz(300);
        servo(27, 20);
        delay(300000);    
      }
    delay(200);
  }

void servo_green()
  {
    green_box = 1;
    bz(100);bz(100);
    servo(20, servo20+50);
    delay(400);
    servo(20, servo20);
    if(red_box == 1 && green_box == 1 && blue_box == 1 && yello_box == 1 && ch_poit >= 2)
      {
        bz(300);bz(300);bz(300);
        servo(27, 20);
        delay(300000);    
      }
    delay(200);
  }

void servo_yello()
  {
    yello_box = 1;
    bz(100);bz(100);bz(100);bz(100);
    servo(28, servo28+50);
    delay(400);
    servo(28, servo28);
    if(red_box == 1 && green_box == 1 && blue_box == 1 && yello_box == 1 && ch_poit >= 2)
      {
        bz(300);bz(300);bz(300);
        servo(27, 20);
        delay(300000);       
      }
    delay(200);
  }
void servo_blue()
  {
    blue_box = 1;
    bz(100);bz(100);bz(100);
    servo(28, servo28-50);
    delay(400);
    servo(28, servo28);
    if(red_box == 1 && green_box == 1 && blue_box == 1 && yello_box == 1 && ch_poit >= 2)
      {
        bz(300);bz(300);bz(300);
        servo(27, 20);
        delay(300000);       
      }
     delay(200);
  }
