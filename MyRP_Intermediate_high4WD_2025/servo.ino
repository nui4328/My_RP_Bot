void servo_red()
  {   
    red_box = 1;
    bz(100);
    servo(27, servo27-60);
    delay(400);
    servo(27, servo27);
     //------------->>  ตรวจสอบนับการวางกล่องสี
    if(red_box == 1 && green_box == 1 && blue_box == 1 && yello_box == 1 && ch_poit >= 2)
      {
        servo_pico26.write(120);
        bz(300);bz(300);bz(300);
        
        begin_robot();    
      }
    delay(200);
  }

void servo_green()
  {
    green_box = 1;
    bz(100);bz(100);
    servo(27, servo27+60);
    delay(400);
    servo(27, servo27);
    if(red_box == 1 && green_box == 1 && blue_box == 1 && yello_box == 1 && ch_poit >= 2)
      {
        servo_pico26.write(120);
        bz(300);bz(300);bz(300);
        
        begin_robot();   
      }
    delay(200);
  }

void servo_yello()
  {
    yello_box = 1;
    bz(50);bz(50);bz(50);bz(50);
    servo(28, servo28+60);
    delay(400);
    servo(28, servo28);
    if(red_box == 1 && green_box == 1 && blue_box == 1 && yello_box == 1 && ch_poit >= 2)
      {
        servo_pico26.write(120);
        bz(300);bz(300);bz(300);
        
        begin_robot();      
      }
    delay(200);
  }
void servo_blue()
  {
    blue_box = 1;
    bz(100);bz(100);bz(100);
    servo(28, servo28-60);
    delay(400);
    servo(28, servo28);
    if(red_box == 1 && green_box == 1 && blue_box == 1 && yello_box == 1 && ch_poit >= 2)
      {
        servo_pico26.write(120);
        bz(300);bz(300);bz(300);
        
        begin_robot();       
      }
     delay(200);
  }
