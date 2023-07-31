 void servo_mission() 
    {                         
      servo(23,120);  //หุบ      
      delay(300); 
      check_box();    
      servo(8,servo_up );  //ยก
      //delay(300); 
      

    }
void servo_store() // วางพื้นขาวดำ
    {    
       servo(8,servo_down );  //ลง   
       delay(400);
       slow_servo_open();
       delay(200);
       servo(8,servo_up);  //ลง
       delay(300);
    }

  void down_servo() // วาง
    {
       servo(8,servo_down );  //ลง
       servo(23,60);
       delay(200);
    }
 
 void slow_servo_down()
  {
    for(int i = servo_up; i > servo_down + 10; i-- )
      {
        servo(8, i);
        delay(6);
      }
  }
  
 void slow_servo_open()
  {
    for(int i = 120; i > 90; i-- )
      {
        servo(23, i);
        delay(10);
      }
  }
