 void servo_mission() 
    {                       
      servo(8,120);  //หุบ       
      delay(300);  
  
      servo(23,servo_up );  //ยก
      delay(300); 
    
    }
void servo_store() // วางพื้นขาวดำ
    {
       servo(23,servo_down );  //ลง
       delay(100);
       servo(8,90);   // กาง
       delay(100);
       servo(23,120 );  //ยก
       delay(300);
    }

  void down_servo() // วาง
    {
       servo(7,servo_down );  //ลง
       servo(6,50);
       delay(2);
    }
 
 void slow_servo()
  {
    for(int i = servo_up; i > servo_down + 10; i-- )
      {
        servo(23, i);
        delay(3);
      }
  }
