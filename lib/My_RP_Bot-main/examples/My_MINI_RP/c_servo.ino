float PID_outputs=0,errorss=0,Is=0,Ds=0;

void  servo_upp()  // ยกกล่อง
  {
    for(int i=60; i<140; i++) 
      {
        servo(8, i);
        delay(5);
      }
        
  }

void  servo_downs()  // ยก กางออก
  {
    for(int i=140; i>60; i--) 
      {
        servo(8, i);
        delay(5);  
      }
      
  }
void  servo_up_open()  // ยก กางออก
  {
    servo(8,servo_up );  //กลาง
    servo(0,servo_close-50); //ซ้าย  
    
  }

void servo_down_open()   // ลง กางออก
  {
    servo(0,servo_close-55); //ซ้าย   
    servo(8,servo_down );  //กลาง
  }

void  servo_up_close()  // ยก หุบเข้า
  {
     servo(0,servo_close); //ซ้าย    
     servo(8,servo_up );  //กลาง
  }

void  servo_down_close()  // ลง หุบเข้า
  {    
      servo(0, servo_close); //ซ้าย   
    servo(8,servo_down );  //กลาง
  }
