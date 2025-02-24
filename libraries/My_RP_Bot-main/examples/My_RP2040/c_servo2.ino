/*
float PID_outputs=0,errorss=0,Is=0,Ds=0;


void  servo_up_open()  // ยก กางออก
  {
    servo(8,servo_up );  //กลาง
    
    servo(7,15); //ขวา
    servo(23,165); //ซ้าย  
    
  }

void servo_down_open()   // ลง กางออก
  {
    servo(7,servoR_open); //ขวา
    servo(23,servoL_open); //ซ้าย   
    servo(8,servo_down );  //กลาง
  }

void  servo_up_close()  // ยก หุบเข้า
  {
     
     servo(23,servoL_open - 110); //ซ้าย 
     servo(7,servoR_open + 110); //ขวา    
     servo(8,servo_up );  //กลาง
  }

void  servo_down_close()  // ลง หุบเข้า
  {    
     servo(23,servoL_open - 110); //ซ้าย 
     servo(7,servoR_open + 110); //ขวา    
     servo(8,servo_down );  //กลาง
  }

void servo_big()    //
  {
      servo(23,servoL_open - 100); //ซ้าย 
      servo(7,servoR_open + 100); //ขวา 
      servo(8,servo_down );  //กลาง  
  }

void  mission_L()  //  a
     {
          Motor(-1,20);
          delay(200);
          Motor(1,-30);
          delay(20);
          Motor(1,1);
          delay(20);
          
          servo_down_open();
    
          delay(300);
          Motor(1,-20);
          delay(200);
          Motor(-1,30);
          delay(20);
          Motor(1,1);
          delay(20);   
     }
void  mission_R()
     {
          Motor(20,-1);
          delay(180);
          Motor(-30,1);
          delay(20);
          Motor(1,1);
          delay(40);
          
          servo_down_open();
    
          delay(300);
          Motor(-20,1);
          delay(180);
          Motor(30,-1);
          delay(20);
          Motor(1,1);
          delay(20);   
     }
void  mission_L_M()
     {
          Motor(-1,30);
          delay(350);
          Motor(1,-30);
          delay(30);
          Motor(0,0);
          delay(20);
          
          servo_down_open();
    
          delay(300);
          Motor(1,-30);
          delay(350);
          Motor(-1,30);
          delay(20);
          Motor(0,0);
          delay(20);   
     }
     
void  mission_R_M()
     {
          Motor(30,-1);
          delay(380);
          Motor(-30,1);
          delay(30);
          Motor(0,0);
          delay(40);
          
          servo_down_open();
    
          delay(300);
          Motor(-30,1);
          delay(390);
          Motor(30,-1);
          delay(20);
          Motor(0,0);
          delay(20);   
     }
    
void mission(int SPL ,int SPR, float KP,int TIM,String box) 
    {         
          servo(23,servoL_open - 70); //ซ้าย 
          servo(7,servoR_open + 70); //ขวา    
          servo(8,servo_down );  //กลาง
          
          
          delay(300);

           lasts_time=millis();
                 while(millis()-lasts_time<TIM  )
                       {
                         PID_output = (KP * error_F()) + (0.00015 * I) + (0.4* D);
                         Motor(SPL - PID_output,SPR + PID_output);                      
                        }
           Motor(-20,-20);delay(15);
           Motor(1,1);delay(100);

          if (box == "big")
            {
               servo_big();        
            }
          else
            {
                servo_down_close();
            }
        
                  
    }
  */ 
