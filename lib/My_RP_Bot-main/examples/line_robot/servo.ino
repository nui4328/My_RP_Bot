
float PID_outputs=0,errorss=0,Is=0,Ds=0;


void  servo_up_open()  // ยก กางออก
  {
    servo(8,servo_up );  //กลาง    
    servo(7,15); //ขวา
    servo(23,155); //ซ้าย      
  }

void servo_down_open()   // ลง กางออก
  {
    servo(7,15); //ขวา
    servo(23,165); //ซ้าย   
    servo(8,servo_down );  //กลาง
  }

void  servo_up_close()  // ยก หุบเข้า
  {
    servo(7,150); //ขวา
    servo(23,40); //ซ้าย 
    servo(8,servo_up );  //กลาง
  }

void  servo_down_close()  // ลง หุบเข้า
  {    
    servo(7,140); //ขวา
    servo(23,40); //ซ้าย   
    servo(8,servo_down );  //กลาง
  }
void servo_st()       // เตรียมพร้อมรอคีบกล่อง
  {
     servo(7,80); //ขวา
     servo(23,95); //ซ้าย    
     servo(8,servo_down );  //กลาง  
  }

void servo_big()    
  {
      servo(7,130); //ขวา
      servo(23,45); //ซ้าย     
      servo(8,servo_down );  //กลาง  
  }

void  mission_L()
     {
          Motor(-1,30);
          delay(220);
          Motor(1,-30);
          delay(20);
          Motor(0,0);
          delay(20);
          
          servo_down_open();
    
          delay(300);
          Motor(1,-30);
          delay(220);
          Motor(-1,30);
          delay(20);
          Motor(0,0);
          delay(20);   
     }
void  mission_R()
     {
          Motor(30,-1);
          delay(220);
          Motor(-30,1);
          delay(20);
          Motor(0,0);
          delay(40);
          
          servo_down_open();
    
          delay(300);
          Motor(-30,1);
          delay(220);
          Motor(30,-1);
          delay(20);
          Motor(0,0);
          delay(20);   
     }
void  mission_L_M()
     {
          Motor(-1,30);
          delay(380);
          Motor(1,-30);
          delay(30);
          Motor(0,0);
          delay(20);
          
          servo_down_open();
    
          delay(300);
          Motor(1,-30);
          delay(390);
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


void mission(int SPL ,int SPR, float KP,unsigned long TIM, String box) 
    {                 
          delay(100);

           lasts_time_servo=millis();
           while(millis()-lasts_time_servo<TIM  )
               {
                   PID_output = (KP * error_F()) + (0.00015 * I) + (0.3* D);
                   Motor(SPL - PID_output,SPR + PID_output);                                           
               }
           Motor(-20,-20);delay(15);
           Motor(0,0);delay(100);

          if (box == "big")
            {
               servo_big();        
            }
          else
            {
                servo_down_close();
            }         
                  
    }
  
