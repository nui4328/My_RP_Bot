/*
float PID_outputs=0,errorss=0,Is=0,Ds=0;


void  servo_up_open()  // ยก กางออก
  {
    servo(8,servo_up );  //กลาง
    servo(23,servo_close-50); //ซ้าย  
    
  }

void servo_down_open()   // ลง กางออก
  {
    servo(23,servo_close-55); //ซ้าย   
    servo(8,servo_down );  //กลาง
  }

void  servo_up_close()  // ยก หุบเข้า
  {
     servo(23,servo_close); //ซ้าย    
    servo(8,servo_up );  //กลาง
  }

void  servo_down_close()  // ลง หุบเข้า
  {    
      servo(23, servo_close); //ซ้าย   
    servo(8,servo_down );  //กลาง
  }


void mission(int spl ,int spr, float kp,int _dis,int ofset) 
    {    
         servo(23,servo_close-75); 
         servo(8,servo_down);  delay(200);
        while(1)
          {

            float error_L = map(mcp_f(3), min_mcp_f(3), max_mcp_f(3), 0, 30 );
            float error_R = map(mcp_f(4), min_mcp_f(4), max_mcp_f(4), 0, 30 );
            errors = error_L - error_R;  
            I = 0;
            previous_I = 0;
            previous_error = 0;
            P = errors;
            I = I + previous_I;
            D = errors - previous_error ;            
            previous_I=I;
            previous_error=errors  ;  
            PID_output = (kp * P) + (0.0000015 * I) + (0.5* D); 

            Motor(spl + PID_output, spr - PID_output);
            
            if( mcp_f(3) > md_mcp_f(3) && mcp_f(4) > md_mcp_f(4) || mcp_f(2) < md_mcp_f(2) && mcp_f(5) < md_mcp_f(5) )               
              {
                Motor(-15,-15);
                delay(50);
                break;
              }  
           if( dis.readRangeSingleMillimeters() < _dis )               
              {
                Motor(-15,-15);
                delay(50);
                break;
              }                                
          }
        Motor(-1,-1);
        delay(10);
        servo(23,servo_close-40); //คีบ  
        delay(100);
                  
    }
    
  void slow_servo()
    {
      for(int i = 110; i > 20 ; i-- )
        {
          servo(8, i);
          delay(4);
        }
    }
    
void slow_down()
    {
      for(int i = servo_up; i > servo_down ; i-- )
        {
          servo(8, i);
          delay(4);
        }
    }

void  mis_r()
    {
      do{Motor(50, -50);}while(mcp_m(7) > md_mcp_m(7));delay(20);
      do{Motor(50, -50);}while(mcp_m(7) < md_mcp_m(7));
      Motor(-40, 40); delay(20);
      Motor(1, 1); delay(100);
      servo(23,servo_close-55); delay(300);
      servo(8,servo_up );delay(500);

      Motor(-50, 50);delay(150);
      do{Motor(-50, 50);}while(mcp_b(4) > md_mcp_b(4));
      Motor(30, -30); delay(20);
      Motor(1, 1); delay(100);
    }

void  mis_l()
    {
      do{Motor(-50, 50);}while(mcp_m(6) > md_mcp_m(6));delay(20);
      do{Motor(-50, 50);}while(mcp_m(6) < md_mcp_m(6));
      Motor(40, -40); delay(20);
      Motor(1, 1); delay(100);
      servo(23,servo_close-55); delay(300);
      servo(8,servo_up );delay(500);

      Motor(50, -50);delay(150);
      do{Motor(50, -50);}while(mcp_b(3) > md_mcp_b(3));
      Motor(-30, 30); delay(20);
      Motor(1, 1); delay(100);
    }
 */
