
void fw(int spl, int spr, int _time, String offset)
  {

    last_time = millis();
    while(millis()-last_time < _time)
      {
        Motor(spl, spr);
        if(mcp_f(0) < md_mcp_f(0)-100 && mcp_f(3) > md_mcp_f(3))
              { 
                Motor(spl, -10);                
              }
        else if(mcp_f(0) > md_mcp_f(0) && mcp_f(3) < md_mcp_f(3)-100)
              {
                Motor(-10, spr);
              }
        else if(mcp_f(0) < md_mcp_f(0) && mcp_f(3) < md_mcp_f(3)-100)
              {
                Motor(-40, -40); delay(50);
                Motor(-2, -2);
                delay(300);
                goto end_time;
              }
      }
 
    
    if(offset == "line")
      {  
        while(1)      
           {    
              Motor(motor_slow, motor_slow);        
              if(mcp_f(0) < md_mcp_f(0)-100 && mcp_f(3) > md_mcp_f(3))
                  {
                    Motor(40, -10);
                  }
              else if(mcp_f(0) > md_mcp_f(0) && mcp_f(3) < md_mcp_f(3)-100)
                  {
                    Motor(-10, 40);
                  }
              else if(mcp_f(1) < md_mcp_f(1) || mcp_f(2) < md_mcp_f(2))
                  {
                    Motor(-40, -40); delay(80);  
                    Motor(-2, -2);delay(100);
                    
                    break;                  
                  }                  
          }
  
        line = true;                     
      } 
    else
      {
        Motor(-40, -40); delay(30);
        Motor(-2, -2);
        delay(300);
        line = false;
      }
    end_time:
    en_for:delay(10);
    ch_set_fb = false;      
  }

void bw(int spl, int spr, int _time, String offset)
  {
    for(int i = 20; i < spl; i ++)  // 
          {           
            Motor(-i, -i);
            delay(2);
          }
    last_time = millis();
    while(millis()-last_time < _time)
      {
        Motor(-spl, -spr);
        if(mcp_f(7) < md_mcp_f(7) && mcp_f(4) > md_mcp_f(4))
              {
                Motor(20, -spr);
              }
        else if(mcp_f(7) > md_mcp_f(7) && mcp_f(4) < md_mcp_f(4))
              {
                Motor(-spl, 20);
              }
        else if(mcp_f(7) < md_mcp_f(7) && mcp_f(4) < md_mcp_f(4))
              {
                goto end_time;
              }
      }
    for(int i = spl; i > 30; i -= 1)  // 
          {           
            Motor(-i, -i);
            delay(3);
          } 
     if(offset == "line")
      {  
        while(1)      
           {
              Motor(-motor_slow, -motor_slow);
            
              if(mcp_f(6) < md_mcp_f(6)-100 || mcp_f(5) < md_mcp_f(5)-100)
                  {
                    Motor(60, 60);delay(50);
                    while(1)
                      {
                        if(mcp_f(4) < md_mcp_f(4) && mcp_f(7) > md_mcp_f(7)) 
                          {
                            do{Motor(2, -30);}while(mcp_f(7) > md_mcp_f(7)+50);    
                            Motor(-2, 30);delay(30);
                            Motor(0, 0); delay(10);
            
                            break; 
                            
                          }
                        else if(mcp_f(4) > md_mcp_f(4) && mcp_f(7) < md_mcp_f(7))
                          {
                            do{Motor(-30, 2);}while(mcp_f(4) > md_mcp_f(4)+50);    
                            Motor(30, -2);delay(30);
                            Motor(0, 0); delay(10);
                            break; 
                          }
            
                        else
                          {
                            Motor(-motor_slow, -motor_slow);
                          }
                      }
                    break;
                  }
               else if(mcp_f(7) < md_mcp_f(7) && mcp_f(4) > md_mcp_f(4))
                  {
                    Motor(10, -40);
                    //delay(5);
                  }
               else if(mcp_f(7) > md_mcp_f(7) && mcp_f(4) < md_mcp_f(4))
                  {
                    Motor(-40, 10);
                    //delay(5);
                  }

          }  
        line = true;
        ch_bw = true;                     
      }  
    else
      {
        Motor(50, 50); delay(40);
        Motor(0, 0);
        delay(200);
        line = false;
      }
    end_time:
    en_for:delay(10);
    ch_set_fb = false;
    
  }

void fw_no_sensor(int spl, int spr, int _time, String offset)
  {

    last_time = millis();
    while(millis()-last_time < _time)
      {
        Motor(spl, spr);
      }
 
    
    if(offset == "line")
      {  
        while(1)      
           {    
              Motor(motor_slow, motor_slow);        
              if(mcp_f(0) < md_mcp_f(0)-100 && mcp_f(3) > md_mcp_f(3))
                  {
                    Motor(40, -10);
                  }
              else if(mcp_f(0) > md_mcp_f(0) && mcp_f(3) < md_mcp_f(3)-100)
                  {
                    Motor(-10, 40);
                  }
              else if(mcp_f(1) < md_mcp_f(1) || mcp_f(2) < md_mcp_f(2))
                  {
                    Motor(-40, -40); delay(80);  
                    Motor(-2, -2);delay(100);
                    
                    break;                  
                  }                  
          }
  
        line = true;                     
      } 
    else
      {
        Motor(-40, -40); delay(30);
        Motor(-2, -2);
        delay(300);
        line = false;
      }
    end_time:
    en_for:delay(10);
    ch_set_fb = false;      
  }
