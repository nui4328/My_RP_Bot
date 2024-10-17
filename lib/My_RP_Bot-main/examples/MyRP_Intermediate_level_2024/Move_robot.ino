void fw(int spl, int spr, int en_pos, String _line)
  {
    lines_fw = true; 
    encoderPos = 0;
    while(encoderPos < en_pos)
      {
        Motor(spl, spr);
        if(mcp_f(0) < md_mcp_f(0)-100 && mcp_f(3) > md_mcp_f(3))
              {
                line_l = false;
                Motor(spl, -10);
              }
        else if(mcp_f(0) > md_mcp_f(0) && mcp_f(3) < md_mcp_f(3)-100)
              {
                line_r = false;
                Motor(-10, spr);
              }
        else if(mcp_f(0) < md_mcp_f(0) && mcp_f(3) < md_mcp_f(3)-100)
              {     
                Motor(-40, -40); delay(30);
                Motor(-1, -1);
                delay(100);           
                break;
              }
      }
     Motor(-1, -1);
     delay(10);
     if(_line == "line")
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
                    Motor(-40, -40); delay(20);  
                    Motor(-1, -1);delay(10);  
                    while(1)
                      {
                        if(mcp_f(1) < md_mcp_f(1) && mcp_f(2) > md_mcp_f(2)) 
                          {
                             Motor(-10, 40);        
                          }
                        else if(mcp_f(1) > md_mcp_f(1) && mcp_f(2) < md_mcp_f(2))
                          {
                            Motor(40, -10);           
                          }
                        else if(mcp_f(0) < md_mcp_f(0) && mcp_f(3) < md_mcp_f(3)
                              || mcp_f(1) < md_mcp_f(1) && mcp_f(2) < md_mcp_f(2))
                          {   
                            Motor(-15, -15);delay(20);
                            Motor(1, -1);delay(10);
                            Motor(0, 0); delay(10);
                            break;            
                          }
                         else
                          {
                            Motor(10, 10);
                          }
                        }                  
                      break;                  
                      
                  }                  
          }
  
        lines = true;         
                           
      } 
    else
      {
        Motor(-30, -30); delay(20);
        Motor(-1, -1);
        delay(50);
        lines = false;
      }   
  }

 /////-------------------------------------------------------->>>>>>>>>>>>>>>>

 void bw(int spl, int spr, int en_pos, String _line)
  {
    
    lines_fw = false; 
    encoderPos = 0;
    while(encoderPos > -en_pos)
      {
        
        if(mcp_f(7) < md_mcp_f(7) && mcp_f(4) > md_mcp_f(4))
              {
                Motor(20, -spr);
              }
        else if(mcp_f(7) > md_mcp_f(7) && mcp_f(4) < md_mcp_f(4))
              {
                Motor(-spl, 20);
              }
        
        else if( mcp_f(6) < md_mcp_f(6))
              {
                Motor(40, 40); delay(30);
                Motor(1, 1);
                delay(100);           
                break;
              }
         
         else
            {
              Motor(-spl, -spr);
            }
      }
     Motor(1, 1);
     delay(50); 
     if(_line == "line")
      {  
        while(1)      
           {    
                      
              if(mcp_f(4) < md_mcp_f(4)-100 && mcp_f(7) > md_mcp_f(7))
                  {
                    Motor(10, -40);
                  }
              else if(mcp_f(4) > md_mcp_f(4) && mcp_f(7) < md_mcp_f(7)-100)
                  {
                    Motor(-40, 10);
                  }
              else if( mcp_f(6) < md_mcp_f(6))
                  {
                    Motor(40, 40); delay(30);  
                    Motor(1, 1);delay(100);
                    Motor(0, 0);delay(10);                     
                    break;                  
                  }  
              else
                 {
                   Motor(-motor_slow, -motor_slow);              
                 }
          }
        lines = true;
        encoderPos = 0;
        do{Motor(30, 30);}while(encoderPos < 350);
        delay(10);
        Motor(1, 1);
      }  
    else
      {
        Motor(30, 30); delay(30);
        Motor(-1, -1);
        delay(50);
        Motor(0, 0);
        lines = false;
      }
    
  }
