void set_f(int _time)
  {
    ch_set_fb = true ;
    if(mcp_f(1) > md_mcp_f(1) && mcp_f(2) > md_mcp_f(2))
      {
        Motor(-35, -35); delay(120);
        Motor(-2, -2); delay(50);
      }
    else
      {
        Motor(-35, -35); delay(50);
        Motor(-2, -2); delay(50);
      }
    
    last_time = millis();
    while(millis()-last_time < 35)
      {
        Motor(-35, -35);
        if(mcp_f(0) > md_mcp_f(0) && mcp_f(3) > md_mcp_f(3)
        && mcp_f(1) > md_mcp_f(1) && mcp_f(2) > md_mcp_f(2))
          {
            break;
          }
      }
    Motor(35, 35); delay(20);
    Motor(-2, -2); delay(50);
    for(int i = 0; i<_time; i++)
      {
        while(1)
          {
            Motor(20, 20);
            if(mcp_f(1) < md_mcp_f(1) && mcp_f(2) > md_mcp_f(2)) 
              {
                while(1)
                  {
                    Motor(-2, 35);
                    if(mcp_f(2) < md_mcp_f(2) || mcp_f(3) < md_mcp_f(3))
                      {
                        Motor(2, -35);delay(20);
                        Motor(0, 0); delay(10);
                        goto en_wh;
                      }
                  }          
              }
            else if(mcp_f(1) > md_mcp_f(1) && mcp_f(2) < md_mcp_f(2))
              {
                while(1)
                  {
                    Motor(35, -2);
                    if(mcp_f(0) < md_mcp_f(0) || mcp_f(1) < md_mcp_f(1))
                      {
                         Motor(-35, 3);delay(20);
                         Motor(0, 0); delay(10);
                         goto en_wh;
                      }
                  }           
              }
            else if(mcp_f(0) < md_mcp_f(0) && mcp_f(3) < md_mcp_f(3)
                  || mcp_f(1) < md_mcp_f(1) && mcp_f(2) < md_mcp_f(2))
              {   
                Motor(-35, -35);delay(35);
                Motor(0, 0); delay(10);
                break;            
              }
          }
        en_wh:
        Motor(-35, -35);delay(50);
        last_time = millis();
        while(millis()-last_time < 35)
          {
            Motor(-35, -35);
            if(mcp_f(0) > md_mcp_f(0)+50 && mcp_f(1) > md_mcp_f(1)+50
            && mcp_f(2) > md_mcp_f(2)+50 && mcp_f(3) > md_mcp_f(3)+50)
              {
                break;
              }
          }
        Motor(35, 35); delay(20);
        Motor(0, 0); delay(2);
        
      }

      Motor(0, 0); delay(400);
  }

void set_b(int _time)
  {
    while(1)
      {
         Motor(-25, -25);
         if(mcp_f(5) < md_mcp_f(5)||mcp_f(6) < md_mcp_f(6)
           ||mcp_f(4) < md_mcp_f(4)||mcp_f(7) < md_mcp_f(7))
           {
             Motor(30, 30); delay(60);
             Motor(0, 0); delay(200);
             break;
           }

      }   
    Motor(30, 30); delay(60);
    Motor(0, 0); delay(2);   
    last_time = millis();
    while(millis()-last_time < 50)
      {
        Motor(30, 30);
        if(mcp_f(4) > md_mcp_f(4) && mcp_f(5) > md_mcp_f(5)
        && mcp_f(7) > md_mcp_f(7) && mcp_f(6) > md_mcp_f(6))
          {
            break;
          }
      }
    Motor(-30, -30); delay(40);
    Motor(0, 0); delay(100);
   
    for(int i = 0; i< _time; i++)
      {
        while(1)
          {
            if(mcp_f(5) < md_mcp_f(5) && mcp_f(6) > md_mcp_f(6)) 
              {
                while(1)
                  {
                    Motor(2, -30);
                    if(mcp_f(7) < md_mcp_f(7) || mcp_f(6) < md_mcp_f(6))
                      {
                        break;
                      }
                  }  
                Motor(-2, 30);delay(20);
                Motor(0, 0); delay(10);

                break; 
                
              }
            else if(mcp_f(5) > md_mcp_f(5) && mcp_f(6) < md_mcp_f(6))
              {
                while(1)
                  {
                    Motor(-30, 2);
                    if(mcp_f(4) < md_mcp_f(4) || mcp_f(5) < md_mcp_f(5))
                      {
                        break;
                      }
                  }      
                Motor(30, -2);delay(20);
                Motor(0, 0); delay(10);
                break; 
              }
             else if(mcp_f(5) < md_mcp_f(5) && mcp_f(6) < md_mcp_f(6)
                     || mcp_f(4) < md_mcp_f(4) && mcp_f(7) < md_mcp_f(7))
              {   
                Motor(-20, -20);delay(35);
                Motor(0, 0); delay(10);
                break;            
              }

            else
              {
                Motor(-20, -20);
              }
          }
        Motor(30, 30);delay(20);
        while(1)
          {
            Motor(30, 30);
            if(mcp_f(4) > md_mcp_f(4) && mcp_f(7) > md_mcp_f(7)
            && mcp_f(6) > md_mcp_f(6) && mcp_f(5) > md_mcp_f(5))
              {
                break;
              }
          }
        Motor(-30, -30); delay(50);
        //Motor(0, 0); delay(10);       
      }
     
     Motor(0, 0); delay(200); 
 
  }
