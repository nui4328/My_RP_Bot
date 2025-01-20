void ch_line_to_left()
  {
    unsigned long lastTime = millis();
            while(1)
              {
                Motor(15, 15);
                if(mcp_f(0) < md_mcp_f(0)-100 && mcp_f(1) > md_mcp_f(1)-100) 
                  {
                    lastTime = millis();
                     ch_lr = 0;
                     do{Motor(-5, 25);}while(mcp_f(1) > md_mcp_f(1)-100);        
                  }
                else if(mcp_f(0) > md_mcp_f(0)-100 && mcp_f(1) < md_mcp_f(1)-100)
                  {
                    lastTime = millis();
                    ch_lr = 1;
                    do{Motor(25, -5);}while(mcp_f(0) > md_mcp_f(0)-100); 
                               
                  }
                else if(mcp_f(0) < md_mcp_f(0)-100 && mcp_f(1) < md_mcp_f(1)-100)
                  {  
                    lastTime = millis(); 
                    if(ch_lr == 1)
                      {
                        Motor(-15, 5);delay(20);
                      }
                    else
                      {
                        Motor(5, -15);delay(20);
                      }
                    Motor(-2, -2);delay(30);
                    check_colors();
                    if(check_color != "none") 
                      { 
                        display_color();
                        mision_color();
                        goto end_line;
                      }
                    Motor(-20, -20);delay(200);
                  }

                if (millis() - lastTime >= 400) 
                  {
                    bz(100);
                    Motor(-10, -10);delay(100);
                    Motor(0, 0);delay(400);  
                    Motor(5, 5);delay(200); 
                    move_fw(20,Go_back_before_sticks);
                    move_tr(50, set_rotate_Right);  //-------------->>  หมุนซ้าย เมื่อเดินได้ระยะทาง 30 cm  แล้ว
                    goto end_line;
                  }
                  //------------------------------------------------------\\

                if(mcp_f(0) < md_mcp_f(0)+100 && mcp_f(1) > md_mcp_f(1)+100) 
                  {
                     ch_lr = 0;
                     Motor(-5, 15);        
                  }
                else if(mcp_f(0) > md_mcp_f(0)+100 && mcp_f(1) < md_mcp_f(1)+100)
                  {
                    ch_lr = 1;
                    Motor(15, -5);           
                  }
                else if(mcp_f(0) < md_mcp_f(0)+100 && mcp_f(1) < md_mcp_f(1)+100)
                  {   
                    if(ch_lr == 1)
                      {
                        Motor(-15, 5);delay(10);
                      }
                    else
                      {
                        Motor(5, -15);delay(10);
                      }
                    Motor(1,1);delay(200);
                    check_colors();
                    if(check_color != "none") 
                      { 
                        display_color();
                        mision_color();
                        goto end_line;
                      }
                    move_bw(20, Go_back_before_turning_left);
                    move_tl(60, set_rotate_Left);
                    break;
                  }
                else
                  {
                    Motor(15, 15); 
                  }                
              }
            end_line:
            Motor(0, 0);delay(20);
  }
