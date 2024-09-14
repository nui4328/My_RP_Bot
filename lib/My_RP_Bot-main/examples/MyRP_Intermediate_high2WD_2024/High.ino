void move_fw(int spl, int spr, int en_pos)
  {
    if(start_begin == 0)
      {
        goto ut_R;
      }
    if(LR_free == true)
      {
        encoderPos = 0;
        while(encoderPos < Forward_not_finding_line)
          {            
            Motor(spl, spl);
            check_collor();
            if(color_ch == "yello")
              {
                mision_color_yello();
                if(ch_point == true )
                  {
                    ch_point = false;   
                    LR_free = false;     
                    goto ut_R;
                  }
                else
                  {
                    LR_free = false;
                    goto from_color;
                  }
                
              }
            
            if(mcp_f(0) < md_mcp_f(0) && mcp_f(7) > md_mcp_f(7)) 
                  {
                     Motor(-10, 35);        
                  }
             else if(mcp_f(0) > md_mcp_f(0) && mcp_f(7) < md_mcp_f(7))
                  {
                    Motor(35, -10);           
                  }
             else if(mcp_f(0) < md_mcp_f(0) && mcp_f(7) < md_mcp_f(7))
                  {   
                    Motor(-30, -30);delay(30);  Motor(1, 1); delay(400); Motor(0, 0);delay(100);
                    
                    check_collor();  //---------->> ฟังก์ชันอ่านค่าสีพื้น
                    if(color_ch != "none")
                      {
                        bz(100);
                        mision_color();
                        LR_free = false;
                        goto from_color;
                      }
                    delay(200);
                    break;
                  }
          }
        
        LR_free = false;
        
        goto ut_R;
      }
    //----------------------------------------------------------->> 
    encoderPos = 0;
    while(encoderPos < en_pos)       //------------------------>>  เดินหน้าตามค่า  en_pos
      {        
        Motor(spl, spr);
        check_collor();   //---------->> ฟังก์ชันอ่านค่าสีพื้น
        if(color_ch == "yello")
          {
            mision_color_yello();
            if(ch_point == true )
              {
                ch_point = false;        
                goto ut_R;
              }
            else
              {
                goto from_color;
              }
          }
                          //---------->>
                          
        if(mcp_f(0) < md_mcp_f(0) || mcp_f(7) < md_mcp_f(7))  //---------->> ตรวจสอบเส้นข้างหน้า
          {
            Motor(-30, -30);delay(15);
            while(1)
              {
                if(mcp_f(0) < md_mcp_f(0) && mcp_f(7) > md_mcp_f(7)) 
                  {
                     Motor(-10, 35);        
                  }
                else if(mcp_f(0) > md_mcp_f(0) && mcp_f(7) < md_mcp_f(7))
                  {
                    Motor(35, -10);           
                  }
                else if(mcp_f(0) < md_mcp_f(0) && mcp_f(7) < md_mcp_f(7))
                  {   
                    Motor(-30, -30);delay(30);  Motor(-1, -1); delay(200); Motor(0, 0);delay(100);
                    
                    check_collor();  //---------->> ฟังก์ชันอ่านค่าสีพื้น
                    if(color_ch != "none")
                      {
                        mision_color();
                        goto from_color;
                      }
                      
                    encoderPos = 0;
                    do{Motor(-20, -20);}while(encoderPos > -450);
                    Motor(20, 20); delay(20); Motor(-1, -1); delay(50); Motor(0, 0); delay(100);

                   
                    
                    encoderPos = 0;
                    do{Motor(-speed_rotate, speed_rotate);}while(encoderPos > -(rotate_left));   //------------------>> หมุ่นซ้าย 
                    Motor(30, -30); delay(40);  Motor(1, 1); delay(100); 
                    
                    from_color: //------------------>> มาจากการวางกล่อง
                       
                    encoderPos = 0;
                    while(encoderPos < 850)
                      {
                        get_encode_toline = encoderPos;
                        if(mcp_f(0) < md_mcp_f(0) || mcp_f(7) < md_mcp_f(7))
                          {
                            while(1)
                              {
                                if(mcp_f(0) < md_mcp_f(0) && mcp_f(7) > md_mcp_f(7)) 
                                  {
                                     Motor(-10, 35);        
                                  }
                                else if(mcp_f(0) > md_mcp_f(0) && mcp_f(7) < md_mcp_f(7))
                                  {
                                    Motor(35, -10);           
                                  }
                                else if(mcp_f(0) < md_mcp_f(0) && mcp_f(7) < md_mcp_f(7))
                                  {   
                                    Motor(-30, -30);delay(20); Motor(1, 1);delay(10);
                                    
                                    encoderPos = 0;
                                    do{Motor(-20, -20);}while(encoderPos > -450);
                                    Motor(20, 20); delay(20); Motor(-1, -1); delay(50); Motor(0, 0); delay(100);
                
                                    encoderPos = 0;
                                    do{Motor(-speed_rotate, speed_rotate);}while(encoderPos > -(rotate_left));
                                    Motor(30, -30); delay(30); Motor(1, 1); delay(100);  
                                    goto ent_r;            
                                  }
                              }
                          }
                        else
                          {
                            Motor(20, 20);
                          }
                      }
                    LR_free = true;
                    goto ent_r;
                  }
                else
                  {
                    Motor(20, 20);
                  }
              }
          }
      }

    ut_R: delay(10);
    Motor(-20, -20); delay(20);
    Motor(-1, -1);  delay(100);  

    encoderPos = 0;
    do{Motor(speed_rotate, -speed_rotate);}while(encoderPos < rotate_right);  ///--------------->>  เมื่อเดินถึงกลางบล๊อก ให้หมุนขวา
    Motor(-20, 20); delay(20); Motor(1, 1);  delay(100);  

    encoderPos = 0;
    while(encoderPos < 850)   ///---------------------------------->>   เดินเข้าไปเช็คเส้น 
      {
        get_encode_toline = encoderPos;
        if(mcp_f(1) < md_mcp_f(1)-50)
          {
            Motor(-30, -30); delay(40);
            Motor(1, 1);   delay(100);  
            Motor(0, 0);   delay(100); 
            Motor(-30, -30); delay(100);
          }
        if(mcp_f(0) < md_mcp_f(0)+10 || mcp_f(7) < md_mcp_f(7)+10)   ///--------------->>  เมื่อมีเส้น
          {
            while(1)
              {
                if(mcp_f(0) < md_mcp_f(0) && mcp_f(7) > md_mcp_f(7)) 
                  {
                     Motor(-10, 35);        
                  }
                else if(mcp_f(0) > md_mcp_f(0) && mcp_f(7) < md_mcp_f(7))
                  {
                    Motor(35, -10);           
                  }
                else if(mcp_f(0) < md_mcp_f(0)+10 && mcp_f(7) < md_mcp_f(7)+10)
                  {   
                    Motor(-20, -20);delay(20);
                    Motor(1, 1);delay(300);
                    Motor(0, 0);delay(100);
                    check_collor();  //---------->> ฟังก์ชันอ่านค่าสีพื้น
                    if(color_ch != "none")
                      {
                        mision_color();
                        goto from_color;
                      }
                    encoderPos = 0;
                    do{Motor(-30, -30);}while(encoderPos > -Back_from_line);  ///--------------->>  เมื่อมีเส้นให้ถอยหลังออกมา
                    Motor(20, 20); delay(20);
                    Motor(-1, -1); delay(100);
                    Motor(0, 0);   delay(100);

                    encoderPos = 0;
                    do{Motor(-speed_rotate, speed_rotate);}while(encoderPos > -(rotate_left)); ///--------------->>  หมุนซ้ายเพื่อเดินหน้าต่อ
                    Motor(30, -30); delay(30);
                    Motor(1, 1);   delay(100);  
                    Motor(0, 0);   delay(100); 
                    goto ent_r;            
                  }
              }
          }
        else
          {
            Motor(25, 25);
          }
      }
     LR_free = true;
     ent_r: delay(10);
     Motor(-1, -1);delay(3);
     Motor(0, 0);delay(10);
     start_begin = 1;
  }
