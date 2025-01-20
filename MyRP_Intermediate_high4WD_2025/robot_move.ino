void move_fw(int spl, int spr, int targetDistanceCm)
  {
    float targetPulses = targetDistanceCm * pulsesPerCm;
    if(start_begin == 0)
      {
        goto ut_R;
      }
    if(LR_free == true)
      {
        //----------------------------------------------------------------->> เดินหน้าตามบล๊อก  *********
        encoder.resetEncoders();
        move_fw_free(20,4);
        while(1)
          {            
            float leftPulses = encoder.Poss_L();
            float rightPulses = encoder.Poss_R();
            float currentPulses = (leftPulses + rightPulses) / 2;
            //float currentPulses = encoder.Poss_L();
            
            Motor(spl, spl);
            
            if (encoder.Poss_L() >= targetPulses) 
              {
                Motor(-1, -1); // หยุดมอเตอร์
                delay(50);
               
                break;       // ออกจากลูป
              }

            //------------------------------------------>> เช็คสี  
            check_colors();
            if(check_color != "none") 
              {                            
                Motor(-45, -45); delay(50);
                Motor(-1, -1); delay(100);
                bz(50);                              
                if(check_color == "yellow")
                  {
                    display_color();
                    mision_color_yello();
                    if(ch_point == true )
                      {
                        ch_point = false;      
                        goto ut_R;
                      }
                    else
                      {
                        goto After_check_line;
                      }                    
                  }
                 else
                  {
                     display_color();
                     mision_color();
                     goto ent_r;
                  }
              }
            //------------------------------------------>> จบเช็คสี   
               
           if(mcp_f(0) < md_mcp_f(0)+50 || mcp_f(1) < md_mcp_f(1)+50)  //---------->> ตรวจสอบเส้นข้างหน้า
              {
                Motor(-25, -25); delay(50);
                Motor(1, 1); delay(100);
                check_colors();
                if(check_color != "none") 
                  { 
                    display_color();
                    mision_color();
                    goto ent_r;
                  }
    
                while(1)
                  {
                    Motor(-20, -20);
                    if(mcp_f(0) > md_mcp_f(0) && mcp_f(1) > md_mcp_f(1))
                      {
                        delay(50);
                        break;
                      }
                  }
                
                ch_line_to_left();
                goto After_check_line;
              }
          }
                    
     }
        ///------------------------------------------------------------->> จบการเดินหน้าตามบล๊อก *********
        LR_free = false;
     
    ut_R: delay(10);
    Motor(-20, -20); delay(20);
    Motor(-1, -1);  delay(100); 
     
    move_tr(60,set_rotate_Right);  //-------------->>  หมุนขวา เมื่อเดินได้ระยะทาง 30 cm  แล้ว
    
    encoder.resetEncoders();
    while(1)   //---------------------------->>  เดินเข้าหาเส้นเป็นระยะทาง 30 cm
      { 
        // อ่านค่าจาก Encoder
        float leftPulses = encoder.Poss_L();
        float rightPulses = encoder.Poss_R();
        // คำนวณระยะทางที่เคลื่อนที่แล้ว (เฉลี่ยจากล้อซ้ายและขวา)
        float currentPulses = (leftPulses + rightPulses) / 2;
        // ตรวจสอบและอัปเดตค่ามากที่สุด
        
        Motor(spl-10, spr-10);
        if (currentPulses >= targetPulses) 
              {
                Motor(-1, -1); // หยุดมอเตอร์
                delay(100);
                move_tr(50, set_rotate_Right+2);  //-------------->>  หมุนซ้าย เมื่อเดินได้ระยะทาง 30 cm  แล้ว
            
                break;       // ออกจากลูป
              }

            //------------------------------------------>> เช็คสี  
            check_colors();
            if(check_color != "none") 
              {                            
                Motor(-45, -45); delay(50);
                Motor(-1, -1); delay(100);
                bz(50); bz(50);                             
                if(check_color == "yellow")
                  {
                    display_color();
                    mision_color_yello();
                    if(ch_point == true )
                      {
                        ch_point = false;   
                        goto ut_R;
                      }
                    else
                      {
                        goto After_check_line;
                      }                    
                  }
                 else
                  {
                     display_color();
                     mision_color();
                     goto After_check_line;
                  }
              }
            //------------------------------------------>> จบเช็คสี   
               
           if(mcp_f(0) < md_mcp_f(0) || mcp_f(1) < md_mcp_f(1))  //---------->> ตรวจสอบเส้นข้างหน้า
              {
                Motor(-25, -25); delay(50);
                Motor(1, 1); delay(100);
                check_colors();
                if(check_color != "none") 
                  { 
                    display_color();
                    mision_color();
                    goto After_check_line;
                  }
    
                while(1)
                  {
                    Motor(-20, -20);
                    if(mcp_f(0) > md_mcp_f(0) && mcp_f(1) > md_mcp_f(1))
                      {
                        Motor(1, 1);delay(50);
                        break;
                      }
                  }
                delay(50);
                ch_line_to_left();
                
              }
        
      }
    After_check_line: delay(100);

    LR_free = true;
    ent_r: delay(10);
    Motor(-1, -1);delay(100);
    Motor(0, 0);delay(10);
    start_begin = 1;
  }
