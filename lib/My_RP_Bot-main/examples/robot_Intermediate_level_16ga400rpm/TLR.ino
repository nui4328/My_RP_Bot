
void tl(int pw, int offset)
  {
    if(line == true)
      {
        if(ch_set_fb == true)  // เลือกใช้คำสั่ง set_f(1); set_b(1);
          {
            if(ch_bw == true)  // เลือกใช้คำสั่ง bw( 55, 56, 300, "line");
              {
                Motor(25, 25); delay(line_set_fb);
                Motor(-20, -20); delay(20);
              }
            else              // เลือกใช้คำสั่ง fw( 55, 56, 300, "line");
              {
                Motor(-25, -25); delay(line_set_fb);
                Motor(20, 20); delay(50);
              }
            
          }
        else
          {
            if(ch_bw == true)
              {
                Motor(30, 30); delay(line_none_set_fb);
                Motor(-30, -30); delay(50);
              }
            else
              {
                Motor(-30, -30); delay(line_none_set_fb);
                Motor(30, 30); delay(50);
              }
          }
        ch_set_fb = false;
      }
    else{}
    
    Motor(0, 0);
    delay(100);
    last_time = millis();
    while(millis()-last_time < offset)
      {
        Motor(-pw, pw);
        if(mcp_f(7) < md_mcp_f(7) || mcp_f(4) > md_mcp_f(4))
          {
            ch_lr = true;
          }
      }
    Motor(pw, -pw);
    delay(40);
    if(ch_lr == true)
      {
        Motor(30, 30);
        delay(80);
      }
    
    Motor(0, 0);
    delay(100);
    ch_lr = false;
    ch_bw = false;
  }


void tr(int pw, int offset)
  {
    if(line == true)
      {
        if(ch_set_fb == true)  // เมื่อมีการใช้งานฟังก์ชัน set_f และ set_b
          {
            if(ch_bw == true)
              {
                Motor(25, 25); delay(line_set_fb);
                Motor(-20, -20); delay(20);
              }
            else
              {
                Motor(-25, -25); delay(line_set_fb);
                Motor(20, 20); delay(50);
              }
            
          }
        else
          {
            if(ch_bw == true)
              {
                Motor(30, 30); delay(line_none_set_fb);
                Motor(-30, -30); delay(50);
              }
            else
              {
                Motor(-30, -30); delay(line_none_set_fb);
                Motor(30, 30); delay(50);
              }
          }
        ch_set_fb = false; //  
      }
    else{}
    Motor(0, 0);
    delay(300);
    last_time = millis();
    while(millis()-last_time < offset)
      {
        Motor(pw, -pw);
        if(mcp_f(7) < md_mcp_f(7) || mcp_f(4) > md_mcp_f(4))
          {
            ch_lr = true;
          }
      }
    Motor(-pw, pw);
    delay(40);
    if(ch_lr == true)
      {
        Motor(30, 30);
        delay(80);
      }
    
    Motor(0, 0);
    delay(100);
    ch_lr = false;
    ch_bw = false;
  }
