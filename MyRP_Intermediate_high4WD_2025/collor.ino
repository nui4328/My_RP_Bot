
void mision_color()
  { 
    Motor(20, 20);delay(150);
    Motor(-1, -1);delay(100);
    check_colors();
    if(check_color == "red")
      {
        bz(50);
        do{Motor(-15, -15);}while(mcp_f(0) < md_mcp_f(0)+50&& mcp_f(1) < md_mcp_f(1)+ 50);
        move_bw(15,4);
        Motor(1, 1);delay(200);
        servo_red();         
      }
    else if(check_color == "green")
      {
        bz(50);bz(50);        servo(20, servo27+50);
        do{Motor(-15, -15);}while(mcp_f(0) < md_mcp_f(0)+50&& mcp_f(1) < md_mcp_f(1)+ 50);
        move_bw(15,4);
        Motor(1, 1);delay(200);
        servo_green();       
      }
    else if(check_color == "blue")
      {
        bz(50);bz(50);bz(200);
        do{Motor(-15, -15);}while(mcp_f(0) < md_mcp_f(0)+50&& mcp_f(1) < md_mcp_f(1)+ 50);
        move_bw(15,4);
        Motor(1, 1);delay(200);
        servo_blue();   
              
      }
    //do{Motor(-20, -20);}while(mcp_f(0) < md_mcp_f(0)&& mcp_f(1) < md_mcp_f(1));
    while(1)
              {
                if(mcp_f(0) < md_mcp_f(0) && mcp_f(1) > md_mcp_f(1)) 
                  {
                     Motor(-10, 35);        
                  }
                else if(mcp_f(0) > md_mcp_f(0) && mcp_f(1) < md_mcp_f(1))
                  {
                    Motor(35, -10);           
                  }
                else if(mcp_f(0) < md_mcp_f(0) && mcp_f(1) < md_mcp_f(1))
                  {   
                    Motor(-20, -20);delay(10);
                    Motor(1, 1);delay(300);
                    break;
                  }
                else
                  {
                    Motor(15, 15);
                  }
              }
    move_bw(30, Go_back_before_color);
    Motor(1, 1);delay(100);
    move_tl(50, rotate_color);//------------------>> หมุ่นซ้าย
    
     
  }

void mision_color_yello()
  {
    move_fw(20,7); 
    num_encoder = encoder.Poss_L();    
    check_color= "none";
    check_colors();
    display_color();
    if(check_color == "yellow")
      {
        while(1)
            {
               if(mcp_f(0) < md_mcp_f(0) && mcp_f(1) > md_mcp_f(1)) 
                 {
                    _ch_lr = 0;
                    Motor(-5, 25);        
                 }
               else if(mcp_f(0) > md_mcp_f(0) && mcp_f(1) < md_mcp_f(1))
                 {
                    _ch_lr = 1;
                    Motor(25, -5);           
                 }
               else if(mcp_f(0) < md_mcp_f(0)-100 && mcp_f(1) < md_mcp_f(1)-100)
                 {  
                    if(_ch_lr == 1)
                      {
                         Motor(-25, 5);delay(20);
                      }
                    else
                      {
                         Motor(5, -25);delay(20);
                      }
                    Motor(-20, -20);delay(50);
                    Motor(0, 0);delay(50);
                    break;
                  }
                else
                  {
                     Motor(10, 10);
                  }
            }
         Motor(-20, -20); delay(200);
         do{check_colors();Motor(-15, -15);}while(check_color == "yellow");         
         Motor(30, 30); delay(40);
         Motor(-1, -1); delay(10);
         bz(100); bz(100); 
         servo_yello();delay(100);
      }
     else
      {
         ch_poit ++;
         bz(100);
         move_bw(20, 2);
         ch_point = true; 
         LR_free = false;
         if(red_box == 1 && green_box == 1 && blue_box == 1 && yello_box == 1 && ch_poit >= 2)
          {
            servo_pico26.write(120);
            bz(300);bz(300);bz(300);
            
            begin_robot();     // รอกดปุ่ม  
          }
         
         goto non_yello;
      }     
   
    num_encoder = 0;
    move_bw(30, Go_back_before_yelow);
    move_tl(50, rotate_yelow);//------------------>> หมุ่นซ้าย  
    non_yello: delay(10);  
  }

void display_color_loop()
  {
    while(1)
      {
        check_colors();
        String color = String(check_color);
        mydisplay_background(black);
        mydisplay(color, 20, 30, 2, white);

        if(check_color == "red")
          {
            bz(50);
            servo(27, servo27-40);
            delay(400);
            servo(27, servo27);         
          }
        else if(check_color == "green")
          {
            bz(50);bz(50);
            servo(27, servo27+50);
            delay(400);
            servo(27, servo27);        
          }
        else if(check_color == "blue")
          {
            bz(50);bz(50);bz(200);
            servo_blue();         
          }
      }
      
  }
  
void display_color()
  {
    check_colors();
    String color = String(check_color);
    mydisplay_background(black);
    mydisplay(color, 20, 30, 2, white);
  }
