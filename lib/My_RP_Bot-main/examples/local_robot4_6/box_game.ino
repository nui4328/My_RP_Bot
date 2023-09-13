void box_1()
  {
     Motor(30,30);delay(200);
     down_servo();     
     fline(40,40, 0.3, 200, 'n', 's',40, "a4", 1); //--->>วิ่งเป็นเวลา 200 แล้วหยุด
     fw_a3_a4(13, 13, 0.3, 20); //--->เดินชนเส้น     
     if(analogRead(28) < 1800)  //------------------->> ตรวจสอบว่ามีกระป๋องหรือไม่ (ไม่เจอกระป๋อง)
        {
          no_can = 1;
        }
     servo_mission();  //------------------------------->> หยิบกระป๋อง
     fw_ch_07(15, 15, 10);   
     if(mcp_f(0) > md_mcp_f(0)&&mcp_f(7) < md_mcp_f(7))
        {  
            if(no_can == 1)  //------------------->> ตรวจสอบว่ามีกระป๋องหรือไม่ (ไม่เจอกระป๋อง)
              {
                no_can_L();
              }
           if(color_box == color_floor_L) // ----------------->> เช็คสีกระป๋อง กับ สีพื้น
              {
                stL_can1_to_L(); 
              }
           else 
              {
                stL_can1_to_MD();
              }
           start_robot = 'l';
        }
      else
        {
           bz(100);
           if(no_can == 1)  //------------------->> ตรวจสอบว่ามีกระป๋องหรือไม่ (ไม่เจอกระป๋อง)
              {
                no_can_R();
              }
           if(color_box == color_floor_R) // ----------------->> เช็คสีกระป๋อง กับ สีพื้น
              {
                stR_can1_to_R(); 
              }
           else 
              {
                stR_can1_to_MD();
              }
        }
  }

void box_2()
  {
     down_servo();
     fline(40,40, 0.3, 250, 'n', 's',40, "a4", 1); //--->>วิ่งเป็นเวลา 200 แล้วหยุด
     
     fw_a3_a4(13, 13, 0.3, 20); //--->เดินชนเส้น
     servo_mission();  //------------------------------->> หยิบกระป๋อง
     
     if(start_robot == 'l')
        {
          if(color_box == color_floor_L) // ----------------->> เช็คสีกระป๋อง กับ สีพื้น
            {
              stL_can2_to_L(); 
            }
         else if(color_box == color_floor_R)
            {
              stL_can2_to_R(); 
            }
         else 
            {
              stL_can2_to_MD();
            }
        }
      else
        {
           if(color_box == color_floor_L) // ----------------->> เช็คสีกระป๋อง กับ สีพื้น
              {
                stR_can2_to_L(); 
              }
           else if(color_box == color_floor_R)
              {
                stR_can2_to_R(); 
              }
           else 
              {
                stR_can2_to_MD();
              }
        }
     

  }
void box_3()
  {
    down_servo();
    fline(30,30, 0.3, 0, 'c', 'r',40, "a4", 20); 
    
    fw_to_box(40, 40, 0.3, 100 , 10);  //-------------->>  วิ่งเข้าไปคีบกระป๋อง วิ่งไม่ถึงให้เพิ่ม 200 ขึ้น ถ้าชนให้ลดลง
    
    servo_mission();
    bline(40,40, 0.3, 0, 'c', 'r',40, "a4", 20);
    check_box_to_floor();  
  }
void box_4()
  {
    down_servo();
    fline(30,30, 0.3, 0, 'c', 'l',40, "a3", 20); 
    fw_to_box(40, 40, 0.3, 100 , 10);
    servo_mission();
    bline(40,40, 0.3, 0, 'c', 'l',40, "a3", 20);
    check_box_to_floor();  
  }
void box_5()
  {
    down_servo();  
    fline(40,40,0.3,0,'c','p',45, "a5", 25);   
    fline(40,40,0.5,300,'n','s',45, "a5", 1);   
    fw_a3_a4(40, 40, 0.3, 0); 
    fw_stop_center();
    tx_right(35, 25);
    fw_to_box(40, 40, 0.3, 100 , 10);
    servo_mission();
    bline(30,30, 0.3, 400, 'n', 's',50, "a2", 1); 
    bw_a3_a4(50, 50, 0.3, 0);
    bw_stop_ba7();
    Motor(0,0);delay(100); 
    tx_right(35, 25);   
    fline(40,40, 0.3, 0, 'c', 's',40, "a3", 5);   
    
    check_box_to_floor();  
  }
void box_6()
  {
    down_servo(); 
    fline(40,40,0.3,0,'c','p',45, "a5", 25);
    fline(40,40,0.5,300,'n','s',45, "a5", 1);    
    fw_a3_a4(40, 40, 0.3, 0); 
    fw_stop_center();
    tx_left(35, 25);
    fw_to_box(40, 40, 0.3, 100 , 10);
    servo_mission();
    bline(30,30, 0.3, 400, 'n', 's',50, "a2", 1); 
    bw_a3_a4(50, 50, 0.3, 0);
    bw_stop_ba0();
    Motor(0,0);delay(100); 
    tx_left(35, 25);   
    fline(40,40, 0.3, 0, 'c', 's',40, "a3", 5);   
    
    check_box_to_floor();  
  }

void box_7()
  {
     down_servo();
     fline(40,40,0.3,0,'c','p',45, "a5", 25);  
     fline(40,40,0.5,0,'f','s',45, "a5", 1);        
     fline_2sensor(30,30,0.2,0,"3:4","27",'p',0);
     fline_2sensor(30,30,0.2,0,"3:4","a0",'p',20);
     do{Motor(40,-10);}while(mcp_f(4) > md_mcp_f(4));
     Motor(-40, 10); delay(20);
     Motor(0, 0); delay(100);

     fline_2sensor(20,20,0.7,0,"3:4","a7",'p',0);
     fline(20,25, 0.2, 100, 'n', 'r',40, "a4", 30);  
     fline(20,20, 0.3, 0, 'f', 's',40, "a4", 2);
     fw_a3_a4(20, 20, 0.3, 3);
     
     
     servo_mission();
     
     bline(30,30, 0.3, 0, 'c', 'r',40, "a4", 20); 
     fline_2sensor(30,30,0.7,0,"3:4","a0",'p',20);
     fline(20,20, 0.1, 50, 'n', 'l',40, "a3", 20); 
     fw_a3_a4(40, 40, 0.3, 0); 
     fw_stop_center();     
     fline(50,50,0.45,0,'c','s',45, "a5", 1);
     
     check_box_to_floor();  
  }
void box_8()
  {
     down_servo();
     fline(40,40,0.3,0,'c','p',45, "a5", 25);  
     fline(40,40,0.5,0,'f','s',45, "a5", 1);       
     fline_2sensor(30,30,0.2,0,"3:4","27",'p',0);
     fline_2sensor(30,30,0.7,0,"3:4","a0",'p',20);
     do{Motor(-10,40);}while(mcp_f(3) > md_mcp_f(3));
     Motor(10,-40); delay(20);
     Motor(0, 0); delay(100);

     fline_2sensor(20,20,0.7,0,"3:4","a0",'p',20);
     fline(25,20, 0.1, 100, 'n', 'l',40, "a3", 30);  
     fw_to_box(30, 30, 0.3, 100 , 10);
     
     servo_mission();
     
     bline(30,30, 0.3, 0, 'c', 'l',40, "a3", 20); 
     fline_2sensor(30,30,0.7,0,"3:4","a7",'p',20);
     fline(20,20, 0.1, 50, 'n', 'r',40, "a4", 20);  
     fw_a3_a4(40, 40, 0.3, 0); 
     fw_stop_center();     
     fline(50,50,0.45,0,'c','s',45, "a5", 1);
     
     check_box_to_floor();      
  }
void box_9()
  {
     down_servo();
     fline(40,40,0.3,0,'c','p',45, "a5", 25);  
     fline(40,40,0.5,0,'f','s',45, "a5", 1);        
     fline_2sensor(30,30,0.2,0,"3:4","27",'p',0);

     fline_2sensor(20, 20, 0.7, 0, "3:4", "a0", 'p', 0);delay(30);
     do {Motor(40, -10);} while (mcp_f(3) > md_mcp_f(3));
     Motor(-40, 10); delay(20);
  
     fline_2sensor(25, 25, 0.7, 0, "3:4", "a7", 'p', 0);
     do {Motor(-30, 40);} while (mcp_f(4) > md_mcp_f(4));
     
     fline_2sensor(25,25,0.7,0,"3:4","a7",'p',0);
     fline(20,20, 0.1, 100, 'n', 'r',40, "a4", 20); 

     fline_2sensor(25,25,0.7,0,"3:4","a7",'p',20);
     fline(20,25, 0.1, 100, 'n', 'r',40, "a4", 30);  
     fw_to_box(30, 30, 0.3, 100 , 10);
     
     servo_mission();
     
     bline(30,30, 0.3, 0, 'c', 'r',40, "a4", 20); 
     fline_2sensor(25,25,0.7,0,"3:4","a0",'p',0);
     fline(20,20, 0.1, 70, 'n', 'l',40, "a3", 20);
          
     fline_2sensor(25,25,0.8,0,"3:4","a0",'p',0);delay(10);
     do{Motor(30,-15);}while(mcp_f(4) > md_mcp_f(4));
     fline_2sensor(30,30,0.8,0,"3:4","a0",'p',0);
     fline(20,20, 0.1, 50, 'n', 'l',40, "a3", 20);  
     fw_a3_a4(40, 40, 0.3, 0); 
     fw_stop_center();     
     fline(50,50,0.45,0,'c','s',45, "a5", 1);
     
     check_box_to_floor(); 
  }



void box_10()
  {
     down_servo();
     fline(40,40,0.3,0,'c','p',45, "a5", 25);  
     fline(40,40,0.5,0,'f','s',45, "a5", 1);      
     fline_2sensor(30,30,0.2,0,"3:4","27",'p',0);
     
     fline_2sensor(30,30,0.7,0,"3:4","a0",'p',0);delay(30);
     do{Motor(-10,40);}while(mcp_f(3) > md_mcp_f(3));
     Motor(10,-40); delay(20);
     Motor(0, 0); delay(100);

     fline_2sensor(20,20,0.7,0,"3:4","a0",'p',0);
     do{Motor(40,-20);}while(mcp_f(4) > md_mcp_f(4));
     fline_2sensor(25,25,0.7,0,"3:4","a0",'p',0);
     fline(20,20, 0.1, 100, 'n', 'l',40, "a3", 20); 

     fline_2sensor(25,25,0.7,0,"3:4","a0",'p',0);
     fline(25,20, 0.1, 100, 'n', 'l',40, "a3", 30);  
     fw_to_box(30, 30, 0.3, 100 , 10);
     
     servo_mission();
     
     bline(30,30, 0.3, 0, 'c', 'l',40, "a3", 20); 
     fline_2sensor(25,25,0.7,0,"3:4","a7",'p',0);
     fline(20,20, 0.1, 50, 'n', 'r',40, "a4", 20);
          
     fline_2sensor(25,25,0.7,0,"3:4","a7",'p',0);
     do{Motor(10,30);}while(mcp_f(7) < md_mcp_f(7));
     do{Motor(-15,30);}while(mcp_f(3) > md_mcp_f(3));
     fline_2sensor(25,25,0.7,0,"3:4","a7",'p',0);
     fline(20,20, 0.1, 50, 'n', 'r',40, "a4", 20);  

     fw_a3_a4(40, 40, 0.3, 0); 
     fw_stop_center();     
     fline(50,50,0.45,0,'c','s',45, "a5", 1);
     
     check_box_to_floor(); 
  }

void box_11()
  {
     down_servo();
     fline(50,50,0.3,0,'c','p',45, "a5", 25);  
     fline(40,40,0.5,0,'f','s',45, "a5", 1);        
     fline_2sensor(30,30,0.2,0,"3:4","27",'p',0);

     circle();

     fw_a3_a4(40, 40, 0.3, 0); 
     fw_stop_center();
     tx_right(35, 25);
     fw_to_box(40, 40, 0.3, 100 , 10);
     servo_mission();
     bline(30,30, 0.3, 400, 'n', 's',50, "a2", 1); 
     bw_a3_a4(50, 50, 0.3, 0);
     bw_stop_ba7();
     Motor(0,0);delay(100); 
     tx_right(35, 25); 

     circle();       

     fw_a3_a4(40, 40, 0.3, 0); 
     fw_stop_center();     
     fline(50,50,0.45,0,'c','s',45, "a5", 1);
     
     check_box_to_floor(); 
  }

void box_12()
  {
     down_servo();
     fline(50,50,0.3,0,'c','p',45, "a5", 25);  
     fline(40,40,0.5,0,'f','s',45, "a5", 1);         
     fline_2sensor(30,30,0.2,0,"3:4","27",'p',0);

     circle();

     fw_a3_a4(40, 40, 0.3, 0); 
     fw_stop_center();
     tx_left(35, 25);
     fw_to_box(40, 40, 0.3, 100 , 10);
     servo_mission();
     bline(30,30, 0.3, 400, 'n', 's',50, "a2", 1); 
     bw_a3_a4(50, 50, 0.3, 0);
     bw_stop_ba0();
     Motor(0,0);delay(100); 
     tx_left(35, 25);  

     circle();       

     fw_a3_a4(40, 40, 0.3, 0); 
     fw_stop_center();     
     fline(50,50,0.45,0,'c','s',45, "a5", 1);
     
     check_box_to_floor(); 
  }


void box_13()
  {
     down_servo();
     fline(50,50,0.3,0,'c','p',45, "a5", 25);  
     fline(40,40,0.5,0,'f','s',45, "a5", 1);       
     fline_2sensor(30,30,0.2,0,"3:4","27",'p',0);

     circle();
          
    fw_a3_a4(40, 40, 0.3, 0); 
    fw_stop_center();
    fline(0, 0, 0.4, 10,'n','r',40, "a4", 30);
    fline(30,30,0.5,200,'n','s',45, "a5", 1); 
    fw_to_box(40, 40, 0.3, 100 , 10);
    servo_mission();

    bw_a3_a4(50, 50, 0.3, 0);
    bw_stop_ba7();
    Motor(0,0);delay(100); 
    fline(0,0,0.3,10,'n','r',50, "a5", 2); 
    fline(0,0,0.3,10,'n','r',50, "a5", 2); 
    fline(0,0,0.3,10,'n','r',50, "a4", 30);

     circle();       

     fw_a3_a4(40, 40, 0.3, 0); 
     fw_stop_center();     
     fline(50,50,0.45,0,'c','s',45, "a5", 1);
     
     check_box_to_floor();  
  }
void box_14()
  {
     down_servo();
     fline(50,50,0.3,0,'c','p',45, "a5", 25);  
     fline(40,40,0.5,0,'f','s',45, "a5", 1);        
     fline_2sensor(30,30,0.2,0,"3:4","27",'p',0);

     circle();      
    fw_a3_a4(40, 40, 0.3, 0); 
    fw_stop_center();
    fline(0, 0, 0.4, 10,'n','l',40, "a3", 30);
    
    fline(30,30,0.5,200,'n','s',45, "a5", 1); 
    fw_to_box(40, 40, 0.3, 100 , 10);
    servo_mission();

    bw_a3_a4(50, 50, 0.3, 0);
    bw_stop_ba0();
    Motor(0,0);delay(100); 
    fline(0,0,0.3,10,'n','l',50, "a2", 2); 
    fline(0,0,0.3,10,'n','l',50, "a2", 2); 
    fline(0,0,0.3,10,'n','l',50, "a3", 30);

     circle();       

     fw_a3_a4(40, 40, 0.3, 0); 
     fw_stop_center();     
     fline(50,50,0.45,0,'c','s',45, "a5", 1);
     
     check_box_to_floor();  
  }

void box_15()
  {
     down_servo();
     fline(50,50,0.3,0,'c','p',45, "a5", 25);  
     fline(40,40,0.5,0,'f','s',45, "a5", 1);         
     fline_2sensor(30,30,0.2,0,"3:4","27",'p',0);

     circle();
           
    fw_a3_a4(40, 40, 0.3, 0); 
    fw_stop_center();
    fw_to_box(30, 30, 0.3, 50 , 10);
    servo_mission();
    
    fline(0,0,0.3,50,'n','r',40, "a4", 30); 

    fw_a3_a4(30, 30, 0.3, 0);
    fw_stop_center();

     circle();       

     fw_a3_a4(40, 40, 0.3, 0); 
     fw_stop_center();     
     fline(50,50,0.45,0,'c','s',45, "a5", 1);
     
     check_box_to_floor();  

  }
void end_game()
  {
     Motor(-30, -30); delay(300);
     Motor(0, 0); delay(300);
  }
