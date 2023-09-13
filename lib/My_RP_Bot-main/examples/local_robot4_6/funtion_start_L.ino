void no_can_L()
  {
      bline(30,30,0.3,0,'c','l',50, "a3", 20);
      fw_2sensor(30,30,0.4,0,"2:3","b0",10);
  
      do{Motor(25,25);}while(mcp_b(0) < md_mcp_b(0));delay(20);
      Motor(-30, -30); delay(30);
      Motor(0, 0); delay(100);
      
      do{Motor(-20,60);}while(mcp_f(5) > md_mcp_f(5));delay(40); 
      Motor(20,-60); delay(20); 
      do{Motor(20,-20);}while(mcp_f(2) > md_mcp_f(2));
      fw_2sensor(20,20,0.2,0,"1:2","27",20);
      
      fline(0,0,0.5,2,'n','r',40, "a4", 30);
      color_box= "none";

      test() ; 

      box_15();
      box_14();
      box_13();
      box_12();
      box_11();
      box_10();
      box_9();
      box_8();
      box_7();
      box_6();
      box_5();
      box_4();
      box_3();
      delay(200000);
  }
void stL_can1_to_L()
  {
    
     fline(30,30,0.1,200,'n','s',40, "b3", 30);
     do{Motor(50,-20);}while(mcp_f(7) > md_mcp_f(7));  //-----> หมุนขวา ใช้เซนเซอร์ 7 จากขาว ไปหา ดำ
     do{Motor(50,-20);}while(mcp_f(4) > md_mcp_f(4));  //-----> หมุนขวา ใช้เซนเซอร์ 4 จากขาว ไปหา ดำ
     Motor(-40,20); delay(20);  //----------------------------> เบรคการหมุนตัว 
     Motor(0,0); delay(50);    //----------------------------> มอเตอร์ไม่หมุน หน่วงเวลา 100
          
     slow_servo_down();         //----------------------------> ฟังก์ชัน เอาเซอร์โวลงแบบช้า ๆ     
     fw_a3_a4(20, 20, 0.3, 20);  ////เดินเข้าหากรอบ

     Motor(20,15);delay(200);
     Motor(-20,-15);delay(10);
     Motor(0,0);delay(100);
                    
     servo_store();   
     bline(30,30,0.15,0,'c','r',50, "a5", 15);
  
     fline(50,50,0.5,150,'c','l',40, "b3", 30);
     fw_2sensor(30, 30, 0.3, 0, "4:5","b7", 20);
  
     do{Motor(20,20);}while(mcp_b(7) < md_mcp_b(7));delay(20);
          
     do{Motor(40,-40);}while(mcp_f(3) > md_mcp_f(3));delay(20);          
     Motor(-50,40); delay(20); 
     if(mcp_f(1)> md_mcp_f(1)&&mcp_f(2)> md_mcp_f(2))
       {
         do{Motor(-2,20);}while(mcp_f(2) > md_mcp_f(2));
       }
      
    fw_2sensor(30, 30, 0.3, 0, "2:3","b7", 20);  

    do{Motor(60,-20);}while(mcp_f(0) > md_mcp_f(0));delay(10);
    Motor(-60,20); delay(20); 
    fw_2sensor(20, 20, 0.3, 0, "3:4","26", 20);
    fline(0,0,0.3, 10, 'n', 'l',40, "a3", 20);  
      
  }
void stL_can1_to_MD()
  {
    bline(30,30, 0.3, 0, 'c', 'r',40, "a5", 20);  //--->เดินชนเส้น
    fw_2sensor(30, 30, 0.3, 0, "4:5","b7", 20);

    do{Motor(20,20);}while(mcp_b(7) < md_mcp_b(7));delay(20);
          
    do{Motor(40,-40);}while(mcp_f(3) > md_mcp_f(3));delay(20);          
    Motor(-50,40); delay(20); 
    if(mcp_f(1)> md_mcp_f(1)&&mcp_f(2)> md_mcp_f(2))
      {
         do{Motor(-2,20);}while(mcp_f(2) > md_mcp_f(2));
      }
      
    fw_2sensor(30, 30, 0.3, 0, "2:3","b0", 20);  
    do{Motor(30,30);delay(5);}while(mcp_b(0) < md_mcp_b(0));delay(60);       
    fline(0,0,0.5,10,'n','l',40, "a4", 10);
    slow_servo_down();
    fw_2sensor(20, 20, 0.3, 100, "3:4","27", 20);
           
    if(tmd == 0)
      {
         Motor(20,15);delay(200);
         Motor(-20,-15);delay(10);
         Motor(0,0);delay(100);
                    
         servo_store();                
         tmd = 1;
        }
     else
        {
           Motor(15,20);delay(200);
           Motor(-15,-20);delay(10);
           Motor(0,0);delay(100);
                    
           servo_store();
           tmd = 0; 
        }
      do{Motor(15,15);}while(mcp_b(7) > md_mcp_b(7));delay(10);
      do{Motor(15,15);}while(mcp_b(7) < md_mcp_b(7));delay(100);
      Motor(-20, -20); delay(20);
      Motor(0, 0); delay(100);
            
      bline(20,20,0.3, 0, 'c', 'r',40, "b2", 10);
      do{Motor(25,25);}while(mcp_b(7) > md_mcp_b(7));delay(10);
      do{Motor(25,25);}while(mcp_b(7) < md_mcp_b(7));delay(20);
      Motor(-30, -30); delay(20);
      Motor(0, 0); delay(100);
      do{Motor(60,-20);}while(mcp_f(0) > md_mcp_f(0));delay(10);
      Motor(-60,20); delay(20); 
      fw_2sensor(20, 20, 0.3, 0, "3:4","26", 20);
      fline(0,0,0.3, 10, 'n', 'l',40, "a3", 20);    
  }

void stL_can2_to_L()
  {  
     bline(40,40,0.5,0,'c','r',40, "a4", 30);
     fw_2sensor(30, 30, 0.3, 0, "4:5","b7", 20);
  
     do{Motor(20,20);}while(mcp_b(7) < md_mcp_b(7));delay(20);
          
     do{Motor(40,-40);}while(mcp_f(3) > md_mcp_f(3));delay(20);          
     Motor(-50,40); delay(20); 
     if(mcp_f(1)> md_mcp_f(1)&&mcp_f(2)> md_mcp_f(2))
       {
         do{Motor(-2,20);}while(mcp_f(2) > md_mcp_f(2));
       }
      
    fw_2sensor(30, 30, 0.3, 0, "2:3","b7", 20);  

    do{Motor(60,-20);}while(mcp_f(0) > md_mcp_f(0));delay(10);
    Motor(-60,20); delay(20); 
    fw_2sensor(20, 20, 0.3, 0, "3:4","26", 20);
    fline(0,0,0.3, 10, 'n', 'l',40, "a3", 20);  

    fline_2sensor(30,30,0.3,0,"3:4","a7",'p',0);delay(30); //--> เดินหน้าข้ามเส้น โดยเลือกเซนเซอร์ 7 เช็คเส้น
     do{Motor(40,-20);}while(mcp_f(7) > md_mcp_f(7));  //-----> หมุนขวา ใช้เซนเซอร์ 7 จากขาว ไปหา ดำ
     do{Motor(40,-20);}while(mcp_f(4) > md_mcp_f(4));  //-----> หมุนขวา ใช้เซนเซอร์ 4 จากขาว ไปหา ดำ
     Motor(-40,20); delay(20);  //----------------------------> เบรคการหมุนตัว 
     Motor(0,0); delay(50);    //----------------------------> มอเตอร์ไม่หมุน หน่วงเวลา 100
          
     slow_servo_down();         //----------------------------> ฟังก์ชัน เอาเซอร์โวลงแบบช้า ๆ     
     fw_a3_a4(20, 20, 0.3, 20);  ////เดินเข้าหากรอบ

     Motor(20,15);delay(200);
     Motor(-20,-15);delay(10);
     Motor(0,0);delay(100);
                    
     servo_store();   
     bline(30,30,0.15,0,'c','r',50, "a5", 15);

      fline(50,50,0.5,150,'c','r',40, "b3", 30);
      fline_2sensor(30,30,0.5,0,"2:3","a0",'p',10);
      do{Motor(20,20);}while(mcp_b(0) > md_mcp_b(0));delay(10);
      do{Motor(20,20);}while(mcp_b(0) < md_mcp_b(0));;
      Motor(-20, -20); delay(20);
      Motor(0, 0); delay(100);
      
      do{Motor(-20,60);}while(mcp_f(7) > md_mcp_f(7));
      do{Motor(-20,60);}while(mcp_f(7) < md_mcp_f(7)); delay(10); 
      Motor(20,-60); delay(20); 
     if(mcp_f(2)> md_mcp_f(2)&&mcp_f(1)> md_mcp_f(1))
        {
          do{Motor(20,-20);}while(mcp_f(1) > md_mcp_f(1));
        }
      fline_2sensor(20,20,0.5,0,"1:2","27",'s',20);
      fline(10,10,0.5,10,'n','r',40, "a4", 30);
      
      color_box = "none";     
  }

void stL_can2_to_R()
  {  
    fline(30,30,0.1,200,'n','s',40, "b3", 20);
    do{Motor(-20,50);}while(mcp_f(0) > md_mcp_f(0));
    do{Motor(-20,50);}while(mcp_f(3) > md_mcp_f(3));
    Motor(20,-40); delay(20);
    Motor(0,0); delay(100);
    
    slow_servo_down();     
    fw_a3_a4(25, 25, 0.3, 20);  ////เดินเข้าหากรอบ
    
    Motor(0, 0); delay(10);
    if(trr == 0)
       {
          Motor(20,10);delay(200);
          Motor(-20,-10);delay(10);
          Motor(0,0);delay(100);
              
          servo_store();                
          trr = 1;
        }
     else
        {
          Motor(15,15);delay(200);
          Motor(-10,-10);delay(10);
          Motor(0,0);delay(100);
              
          servo_store();
          trr = 0; 
        }

      bline(30,30,0.15,0,'c','l',40, "a2", 02);
  
      fline(40,40,0.5,250,'c','l',40, "b4", 30);
      fw_2sensor(30,30,0.4,0,"5:6","b7",10);
  
      do{Motor(25,25);}while(mcp_b(7) < md_mcp_b(7));delay(20);
      Motor(-30, -30); delay(30);
      Motor(0, 0); delay(100);
      
      do{Motor(60,-20);}while(mcp_f(2) > md_mcp_f(2));delay(40); 
      //do{Motor(70,-20);}while(mcp_f(0) < md_mcp_f(0)); 
      Motor(-50,20); delay(20); 
      if(mcp_f(5)> md_mcp_f(5)&&mcp_f(6)> md_mcp_f(6))
        {
          do{Motor(-2,20);}while(mcp_f(6) > md_mcp_f(6));
        }
      fw_2sensor(20,20,0.2,0,"5:6","26",20);
      
      fline(0,0,0.5,2,'n','l',40, "a3", 30);
      color_box= "none";

      test() ; 
  }

void stL_can2_to_MD()
  {
    bline(30,30, 0.3, 0, 'c', 'l',40, "a2", 20);  //--->เดินชนเส้น
    fw_2sensor(30, 30, 0.3, 0, "2:3","b0", 20);

    do{Motor(20,20);}while(mcp_b(0) < md_mcp_b(0));delay(20);
          
    do{Motor(-40,40);}while(mcp_f(4) > md_mcp_f(4));delay(20);          
    Motor(40,-40); delay(20); 
    if(mcp_f(6)> md_mcp_f(6)&&mcp_f(5)> md_mcp_f(5))
      {
         do{Motor(20,-2);}while(mcp_f(5) > md_mcp_f(5));
      }
      
    fw_2sensor(30, 30, 0.3, 0, "4:5","b7", 20);  
    do{Motor(30,30);delay(5);}while(mcp_b(7) < md_mcp_b(7));delay(60);       
    fline(0,0,0.5,10,'n','r',40, "a3", 10);
    slow_servo_down();
    fw_2sensor(20, 20, 0.3, 100, "3:4","26", 20);
           
    if(tmd == 0)
      {
         Motor(20,15);delay(200);
         Motor(-20,-15);delay(10);
         Motor(0,0);delay(100);
                    
         servo_store();                
         tmd = 1;
        }
     else
        {
           Motor(15,20);delay(200);
           Motor(-15,-20);delay(10);
           Motor(0,0);delay(100);
                    
           servo_store();
           tmd = 0; 
        }
     do{Motor(15,15);}while(mcp_b(7) > md_mcp_b(7));delay(10);
     do{Motor(15,15);}while(mcp_b(7) < md_mcp_b(7));delay(100);
     Motor(-20, -20); delay(20);
     Motor(0, 0); delay(100);
            
      bline(20,20,0.3, 0, 'c', 'r',40, "b2", 10);
      do{Motor(25,25);}while(mcp_b(7) > md_mcp_b(7));delay(10);
      do{Motor(25,25);}while(mcp_b(7) < md_mcp_b(7));delay(20);
      Motor(-30, -30); delay(20);
      Motor(0, 0); delay(100);
      //fline_2sensor(30,30,0.5,0,"3:4",7,'p',1);  

      do{Motor(60,-20);}while(mcp_f(0) > md_mcp_f(0));delay(30);
      //do{Motor(60,-20);}while(mcp_f(0) < md_mcp_f(0));delay(20);
      Motor(-60,20); delay(20); 
      fw_2sensor(30,30,0.4,0,"5:6","b7",10);
      do{Motor(25,25);}while(mcp_b(7) < md_mcp_b(7));delay(30);
      Motor(-30, -30); delay(30);
      Motor(0, 0); delay(100);
      
      do{Motor(50,-30);}while(mcp_f(2) > md_mcp_f(2));delay(20); 
      Motor(-50,30); delay(20); 
      if(mcp_f(5)> md_mcp_f(5)&&mcp_f(6)> md_mcp_f(6))
        {
          do{Motor(-2,20);}while(mcp_f(6) > md_mcp_f(6));
        }
      fline_2sensor(20,20,0.2,0,"5:6","26",'s',20);
      fline(0,0,0.5,2,'n','l',40, "a3", 30);
      color_box= "none";

      test() ;   
  }
