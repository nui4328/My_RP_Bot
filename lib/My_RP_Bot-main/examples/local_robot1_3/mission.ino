void check_box() 
  {
     if(can_color == ground_color)
      {
        fl();    // ใส่ช่องที่เข้าไปเก็บค่าสีพื้น เพื่อวางกระป๋อง
      }
    else
      {
        fr();    // ใส่ช่องที่จะวางกระป๋อง อีกฝั่งหนึ่ง
      }    
  }


void fl()
  {
    fline(40,40, 0.3, 0, 'c', 'l',50, "a2", 30);    
    fline(40,40, 0.3, 0, 'c', 's',60, "a7", 12); 
    do{Motor(50, -20);}while(mcp_f(4) > md_mcp_f(4)-150);
    Motor(-40, 20); delay(20); 
    Motor(0, 0); delay(100); 
    slow_servo_down();  delay(200);   
    fw_a3_a4(20, 20, 0.3, 20);  ////เดินเข้าหากรอบ
    
    Motor(0, 0); delay(10);

    if(tll == 0)
       {
          Motor(20,10);delay(200);
          Motor(-20,-10);delay(10);
          Motor(0,0);delay(100);
              
          servo_store();                
          tll = 1;
        }
     else
        {
          Motor(15,15);delay(200);
          Motor(-10,-10);delay(10);
          Motor(0,0);delay(100);
              
          servo_store();
          tll = 0; 
        }
     bline(40,40, 0.1, 0, 'f', 'p',50, "a5", 10); 
     do{Motor(-40, -40);}while(analogRead(27) > md_adc(27));
     Motor(20, 20); delay(10); 
     Motor(0, 0); delay(10); 
      
     do{Motor(40, -40);}while(mcp_f(7) > md_mcp_f(7));
     do{Motor(40, -40);}while(mcp_f(4) > md_mcp_f(4));
     Motor(-50, 50); delay(20); 
     Motor(0, 0); delay(10);
     
     fline(50,50, 0.2, 0, 'c', 'r',50, "a4", 30);  //-------> จบตรงสี่แยกหน้าจุดเริ่มต้น
  }

 
 void  fr()
  {
    fline(40,40, 0.3, 0, 'c', 'r',50, "a5", 30);    
    fline(40,40, 0.3, 0, 'c', 's',60, "a7", 12); 
    do{Motor(-20, 50);}while(mcp_f(3) > md_mcp_f(3)-150);
    Motor(20, -40); delay(20); 
    Motor(0, 0); delay(100); 
    slow_servo_down();  delay(200);     
    fw_a3_a4(20, 20, 0.3, 20);  ////เดินเข้าหากรอบ    
    Motor(0, 0); delay(10);

    if(trr == 0)
       {
          Motor(15,15);delay(200);
          Motor(-10,-10);delay(10);
          Motor(0,0);delay(100);
              
          servo_store();                
          trr = 1;
        }
     else
        {
          Motor(10,20);delay(200);
          Motor(-10,-20);delay(10);
          Motor(0,0);delay(100);
              
          servo_store();
          trr = 0; 
        }
     
     bline(40,40, 0.1, 0, 'f', 'p',50, "a5", 10); 
     do{Motor(-40, -40);}while(analogRead(26) > md_adc(26));
     Motor(20, 20); delay(10); 
     Motor(0, 0); delay(10);
       
     do{Motor(-40, 40);}while(mcp_f(0) > md_mcp_f(0));
     do{Motor(-40, 40);}while(mcp_f(3) > md_mcp_f(3));
     Motor(50, -40); delay(20); 
     Motor(0, 0); delay(10);
      
     fline(50,50, 0.2, 0, 'c', 'l',50, "a3", 30);  //-------> จบตรงสี่แยกหน้าจุดเริ่มต้น
  }

 
