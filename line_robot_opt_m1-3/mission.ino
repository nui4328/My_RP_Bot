void fl()
  {
    fline(40,40, 0.3, 0, 'c', 'l',60, "a2", 30);    
    fline(50,50, 0.3, 0, 'f', 's',60, "a7", 12); 
    stop_f26();         //---------> เดินหน้าหยุดโดยใช้เซนเซอร์ GP27 
    do{Motor(50, -20);}while(mcp_f(4) > md_mcp_f(4)-150);
    Motor(-20, 10); delay(10); 
    Motor(0, 0); delay(10); 
        
    fw_a3_a4(30, 30, 0.3, 0.2, 20);  ////เดินเข้าหากรอบ
    
    Motor(0, 0); delay(10);

    if(tll == 0)
       {
          Motor(20,15);delay(200);
          Motor(-20,-10);delay(10);
          Motor(0,0);delay(10);
              
          servo_store();                
          tll = 1;
        }
     else
        {
          Motor(15,15);delay(200);
          Motor(-10,-20);delay(10);
          Motor(0,0);delay(10);
              
          servo_store();
          tll = 0; 
        }
     bline(40,40, 0.2, 0, 'f', 's',50, "a5", 10);  
     do{Motor(-40, -40);}while(analogRead(27) > md_adc(27));
     Motor(-30, -30); delay(20); 
     Motor(0, 0); delay(10); 
     fline(0,0, 0.2, 10, 'n', 'r',50, "a5", 20);
     fline(50,50, 0.2, 0, 'c', 'r',50, "a4", 30);  //-------> จบตรงสี่แยกหน้าจุดเริ่มต้น
  }

 
 void  fr()
  {
    fline(40,40, 0.3, 0, 'c', 'r',60, "a5", 30);    
    fline(50,50, 0.3, 0, 'f', 's',60, "a7", 12); 
    stop_f27();         //---------> เดินหน้าหยุดโดยใช้เซนเซอร์ GP27 
    do{Motor(-20, 50);}while(mcp_f(3) > md_mcp_f(3)-150);
    Motor(10, -20); delay(10); 
    Motor(0, 0); delay(10); 
        
    fw_a3_a4(30, 30, 0.3, 0.2, 20);  ////เดินเข้าหากรอบ    
    Motor(0, 0); delay(10);

    if(trr == 0)
       {
          Motor(20,15);delay(200);
          Motor(-20,-10);delay(10);
          Motor(0,0);delay(10);
              
          servo_store();                
          trr = 1;
        }
     else
        {
          Motor(15,15);delay(200);
          Motor(-10,-20);delay(10);
          Motor(0,0);delay(10);
              
          servo_store();
          trr = 0; 
        }
     bline(40,40, 0.2, 0, 'f', 's',50, "a3", 10);
       
     do{Motor(-50, -30);}while(analogRead(26) > md_adc(26));
     Motor(30, 30); delay(20); 
     Motor(0, 0); delay(100); 
     fline(0, 0, 0.2, 10, 'n', 'l',50, "a2", 20);
     fline(50,50, 0.2, 0, 'c', 'l',50, "a3", 30);  //-------> จบตรงสี่แยกหน้าจุดเริ่มต้น
  }

 
