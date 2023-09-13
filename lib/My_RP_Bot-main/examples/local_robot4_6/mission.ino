void mission_L()  //วางกล่องซ้าย
  {
    fline(30,30,0.3,0,'c','l',40, "b3", 20);
    do{Motor(30,30);}while(mcp_b(7) > md_mcp_b(7));
    Motor(-20, -20); delay(10);
    Motor(0, 0); delay(100);
    
    do{Motor(60,-15);}while(mcp_f(0) > md_mcp_f(0));delay(10);
    do{Motor(60,-15);}while(mcp_f(0) < md_mcp_f(0)); delay(30);
    Motor(-50,20); delay(10);
    do{Motor(-30,50);}while(mcp_f(0) > md_mcp_f(0));delay(10);
    do{Motor(-30,50);}while(mcp_f(1) > md_mcp_f(1));delay(10);
    do{Motor(-30,50);}while(mcp_f(1) < md_mcp_f(1));delay(10);
    Motor(30,-50); delay(20);    

    
    fline(40,40,0.3,250,'n','s',50, "a4", 1);
    fw_a3_a4(40, 30, 0.3, 2);delay(50);
    do{Motor(40,-20);}while(mcp_f(7) > md_mcp_f(7));
    do{Motor(40,-20);}while(mcp_f(4) > md_mcp_f(4));
    Motor(-40,10); delay(20);
    Motor(0,0); delay(100);
    
    slow_servo_down();     
    fw_a3_a4(25, 25, 0.3, 20);  ////เดินเข้าหากรอบ
    
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

      bline(30,30,0.3,0,'c','r',50, "a5", 20);
      fline(40,40,0.5,250,'c','r',40, "b3", 30);
      fw_2sensor(30,30,0.4,0,"2:3","b0",10);
  
      do{Motor(25,25);}while(mcp_b(0) < md_mcp_b(0));delay(20);
      Motor(-30, -30); delay(30);
      Motor(0, 0); delay(100);
      
      do{Motor(-20,60);}while(mcp_f(5) > md_mcp_f(5));delay(40); 
      Motor(20,-60); delay(20); 
      if(mcp_f(1)> md_mcp_f(1)&&mcp_f(2)> md_mcp_f(2))
        {
          do{Motor(-2,20);}while(mcp_f(2) > md_mcp_f(2));
        }
      fw_2sensor(20,20,0.2,0,"1:2","27",20);
      
      fline(0,0,0.5,2,'n','r',40, "a4", 30);
      color_box= "none";

      test() ; 
  }

void mission_R() //วางกล่องขวา
  {
    fline(30,30,0.3,0,'c','r',40, "b4", 20);
    do{Motor(30,30);}while(mcp_b(0) > md_mcp_b(0));
    Motor(-20, -20); delay(10);
    Motor(0, 0); delay(100);
    
    do{Motor(-15,60);}while(mcp_f(7) > md_mcp_f(7));delay(10);
    do{Motor(-15,60);}while(mcp_f(7) < md_mcp_f(7)); delay(30);
    Motor(15,-50); delay(10);
    do{Motor(50,-30);}while(mcp_f(7) > md_mcp_f(7));delay(10);
    do{Motor(50,-30);}while(mcp_f(6) > md_mcp_f(6));delay(10);
    do{Motor(50,-30);}while(mcp_f(6) < md_mcp_f(6));delay(10);
    Motor(-50,30); delay(20);    

    
    fline(40,40,0.3,250,'n','s',50, "a3", 1);
    fw_a3_a4(40, 40, 0.3, 2);delay(40);
    do{Motor(-20,40);}while(mcp_f(0) > md_mcp_f(0));
    do{Motor(-20,40);}while(mcp_f(3) > md_mcp_f(3));
    Motor(20,-40); delay(20);
    Motor(0,0); delay(100);
    
    slow_servo_down();     
    fw_a3_a4(25, 25, 0.3, 20);  ////เดินเข้าหากรอบ
    
    Motor(0, 0); delay(10);
    if(trr == 0)
       {
          Motor(10,20);delay(200);
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

      bline(30,30,0.3,0,'c','l',50, "a2",20);
  
      fline(40,40,0.5,250,'c','l',40, "b4", 30);
      fw_2sensor(30,30,0.4,0,"5:6","b7",10);
  
      do{Motor(25,25);}while(mcp_b(7) < md_mcp_b(7));delay(20);
      Motor(-30, -30); delay(30);
      Motor(0, 0); delay(100);
      
      do{Motor(60,-20);}while(mcp_f(2) > md_mcp_f(2));delay(40); 
      //do{Motor(70,-20);}while(mcp_f(0) < md_mcp_f(0)); 
      Motor(-50,20); delay(20); 
  
     do{Motor(-20,20);}while(mcp_f(6) > md_mcp_f(6));
       
      fw_2sensor(20,20,0.2,0,"5:6","26",20);
      
      fline(0,0,0.5,2,'n','l',40, "a3", 30);
      color_box= "none";

      test() ; 
  }


void mission_MD() //วางกล่องกลาง
  {
    fline(30,30,0.3,0,'c','l',40, "b3", 10);
    do{Motor(30,30);}while(mcp_b(7) > md_mcp_b(7));
    //do{Motor(30,30);}while(mcp_b(7) < md_mcp_b(7));
    Motor(-30, -30); delay(20);
    Motor(0, 0); delay(100);
    
    do{Motor(60,-20);}while(mcp_f(1) > md_mcp_f(1));delay(20);
    //do{Motor(70,-20);}while(mcp_f(0) < md_mcp_f(0));  
    Motor(-60,20); delay(20); 

    do{Motor(-20,20);}while(mcp_f(5) > md_mcp_f(5));
 
    
    fw_2sensor(30, 30, 0.3, 0, "4:5","b7", 20);

    do{Motor(20,20);}while(mcp_b(7) < md_mcp_b(7));delay(20);
          
    do{Motor(40,-40);}while(mcp_f(3) > md_mcp_f(3));delay(20);          
    Motor(-50,40); delay(20); 
  
    do{Motor(-20,20);}while(mcp_f(2) > md_mcp_f(2));
     
      
    fw_2sensor(30, 30, 0.3, 0, "2:3","b0", 20);  
    do{Motor(25,25);}while(mcp_b(0) < md_mcp_b(0));delay(60);       
    fline(0,0,0.5,10,'n','l',30, "a4", 20);
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
      //fw_2sensor(30,30,0.5,0,"3:4","a7",1);  

      do{Motor(60,-20);}while(mcp_f(1) > md_mcp_f(1));delay(30);
      //do{Motor(60,-20);}while(mcp_f(0) < md_mcp_f(0));delay(20);
      Motor(-60,20); delay(20); 
      do{Motor(-20,20);}while(mcp_f(6) > md_mcp_f(6));
      fw_2sensor(30,30,0.4,0,"5:6","b7",10);
  
     do{Motor(25,25);}while(mcp_b(7) < md_mcp_b(7));delay(20);
      Motor(-30, -30); delay(30);
      Motor(0, 0); delay(100);
      
      do{Motor(60,-20);}while(mcp_f(2) > md_mcp_f(2));delay(40); 
      //do{Motor(70,-20);}while(mcp_f(0) < md_mcp_f(0)); 
      Motor(-50,20); delay(20); 
   
      do{Motor(-2,20);}while(mcp_f(6) > md_mcp_f(6));
       
      fw_2sensor(20,20,0.2,0,"5:6","26",10);
      
      fline(0,0,0.5,2,'n','l',40, "a4", 30);
      color_box= "none";

      test() ;   
  }
void test()
  {
      delay(50); 
  }
