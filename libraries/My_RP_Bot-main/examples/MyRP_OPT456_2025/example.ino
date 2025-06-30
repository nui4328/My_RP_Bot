
void test_move01()
  {
    goto_missiom_5cm();
    bw(20, 20, 1.5, 30, "line",-1000); 
    moveLR(40, -90); 
    fw(40, 40, 1.5, 30, "line"); 
    set_f(2);
    moveLR(300, 40, 90); 
    set_b(2);

    arm_Slide(700);  //----->>  เลื่อนแขนลงให้สุดเพื่อเริ่มต้น
    fw_distance(30, 30, 1.5, 2500, 200) ;  
    Motor(15, 15),delay(100);
    Motor(-1, -1),delay(50);
    arm_Slide(-100);  //----->>  เลื่อนแขนลงให้สุดเพื่อเริ่มต้น
    shake_servo_L(45, 5);  //----->>  มือซ้ายวางกระป๋องทางซ้ายสุด    50 คือองศา  5 คือจำนวนครั้งที่แกว่ง
    shake_servo_R(50, 5);  //----->>  มือขวาวางกระป๋องทางขวาสุด     50 คือองศา  5 คือจำนวนครั้งที่แกว่ง
    
    bw(20, 20, 1.5, 30, "line",-500); 
  }

void st1()  
  {
    fw(50, 50, 1.5, 90, "none_line"); 
    moveLR(70, 90); 
    fw(50, 50, 1.5, 60, "none_line"); 
    moveLR(70, 90); 
    set_b(1);
    fw(50, 50, 1.5, 26, "none_line"); 
    moveLR(70, 90); 
    fw(50, 50, 1.5, 25, "line"); 
    moveLR(70, -90); 
    set_b(1);
    fw(50, 50, 1.5, 30, "none_line"); 
    moveLR(70, -90); 
    set_b(1);
    fw_bridge(50, 50, 1.5, 120, "none_line");  
    moveLR(70, 90); 

    fw(50, 50, 1.5, 30, "line");
    moveLR(70, 90); 
    set_b(1);

    fw_chopsticks (20, 20, 2.0, 60, "none_line");
    fw(50, 50, 1.5, 30, "line");
    
    set_f(2);
    bw(40, 40, 1.5, 40, "none_line");
    bw_chopsticks (20, 20, 2.0, 60, "line");

    moveLR(70, 90); 
    set_b(1);

    fw(40, 40, 1.5, 27, "none_line"); 
    moveLR(70, -90); 

    fw_bridge(50, 50, 1.5, 120, "line");  
    moveLR(70, 90); 

    fw(50, 50, 1.5, 25, "line");  
    moveLR(70, 90); 
    fw(50, 50, 1.5, 25, "none_line");  
    moveLR(70, -90); 

    fw(50, 50, 1.5, 30, "line");  
    moveLR(70, -90); 

    set_b(1);

    fw(40, 40, 1.5, 60, "none_line"); 
    moveLR(70, -90);

    set_b(1);

    fw(40, 40, 1.5, 90, "line"); 
  





  }
