void can_1()
  {
    arm_ready();
    do{ Motor(20, 20); }while(read_sensor(5) > md_sensor(5));
    delay(100);
    do{ Motor(20, 20); }while(read_sensor(5) < md_sensor(5));
    delay(100);
    //----------------------------------------->> ออกจากกรอบ สตาร์ท
    fw_line(45, 45, 0.25, 's', "a5", 30);
    turn_left_wheel(6, 44, "a1", 50);   //------->>หมุนซ้ายฉาก
    fw_mission(10, 10, 0.25, 6, 10);  
    arm_close_up();       //------->> หุบมือเข้า เอาแขนขึ้น    
    bw_distance(20, 20, 8, 5);
    turn_right_wheel(0, 44, "a4", 50);
    ///---------------------------------------->>  เข้า Y 
    fw_line(40, 40, 0.25, 's', "a5", 50);
    turn_gyro_sensor(1, 70, 35, 50) ;
    fw_line(40, 40, 0.25, 's', "a0", 30);
    turn_gyro_sensor(10, 70, 25, 50) ;
    fw_chopsticks(17,17, 1.85, 50, 5);  //----------------->>ข้ามตะเกียบ     
    servo(28, servo_down + 70); delay(100);
    turn_right_sensor(1, 64, "a3", 50);  //---------------->>  กรอบสี่เหลี่ยม
    turn_left_sensor(2, 64, "a1", 50);
    fw_line(30, 30, 0.25, 'p', "a0", 30);
    turn_left_sensor(1, 64, "a2", 50);
    turn_right_sensor(1, 64, "a3", 30);  //---------------->> กรอบสี่เหลี่ยม
    fw_line(50, 50, 0.55, 'p', "a0", 40);
    fw_line(30, 30, 0.55, 's', "a0", 40);
    arm_down_open_up();sw();
   
  }

void can_2()
  {

    bw_distance(20, 20, 0.25,5, 2);
    turn_left_wheel(0, 54, "a1", 40);
    arm_ready();

    fw_line(40, 40, 0.35, 's', "a0", 20); 
    turn_left_sensor(1, 54, "a1", 20);
    turn_gyro_sensor(2, 70, 30, 20) ; 
    turn_gyro_sensor(4, 70, -45, 0) ;
    turn_left_sensor(0, 54, "a2", 30);  

    fw_distance(40, 40, 1.10, 20, 0);
    fw_mission(10, 10, 0.95, 50, 10);  
    arm_close_up();       //------->> หุบมือเข้า เอาแขนขึ้น 
    
    turn_left_wheel(0, 44, "a1", 30);

    fw_distance(40, 40, 1.25, 22, 5);
    fw_line(10, 10, 0.35, 's', "a0", 50); 
    turn_right_sensor(2, 54, "a2", 30);
    turn_gyro_sensor(2, 70, -40, 20) ;
    turn_right_wheel(0, 44, "a3", 40);  //------>> หมุนต่อไปโดยใช้ "a3" แตะเส้นหยุด
 
    fw_line(50, 50, 0.65, 's', "a0", 30); 
    turn_right_wheel(1, 44, "a4", 40);
    sw();
    ///-------------------------------------------------->> เข้าสีแยกก่อนข้าตะเกียบ
    fw_line(25, 25, 0.15, 's', "a0", 10);
    turn_right_sensor(3, 44, "a3", 30);
    fw_line(20, 20, 0.15, 's', "a0", 5);
    turn_left_sensor(3, 44, "a2", 30);
    fw_line(20, 20, 0.15, 's', "a0", 5);
    turn_gyro_sensor(5, 54, -85, 20);
    fw_line(15, 15, 0.55, 's', "a0", 50);
    turn_right_wheel(0, 34, "a3", 50);
    ////-------------------------------------------------->>
    arm_slow_down();
    fw_chopsticks(17, 17, 1.85, 40, 5);  //----------------->>ข้ามตะเกียบ
    

    ///---------------------------------------->>  เข้า Y 
    //fw_line(20, 20, 0.25, 's', 20);  
    turn_right_sensor(0, 44, "a3", 40);
    fw_line(20, 20, 0.25, 's', "a0", 40);
    turn_gyro_sensor(6, 44, 30, 30);

    servo(28, servo_down + 70);

    fw_line(30, 30, 0.35, 's', "a0", 20); 
    turn_left_wheel(6, 34, "a2", 30);

    fw_line(30, 30, 0.25, 's', "a0", 40);
    arm_down_open_up();
     
  }


void can_3()
  {

    bw_distance(20, 20, 0.25,6, 2);
    turn_left_wheel(0, 34, "a4", 1);
    turn_left_wheel(0, 34, "a2", 30);
    arm_ready();

    fw_mission(15, 15, 0.25, 5, 10);  
    arm_close_up();       //------->> หุบมือเข้า เอาแขนขึ้น 
    
    bw_distance(20, 20, 7, 5);
    turn_right_wheel(0, 44, "a4", 40);

    ///---------------------------------------->>  เข้า Y 
    fw_line(30, 30, 0.25, 's', "a5", 10);
    turn_left_wheel(2, 44, "a1", 30);

    fw_line(20, 20, 0.25, 's', "a5", 40);
    turn_gyro_sensor(5, 40, -43, 40) ;
    arm_slow_down();

    fw_chopsticks(17, 17,  1.85, 40, 5);  //----------------->>ข้ามตะเกียบ

    //fw_line(20, 20, 0.55, 's',  20);
    servo(28, servo_down + 70); delay(100);
    turn_right_sensor(1, 54, "a1", 50);
    turn_left_sensor(2, 54, "a2", 50);
    fw_line(20, 20, 0.25, 's', "a0", 10);
    turn_left_sensor(2, 54, "a2", 50);
    turn_right_sensor(3, 44, "a3", 40); 
    servo(28, servo_down + 70);

    fw_line(30, 30, 0.35, 's', "a0", 30); 
    turn_right_sensor(1, 44, "a3", 30);

    fw_line(20, 20, 0.35, 's', "a5", 10);   //-------->>หยุดตรงเริ่มต้นตัว S
    fw_distance(15, 15, 0.40, 36, 10); 

    fw_line(20, 20, 0.05, 's', "a0", 30); 
    turn_left_sensor(3, 44, "a2", 30);

    fw_line(20, 20, 0.05, 's', "a5", 10); 
    turn_left_sensor(7, 44, 45, 30);
    arm_down_open_up();
     
  }

void can_4()
  {

    bw_distance(20, 20, 0.70, 4, 10);
    turn_right_wheel(0, 34, 35, 1);    //---------->>หมุนตัว ลงหาเส้น 45 องศา
    turn_right_wheel(0, 34, "a3", 30);
    arm_ready();

    fw_line(20, 20, 0.35, 's', "a5", 10); 
    turn_right_sensor(3, 44, "a3", 30);
    fw_line(20, 20, 0.35, 'p', "a5", 10);

    fw_line(30, 30, 0.25, 's', "a0", 20);
    turn_left_sensor(4, 34, "a3", 50); 
    fw_mission(10, 10, 0.55, 30, 10);

    arm_close_up();       //------->> หุบมือเข้า เอาแขนขึ้น  
    turn_left_wheel(2, 34, "a2", 40); 

    fw_line(20, 20, 0.25, 's', "a5", 10); 
    turn_right_sensor(3, 44, "a3", 30);

    fw_line(20, 20, 0.25, 's', "a0", 10); 
     ///---------------------------------------->>  เข้า Y 
    turn_left_sensor(3, 44, "a2", 30);
    fw_line(20, 20, 0.25, 's', "a5", 40);
    turn_left_sensor(4, 34, "a3", 50); 

    fw_line(30, 30, 0.35, 's', "a5", 10);   //-------->>หยุดตรงเริ่มต้นตัว S
    fw_distance(15, 15, 0.50, 40, 10);     //-------->>ข้ามต้นตัว S
    bz(50);

    fw_line(20, 20, 0.35, 's', "a5", 10); 
    turn_left_sensor(3, 44, "a2", 30);

   ///-------------------------------------------------->> เข้าสีแยกก่อนข้าตะเกียบ
    fw_line(25, 25, 0.15, 's', "a0", 10);
    turn_right_sensor(3, 44, "a3", 30);
    fw_line(20, 20, 0.15, 's', "a0", 5);
    turn_left_sensor(3, 44, "a2", 30);
    fw_line(20, 20, 0.15, 's', "a0", 5);
    turn_gyro_sensor(5, 54, -85, 20);
    fw_line(15, 15, 0.55, 's', "a0", 50);
    turn_right_wheel(0, 34, "a3", 50);
    ////-------------------------------------------------->>
    
    arm_slow_down();
    fw_chopsticks(17, 17, 1.85, 40, 5);  //----------------->>ข้ามตะเกียบ


    //fw_line(20, 20, 0.05, 's',  10);  //---------->> เข้า Y ออกซ้าย
    turn_gyro_sensor(3, 44, -45, 30);
    fw_line(20, 20, 0.25, 's', "a5", 40);
    turn_left_sensor(6, 44, "a3", 50);

    fw_line(50, 50, 0.25, 'p', "a5", 0);
    fw_line(30, 30, 0.25, 's', "a5", 10);
    fw_distance(15, 15, 10, 10); 

    arm_down_open_up();
     
  }


void can_5()
  {

    bw_distance(20, 20, 2, 10); 
    turn_right_wheel(0, 34, "a3", 30);

    fw_line(30, 30, 0.35, 's', "a0", 10); 
    turn_left_sensor(2, 44, "a2", 30); 

    fw_line(30, 30, 0.35, 's', "a5", 10); 
    turn_right_sensor(2, 44, "a3", 30); 
    arm_ready();

    fw_line(60, 60, 0.35, 'p', "a5", 0); 
    fw_line(30, 30, 0.35, 's', "a5", 10); 
    turn_right_sensor(2, 44, "a3", 30); 

    fw_line(30, 30, 0.55, 's', "a5", 10); 
    turn_right_sensor(2, 44, "a3", 30);

    fw_line(30, 30, 0.35, 's', "a0", 10); 
    turn_left_sensor(2, 44, "a2", 30); 
    
    fw_line(30, 30, 0.55, 's', "a5", 10); 
    turn_right_wheel(6, 34, "a3", 30);

    fw_mission(10, 10, 0.55, 5, 10);
    arm_close_up();       //------->> หุบมือเข้า เอาแขนขึ้น  

    bw_distance(20, 20, 5, 10); 
    turn_right_wheel(0, 34, "a3", 30);

    fw_line(20, 20, 0.55, 's', "a5", 10); 
    turn_right_sensor(1, 44, "a3", 30);

    fw_line(30, 30, 0.55, 's', "a0", 10); 
    turn_left_sensor(1, 44, "a2", 30); 
    

    fw_line(30, 30, 0.55, 's', "a0", 10); 
    turn_left_sensor(2, 44, "a2", 30); 

    fw_line(50, 50, 0.25, 'p', "a0", 0); 
    fw_line(30, 30, 0.25, 's', "a0", 10); 
    turn_left_sensor(2, 44, "a2", 30); 

    fw_line(20, 20, 0.35, 's', "a5", 10); 
    turn_right_sensor(2, 44, "a3", 30); 

    fw_line(20, 20, 0.25, 's', "a0", 10); 
    turn_left_sensor(2, 44, "a2", 30); 

    fw_line(30, 30, 0.25, 'p', "a0", 0);  

    fw_line(20, 20, 0.25, 's', "a5", 30);
    turn_gyro_sensor(5, 50, 40, 50) ;

    fw_line(20, 20, 0.25, 's', "a0", 40);
    turn_gyro_sensor(9, 50, 33, 50) ;
    arm_slow_down();

   fw_chopsticks(17,17, 1.85, 40, 5);  //----------------->>ข้ามตะเกียบ
     
   // fw_line(20, 20, 0.55, 's',  20);
    fw_line(20, 20, 0.55, 's',  20);
    servo(28, servo_down + 70); delay(100);
    turn_right_sensor(1, 54, "a1", 50);
    turn_left_sensor(2, 54, "a2", 50);
    fw_line(20, 20, 0.25, 's', "a0", 10);
    turn_left_sensor(2, 54, "a2", 50);
    turn_right_sensor(3, 44, "a3", 40); 

    fw_line(30, 30, 0.35, 's', "a0", 30); 
    turn_right_sensor(1, 44, "a3", 30);

   fw_line(20, 20, 0.35, 's', "a5", 10);   //-------->>หยุดตรงเริ่มต้นตัว S
    fw_distance(15, 15, 0.40, 36, 10); 

    fw_line(20, 20, 0.05, 's', "a0", 30); 
    turn_left_sensor(3, 44, "a2", 30);

    fw_line(20, 20, 0.35, 's', "a5", 10); 
    arm_slow_down();
    turn_left_sensor(7, 24, 45, 30);
    arm_down_open_up();

    
     
  }

void can_6()
  {

    bw_distance(20, 20, 0.70, 3, 10);
    turn_left_wheel(0, 34, 50, 1);    //---------->>หมุนตัว ลงหาเส้น 120 องศา
    turn_left_wheel(0, 34, "a2", 1);
    arm_ready();

    fw_line(20, 20, 0.35, 's', "a0", 10); 
    turn_right_sensor(5, 44, "a3", 30);
    
    fw_line(20, 20, 0.35, 's', "a5", 10);   //-------->>หยุดตรงเริ่มต้นตัว S
    fw_distance(15, 15, 0.50, 40, 10);     //-------->>ข้ามต้นตัว S
    bz(50);

    fw_line(20, 20, 0.35, 's', "a5", 10); 
    turn_left_sensor(3, 44, "a2", 30);

    ///-------------------------------------------------->> เข้าสีแยกก่อนข้าตะเกียบ
    fw_line(25, 25, 0.15, 's', "a0", 10);
    turn_right_sensor(3, 44, "a3", 30);
    fw_line(20, 20, 0.15, 's', "a0", 5);
    turn_left_sensor(3, 44, "a2", 30);
    fw_line(20, 20, 0.15, 's', "a0", 5);
    turn_gyro_sensor(5, 54, -85, 20);
    fw_line(15, 15, 0.55, 's', "a0", 50);
    turn_right_wheel(0, 34, "a3", 50);
    ////-------------------------------------------------->>;
    
    
    fw_chopsticks(17, 17, 1.85, 30, 5);  //----------------->>ข้ามตะเกียบ      
    fw_line(20, 20, 0.05, 's', 10);  //---------->> เข้า Y ออกซ้าย

    
    turn_gyro_sensor(4, 44, -40, 30);
    fw_line(20, 20, 0.25, 's', "a5", 40);
    turn_left_sensor(4, 34, "a3", 50);

   
    fw_line(30, 30, 0.25, 's', "a5", 10);
    turn_left_wheel(6, 44, "a2", 30); 
    arm_ready();
    fw_distance(20, 20, 0.50, 30, 10); 
    fw_mission(10, 10, 0.6, 40, 10);
    arm_close_up();       //------->> หุบมือเข้า เอาแขนขึ้น

    
    turn_right_wheel(0, 34, "a3", 30);

     fw_distance(20, 20, 0.50, 30, 10);
    fw_line(20, 20, 0.25, 's', "a5", 10);
    turn_right_sensor(3, 44, "a3", 20); 
    

    ///---------------------------------------->>  เข้า Y 
    //---------------------------------------->>  เข้า Y 
    fw_line(20, 20, 0.25, 's', "a5", 30);
    turn_gyro_sensor(5, 50, 40, 50) ;

    fw_line(20, 20, 0.25, 's', "a0", 40);
    turn_gyro_sensor(9, 50, 33, 50) ;
    arm_slow_down();

   fw_chopsticks(17,17, 1.85, 40, 5);  //----------------->>ข้ามตะเกียบ
     
    //fw_line(20, 20, 0.55, 's',  20);
    fw_line(20, 20, 0.55, 's',  20);
    servo(28, servo_down + 70); delay(100);
    turn_right_sensor(1, 54, "a1", 50);
    turn_left_sensor(2, 54, "a2", 50);
    fw_line(20, 20, 0.25, 's', "a0", 10);
    turn_left_sensor(2, 54, "a2", 50);
    turn_right_sensor(3, 44, "a3", 40); 
    

    arm_slow_down(); delay(100);
    fw_line(40, 40, 0.25, 'p', "a0", 40);
    fw_line(20, 20, 0.25, 's', "a0", 40);
    arm_down_open_up();

    bw_distance(20, 20, 5, 10); 
    turn_left_wheel(0, 34, "a2", 30);
    fw_line(25, 25, 0.25, 's', "a0", 10);
    fw_distance(20, 20, 15, 10); 


     
  }
