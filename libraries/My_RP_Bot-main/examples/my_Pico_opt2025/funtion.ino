
void out_left()
  {
    fw_line(30, 30, 0.35, 's', "a0", 20);
    turn_right_sensor(0, 30, "a3", 30); 
  }

  
void curve_y()   //---->> แยก
  {
    arm_ready();
    fw_line(30, 30, 0.25, 's', "a0", 20);   //---->> เดินหน้าใช้ เซนเซอร์ a0 แตะเส้นหยุด
    turn_left_sensor(3, 37, "a2", 30);      //---->> หมุนซ้าย โดยเซนเซอร์ a0 แตะเส้นหยุด
    fw_distance(10, 10, 0.85, 5, 10);      //---->> เดินหน้าระยะ 10 cm
    turn_right_sensor(0, 20, 30, 30);       //---->> เดินหน้า 0 cm หมุนขวาด้วยความเร็ว 20 องศา 30
    fw_distance(10, 10, 0.0, 3, 10);        //---->> เดินหน้าระยะ 2 cm
    arm_close_up();                         //---->> คีบ และ ยก
  }
  
void curve_S()
  {
    arm_ready();
    fw_line(10, 10, 0.85, 's', "a5", 20); //--->เดินแตะเส้นก่อนโค้ง
    turn_right_sensor(2, 34, 45, 30);     //--->หมุนก่อนโค้ง หมุนที่ 45 องศา
    
    fw_distance(30, 30, 0.85, 7, 2);   //----->> โค้งตัว S
    fw_line(30, 30, 0.85, 's', "a0", 30);    //----->> โค้งตัว S
    turn_left_sensor(1, 24, "a2", 30);
  }

void example_01()
  {
    arm_ready();
    fw_line(30, 30, 0.85, 's', "a5", 20);//---->> ทางโค้งค่า pid = 0.85
    turn_right_sensor(1, 34, "a3", 30); 
    
    fw_line(30, 30, 0.85, 's', "a5", 20);
    turn_right_sensor(1, 34, "a3", 30); 
    
    fw_distance(30, 30, 0.85, 5, 2);   //----->> โค้งตัว S
    fw_line(30, 30, 0.85, 's', "a0", 30);   
    turn_left_sensor(1, 34, "a2", 30); 
    
    fw_line(30, 30, 0.15, 's', "a5", 10);
    turn_right_sensor(1, 44, "a3", 30); 

    fw_line(30, 30, 0.15, 's', "a0", 20);
    turn_left_sensor(1, 44, "a2", 30); 

     fw_line(30, 30, 0.85, 's', "a0", 10);
    turn_right_sensor(1, 44, "a3", 30); 

     fw_line(30, 30, 0.25, 's', "a0", 10);  //---->> ทางตรงค่า pid = 0.25
    turn_right_sensor(2, 44, "a3", 30); 

     fw_line(30, 30, 0.25, 's', "a5", 30);
    turn_right_wheel(5, 34, "a3", 30); 

    delay(1000);

    turn_right_wheel(0, 34, "a3", 30); 

    fw_line(30, 30, 0.15, 's', "a5", 20);
    turn_left_sensor(1, 44, "a2", 30); 

    fw_line(30, 30, 0.85, 's', "a5", 20);
    turn_left_sensor(3, 34, "a2", 30);            

     fw_line(20, 20, 0.85, 's', "a5", 10);
    turn_right_sensor(1, 44, "a3", 30); 

     fw_line(30, 30, 0.85, 's', "a0", 10);
    turn_left_sensor(1, 44, "a2", 30); 

    fw_line(10, 10, 0.85, 's', "a5", 20); //--->เดินก่อนโค้ง
    turn_right_sensor(2, 34, 45, 30);     //--->หมุนก่อนโค้ง หมุนที่ 45 องศา
    //sw();
    fw_distance(30, 30, 0.85, 5, 2);   //----->> โค้งตัว S
    fw_line(30, 30, 0.85, 's', "a0", 30);    //----->> โค้งตัว S
    turn_left_sensor(1, 34, "a2", 30); 

    fw_line(30, 30, 0.85, 's', "a0", 20);
    turn_left_sensor(3, 34, "a2", 30); 

    fw_distance(30, 30, 0.55, 25, 1);

    fw_line(30, 30, 0.15, 's', "a5", 20);

  }
