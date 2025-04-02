void can_1()
  {
    arm_open_up();
    fw_line(30, 30, 0.85, 's', "a5", 20);
    turn_right_sensor(1, 34, "a3", 30); 

    arm_ready();
    fw_distance(15, 15, 0.85, 5, 20);  

    arm_close_up();       //------->> หุบมือเข้า เอาแขนขึ้น

    bw_distance(30, 30, 0.85, 5, 2);
    turn_left_wheel(3, 34, "a2", 30);

    fw_line(30, 30, 0.85, 's', "a5", 20);
    turn_right_sensor(1, 34, "a3", 30); 

    
    fw_line(30, 30, 0.85, 's', "a5", 20);
    arm_down_open_up();   //------->> เอาแขนลง กางมือออก ยกมือขึ้น    
  }

void can_2()
  {
    arm_open_up();
    fw_line(30, 30, 0.85, 's', "a5", 20);
    turn_right_sensor(1, 34, "a3", 30); 

    arm_ready();
    fw_distance(15, 15, 0.85, 5, 20);  

    arm_close_up();       //------->> หุบมือเข้า เอาแขนขึ้น

    bw_distance(30, 30, 0.85, 5, 2);
    turn_left_wheel(3, 34, "a2", 30);

    fw_line(30, 30, 0.85, 's', "a5", 20);
    turn_right_sensor(1, 34, "a3", 30); 

    
    fw_line(30, 30, 0.85, 's', "a5", 20);
    arm_down_open_up();   //------->> เอาแขนลง กางมือออก ยกมือขึ้น    
  }
