void goto_missiom_5cm()
  {
    arm_Slide(0);  //----->>  เลื่อนแขนลงให้สุดเพื่อเริ่มต้น
    servo(27, servo_27_close-90);   
    servo(28, servo_28_close-90); 

    fw_distance(10, 10, 1.5, 2700, 300) ;  //---->>เดินเข้าไปคีบที่ระยะ 2700 ยกแขนขึ้นที่ 400

    servo(27, servo_27_close);   
    servo(28, servo_28_close);
    delay(300);
    arm_Slide(600);  //----->>  เลื่อนแขนลงให้สุดเพื่อเริ่มต้น

    bw(20, 20, 1.5, 30, "none_line",-1000); 

  }

void mission_5cm()
  {
    fw_distance(20, 20, 1.5, 2700, 600) ;  //---->>เดินเข้าไปคีบที่ระยะ 2700 ยกแขนขึ้นที่ 400

    arm_Slide(-100);  //----->>  เลื่อนแขนลงให้สุดเพื่อเริ่มต้น
    delay(300);
    servo(27, servo_27_close-40);  //---> เซอร์กางออก 
    servo(28, servo_28_close-40); 
    delay(200);
    servo(27, servo_27_close-90);  //---> เซอร์กางออก 
    servo(28, servo_28_close-90); 
    delay(200);

    bw(20, 20, 1.5, 30, "line",-500); 
  }