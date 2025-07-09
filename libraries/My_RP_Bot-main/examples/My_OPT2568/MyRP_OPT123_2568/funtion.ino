
void goto_missiom_5cm()
  {
    arm_Slide(0);  //----->>  เลื่อนแขนลงให้สุดเพื่อเริ่มต้น
    servo(27, servo_27_close-90);   
    servo(28, servo_28_close-90);     // กางฝ่ามือออก

    fw_distance(10, 10, 1.5, 2600, 600) ;  //---->>เดินเข้าไปคีบที่ระยะ 2700 ยกแขนขึ้นที่ 400
    Motor(15, 15),delay(100);
    Motor(-1, -1),delay(50);
     
    servo(27, servo_27_close);   
    servo(28, servo_28_close);
    delay(300);
    arm_Slide(500);  //----->>  เลื่อนแขน-ขึ้น

    

  }

void mission_5cm()
  {
    arm_Slide(0);  //----->>  เลื่อนแขนลงให้สุดเพื่อเริ่มต้น
    fw_distance(20, 20, 1.5, 2500, 500) ;  //---->>เดินเข้าไปคีบที่ระยะ 2700 ยกแขนขึ้นที่ 400

    arm_Slide(-100);  //----->>  เลื่อนแขนลงให้สุดเพื่อเริ่มต้น
    delay(300);
    servo(27, servo_27_close-40);  //---> เซอร์กางออก 
    servo(28, servo_28_close-40); 
    delay(200);
    servo(27, servo_27_close-90);  //---> เซอร์กางออก 
    servo(28, servo_28_close-90); 
    delay(200);

    
  }