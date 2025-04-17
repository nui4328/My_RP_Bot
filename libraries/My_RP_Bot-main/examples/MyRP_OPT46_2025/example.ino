void box1()
  {
    fw_pid(50, 50, 30, "none_line");
    moveLR(50, 92); 
    set_b(2);
    moveLR(150, 50, -90);
     
    servo(27, servo_27_close-110);   
    servo(28, servo_28_close-110);
    
    fw_pid_distance(10, 10, 2700,23);delay(300);  //------>> เข้าคีบที่แท่น    
    servo(27, servo_27_close);   
    servo(28, servo_28_close);   delay(300); 
    arm_up_down(40);             //------>> เข้าคีบที่แท่น  
     _servo(0, 0);     //----->แขนซ้ายกางออก
    _servo(1, 180);     //----->แขนขวากางออก 
    delay(300);
    bw_pid(30, 30, 28, "none_line");
    
    reset_arm(1000);
    servo(27, servo_27_close-40);   
    servo(28, servo_28_close-40);   delay(300); 
    servo(27, servo_27_close);   
    servo(28, servo_28_close);   delay(100); 
    arm_up_down(5);
    
    _servo(0, 0);     //----->แขนซ้ายกางออก
    _servo(1, 180);     //----->แขนขวากางออก 
    
    //robot_start();    //----------------------------->> รอกดปุ่ม
    moveLR(40, 90); 

    set_b(3);
    
    _servo(1, 90);     //----->แขนขวา้ไปขวาชี้ไปข้างหน้า
    fw_pid(35, 35, 93, "none_line",15);  //------->> ขึ้นสะพาน

    //robot_start();    //----------------------------->> รอกดปุ่ม
    _servo(0, 90);     //----->แขนซ้ายชี้ไปข้างหน้า
    _servo(1, 180);     //----->แขนขวากางออก 
    delay(300);
    moveLR(30, -87); 

    fw_pid(30, 30, 50, "line");       //------->> ลงสะพาน    
    _servo(0, 0);     //----->แขนซ้ายกางออก
    _servo(1, 180);     //----->แขนขวากางออก 
    reset_arm(600);
    servo(27, servo_27_close-40);   
    servo(28, servo_28_close-40);   delay(300); 
    servo(27, servo_27_close);   
    servo(28, servo_28_close);   delay(100); 
    arm_up_down(5);
    set_f(2);
    moveLR(45, -92);
    _servo(0, 90);   //----->แขนซ้ายชี้ไปข้างหน้า
   _servo(1, 180);     //----->แขนขวากางออก 
   

   //robot_start();    //----------------------------->> รอกดปุ่ม
    
    fw_pid(50, 50, 75, "line"); 
    set_f(2);delay(500);
    bw_pid(40, 40, 60, "none_line");delay(400);
    _servo(0, 0);     //----->แขนซ้ายกางออก
    _servo(1, 180);     //----->แขนขวากางออก 
    moveLR(40, 90);
    set_f(2);
    moveLR(40, 90);

    fw_pid(50, 50, 90, "line");
    
    _servo(0, 0);     //----->แขนซ้ายกางออก
    _servo(1, 180);     //----->แขนขวากางออก 
    moveLR(40, 90);
    set_b(2);
    reset_arm(500);
    servo(27, servo_27_close-40);   
    servo(28, servo_28_close-40);   delay(300); 
    servo(27, servo_27_close);   
    servo(28, servo_28_close);   delay(100); 

    fw_pid(50, 50, 90, "line", 10);
    moveLR(40, 90);
    set_b(2);

    fw_pid(40, 40, 20, "none_line",30); 
    _servo(0, 90);     //----->แขนซ้ายชี้ไปข้างหน้า
    _servo(1, 90);     //----->แขนขวาชี้ไปข้างหน้า
    
    fw_pid_distance(10, 10, 2700,58);delay(300);  //------>> วางสีแดง 
    arm_up_down(50); 
    servo(28, servo_28_close-60);   delay(300);    
    servo(28, servo_28_close-110);   delay(100);

    bw_pid(20, 20, 15, "none_line");
    reset_arm(1000); 
    servo(27, servo_27_close);   
    servo(28, servo_28_close);   delay(100);   

    bw_pid(30, 30, 30, "line");    
    moveLR(40, 88);
    set_b(2);
    reset_arm(500);
    
    fw_pid(40, 40, 25, "none_line",10);
    moveLR(40, -90);
    set_b(2);

    fw_pid_distance(10, 10, 2600,40);delay(300);  //------>> วางสีเหลือง
    arm_up_down(32);
    servo(27, servo_27_close-60);   delay(300);    
    servo(27, servo_27_close-110);   delay(100);

    bw_pid(40, 40, 20, "line");       
    reset_arm(1000);
    moveLR(50, 88);

    fw_pid(30, 30, 26, "none_line");
    moveLR(45, 90);
    set_b(2);   

    servo(27, servo_27_close-110);   
    servo(28, servo_28_close-110);delay(300);
    fw_pid(15, 15, 28, "none_line");   //--------->> เดินเข้าเก็บกระป๋องชุดที่ 2
    servo(27, servo_27_close);   
    servo(28, servo_28_close);delay(300);
    arm_up_down(10); 
    _servo(0, 0);     //----->แขนซ้ายกางออก
    _servo(1, 180);     //----->แขนขวากางออก
    delay(300);
    bw_pid(50, 50, 20, "line");
    moveLR(40, 90);

    fw_pid(30, 30, 38, "none_line");
    moveLR(45, 90);
    set_b(2);

     _servo(1, 70);     //----->แขนขวากางออก 
    fw_pid_distance(10, 10, 2600,35);delay(300);  //------>> วางสีเหลืองที่ 2 
    arm_up_down(32);    
    servo(27, servo_27_close-60);   delay(300);    
    servo(27, servo_27_close-110);   delay(100);

    bw_pid(40, 40, 20, "line");       
    reset_arm(1000);
    arm_up_down(10);
    set_b(1);
    moveLR(40, 88);

    fw_pid(50, 50, 60, "line");
    moveLR(45, -90);
    set_b(2);

    fw_pid(50, 50, 58, "none_line");
    moveLR(50, -90);
    set_b(2);
    arm_up_down(10);
    _servo(0, 90);      //------------->> แขนซ้ายชี้ไปข้างหน้า    
    fw_pid(40, 40, 54, "none_line", 20);    //------>>ขึ้นสะพาน
    moveLR(30, 87);  
    _servo(1, 90);      //------------->> แขนขวาตรงหน้า
    _servo(0, 90);      //------------->> แขนขวากางออก
    
    fw_pid(20, 20, 50, "none_line");    //------>> ลงสะพาน
    _servo(1, 180);    
    _servo(0, 0);     
    moveLR(40, -90);
    
    fw_pid(40, 40, 20, "line");
    moveLR(45, -90);
    set_b(2);
    _servo(0, 70);      //------------->> แขนขวากางออก
    fw_pid_distance(10, 10, 2600,100);delay(300);  //------>> วางสีแดงที่ 2  

     
     
     //robot_start();    //----------------------------------------------->> รอกดปุ่ม
    servo(28, servo_27_close-60);   delay(300);    
    servo(28, servo_27_close-110);   delay(100);    

    bw_pid(20, 20, 20, "line");
    reset_arm(2000);
        
    _servo(0, 90); 
    _servo(1, 90);
    moveLR(45, -90);
    set_b(2);
    
    fw_pid(50, 50, 60, "line");
    moveLR(45, 90);
    servo(27, servo_27_close-110);   
    servo(28, servo_28_close-110);delay(300);
    fw_pid(10, 10, 24, "none_line");   //--------->> เดินเข้าเก็บกระป๋องชุดที่ 2
    Motor(10,10);delay(200);
    Motor(-1,-1);delay(30);
    servo(27, servo_27_close);   
    servo(28, servo_28_close);delay(300);
    arm_up_down(10);

    bw_pid(30, 30, 30, "none_line");
    _servo(0, 0); 
    _servo(1, 180);
    moveLR(35, 90);
    fw_pid(40, 40, 55, "line");   
    set_f(2);
    
    moveLR(100, 45, -90);
    set_b(2);
    arm_up_down(40);
    _servo(0, 130);      //------------->> แขนขวากางออก
    fw_pid_distance(10, 10, 2600,100);delay(300);  //------>> วางสีแดงที่ 2 
    servo(28, servo_27_close-60);   delay(300);    
    servo(28, servo_27_close-110);   delay(100);    

    bw_pid(20, 20, 20, "line");
    _servo(0, 90); 
    servo(28, servo_27_close); 
    reset_arm(2000);
    arm_up_down(10);

    moveLR(40, -90);
    set_b(4);
    _servo(1, 90);
    fw_pid(30, 30, 25, "none_line");
    moveLR(35, 90);

    arm_up_down(15);
    fw_pid(40, 40, 55, "none_line");  //------->> ขึ้นสะพาน

    //robot_start();    //----------------------------->> รอกดปุ่ม
    moveLR(30, -87); 
    fw_pid(30, 30, 50, "line");       //------->> ลงสะพาน
    set_f(2);
    _servo(0, 0);     //----->แขนซ้ายกางออก
    _servo(1, 180);     //----->แขนขวากางออก 
    reset_arm(500);
    arm_up_down(10);
    moveLR(45, 90);

    fw_pid(50, 50, 55, "line"); 
    moveLR(40, 90);
    set_b(2);
    reset_arm(500);    

    fw_pid(70, 70, 90, "line", 10);
    moveLR(40, 90);
    set_b(2);

    fw_pid(40, 40, 20, "none_line",30); 
    _servo(0, 0);     //----->แขนซ้ายชี้ไปข้างหน้า
    _servo(1, 110);     //----->แขนขวาชี้ไปข้างหน้า
    
    fw_pid_distance(10, 10, 2700,58);delay(300);  //------>> วางสีแดง2
    arm_up_down(50); 
    servo(27, servo_27_close-60);   delay(300);    
    servo(27, servo_27_close-110);   delay(100);

    bw_pid(20, 20, 15, "none_line");
    reset_arm(1500); 
    servo(27, servo_27_close);   
    servo(28, servo_28_close);   delay(100);   

    bw_pid(30, 30, 30, "line");    
    moveLR(40, 88);
    set_b(2);
    reset_arm(500);

    arm_up_down(2);
    _servo(0, 90);     //----->แขนซ้ายชี้ไปข้างหน้า
    _servo(1, 90);     //----->แขนขวาชี้ไปข้างหน้า
    
    fw_pid(70, 70, 90, "line");    
    moveLR(45, -88);
    set_b(2);

    fw_pid(50, 50, 60, "none_line");
    moveLR(50, -90);
    set_b(2);
    arm_up_down(15);
    _servo(0, 90);      //------------->> แขนซ้ายชี้ไปข้างหน้า    
    fw_pid(40, 40, 54, "none_line");    //------>>ขึ้นสะพาน
    moveLR(40, 88);         
    fw_pid(50, 50, 90, "line");    //------>> ลงสะพาน
    moveLR(45, -90);
    fw_pid(30, 30, 25, "line");
        

    
    
   
       
  }
