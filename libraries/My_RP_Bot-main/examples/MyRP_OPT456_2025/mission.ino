void arm_right()
  {
    if(color_right == stsnd_1[0])
      {
        servo(27, servo_27_close);  
        servo(28, servo_28_close); 
        delay(200);
        bw(10, 10, 1.5, 2, "none_line"); 
            delay(300);
        shake_servo_R(80, 5);
      }
    else if(color_right == stsnd_1[1])
      {
        //bw(50, 50, 1.5, 10, "none_line"); 
        //moveLR(70, 90);
        //set_f(2); 
        //moveLR(150, 70, 90);      //------------------>> ปรับการถอยหลัง
        //fw_distance(10, 10, 1.5, 2500) ;
        _servo(0, 90);
        servo(27, servo_27_close);  
        servo(28, servo_28_close); 
        delay(200);
        shake_servo_R(30, 5);
        _servo(1, 40);
      }
    else  
      {
        if(color_left == stsnd_1[0])   //----->> ถ้ามือซ้ายสีตรงกับขวาสุด
          {
            //---------------------->> ให้วางตรงกลางก่อน
            _servo(0, 90);
            servo(27, servo_27_close);  
            servo(28, servo_28_close); 
            delay(200);
            shake_servo_R(30, 5);
            _servo(1, 40);
            
            bw(20, 20, 1.5, 10, "none_line");  //-------------->> ถอยหลังย้ายมือ

            servo(28, servo_28_close-70); 
             _servo(1, 20); delay(300);
            _servo(0, 10); delay(200);
            servo(28, servo_28_close); 
            delay(500);
            //robot_start(); 
            servo(27, servo_27_close-140); 
            delay(200);
            _servo(1, 60); delay(200);
            _servo(0, 60); delay(200);

            servo(27, servo_27_close); 
            _servo(0, 85);     //----------->> กางแขนทั้ง2 ออกไป 85 องศา
            _servo(1, 85);
            arm_Slide(200);
            fw_distance(10, 10, 1.5, 2500) ;
            bw(10, 10, 1.5, 1.5, "none_line"); 
            delay(300);
            arm_Slide(-200);
            shake_servo_R(80, 5);
            mission_left0 = 1;     //----------->> 
            

            bw(20, 20, 1.5, 10, "none_line");    //------>> ถอยหลังออกมา
            servo(28, servo_28_close); 
            servo(27, servo_27_close-100); 
            _servo(0, 30);
            _servo(1, 170);
            delay(300);
            arm_Slide(-400);
            fw_distance(10, 10, 1.5, 2500) ;
            delay(300);
            servo(27, servo_27_close);
            delay(300);
            arm_Slide(400);
            delay(300);
            bw(10, 10, 1.5, 2, "none_line"); 
            delay(300);
            shake_servo_L(85, 5);   
            mission_right0 = 1;       

          }
      }
  }

void arm_left()
  {
    if(mission_left0 == 0)
      {      
        if(color_left == stsnd_1[0])
          {
            
            
          }
        else if(color_right == stsnd_1[1])
          {
  
          }
        else  
          {
            fw_distance(10, 10, 1.5, 2500) ;
            delay(200);
            bw(10, 10, 1.5, 2, "none_line"); 
            delay(300);
            shake_servo_L(80, 5);
          }
      }
    else{}
  }