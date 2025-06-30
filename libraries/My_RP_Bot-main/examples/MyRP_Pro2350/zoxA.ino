void box_1()
  { 
      arm_down_open();
      fline(80, 80, 0.35, 25, 'f', 'r', 100, "a6", 2);
      arm_ready();
      fline(80, 80, 0.35, 10, 'f', 's', 100, "a2", 10);

      arm_down_close();delay(200);

      fline(0, 0, 0.35, 0, 'c', 'l', 50, "a3", 10);
      fline(80, 80, 0.35, 0, 'c', 'p', 45, "a3", 10); 
      fline(20, 20, 0.35, 0, 'f', 'l', 100, "a1", 2);
      fline(20, 20, 0.35, 0, 'f', 'r', 100, "a6", 2);
      place_left_in(40, 65, 30); 
      do{Motor(15 ,15);delayMicroseconds(50);}while(read_sensorA(3) > md_sensorA(3)); 
      delay(150);
      Motor(-20 ,-20);delay(30);
      Motor(-1 ,-1);delay(30);
      arm_open_min_up();

  }
void box_2()
  {
    arm_down_open();
    bline(15, 15, 0.0, 0, 'c', 'r', 50, "a3", 10); 
      fline(80, 80, 0.35, 0, 'c', 'p', 100, "a6", 0);  
      fline(20, 20, 0.0, 0, 'c', 'l', 50, "a1", 10);  
      fline(30, 30, 0.35, 0, 'c', 'l', 70, "a2", 7); 
      fline(60, 60, 0.35, 0, 'c', 'p', 100, "a6", 0); 
      arm_up_open();

      fline(30, 30, 0.35, 0, 'c', 'r', 50, "a4", 10); 
      arm_down_open();delay(200);
    Motor(10 ,10);delay(200);
     Motor(-10 ,-10);delay(30);
    Motor(-1 ,-1);delay(30);
    arm_down_close();delay(200);
    bline(30, 30, 0.0, 0, 'c', 'l', 80, "a1", 10);
      fline(80, 80, 0.35, 15, 'f', 'l', 100, "a1", 2); 
    fline(20, 20, 0.35, 0, 'f', 'r', 100, "a6", 2); 
      fline(50, 50, 0.35, 0, 'c', 'p', 45, "a3", 10); 
      fline(80, 80, 0.35, 30, 'f', 'l', 100, "a1", 2); 
      fline(80, 80, 0.55, 11, 'f', 'r', 100, "a6", 2); 
      fline(80, 80, 0.55, 0, 'c', 'p', 100, "a6", 2); 
    fline(20, 20, 0.35, 0, 'f', 'l', 100, "a1", 2); 
      fline(80, 80, 0.55, 13, 'f', 'l', 100, "a1", 2); 
      fline(50, 50, 0.35, 0, 'c', 'l', 50, "a3", 10); 
      fline(65, 65, 0.35, 15, 'c', 's', 100, "a6", 10); 
   place_left_in(30, 45, 30) ;
   arm_down_open();delay(400);
   place_left_out(30, 45, 30) ;
  }

void box_3()
  {
    
      arm_up_open();
    bline(50, 50, 0.35, 0, 'c', 'l', 40, "a3", 10);
    arm_down_open();delay(200);
    Motor(10 ,10);delay(200);
     Motor(-10 ,-10);delay(30);
    Motor(-1 ,-1);delay(30);
    arm_down_close();delay(200);
      bline(50, 50, 0.35, 0, 'c', 'p', 45, "a3", 0);
      bline(50, 50, 0.35, 0, 'c', 'l', 45, "a3", 10);
    fline(80, 80, 0.35, 13, 'f', 'l', 100, "a1", 2); 
      fline(80, 80, 0.55, 11, 'f', 'r', 100, "a6", 2);
      fline(80, 80, 0.35, 0, 'c', 'p', 45, "a3", 10); 
      fline(20, 20, 0.35, 0, 'f', 'l', 100, "a1", 2);
      fline(20, 20, 0.35, 0, 'f', 'r', 100, "a6", 2);
      fline(10, 10, 0.0, 25, 'n', 's', 100, "a6", 10);  
      place_left_in(40, 90, 30);  
      do{Motor(15 ,15);delayMicroseconds(50);}while(read_sensorA(3) > md_sensorA(3)); 
      delay(150);
      Motor(-20 ,-20);delay(30);
      Motor(-1 ,-1);delay(30);
      arm_open_min_up(); 
    
  }

void box_4()
  {
    arm_down_open();
    bline(15, 15, 0.0, 0, 'c', 'r', 50, "a3", 10); 
    fline(80, 80, 0.35, 15, 'f', 'r', 100, "a6", 2);  
    fline(80, 80, 0.35, 15, 'f', 'r', 100, "a6", 2); 
    fline(50, 50, 0.35, 20, 'n', 's', 100, "a6", 2);  
    arm_down_close();delay(200);
    bline(80, 80, 0.35, 15, 'c', 'r', 70, "a6", 2); 
    fline(80, 80, 0.35, 15, 'f', 'l', 100, "a1", 2); 
    fline(80, 80, 0.35, 35, 'f', 'l', 100, "a1", 2); 
    fline(20, 20, 0.35, 0, 'f', 'r', 100, "a6", 2); 
      fline(50, 50, 0.35, 0, 'c', 'p', 45, "a3", 10); 
      fline(80, 80, 0.35, 30, 'f', 'r', 100, "a6", 2); 
    fline(80, 80, 0.35, 15, 'f', 'l', 100, "a1", 2); 
    fline(70, 70, 0.35, 20, 'f', 's', 100, "a6", 2); 
    
   place_left_in(30, 45, 30) ;
   arm_down_open();delay(400);
   place_left_out(30, 45, 30) ;
  }

void box_5()
  { 
    bline(10, 10, 0.35, 5, 'n', 'r', 70, "a5", 10); 

     arm_down_open();
      fline(80, 80, 0.35, 15, 'f', 'r', 100, "a6", 2);
      fline(80, 80, 0.35, 0, 'c', 'p', 100, "a2", 0);
      fline(50, 50, 0.35, 20, 'c', 's', 100, "a2", 10);

      arm_down_close();
      
      fline(30, 30, 0.35, 10, 'f', 's', 100, "a2", 20);
      bline(70, 70, 0.35, 25, 'c', 'l', 70, "a1", 10);
      fline(80, 80, 0.35, 0, 'c', 'p', 45, "a3", 10); 
      fline(20, 20, 0.35, 0, 'f', 'l', 100, "a1", 2);
      fline(20, 20, 0.35, 0, 'f', 'r', 100, "a6", 2);
      fline(30, 30, 0.0, 45, 'n', 's', 100, "a6", 10);  
      place_left_in(40, 90, 30); 
      do{Motor(15 ,15);delayMicroseconds(50);}while(read_sensorA(3) > md_sensorA(3)); 
      delay(150);
      Motor(-20 ,-20);delay(30);
      Motor(-1 ,-1);delay(30);
      arm_open_min_up();
  }

void box_6()
  {
     bline(30, 30, 0.0, 0, 'c', 'l', 80, "a1", 10);
      fline(80, 80, 0.35, 15, 'f', 'l', 100, "a1", 2); 
    fline(20, 20, 0.35, 0, 'f', 'r', 100, "a6", 2); 
    fline(20, 20, 0.35, 0, 'f', 'l', 100, "a1", 2); 
    arm_ready_min();
      fline(50, 50, 0.35, 11, 'f', 's', 100, "a6", 10);
      arm_down_close();delay(200);
      bline(10, 10, 0.35, 5, 'n', 'r', 70, "a5", 10); 
      fline(20, 20, 0.35, 0, 'f', 'l', 100, "a1", 2); 
      fline(80, 80, 0.35, 30, 'f', 'r', 100, "a6", 2); 
    fline(80, 80, 0.35, 15, 'f', 'l', 100, "a1", 2); 
    fline(70, 70, 0.35, 20, 'f', 's', 100, "a6", 2); 
    
   place_left_in(30, 45, 30) ;
   arm_down_open();delay(400);
   place_left_out(30, 45, 30) ;

  }
