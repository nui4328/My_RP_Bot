void box1()
  {
      arm_ready();
      fline(80, 80, 0.35, 30, 'n', 's', 100, "a1", 0);  //
      fline(10, 10, 0.35, 0, 'f', 's', 100, "a1", 10);  
      arm_down_close(); delay(300);


      bline(50, 50, 0.35, 0, 'f', 's', 100, "a4", 20);  


      place_left_in(30, 95, 30) ;
      fline(15, 15, 0.35, 0, 'f', 's', 100, "a4", 20);  
      arm_down_open();delay(400);
      bline(30, 30, 0.35, 0, 'c', 'r', 60, "a5", 20);  

  }

void box2()
  {
     
      arm_ready();
      fline(80, 80, 0.35, 35, 'n', 's', 100, "a1", 0);
      arm_down_close();
      fline(20, 20, 0.35, 0, 'f', 'l', 100, "a1", 2);  
      fline(60, 60, 0.35, 15, 'n', 's', 100, "a1", 10); 
      place_left_in(30, 95, 30) ;
      arm_down_open();delay(200);

      fline(20, 20, 0.35, 0, 'f', 's', 100, "a4", 20);  
      bline(35, 35, 0.35, 0, 'c', 'r', 60, "a5", 20);  

  }

void box3()
  {
      arm_ready();
      fline(80, 80, 0.35, 35, 'n', 's', 100, "a1", 0);
      arm_down_close();
      fline(20, 20, 0.35, 0, 'c', 'r', 70, "b3", 10);  
      
      bline(40, 40, 0.35, 5, 'n', 's', 100, "a1", 30); 
      place_right_in(60, 85, 30) ;
      
      arm_open_min_up();delay(200);

      fline(20, 20, 0.35, 0, 'f', 's', 100, "a4", 20); 
      bline(35, 35, 0.35, 0, 'c', 'l', 60, "a2", 20);  

      arm_down_open();
  }
