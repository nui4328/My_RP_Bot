void set_motor()
  {
    set_Freq("Coreless_Motors");      // กำหนดความถี่ให้กับมอเตอร์เกาหลีหรือ มอเตอร์ Coreless ("Coreless_Motors")
      //set_Freq("DC_Motors");        // กำหนดความถี่ให้กับมอเตอร์ธรรมกา(DC_Motors)

    distance_scale_fw(1.00);       // ปรับค่าเพื่อให้ได้ระยะทางที่เป็นจริงสำหรับเดินหน้า
    distance_scale_bw(1.10);        // ปรับค่าเพื่อให้ได้ระยะทางที่เป็นจริงสำหรับถอยหลัง
   
    set_slow_motor(20, 20);     
    set_turn_center_l(-90, 90);
    set_turn_center_r(90, -90);
    set_turn_front_l(-15, 100);
    set_turn_front_r(100, -15);
    set_brake_fc(5, 20);
    set_brake_bc(5, 20);
    set_delay_f(10);
    
  }
void fw_ch_line(int num)
  {
    for(int i=0; i< num; i++)
      {
        while(1)
          {
            delay(5);      
            if(read_sensorA(0) < md_sensorA(0)-50 && read_sensorA(7) > md_sensorA(7)-50)
              {
                Motor(-5 ,20);
              }
            else if(read_sensorA(0) > md_sensorA(0)-50 && read_sensorA(7) < md_sensorA(7)-50)
              {
                Motor(20 ,-5);
              }
            else if(read_sensorA(0) > md_sensorA(0)-50 && read_sensorA(7) > md_sensorA(7)-50)
              {          
                Motor(15 ,15);
              }
            else 
              {
                Motor(-1 ,-1);
                break;
              }      
          }
        if(num > 1)
          {
            Motor(-15 ,-15);
            delay(50);
            Motor(-1 ,-1);
          }
      }
    
  }

void bw_ch_line(int num)
  {
    for(int i=0; i< num; i++)
      {
        while(1)
          {
            delay(5);      
            if(read_sensorB(0) < md_sensorB(0)-50 && read_sensorB(7) > md_sensorB(7)-50)
              {
                Motor(-20 ,5);
              }
            else if(read_sensorB(0) > md_sensorB(0)-50 && read_sensorB(7) < md_sensorB(7)-50)
              {
                Motor(5 ,-20);
              }
            else if(read_sensorB(0) > md_sensorB(0)-50 && read_sensorB(7) > md_sensorB(7)-50)
              {          
                Motor(-15 ,-20);
              }
            else 
              {
                Motor(-1 ,-1);
                break;
              }      
          }
        if(num > 1)
          {
            Motor(10 ,10);
            delay(50);
            Motor(-1 ,-1);
          }
      }
    
  }
