void example()
  {
     arm_ready();
  fline(70,70,0.28,22,'n','s',80, "a1", 40);
  arm_down_close();
  fline(20,20,0.28,0,'f','l',80, "a2", 0);
  fline(20,20,0.28,0,'f','r',80, "a5", 0);
  fline(70,70,0.28,25,'f','r',80, "a5", 0);     
  fline(20,20,0.28,0,'f','r',80, "a5", 0);
  fline(90,90,0.38,55,'f','s',80, "a1", 2);
  place_letf_in(30, 50);delay(100);
  arm_down_open();
  place_letf_out(20, 50);

  bline(60,60,0.38,0,'c','l',50, "a1", 50);
  arm_ready();
  fline(70,70,0.28,0,'f','s',80, "a1", 2);
  fline(20,20,0.28,18,'n','s',80, "a1", 20);
  arm_down_close(); delay(200);

  bline(70,70,0.28,65,'c','r',50, "a6", 50);
  fline(70,70,0.28,15,'n','s',80, "a1", 20);
  place_letf_in(30, 50);delay(100);
  arm_down_open();
  place_letf_out(20, 50);
  fline(20,20,0.28,0,'f','p',80, "a6", 30);delay(200);
  
  bline(60,60,0.38,0,'c','r',50, "a6", 1);
  bline(0,0,0.38,0,'n','r',50, "a6", 50);
  arm_ready();
  fline(70,70,0.38,0,'f','s',80, "a1", 2);
  fline(20,20,0.28,18,'n','s',80, "a1", 20);
  arm_down_close(); delay(100);

  bline(0,0,0.38,0,'c','r',50, "a6", 1);
  bline(0,0,0.38,0,'n','r',50, "a6", 70);

  fline(80,80,0.38,50,'n','s',80, "a1", 20);
  place_letf_in(20, 50); delay(100);
  arm_down_open();
  place_letf_out(20, 40);
  
  fline(10,10,0.28,0,'f','p',80, "a6", 30);

  bline(70,70,0.28,70,'c','l',50, "a1", 50);
  arm_ready();
  fline(70,70,0.38,40,'n','s',50, "a6", 30);
  arm_down_close();
  fline(20,20,0.28,0,'f','r',80, "a5", 0);
  fline(70,70,0.28,65,'f','r',50, "a5", 0);
  fline(50,50,0.38,2,'n','s',50, "a6", 30);delay(200);
  place_letf_in(20, 40); delay(100);
  arm_down_open();
  place_letf_out(20, 35);
  
  fline(10,10,0.28,20,'n','s',80, "a6", 30);
  }
void test_bord()
  {
    
    for(int i=0; i<3; i++)
      {
        servo(23, 160); servo(8, 160); servo(7, 160);delay(1000);
        servo(23, 10);servo(8, 10); servo(7, 10); delay(1000);
      }
    bz(200);
  
    encoder.resetEncoders();
    while(1)
      {
        Serial.print("Encoder_L: ");
        Serial.print(encoder.Poss_L());
        Serial.print("     ");
        Serial.print("Encoder_R: ");
        Serial.println(encoder.Poss_R());
    
        delay(10);  // Delay for better readability in serial out
        if(digitalRead(9) == 0)
          {
            break;
          }
      }
    bz(200);
    while(digitalRead(9)== 1)
      {
        Motor(30, 30);delay(1000);
        Motor(-30, -30);delay(1000);
        Motor(0, 0);delay(1000);
        
      }
    bz(200);
    /*
    while(1)
      {
        Serial.print(my_tcs('r'));
        Serial.print("  ");
        Serial.print(my_tcs('g'));
        Serial.print("  ");
        Serial.println(my_tcs('b'));
    
        delay(10);  // Delay for better readability in serial out
        if(digitalRead(9) == 0)
          {
            break;
          }
      }
    bz(200);
    */
  }
  
void encoder_poss()
  {    
    Serial.print("Encoder_L: ");
    Serial.print(encoder.Poss_L());
    Serial.print("     ");
    Serial.print("Encoder_R: ");
    Serial.println(encoder.Poss_R());

    delay(100);  // Delay for better readability in serial out
  }
