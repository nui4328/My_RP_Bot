void test_bord()
  {
    
    for(int i=0; i<3; i++)
      {
        servo(0, 60);servo(8, 150); servo(1, 10); Motor(-30, -30);delay(500);
        servo(0, 140); servo(8, 50); servo(1, 140);Motor(30, 30);delay(500);
        
      }
    bz(200);
    Motor(0, 0);
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
