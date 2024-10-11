
void encoder_poss()
  {    
    Motor(30, 30); delay(1000);
    Motor(-30, -30); delay(1000);
    Motor(0, 0); delay(1000);
    while(1)
      {
        Serial.print("Encoder_L: ");
        Serial.print(encoder.Poss_L());
        Serial.print("     ");
        Serial.print("Encoder_R: ");
        Serial.println(encoder.Poss_R());
    
        delay(10);  // Delay for better readability in serial out
      }
  }
