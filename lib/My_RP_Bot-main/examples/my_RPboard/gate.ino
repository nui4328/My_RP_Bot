void gate()
{
bz(200);bz(200);
testdrawtext("0.00", 40, 30, 3, white, black);
 while(analogRead(26)<2500)
    {      
      Serial.println(analogRead(26));
      
    }
 bz(200);
 lasts_time = millis();
 delay(200);
 while(1)
    {
      unsigned long elapsedTime = millis() - lasts_time;
    float seconds = elapsedTime / 1000.0;
    Serial.print("Elapsed time: ");
    Serial.print(seconds);
    Serial.println(" seconds");
    String num = String(seconds);
    testdrawtext(num, 40, 30, 3, white, black);
    delay(10);
       if(analogRead(26)>2700)
        {
          break;
        }
    }
  bz(100);
  bz(100);
  delay(300);
  bz(300);
 }
