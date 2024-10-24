void _bz(int _time)
  {
    pinMode(21, OUTPUT);
    digitalWrite(21, 1);
    delay(_time);
    digitalWrite(21, 0 );
    delay(_time);
  }
void _sw()
  {
    _bz(100);
    _bz(100);
    

    while(digitalRead(9)==1 )
      {
        if(analogRead(29) > 4000)
           {
            cal_censor(0, 0);
           }
        String mcp_f0 = String(mcp_f(0));
        String mcp_f1 = String(mcp_f(1));
        String mcp_f2 = String(mcp_f(2));
        String mcp_f3 = String(mcp_f(3));
        String mcp_f4 = String(mcp_f(4));
        String mcp_f5 = String(mcp_f(5));
        String mcp_f6 = String(mcp_f(6));
        String mcp_f7 = String(mcp_f(7));
        String _clock = String(millis());
        String knob = String(analogRead(29));
        mydisplay_background(black);
        mydisplay(mcp_f0 +"  "+ mcp_f1 +"  "+ mcp_f2+"  "+ mcp_f3, 5, 10 ,1, white);
        mydisplay(mcp_f4 +"  "+ mcp_f5 +"  "+ mcp_f6+"  "+ mcp_f7, 5, 30 ,1, white);
        mydisplay(knob, 5, 50 ,2, white);
        for(int i = 0; i<6; i++)
          {
            Serial.print(mcp_f(i));
            Serial.print("  ");
            delay(10);
          }
        Serial.println(" ");
      }
    mydisplay_background(black);
    mydisplay("End", 5, 10 ,2, white);

    _bz(500);
  }
void _sw_md()
  {
    _bz(100);
    _bz(100);
    

    while(digitalRead(9)==1 )
      {
        String mcp_f0 = String(md_mcp_f(0));
        String mcp_f1 = String(md_mcp_f(1));
        String mcp_f2 = String(md_mcp_f(2));
        String mcp_f3 = String(md_mcp_f(3));
        String mcp_f4 = String(md_mcp_f(4));
        String mcp_f5 = String(md_mcp_f(5));
        String _clock = String(millis());
        mydisplay_background(black);
        mydisplay(mcp_f0 +" "+ mcp_f1 +" "+ mcp_f2, 5, 10 ,2, white);
        mydisplay(mcp_f3 +" "+ mcp_f4 +" "+ mcp_f5, 5, 30 ,2, white);
        mydisplay(_clock, 5, 50 ,2, white);
        for(int i = 0; i<6; i++)
          {
            Serial.print(mcp_f(i));
            Serial.print("  ");
            delay(10);
          }
        Serial.println(" ");
      }
    mydisplay_background(black);
    mydisplay("End", 5, 10 ,2, white);

    _bz(500);
  }

void _en()
  {
    _bz(100);
    _bz(100);
    

    while(digitalRead(9)==1 )
      {
        mydisplay_background(black);
        mydisplay( "Mission Completed" , 10, 20 ,1, white);
        mydisplay( "MY MAKERS", 10, 50 ,2, white);
      }
    _bz(500);
  }
