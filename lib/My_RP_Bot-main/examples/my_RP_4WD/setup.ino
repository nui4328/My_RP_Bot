

void sw_start()
  {
    bz(100);
    bz(100);
     while(digitalRead(9)==1)
        { 
          String sen_0 = String (mcp_f(0));
          String sen_1 = String (mcp_f(1));
          String sen_2 = String (mcp_f(2));
          String sen_3 = String (mcp_f(3));
          String sen_4 = String (mcp_f(4));
          String sen_5 = String (mcp_f(5));
          String sen_6 = String (mcp_f(6));
          String sen_7 = String (mcp_f(7));  
         
          mydisplay_background(black);
          mydisplay(sen_0 +" "+ sen_1+" "+ sen_2, 10, 5, 2, white);
          mydisplay(sen_3 +" "+ sen_4+" "+ sen_5, 10, 30, 2, white);
          mydisplay(sen_6 +" "+ sen_7, 10, 45, 2, white);
          Serial.print(mcp_f(6));
          Serial.print("  ");
          Serial.println(md_mcp_f(6));        
            
        } 
     bz(300);
  }
