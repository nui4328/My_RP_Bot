void fw(int spl, int spr,int tim, char slr, int set_bls)
  {
    
     if(tim > 0)
      {
        lasts_time = millis();
        while(millis() - lasts_time < tim)
          {
            Motor(spl, spr);
            if(mcp_f(1)<md_sensors(1)-150 ||mcp_f(2)<md_sensors(2)-150 )
              {
                Motor(-spl, -spr);delay(50);
                Motor(0, 0);delay(100);
                Motor(-40, -40);delay(110);
                Motor(0, 0);delay(100);
                set_bl(set_bls);
                goto end_set_bl;

              }
            if(mcp_f(0)<md_sensors(0)) //------------->> มีเส้นด้านข้าง
              {
                do{Motor(spl, spr/2);}while(mcp_f(0)<md_sensors(0));
                delay(20);
                break;
              }
            if(mcp_f(3)<md_sensors(3)) //------------->> มีเส้นด้านข้าง
              {
                do{Motor(spl/2, spr);}while(mcp_f(0)<md_sensors(0));
                delay(20);
                break;
              }
            
          }
        Motor(0, 0 );delay(20);
      }
    else   //--------------->> กำหนดเวลาเป็น 0
      {   
        while(1)
          {     
            Motor(spl, spr);
            if(mcp_f(1)<md_sensors(1)-150 ||mcp_f(2)<md_sensors(2)-150 )
              {
                Motor(-spl, -spr);delay(50);
                Motor(0, 0);delay(100);
                Motor(-40, -40);delay(110);
                Motor(0, 0);delay(100);
                set_bl(set_bls);
                goto end_set_bl;

              }
            if(mcp_f(0)<md_sensors(0)) //------------->> มีเส้นด้านข้าง
              {
                do{Motor(spl, spr/2);}while(mcp_f(0)<md_sensors(0));
                delay(20);
                break;
              }
            if(mcp_f(3)<md_sensors(3)) //------------->> มีเส้นด้านข้าง
              {
                do{Motor(spl/2, spr);}while(mcp_f(0)<md_sensors(0));
                delay(20);
                break;
              }
          }
      } 
    end_set_bl:  

    Motor(-50,-50);delay(70);
    Motor(40,40);delay(10);
    Motor(0,0);delay(150);
    if(slr == 'r')
      {

            Motor(70,-70);delay(450);
            Motor(-60,60);delay(50);
            Motor(0,0);delay(100);
 
      }
    else if(slr == 'l')
      {

            Motor(-70,70);delay(450);
            Motor(60,-60);delay(50);
            Motor(0,0);delay(100);

         
      }
     else
      {
        Motor(0,0);delay(20);
      }
  }

void bw(int spl, int spr,int tim, char slr, int set_bls)
  {
    if(tim > 0)
      {
        lasts_time = millis();
        while(millis() - lasts_time < tim)
          {
            Motor(-spl, -spr);
            if(mcp_f(5)<md_sensors(5)-150 ||mcp_f(6)<md_sensors(6)-150 )
              {
                Motor(spl, spr);delay(50);
                Motor(0, 0);delay(100);
                Motor(40, 40);delay(110);
                Motor(0, 0);delay(100);
                set_bl_b(set_bls);
                goto end_set_bl;

              }
            if(mcp_f(4)<md_sensors(4)) //------------->> มีเส้นด้านข้าง
              {
                do{Motor(-spl, -spr/2);}while(mcp_f(4)<md_sensors(4));
                delay(20);
                break;
              }
            if(mcp_f(7)<md_sensors(7)) //------------->> มีเส้นด้านข้าง
              {
                do{Motor(-spl/2, -spr);}while(mcp_f(7)<md_sensors(7));
                delay(20);
                break;
              }
            
          }
        Motor(0, 0 );delay(20);
      }
    else   //--------------->> กำหนดเวลาเป็น 0
      {   
        while(1)
           {
            Motor(-spl, -spr);
            if(mcp_f(5)<md_sensors(5)-150 ||mcp_f(6)<md_sensors(6)-150 )
              {
                Motor(spl, spr);delay(50);
                Motor(0, 0);delay(100);
                Motor(40, 40);delay(110);
                Motor(0, 0);delay(100);
                set_bl_b(set_bls);
                goto end_set_bl;

              }
            if(mcp_f(4)<md_sensors(4)) //------------->> มีเส้นด้านข้าง
              {
                do{Motor(-spl, -spr/2);}while(mcp_f(4)<md_sensors(4));
                delay(20);
                break;
              }
            if(mcp_f(7)<md_sensors(7)) //------------->> มีเส้นด้านข้าง
              {
                do{Motor(-spl/2, -spr);}while(mcp_f(7)<md_sensors(7));
                delay(20);
                break;
              }
            
          }
        Motor(0, 0 );delay(20);
      } 
    end_set_bl:  

    Motor(50,50);delay(70);
    Motor(-40,-40);delay(10);
    Motor(0,0);delay(150);
    if(slr == 'r')
      {

            Motor(70,-70);delay(450);
            Motor(-60,60);delay(50);
            Motor(0,0);delay(100);
 
      }
    else if(slr == 'l')
      {

            Motor(-70,70);delay(450);
            Motor(60,-60);delay(50);
            Motor(0,0);delay(100);

         
      }
     else
      {
        Motor(0,0);delay(20);
      }
  }


void set_bl(int num)
  {
    for(int i=0; i< num; i++)
      {
        while(1)
          {
            if(mcp_f(1)<md_sensors(1))
              {
                do{Motor(-15,55);}while(mcp_f(2)>md_sensors(2)-50);
                Motor(15,-35);delay(10);
                break;
              }
            else if(mcp_f(2)<md_sensors(2))
              {
                do{Motor(55,-15);}while(mcp_f(1)>md_sensors(1)-50);
                Motor(-35,15);delay(10);
                break;
              }
            else
              {
                Motor(30,30);
              }
          }
         Motor(-40,-40);delay(150);
         Motor(0,0);delay(10);
      }
   
  }
void set_bl_b(int num)
  {
    for(int i=0; i< num; i++)
      {
        while(1)
          {
            if(mcp_f(5)<md_sensors(5))
              {
                do{Motor(15,-55);}while(mcp_f(6)>md_sensors(6)-50);
                Motor(-15,35);delay(10);
                break;
              }
            else if(mcp_f(6)<md_sensors(6)-150)
              {
                do{Motor(-55,15);}while(mcp_f(5)>md_sensors(5)-50);
                Motor(35,-15);delay(10);
                break;
              }
            else
              {
                Motor(-50,-50);
              }
          }
         Motor(40,40);delay(150);
         Motor(0,0);delay(10);
      }
 
  }

void set_bl_box()
  {
 
        while(1)
          {
            if(mcp_f(1)<md_sensors(1))
              {
                do{Motor(-15,55);}while(mcp_f(2)>md_sensors(2)-50);
                Motor(15,-35);delay(10);
                break;
              }
            else if(mcp_f(2)<md_sensors(2))
              {
                do{Motor(55,-15);}while(mcp_f(1)>md_sensors(1)-50);
                Motor(-35,15);delay(10);
                break;
              }
            else
              {
                Motor(45,45);
              }
          }
         Motor(-40,-40);delay(10);
         Motor(0,0);delay(10);
         
  }
void turn_left()
  {
    Motor(-70,70);delay(tl);
    Motor(60,-60);delay(50);
    Motor(0,0);delay(100);
  }
void turn_right()
  {
    Motor(70,-70);delay(tr);
    Motor(-60,60);delay(50);
    Motor(0,0);delay(100);
  }
