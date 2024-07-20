
#include <my_rp_2wd.h>

void setup() 
  {
    Serial.begin(9600);
    sensor_set();
    mydisplay("MY-MAKERS", 20, 30, 2, white);

    sw();
    

  }

void loop() 
  {
    Motor(100,100);
    
  }
