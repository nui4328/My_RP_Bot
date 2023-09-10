
#include <my_rpboard.h>
#include <my_TCS34725.h> 
#include <my_mpu6050.h> 
String sensor;
unsigned long lasts_time = millis();


void setup() {
Serial.begin(9600);
sensor_set();
//setup_mpu();
//calibration_Yak();
      analogWriteResolution(10);
      analogWriteFreq( 1000);
      pinMode(10,OUTPUT);
      pinMode(11,OUTPUT);
      pinMode(6,OUTPUT);
      pinMode(7,OUTPUT);
      
      pinMode(18,OUTPUT);
      pinMode(19,OUTPUT);
      pinMode(20,OUTPUT);
      pinMode(21,OUTPUT);
testdrawtext("MY-MAKERS", 20, 30, 2, white, black);
while(digitalRead(9)==1)
  {    
  }

}

void loop() 
  {
            analogWrite(10, 1023);
      analogWrite(11, 0);

      analogWrite(6, 1023);
      analogWrite(7, 0);

      analogWrite(18, 1023);
      analogWrite(19, 0);

      analogWrite(20, 1023);
      analogWrite(21, 0);
      

      delay(1000);

      analogWrite(11, 1023);
      analogWrite(10, 0);

      analogWrite(7, 1023);
      analogWrite(6, 0);

      analogWrite(19, 1023);
      analogWrite(18, 0);

      analogWrite(21, 1023);
      analogWrite(20, 0);

      delay(1000);
 /*


     
     float color[3]= {my_tcs(0), my_tcs(1),my_tcs(2)};
     for(int i=0; i<3; i++)
      {
        Serial.print( color[i]);Serial.print( " : ");
      }
     Serial.println( "");
     delay(20);
     */
  }
