
#include <Wire.h>
#include <my_GYRO.h>
#include "my_MCP3008.h"
my_MCP3008 adc;

#include <EncoderLibrary.h>
EncoderLibrary encoder(6, 7, 15, 20);


// กำหนดพินควบคุมมอเตอร์


#define PWMA 6     // PWM ซ้าย
#define AIN1 22
#define AIN2 23

#define PWMB 3     // PWM ขวา
#define BIN1 21
#define BIN2 20
//___--------------------------------------------->>
#define EEPROM_ADDRESS 0x50 // ที่อยู่ I2C ของ CAT24C256
const int numSensors = 8;
const int numSamples = 1000;
int sensorValuesA[numSensors][numSamples]; // เก็บข้อมูลเซนเซอร์จาก read_sensorA
int sensorMaxA[numSensors];
int sensorMinA[numSensors];
int sensorValuesB[numSensors][numSamples]; // เก็บข้อมูลเซนเซอร์จาก read_sensorB
int sensorMaxB[numSensors];
int sensorMinB[numSensors];
int sensorValuesC[2][numSamples]; // เก็บข้อมูลจาก analogRead(46) และ analogRead(47)
int sensorMaxC[2]; // Max สำหรับพิน 46 และ 47
int sensorMinC[2]; // Min สำหรับพิน 46 และ 47
int sensorMax_A[numSensors];
int sensorMin_A[numSensors];
int sensorMax_B[numSensors];
int sensorMin_B[numSensors];
int sensorMax_C[2]; // Max สำหรับพิน 26 และ 27
int sensorMin_C[2]; // Min สำหรับพิน 26 และ 27
float P, I, D, previous_I, previous_error,errors, PID_output, present_position, previous_integral; 
int numSensor = 6; 
int state_on_Line = 0;
int setpoint;
int _lastPosition;
int sensor_pin_A[] = {1,2,3,4,5,6}; 
int sensor_pin_B[] = {1,2,3,4,5,6}; 
const float I_MAX = 1000.0; // ขีดจำกัดบนของ integral
const float I_MIN = -1000.0; // ขีดจำกัดล่างของ integral

//-------------------------------------------------------->>fline
int rgb[] = {24, 25, 28};
bool pid_error = true;

//___--------------------------------------------->>
void setup() {
  Wire.begin();
  Serial.begin(115200);
  my_GYRO::begin();
  my_GYRO::resetAngles();
  pinMode(24, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(32, OUTPUT);

  pinMode(33, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  for(int i = 0; i<2; i++)
    {
      for(int i = 0; i<3; i++)
        {
          digitalWrite(rgb[i],1);
          delay(150);
          digitalWrite(rgb[i],0);
          delay(150);
        }
    }
  digitalWrite(rgb[2],1);

  
   
 // arm_down_open();
   get_EEP_Program();
   read_sensorA_program();
   sw(); 

   //place_left_in(30, 45, 30) ;
   //place_right_in(30, 250);
  
  ///-------------------------------------------------------------------->>>>
   Motor(50, 50);
  /*
    box_1();
    box_2();
    box_3();
    box_4();
    box_5();
   
    box_6();

  */
  ///-------------------------------------------------------------------->>>>
  ///-------------------------------------------------------------------->>>>
  


}

void loop() {
   Serial.print( analogRead(46) );   Serial.print( "   " );   Serial.println( analogRead(47) ); 
    //Serial.print( "   " );
  //Serial.print( md_sensorC(0) );   Serial.print( "   " );   Serial.println( md_sensorC(1) );  
  //Serial.print(digitalRead(19));Serial.println("  ");
  //Serial.print( position_B()); Serial.print("  ");  Serial.println(error_B());
  //Serial.println(my.gyro('z'));   
  delay(10);

}

