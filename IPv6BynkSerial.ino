/*
 * Code written by Dipanjan Maji
 * To receive values from Logitech G29 and actuate the Steering Wheel.
*/


#include <SoftwareSerial.h>
#include <BlynkSimpleStream.h> 
#define BLYNK_PRINT Serial
#define outputA 14
#define outputB 15

char auth[] = "4lWMcMZ9uf097dfVQ0rF9kl2hLUavEVx";
int counter = 0; 
int aState;
int aLastState;
int RPWM = 12;//BTS7960
int LPWM = 13;//BTS7960
int Accel = 4;//D2
int Engine = 2; // D4
int MotorSpeed1 = 0; //Steering
int MotorSpeed2 = 0; //Accelerometer
int joyposVert = 512;
int joyposHorz = 512; 

 void setup() {
  Serial.begin(9600); 
  Blynk.begin(Serial, auth);
   
  pinMode(RPWM,OUTPUT);
  pinMode(LPWM,OUTPUT);
  pinMode(Accel,OUTPUT);
}

BLYNK_APP_DISCONNECTED() {
  MotorSpeed1 = 0;
  MotorSpeed2 = 0;
  analogWrite(RPWM, MotorSpeed1);
  analogWrite(LPWM, MotorSpeed2);
  analogWrite(Accel, 0);
}

BLYNK_WRITE(V6)  {  // Read the Joystick X and Y positions
  int joyposHorz = param[0].asInt();
  if (joyposHorz < 460)  {   // Move Left
    int joyposHorzL = map(joyposHorz, 0, 460, 0, 255);
    analogWrite(RPWM, joyposHorzL);
    analogWrite(LPWM, 0);

  }
  if (joyposHorz > 564)  {  // Move Right
    // Map the number to a value of 1023 maximum
    int joyposHorzR = map(joyposHorz, 564, 1023, 0, 255);
    analogWrite(RPWM, 0);
    analogWrite(LPWM, joyposHorzR);
  }
  if (560 > joyposHorz > 461) {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
  }
}

// Acceleration:
BLYNK_WRITE(V1) {
  int acceleration = param[0].asInt();
  int accelerate = map(acceleeration, 0, 1023, 0, 255);
  analogWrite(Accel, accelerate);
  
}

 void loop() {
   Blynk.run();
 }
