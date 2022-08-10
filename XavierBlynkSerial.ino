 //Code to restrick the motor to certain angle with readings from 2 chanel encoder.
 // (PPR/360)*Angle
//2 Encoders to be used, one is the inbuilt and other is the axle, now let's consider the output values to be counter1 and counter2 respectively.
//now (counter1 + counter2)/2 = counter<< this is the value we'll use.  Now |counter1 - counter2| <0; output error to check and calibrate the encoders.
//Ultrasonics to be used aswell for obstacle detection.

//Currently the said angle when acheived the steering will start steering the opposite direction very slowly untill joystick is engaged.
//-140 is the limit for Right turn.

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
  pinMode(outputA,INPUT);
  pinMode(outputB,INPUT);
  pinMode(Accel,OUTPUT);
  pinMode(Engine,OUTPUT);
}

BLYNK_APP_DISCONNECTED() {
  MotorSpeed1 = 0;
  MotorSpeed2 = 0;
  analogWrite(RPWM, MotorSpeed1);
  analogWrite(LPWM, MotorSpeed2);
  digitalWrite(Accel, LOW);
  digitalWrite(Engine, LOW);
}

BLYNK_WRITE(V6)  {  // Read the Joystick X and Y positions
  int joyposHorz = param[0].asInt();
  int joyposVert = param[1].asInt();
  if (joyposHorz < 460)  {  // Move Left
    // As we are going left we need to reverse readings
    joyposHorz = joyposHorz - 460; // This produces a negative number
    joyposHorz = joyposHorz * -1;  // Make the number positive

    // Map the number to a value of 1023 maximum
    joyposHorz = map(joyposHorz, 0, 460, 0, 255);

    MotorSpeed1 = MotorSpeed1 - joyposHorz;
    MotorSpeed2 = MotorSpeed2 + joyposHorz;

    // Don't exceed range of 0-1023 for motor speeds
    if (MotorSpeed1 < 0)MotorSpeed1 = 0;
    if (MotorSpeed2 > 1023)MotorSpeed2 = 255;

  }  else if (joyposHorz > 564)  {  // Move Right
    // Map the number to a value of 1023 maximum
    joyposHorz = map(joyposHorz, 564, 1023, 0, 255);

    MotorSpeed1 = MotorSpeed1 + joyposHorz;
    MotorSpeed2 = MotorSpeed2 - joyposHorz;

    // Don't exceed range of 0-1023 for motor speeds
    if (MotorSpeed1 > 1023)MotorSpeed1 = 255;
    if (MotorSpeed2 < 0)MotorSpeed2 = 0;
  }


  // Adjust to prevent "buzzing" at very low speed
  if (MotorSpeed1 < 8)MotorSpeed1 = 0;
  if (MotorSpeed2 < 8)MotorSpeed2 = 0;
  if (560 > joyposHorz > 461) {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
  }

  // Set the motor speeds
  analogWrite(RPWM, MotorSpeed1);
  analogWrite(LPWM, MotorSpeed2);
}

// Engine Control:
BLYNK_WRITE(2) {
  if (param.asInt() == 1) {
    digitalWrite(2, HIGH); // Engine ON
  } else {
    digitalWrite(2, LOW); // Engine OFF
  }
}
// Acceleration:
BLYNK_WRITE(Accel) {
  if (param.asInt() == 1) {
    analogWrite(Accel, 10); // Engine ON
  } else {
    analogWrite(Accel, 0); // Engine OFF
  }
}


BLYNK_WRITE(V7)  {  // Read the Joystick X and Y positions
  int joyposHorz = param[0].asInt();
  int joyposVert = param[1].asInt();

  if (joyposVert < 460)  {  // This is Backward
    // Set Motor A backward
    analogWrite(RPWM, MotorSpeed1);
    analogWrite(LPWM, MotorSpeed2);
   
    joyposVert = joyposVert - 460; // This produces a negative number
    joyposVert = joyposVert * -1;  // Make the number positive
    MotorSpeed1 = map(joyposVert, 0, 460, 0, 0);
    MotorSpeed2 = map(joyposVert, 0, 460, 0, 0);

  }  
}

 void encoder(){
  aState = digitalRead(outputA); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (aState != aLastState){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(outputB) != aState) { 
       counter ++;
     } else {
       counter --;
     }
     //Serial.print("Position: ");
     //Serial.println(counter);
   } 
   aLastState = aState; // Updates the previous state of the outputA with the current state

   if (counter >= 100){ //(1024/360)*30= 85 counts for 30 degree.
//    motor turns the other way.
//    if running clockwise after hiting threshold will turn counter-clockwise.
    analogWrite(RPWM, 10);
    analogWrite(LPWM, 0);
    digitalWrite(Engine, HIGH);
   }
   if (counter <= -100){
    //Motor turns the other way.
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 10);
    digitalWrite(Engine, LOW);
   }
 }

 void loop() {
   Blynk.run();
 }
