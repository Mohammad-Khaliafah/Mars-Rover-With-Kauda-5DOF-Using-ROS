// Include iBusBM Library
#include <IBusBM.h>
 
#include <Servo.h>

// Create iBus Object
IBusBM ibus;
 
// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = ibus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}
 
// Read the channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}
//FS channel list
int rcCH1 = 0; // Available
int rcCH2 = 0; // Forward - Backward
int rcCH3 = 0; // Available
int rcCH4 = 0; // Right\Left
int rcCH5 = 0; // Speed
int rcCH6 = 0; // Available
bool rcCH7 = 0; // Mode Control
bool rcCH8 = 0; // Available
bool rcCH9 = 0; // Available
bool rcCH10 = 0; // Lock

float d1 = 271; // distance in mm
float d2 = 278;
float d3 = 301;
float d4 = 304;

float thetaInnerFront = 0;
float thetaInnerBack = 0;
float thetaOuterFront = 0;
float thetaOuterBack = 0;

int r = 0;
int s =255;
float speed1, speed2, speed3 = 0;
float speed1PWM, speed2PWM, speed3PWM = 0;

Servo servoW1;  // front right 
Servo servoW3;  // back right
Servo servoW4;  // front left
Servo servoW6;  // back left





//Right motor driver pins
int EN_right = 2; 
int R_PWM_right = 6; //PWM pin
int L_PWM_right = 7; //PWM pin

//Left motor driver pins
int EN_left = 5; 
int R_PWM_left = 12; //PWM pin
int L_PWM_left = 13; //PWM pin




void setup() {
  // put your setup code here, to run once:

// Start serial monitor
  Serial.begin(115200);
 
  // Attach iBus object to serial port
  ibus.begin(Serial1);

   servoW1.attach(9); 
   servoW3.attach(11);
   servoW4.attach(3);
   servoW6.attach(8);
// old pins
  // front right servo 0 w4
  // front left servo 1  w6
  // back right servo 2  w1 back left 
  // back left  servo 3  w3
// new pins
  //w1 front right 12  how T M >>> Wheel w1 - Front Left >> w4
  //w3 back right 13   how T M >>> Wheel w3 - Back Left >> w6
  //w4 front left 5    how T M >>> Wheel w4 - Front Right >> w1
  //w6 back left 4     how T M >>> Wheel w6 - Back Right >> w3

  servoW1.write(90);
  servoW3.write(90);
  servoW4.write(90);
  servoW6.write(90);
    pinMode(EN_right,OUTPUT);
    pinMode(R_PWM_right,OUTPUT);
    pinMode(L_PWM_right,OUTPUT);

    //Left motor driver pin def
    pinMode(EN_left,OUTPUT);
    pinMode(R_PWM_left,OUTPUT);
    pinMode(L_PWM_left,OUTPUT);



}

void loop() {
  // put your main code here, to run repeatedly:

  
    // Get RC channel valuesstopMotor
 // rcCH1 = readChannel(0, -100, 100, 0);
  rcCH2 = readChannel(1, -100, 100, 0);
 // rcCH3 = readChannel(2, 0, 155, 0);
  rcCH4 = readChannel(3, -100, 100, 0);
  rcCH5 = readChannel(4, -100, 100, 0);
 // rcCH6 = readChannel(5, -100, 100, 0);
  rcCH7 = readSwitch(6, false);
 // rcCH8 = readSwitch(7, false);
 // rcCH9 = readSwitch(8, false);
  rcCH10 = readSwitch(9, false);
  delay (50);
 // Serial.println(rcCH4);

//void stopMotor();
  servoW1.write(90);
  servoW3.write(90);
  servoW4.write(90);
  servoW6.write(90);
  
  
  stopMotor();


if(rcCH10==1 && rcCH7 ==0 ){

if (rcCH4 > 4) {
    r = map(rcCH4, 0, 100, 1400, 600);  // turning radius from 1400mm to 600mm
    calculateServoAngle();
 // Servo motors

 
    // Outer wheels
    servoW1.write(94 + thetaOuterFront);
    servoW3.write(96 - thetaOuterBack);
    // Inner wheels
    servoW4.write(97 + thetaInnerFront); // front wheel steer right
    servoW6.write(97 - thetaInnerBack); // back wheel steer left for overall steering to the right of the rover


// DC Motors
    if (rcCH2 > 0) { 

  MoveForward();
    }else if (rcCH2 < 0) { 
  MoveBackward();
    }
}else if (rcCH4 < -4) {
    r = map(rcCH4, 0, -100, 1400, 600); // turning radius from 600mm to 1400mm
    calculateServoAngle();
  // Servo motors
    servoW1.write(94 - thetaInnerFront);
    servoW3.write(96 + thetaInnerBack);
    servoW4.write(97 - thetaOuterFront);
    servoW6.write(97 + thetaOuterBack);
    Serial.println(thetaOuterFront);

    if (rcCH2 > 0 && rcCH10==1) { 

  MoveForward();
    }else if (rcCH2 < 0) { 
  MoveBackward();
    }


}else{

 if (rcCH2 > 0) { 

  MoveForward();
    }else if (rcCH2 < 0) { 
  MoveBackward();
    }

}
}
}
void stopMotor(){
  

//right side
  digitalWrite(EN_right, 1 );
  analogWrite(R_PWM_right, 0);
  analogWrite(L_PWM_right, 0);

//left side
  digitalWrite(EN_left, 1 );
  analogWrite(R_PWM_left, 0);
  analogWrite(L_PWM_left, 0);


 }
void MoveForward(){

 //right side
  digitalWrite(EN_right, 1 );
  analogWrite(R_PWM_right, 255);
  analogWrite(L_PWM_right, 0);
 //left side
  digitalWrite(EN_left, 1 );
  analogWrite(R_PWM_left, 0);
  analogWrite(L_PWM_left, 255);

}
void MoveBackward(){

 //right side
  digitalWrite(EN_right, 1 );
  analogWrite(R_PWM_right, 0);
  analogWrite(L_PWM_right, 255);
 //left side
  digitalWrite(EN_left, 1 );
  analogWrite(R_PWM_left, 255);
  analogWrite(L_PWM_left, 0);

}void calculateServoAngle() {
  // Calculate the angle for each servo for the input turning radius "r"
  thetaInnerFront = round((atan((d3 / (r + d1)))) * 180 / PI); // thetaInnerFront r19--- l42
  thetaInnerBack = round((atan((d2 / (r + d1)))) * 180 / PI); //thetaInnerBack r18   --- l42
  thetaOuterFront = round((atan((d3 / (r - d1)))) * 180 / PI); //thetaOuterFront r 42  --- l42
  thetaOuterBack = round((atan((d2 / (r - d1)))) * 180 / PI);  //thetaOuterBack r40  --- l42
    
      Serial.println("thetaInnerFront= ");
      Serial.print(thetaInnerFront);
      Serial.println("    ");
/*
      Serial.println("thetaInnerBack= ");
      Serial.print(thetaInnerBack);
      Serial.println("    ");

      Serial.println("thetaOuterFront= ");
      Serial.print(thetaOuterFront);
      Serial.println("    ");

      Serial.println("thetaOuterBack= ");
      Serial.print(thetaOuterBack);
      Serial.println("    ");
*/




}
