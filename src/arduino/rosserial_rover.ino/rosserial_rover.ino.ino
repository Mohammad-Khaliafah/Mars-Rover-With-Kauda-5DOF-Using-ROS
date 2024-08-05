// Include iBusBM Library
#include <IBusBM.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

// Create iBus Object
IBusBM ibus;

// ROS NodeHandle
ros::NodeHandle nh;

// ROS Messages
std_msgs::Int32 rcCH1_msg;
std_msgs::Int32 rcCH2_msg;
std_msgs::Int32 rcCH3_msg;
std_msgs::Int32 rcCH4_msg;
std_msgs::Int32 rcCH5_msg;
std_msgs::Int32 rcCH6_msg;
std_msgs::Bool rcCH7_msg;
std_msgs::Bool rcCH8_msg;
std_msgs::Bool rcCH10_msg;

// ROS Publishers
ros::Publisher rcCH1_pub("rcCH1", &rcCH1_msg);
ros::Publisher rcCH2_pub("rcCH2", &rcCH2_msg);
ros::Publisher rcCH3_pub("rcCH3", &rcCH3_msg);
ros::Publisher rcCH4_pub("rcCH4", &rcCH4_msg);
ros::Publisher rcCH5_pub("rcCH5", &rcCH5_msg);
ros::Publisher rcCH6_pub("rcCH6", &rcCH6_msg);
ros::Publisher rcCH7_pub("rcCH7", &rcCH7_msg);
ros::Publisher rcCH8_pub("rcCH8", &rcCH8_msg);
ros::Publisher rcCH10_pub("rcCH10", &rcCH10_msg);

// Forward declarations for callback functions
void rcCH4Callback(const std_msgs::Int32 &msg);
void rcCH2Callback(const std_msgs::Int32 &msg);
void rcCH7Callback(const std_msgs::Bool &msg);
void rcCH10Callback(const std_msgs::Bool &msg);

// ROS Subscribers
ros::Subscriber<std_msgs::Int32> rcCH4_sub("rcCH4", &rcCH4Callback);
ros::Subscriber<std_msgs::Int32> rcCH2_sub("rcCH2", &rcCH2Callback);
ros::Subscriber<std_msgs::Bool> rcCH7_sub("rcCH7", &rcCH7Callback);
ros::Subscriber<std_msgs::Bool> rcCH10_sub("rcCH10", &rcCH10Callback);

// Servo objects
Servo servoW1;  // front right 
Servo servoW3;  // back right
Servo servoW4;  // front left
Servo servoW6;  // back left

// Right motor driver pins
int EN_right = 2; 
int R_PWM_right = 6; // PWM pin
int L_PWM_right = 7; // PWM pin

// Left motor driver pins
int EN_left = 5; 
int R_PWM_left = 12; // PWM pin
int L_PWM_left = 13; // PWM pin

int r = 0;
float thetaInnerFront = 0;
float thetaInnerBack = 0;
float thetaOuterFront = 0;
float thetaOuterBack = 0;

float d1 = 271; // distance in mm
float d2 = 278;
float d3 = 301;
float d4 = 304;

void rcCH4Callback(const std_msgs::Int32 &msg) {
  rcCH4_msg = msg;
  controlServosAndMotors();
}

void rcCH2Callback(const std_msgs::Int32 &msg) {
  rcCH2_msg = msg;
}

void rcCH7Callback(const std_msgs::Bool &msg) {
  rcCH7_msg = msg;
}

void rcCH10Callback(const std_msgs::Bool &msg) {
  rcCH10_msg = msg;
}

void controlServosAndMotors() {
  if(rcCH10_msg.data == 1 && rcCH7_msg.data == 0) {
    if (rcCH4_msg.data > 4) {
      r = map(rcCH4_msg.data, 0, 100, 1400, 600);  // turning radius from 1400mm to 600mm
      calculateServoAngle();
      // Servo motors
      // Outer wheels
      servoW1.write(94 + thetaOuterFront);
      servoW3.write(96 - thetaOuterBack);
      // Inner wheels
      servoW4.write(97 + thetaInnerFront); // front wheel steer right
      servoW6.write(97 - thetaInnerBack); // back wheel steer left for overall steering to the right of the rover

      // DC Motors
      if (rcCH2_msg.data > 0) { 
        MoveForward();
      } else if (rcCH2_msg.data < 0) { 
        MoveBackward();
      }
    } else if (rcCH4_msg.data < -4) {
      r = map(rcCH4_msg.data, 0, -100, 1400, 600); // turning radius from 600mm to 1400mm
      calculateServoAngle();
      // Servo motors
      servoW1.write(94 - thetaInnerFront);
      servoW3.write(96 + thetaInnerBack);
      servoW4.write(97 - thetaOuterFront);
      servoW6.write(97 + thetaOuterBack);
      Serial.println(thetaOuterFront);

      if (rcCH2_msg.data > 0) { 
        MoveForward();
      } else if (rcCH2_msg.data < 0) { 
        MoveBackward();
      }
    } else {
      if (rcCH2_msg.data > 0) { 
        MoveForward();
      } else if (rcCH2_msg.data < 0) { 
        MoveBackward();
      }
    }
  }
}

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

// FS channel list
int rcCH1 = 0; // Available for stepper1
int rcCH2 = 0; // Forward - Backward for stepper2a and stepper2b
int rcCH3 = 0; // Available for stepper3
int rcCH4 = 0; // Right\Left for wrist_angle servo 
int rcCH5 = 0; // Speed
int rcCH6 = 0; // Available 
bool rcCH7 = 0; // Mode Control
bool rcCH8 = 0; // Available for open gripper
bool rcCH9 = 0; // Available
bool rcCH10 = 0; // Lock

void setup() {
  // Start serial monitor
  Serial.begin(9600);
 
  // Attach iBus object to serial port
  ibus.begin(Serial1);

  // Initialize ROS
  nh.initNode();

  // Advertise the topics
  nh.advertise(rcCH1_pub);
  nh.advertise(rcCH2_pub);
  nh.advertise(rcCH3_pub);
  nh.advertise(rcCH4_pub);
  nh.advertise(rcCH5_pub);
  nh.advertise(rcCH6_pub);
  nh.advertise(rcCH7_pub);
  nh.advertise(rcCH8_pub);
  nh.advertise(rcCH10_pub);

  // Subscribe to the topics
  nh.subscribe(rcCH4_sub);
  nh.subscribe(rcCH2_sub);
  nh.subscribe(rcCH7_sub);
  nh.subscribe(rcCH10_sub);

  // Attach servos
  servoW1.attach(9);
  servoW3.attach(11);
  servoW4.attach(3);
  servoW6.attach(9);

  // Set initial servo positions
  servoW1.write(90);
  servoW3.write(90);
  servoW4.write(90);
  servoW6.write(90);

  pinMode(EN_right, OUTPUT);
  pinMode(R_PWM_right, OUTPUT);
  pinMode(L_PWM_right, OUTPUT);

  pinMode(EN_left, OUTPUT);
  pinMode(R_PWM_left, OUTPUT);
  pinMode(L_PWM_left, OUTPUT);
}

void loop() {
  // Get RC channel values
  rcCH1 = readChannel(0, -100, 100, 0);
  rcCH2 = readChannel(1, -100, 100, 0);
  rcCH3 = readChannel(2, -100, 100, 0);
  rcCH4 = readChannel(3, -100, 100, 0);
  rcCH5 = readChannel(4, -100, 100, 0);
  rcCH6 = readChannel(5, -100, 100, 0);
  rcCH7 = readSwitch(6, false);
  rcCH8 = readSwitch(7, false);
  rcCH10 = readSwitch(9, false);

  // Publish the values
  rcCH1_msg.data = rcCH1;
  rcCH2_msg.data = rcCH2;
  rcCH3_msg.data = rcCH3;
  rcCH4_msg.data = rcCH4;
  rcCH5_msg.data = rcCH5;
  rcCH6_msg.data = rcCH6;
  rcCH7_msg.data = rcCH7;
  rcCH8_msg.data = rcCH8;
  rcCH10_msg.data = rcCH10;

  rcCH1_pub.publish(&rcCH1_msg);
  rcCH2_pub.publish(&rcCH2_msg);
  rcCH3_pub.publish(&rcCH3_msg);
  rcCH4_pub.publish(&rcCH4_msg);
  rcCH5_pub.publish(&rcCH5_msg);
  rcCH6_pub.publish(&rcCH6_msg);
  rcCH7_pub.publish(&rcCH7_msg);
  rcCH8_pub.publish(&rcCH8_msg);
  rcCH10_pub.publish(&rcCH10_msg);

  // Print channel values for debugging
  Serial.print("FWD = ");
  Serial.print(rcCH2);

  Serial.print(" DIR = ");
  Serial.print(rcCH4);

  Serial.print(" Speed = ");
  Serial.print(rcCH5);

  Serial.print(" Mode = ");
  Serial.print(rcCH7);

  Serial.print(" Lock = ");
  Serial.println(rcCH10);

  // Handle ROS communication
  nh.spinOnce();

  delay(50);
}

void stopMotor() {
  // Right side
  digitalWrite(EN_right, 1 );
  analogWrite(R_PWM_right, 0);
  analogWrite(L_PWM_right, 0);

  // Left side
  digitalWrite(EN_left, 1 );
  analogWrite(R_PWM_left, 0);
  analogWrite(L_PWM_left, 0);
}

void MoveForward() {
  // Right side
  digitalWrite(EN_right, 1 );
  analogWrite(R_PWM_right, 255);
  analogWrite(L_PWM_right, 0);

  // Left side
  digitalWrite(EN_left, 1 );
  analogWrite(R_PWM_left, 0);
  analogWrite(L_PWM_left, 255);
}

void MoveBackward() {
  // Right side
  digitalWrite(EN_right, 1 );
  analogWrite(R_PWM_right, 0);
  analogWrite(L_PWM_right, 255);

  // Left side
  digitalWrite(EN_left, 1 );
  analogWrite(R_PWM_left, 255);
  analogWrite(L_PWM_left, 0);
}

void calculateServoAngle() {
  // Calculate the angle for each servo for the input turning radius "r"
  thetaInnerFront = round((atan((d3 / (r + d1)))) * 180 / PI); // thetaInnerFront r19--- l42
  thetaInnerBack = round((atan((d2 / (r + d1)))) * 180 / PI); // thetaInnerBack r18   --- l42
  thetaOuterFront = round((atan((d3 / (r - d1)))) * 180 / PI); // thetaOuterFront r 42  --- l42
  thetaOuterBack = round((atan((d2 / (r - d1)))) * 180 / PI);  // thetaOuterBack r40  --- l42
    
  Serial.println("thetaInnerFront= ");
  Serial.print(thetaInnerFront);
  Serial.println("    ");

  Serial.println("thetaInnerBack= ");
  Serial.print(thetaInnerBack);
  Serial.println("    ");

  Serial.println("thetaOuterFront= ");
  Serial.print(thetaOuterFront);
  Serial.println("    ");

  Serial.println("thetaOuterBack= ");
  Serial.print(thetaOuterBack);
  Serial.println("    ");
}
