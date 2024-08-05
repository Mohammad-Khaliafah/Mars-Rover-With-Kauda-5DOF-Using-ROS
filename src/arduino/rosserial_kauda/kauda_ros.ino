#include <AccelStepper.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
//#include <MultiStepper.h>


ros::NodeHandle node_handle;


//steppers 1
#define STEP1_PIN 7  //d6
#define DIR1_PIN 6   // d4
#define ENABLE1_PIN 2

//steppers 2
#define STEP2_PIN 5  //d10
#define DIR2_PIN 4    //d8
#define ENABLE2_PIN 1

//steppers 3
#define STEP3_PIN 9  //d9
#define DIR3_PIN 8  //d7
#define ENABLE3_PIN 3

//steppers 4
#define STEP4_PIN 3 //d11
#define DIR4_PIN 2  // d12
#define ENABLE4_PIN 21



// Define the stepper motor and the pins that is connected to
AccelStepper stepper1(1, STEP1_PIN , DIR1_PIN);  // (Typeof driver: with 2 pins, STEP, DIR)
AccelStepper stepper2a(1, STEP2_PIN , DIR2_PIN);
AccelStepper stepper2b(1, STEP3_PIN , DIR3_PIN);
AccelStepper stepper3(1, STEP4_PIN , DIR4_PIN);
Servo elbow;
Servo wrist;
Servo gripper;

double elbow_angle = 90;
double prev_elbow = 0;
double wrist_angle = 90;
double prev_wrist = 0;

int gripperState = 0;


void kaudacallback(const sensor_msgs::JointState& msg) {
  
  float joint1 = msg.position[0];
  float joint2a = msg.position[1];
  float joint2b = msg.position[1];
  float joint3 = msg.position[2];
  
  
 float Joint1= map(joint1, -1.57, 1.57, -200, 200);
 float Joint2a= map(joint2a, -1.57, 1.57, -180, 180);
 float Joint2b= map(joint2b, -1.57, 1.57, -180, 180);
 float Joint3= map(joint3, -1.57, 1.57, -180, 180);

  stepper1.moveTo(Joint1);
  stepper1.runToPosition();


  stepper2a.moveTo(Joint2a);
  stepper2a.runToPosition();


  stepper2b.moveTo(-Joint2a);
  stepper2b.runToPosition();


  stepper3.moveTo(Joint3);
  stepper3.runToPosition();

  elbow_angle = radiansToDegrees(msg.position[3]);
  elbow.write(elbow_angle);
  wrist_angle = radiansToDegrees(msg.position[4]);
  wrist.write(wrist_angle);

 if (msg.position[6]> 0){
   gripper.write(30);

 }else{
     gripper.write(0);

 }



    /*
    ROS_INFO("stepper1: %d", stepper1);
    ROS_INFO("stepper2a: %d", stepper2a);
    ROS_INFO("stepper2b: %d", stepper2b);
    ROS_INFO("stepper3: %d", stepper3);
    ROS_INFO("wrist: %d", wrist);
    ROS_INFO("gripper: %d", gripper);
*/
//  ROS_INFO_STREAM("Received ");

}


ros::Subscriber<sensor_msgs::JointState> kauda("joint_states", &kaudacallback);


// int angle_to_steps(double x)
// {
//   float steps;
//   steps=((x / M_PI)*stepsPerRevolution)+0.5; // (radians)*(1 revolution/PI radians)*(200 steps/revolution)
//   return steps;
// }

void setup() {
  Serial.begin(115200);

  elbow.attach(8);
  wrist.attach(12);
  gripper.attach(14);


  delay(1);
 
  stepper1.setMaxSpeed(500);      // Set maximum speed value for the stepper
  stepper1.setAcceleration(500);   // Set acceleration value for the stepper
  stepper1.setCurrentPosition(0);  // Set the current position to 0 steps
 
  stepper2a.setMaxSpeed(500);      // Set maximum speed value for the stepper
  stepper2a.setAcceleration(500);   // Set acceleration value for the stepper
  stepper2a.setCurrentPosition(0);  // Set the current position to 0 steps
 
  stepper2b.setMaxSpeed(500);      // Set maximum speed value for the stepper
  stepper2b.setAcceleration(500);   // Set acceleration value for the stepper
  stepper2b.setCurrentPosition(0);  // Set the current position to 0 steps

  stepper3.setMaxSpeed(500);      // Set maximum speed value for the stepper
  stepper3.setAcceleration(500);   // Set acceleration value for the stepper
  stepper3.setCurrentPosition(0);  // Set the current position to 0 steps

  elbow.write(90);
  wrist.write(90);
  gripper.write(0);

  node_handle.getHardware()->setBaud(115200);
  node_handle.initNode();
  node_handle.subscribe(kauda);

}

void loop() {

  //  stepper3.moveTo(0); // Set desired move: 800 steps (in quater-step resolution that's one rotation)
  //  stepper3.runToPosition(); // Moves the motor to target position w/ acceleration/ deceleration and it blocks until is in position
  //
  //  delay(1000);
  //
  //  stepper3.moveTo(200); // Set desired move: 800 steps (in quater-step resolution that's one rotation)
  //  stepper3.runToPosition(); // Moves the motor to target position w/ acceleration/ deceleration and it blocks until is in position
  //
  //  delay(1000);

  node_handle.spinOnce();
  delay(1);
}

double radiansToDegrees(float position_radians) {

  position_radians = position_radians + 1.6;

  return position_radians * 57.2958;
}
