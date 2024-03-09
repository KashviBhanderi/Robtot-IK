//code for inverse kinematics of 3-dof
#include <Braccio.h>
#include <Servo.h>
#include<math.h> 

const float DEG_2_RAD=PI/180.0f;
const float RAD_2_DEG=180.0f/PI;

float l1=60;
float l2=125;
float l3=125;

float la;
float lb;
float lc;
float ld;

float theta1;
float theta2; 
float theta3;
float theta4;
float theta5;
float theta6;
float theta7;
float theta8;

float x;
float y;

float thetaA;
float thetaB; // this 3 theta’s are our joint angle of robotic arm that we have to find
float thetaC;

//define all servo motors
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

void setup() {
  //Initialization functions and set up the initial position for Braccio
  //All the servo motors will be positioned in the "safety" position:
  //Base (M1):90 degrees
  //Shoulder (M2): 45 degrees
  //Elbow (M3): 180 degrees
  //Wrist vertical (M4): 180 degrees
  //Wrist rotation (M5): 90 degrees
  //gripper (M6): 10 degrees
  Braccio.begin();
  Serial.begin(9600); 
}

void loop() {
   /*
   Step Delay: a milliseconds delay between the movement of each servo.  Allowed values from 10 to 30 msec.
   M1=base degrees. Allowed values from 0 to 180 degrees
   M2=shoulder degrees. Allowed values from 15 to 165 degrees
   M3=elbow degrees. Allowed values from 0 to 180 degrees
   M4=wrist vertical degrees. Allowed values from 0 to 180 degrees
   M5=wrist rotation degrees. Allowed values from 0 to 180 degrees
   M6=gripper degrees. Allowed values from 10 to 73 degrees. 10: the toungue is open, 73: the gripper is closed.
  */
  inversekinematics(180*DEG_2_RAD,-120,180); //here parameters 0 is theta(given in degree) and also give X,Y value as input
  delay(2000);
  inversekinematics(0*DEG_2_RAD,150,200);// 2nd time run for different position and angles
  delay(2000);
  }

void inversekinematics(float theta,float X,float Y)
{
  la=sqrt(pow(X,2)+pow(Y,2));
  theta1=atan(X/Y);
  theta2=atan(Y/X);
  lc=l1*cos(theta);
  ld=l1*sin(theta);
  theta3=HALF_PI-theta;
  x=X-lc;
  y=Y+ld;
  lb=sqrt(pow(x,2)+pow(y,2));
  theta4=atan(y/x);
  theta5=atan(x/y);
  theta6=acos((pow(lb,2)+pow(l2,2)-pow(l3,2))/(2*lb*l2));
  theta7=acos((pow(l3,2)+pow(lb,2)-pow(l2,2))/(2*l3*lb));
  theta8=acos((pow(l2,2)+pow(l3,2)-pow(lb,2))/(2*l2*l3));
  //final caluclation of theta;
  thetaA=PI-theta4+theta7;
  thetaB=theta8-HALF_PI;
  thetaC=theta6-theta5+theta3-HALF_PI;
  // this all angle are in radian so we have to convert this into degree
  //(step delay, M1, M2, M3, M4, M5, M6);
 /* M1=base degrees. Allowed values from 0 to 180 degrees
   M2=shoulder degrees.  //HERE WE HAVE TO FIND thetaA ,B,C SO THAT WE WRITE BELOW THIS MANNER //
   M3=elbow degrees.
   M4=wrist vertical degrees.
   M5=wrist rotation degrees. Allowed values from 0 to 180 degrees
   M6=gripper degrees. Allowed values from 10 to 73 degrees. 10: the toungue is open, 73: the gripper is closed.
  */
  //SERVOMOVEMENT THROUGH WE CAN MOVE ROBOTIC ARM SO OUTPUT WILL REFLACT INTO ROBOTIC ARM
  //here we keep gripper close
  Braccio.ServoMovement(20,          0,  thetaA*RAD_2_DEG, thetaB*RAD_2_DEG, thetaC*RAD_2_DEG, 90,  73);
  Serial.print("thetaA = "); 
  Serial.print(thetaA*RAD_2_DEG);  
  Serial.print("thetaB = "); 
  Serial.print(thetaB*RAD_2_DEG);
  Serial.print("thetaC = "); 
  Serial.print(thetaC*RAD_2_DEG);  
}
