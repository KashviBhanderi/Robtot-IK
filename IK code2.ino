#include <Braccio.h>
#include <Servo.h>
#include<math.h> 

const float DEG_2_RAD=PI/180.0f;
const float RAD_2_DEG=180.0f/PI;

float l1=125;
float l2=125;
float l3=60;

float xprime;
float yprime;
float c;

float beta;
float theta1;
float theta2; 
float theta3;

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

void setup() {
  Braccio.begin();
  Serial.begin(9600);
}

void loop() {
   
  inversekinematics(0*DEG_2_RAD,150,200); 
  
  }

void inversekinematics(float theta,float X,float Y)
{
  xprime=X-l3*cos(theta);
  yprime=Y-l3*sin(theta);
  
  c=sqrt(pow(xprime, 2) + pow(yprime, 2));

  beta = acos((pow(l1, 2) + pow(c, 2) - pow(l2, 2)) / (2 * l1*c ));
  theta1 = atan2(yprime, xprime) - beta;

  theta2 = PI - acos((pow(l1, 2) + pow(l2, 2) - pow(c, 2)) / (2 * l1 * l2));
  theta3 = theta - theta1 - theta2;

 
  
  Braccio.ServoMovement(20,           0,  theta1*RAD_2_DEG, theta2*RAD_2_DEG, theta3*RAD_2_DEG, 90,  73);
  Serial.print("theta1 = "); 
  Serial.print(theta1*RAD_2_DEG);  
  Serial.print("theta2 = "); 
  Serial.print(theta2*RAD_2_DEG);
  Serial.print("theta3 = "); 
  Serial.print(theta3*RAD_2_DEG);  
}
//code by kashvi
