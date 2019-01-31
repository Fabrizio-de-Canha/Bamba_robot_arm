#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/JointState.h>

ros::NodeHandle  nh;
Servo joint_0;
Servo joint_1;
Servo joint_2;
Servo joint_3;
Servo joint_4;
Servo joint_5;

double joint_0_angle=90;
double joint_1_angle=90;
double joint_2_angle=90;
double joint_3_angle=90;
double joint_4_angle=90;
double joint_5_angle=90;


void servo_cb(const sensor_msgs::JointState& cmd_msg){
  joint_0_angle=radiansToDegrees(cmd_msg.position[0], 0);
  joint_1_angle=radiansToDegrees(cmd_msg.position[1], 0);
  joint_2_angle=radiansToDegrees(cmd_msg.position[2], 1.6);
  joint_3_angle=radiansToDegrees(cmd_msg.position[3], 1.6);
  joint_4_angle=radiansToDegrees(cmd_msg.position[4], 1.6);
  joint_5_angle=radiansToDegrees(cmd_msg.position[5], 1.6);

  joint_0.write(90 + joint_0_angle);
  joint_1.write(90 + joint_1_angle);
  joint_2.write(180 - joint_2_angle);
  joint_3.write(180 - joint_3_angle);
  joint_4.write(180 - joint_4_angle);
  joint_5.write(180 - joint_5_angle);
  
}


ros::Subscriber<sensor_msgs::JointState> sub("joint_states", servo_cb);

void setup(){
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);

  joint_0.attach(12);
  joint_1.attach(11); 
  joint_2.attach(10); 
  joint_3.attach(9);
  joint_4.attach(8);
  joint_5.attach(13);

  delay(1);
  joint_0.write(90);
  joint_1.write(90);
  joint_2.write(90);
  joint_3.write(90);
  joint_4.write(90);
  joint_5.write(85);
}

void loop(){
  nh.spinOnce();
}

double radiansToDegrees(float position_radians, float adjustment)
{

  position_radians = position_radians + adjustment;

  return position_radians * 57.2958;

}
