#include "ros_servo.h"

/*
 * rosserial Servo Control
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 */

//Classes
Servo servo_yaw,  servo_pitch;

//ROS stuff
ros::NodeHandle  nh;



/*** This callback is called when a message is received from ROS
 *
 *  cmd_msg:  (std_msgs::UInt16) should have a value from 0-180
 */
void servo_cb( const std_msgs::UInt16& cmd_msg){
  servo_yaw.write(cmd_msg.data); //set servo angle, should be from 0-180
  //digitalWrite(LED, HIGH-digitalRead(LED));  //toggle led
}

ros::Subscriber<std_msgs::UInt16> sub("arduino/servo", servo_cb);


void setup(){
  pinMode(LED, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);

  servo_yaw.attach(SERVO_YAW_SIGNAL_PIN); //attach it to the pin where the servo is connected
  servo_yaw.write(YAW_INITIAL_ANGLE); //set YAW motor at an initial angle  (looking to the front) to add resistance when not in use.

  servo_pitch.attach(SERVO_PITCH_SIGNAL_PIN); //attach it to the pin where the servo is connected
  servo_pitch.write(PITCH_INITIAL_ANGLE); //set PITCH motor at an initial angle  (looking to the front) to add resistance when not in use.
}

void loop(){
  servo_yaw.write(YAW_INITIAL_ANGLE);
  servo_pitch.write(PITCH_INITIAL_ANGLE); //set PITCH motor at an initial angle  (looking to the front) to add resistance when not in use.
  digitalWrite(LED, HIGH-digitalRead(LED));
  nh.spinOnce();
  delay(100);
}

