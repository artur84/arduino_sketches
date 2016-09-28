#include "ros_servo.h"

/*
 * rosserial Servo Control
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 */

//Classes
Servo servo;

//ROS stuff
ros::NodeHandle  nh;
ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);


/*** This callback is called when a message is received from ROS
 *
 *  cmd_msg:  (std_msgs::UInt16) should have a value from 0-180
 */
void servo_cb( const std_msgs::UInt16& cmd_msg){
  servo.write(cmd_msg.data); //set servo angle, should be from 0-180
  digitalWrite(LED, HIGH-digitalRead(LED));  //toggle led
}




void setup(){
  pinMode(LED, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);

  servo.attach(SERVO_SIGNAL_PIN); //attach it to the pin where the servo is connected
}

void loop(){
  nh.spinOnce();
  delay(1);
}

