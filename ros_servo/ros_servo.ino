#include "ros_servo.h"
// Do not remove the include below
/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 *
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */


ros::NodeHandle  nh;

Servo servo;

void servo_cb( const std_msgs::UInt16& cmd_msg){
  servo.write(cmd_msg.data); //set servo angle, should be from 0-180
  digitalWrite(LED, HIGH-digitalRead(LED));  //toggle led
}


ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);

void setup(){
  pinMode(LED, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);

  servo.attach(SERVO_SIGNAL_PIN); //attach it to pin 9
}

void loop(){
  nh.spinOnce();
  delay(1);
}

