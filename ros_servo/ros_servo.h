// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section
#ifndef _ros_servo_H_
#define _ros_servo_H_
/*****************************************
 * rosserial Servo Control
 *****************************************
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 */

//#define USE_USBCON This is used only for Leonardo board (ROS needs this)
#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>

//INSTRUCTIONS
#define SERVO_SIGNAL_PIN 9 //Connect Servo Signal cable (white or yellow) to this pin
#define LED 13  // Arduino led





//Do not add code below this line
#endif /* _ros_servo_H_ */
