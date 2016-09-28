// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _ros_servo_H_
#define _ros_servo_H_

//#define USE_USBCON This is used only for Leonardo board (ROS needs this)
#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
//add your includes for the project servo_ros here


//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project servo_ros here




//Do not add code below this line
#endif /* _servo_ros_H_ */
