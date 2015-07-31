// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _ros_dc_motor_H_
#define _ros_dc_motor_H_
#include "Arduino.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
//add your includes for the project ros_dc_motor here

/*******************************
 * Define the material
 ******************************/
#define RIGHT_MOT_POS 2
#define RIGHT_MOT_NEG 7
#define RIGHT_MOT_EN 6 //enable
#define LEFT_MOT_POS 3
#define LEFT_MOT_NEG 4
#define LEFT_MOT_EN 5 //enable
#define BACKWARD -1  //moves backwards
#define FORWARD 1
#define LEFT 1
#define RIGHT 0
#define LED 13 //A led that blinks when receiving a message in any topic

//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project ros_dc_motor here

//Do not add code below this line
#endif /* _ros_dc_motor_H_ */
