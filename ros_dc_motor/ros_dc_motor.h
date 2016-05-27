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
#include <geometry_msgs/Vector3.h>
#include <Encoder.h>
/*******************************
 * Define the material
 ******************************/
//Para los encoders
#define ENCDA 20  //ENCODER DERECHO A (YELLOW)
#define ENCDB 21    //ENCONDER DERECHO B (WHITE)

#define ENCIA 19  //ENCODER IZQUIERDO A (YELLOW)
#define ENCIB 18    //ENCONDER IZQUIERDO B (WHITE)


#define RIGHT_MOT_POS 3
#define RIGHT_MOT_NEG 2
#define RIGHT_MOT_EN 4 //enable
#define LEFT_MOT_POS 6
#define LEFT_MOT_NEG 5
#define LEFT_MOT_EN 7 //enable
#define BACKWARD -1  //moves backwards
#define FORWARD 1
#define LEFT 1
#define RIGHT 0
#define LED 13 //A led that blinks when receiving a message in any topic
#define WHEELRAD 0.015 //The radius of the wheel (m)
#define WHEELDIST 0.11 //Distance between wheels (m)
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
