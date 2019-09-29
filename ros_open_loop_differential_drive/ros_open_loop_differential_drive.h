// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _robotino_H_
#define _robotino_H_
#include "Arduino.h"
#include "Motor.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>


//add your includes for the project ros_dc_motor here

/*******************************
 * Define the material
 ******************************/
//Adjust this to your robot
#define WHEELRAD 0.09 //The radius of the wheels (m)
#define WHEELDIST 0.48 //Distance between wheels (m)
#define MAX_MOTOR_RPM 130.0 //[RPM]

//para MOVER EL MOTOR
#define MOTORD_PINA  10
#define MOTORD_PINB 9
#define MOTORD_ENABLE 11

#define MOTORI_PINA  8
#define MOTORI_PINB 7
#define MOTORI_ENABLE 6


//otros
#define CONTROLLER_TIME 10 //Time in milliseconds
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

//add your function definitions for the project robotino here

//Do not add code below this line
#endif /* _robotino_H_ */
