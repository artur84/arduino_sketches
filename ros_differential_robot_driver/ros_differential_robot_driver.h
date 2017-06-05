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
#include <std_msgs/Int16.h>
#include <geometry_msgs/Vector3.h>
#include <Encoder.h>


//add your includes for the project ros_dc_motor here

/*******************************
 * Define the material
 ******************************/
//Para los encoders
#define RIGHT_ENC_A 20  //ENCODER DERECHO A (YELLOW)
#define RIGHT_ENC_B 21    //ENCONDER DERECHO B (WHITE)

#define LEFT_ENC_A 19  //ENCODER IZQUIERDO A (YELLOW)
#define LEFT_ENC_B 18    //ENCONDER IZQUIERDO B (WHITE)

//para MOVER EL MOTOR
#define MOTORD_PINA  2
#define MOTORD_PINB 3
#define MOTORD_ENABLE 4

#define MOTORI_PINA  5
#define MOTORI_PINB 6
#define MOTORI_ENABLE 7

//Odometry
#define ODOM_PUBLISH_TIME 2 //TIME IN milliseconds
#define CONTROLLER_TIME 10 //Time in milliseconds

//otros
#define BACKWARD -1  //moves backwards
#define FORWARD 1
#define LEFT 1
#define RIGHT 0
#define LED 13 //A led that blinks when receiving a message in any topic
#define WHEELRAD 0.09 //The radius of the wheel (m)
#define WHEELDIST 0.48 //Distance between wheels (m)

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
