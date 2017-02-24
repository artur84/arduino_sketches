// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _motor_pid_control_H_
#define _motor_pid_control_H_
#include "Arduino.h"
#include <PID_v1.h>
#include "Timer.h"    //http://github.com/JChristensen/Timer
#include <Encoder.h>
#include <math.h>


/*******************************
 * Define the material
 ******************************/
//Arduino pin definitions
#define RIGHT_ENC_A 18  //ENCODER DERECHO A (YELLOW)
#define RIGHT_ENC_B 19    //ENCONDER DERECHO B (WHITE)
#define LEFT_ENC_A 20  //ENCODER IZQUIERDO A (YELLOW)
#define LEFT_ENC_B 21    //ENCONDER IZQUIERDO B (WHITE)

#define RIGHT_MOT_DIR_BACK 48
#define RIGHT__MOT_DIR_FRONT 49
#define RIGHT_MOT_EN 2 //enable
#define LEFT_MOT_DIR_BACK 50
#define LEFT_MOT_DIR_FRONT 51
#define LEFT_MOT_EN 3 //enable

#define BACKWARD -1  //moves backwards
#define FORWARD 1
#define LEFT 1
#define RIGHT 0
#define LED 13 //A led that blinks when receiving a message in any topic

//Robot Hardware
#define WHEELRAD 0.045 //[m] The radius of the wheel
#define WHEELDIST 0.42 //[m] Distance between wheels
#define ENCODER_PULSES 8400.0 //[ticks/rev] Total number of pulses per revolution given by the encoders.

//Controller parameters
#define CONTROL_RATE 20 //How often, in milliseconds, the PID will be evaluated. (int>0)
#define OK_RATE  3000   //How often, in milliseconds, the OK message will be sent. (int>0)
#define ODOMETRY_RATE  20   //How often, in milliseconds, the ODOMETRY message will be sent. (int>0)
#define READ_ENCODER_RATE 100.0 //How often to read the encoders in miliseconds

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

