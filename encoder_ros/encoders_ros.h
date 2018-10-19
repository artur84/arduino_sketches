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
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <Encoder.h>
#include <PID_v1.h>
/*******************************
 * Define the material
 ******************************/
//Arduino pin definitions
#define ENC_A 53  //ENCODER A (YELLOW)
#define ENC_B 51    //ENCONDER B (WHITE)

#define LED 13 //A led that blinks when receiving a message in any topic

//Robot Hardware
#define WHEELRAD 0.045 //The radius of the wheel (m)
#define WHEELDIST 0.42 //Distance between wheels (m)
#define ENCODER_CPR 4480 //Counts per revolution given by the encoders.
#define PUBLISH_RATE 1000 // Control the motors every some [ms] aprox.


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
