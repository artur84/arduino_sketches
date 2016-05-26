#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2016-05-26 16:04:13

#include "Arduino.h"
#include "ros_dc_motor.h"
void hard_stop(int motor) ;
void soft_stop(int motor) ;
void move_motor(int motor, float speed) ;
void move_robot(float linearx, float angularz) ;
void cmd_vel_cb(const geometry_msgs::Twist& cmd_msg) ;
void setup() ;
void loop() ;

#include "ros_dc_motor.ino"


#endif
