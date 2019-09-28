#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2016-12-16 10:39:31

#include "Arduino.h"
#include "ros_dc_motor.h"
void hard_stop(int motor) ;
void soft_stop(int motor) ;
void move_right_motor(void) ;
void move_left_motor(void) ;
double wheel_vel_from_encoder(long curr_pos, long last_pos, long delta_t_us) ;
void wheel_vel_from_twist(double linear, double angular, double* vlp, 		double* vrp) ;
void move_robot(double linear, double angular) ;
void cmd_vel_cb(const geometry_msgs::Twist& cmd_msg) ;
void setup() ;
void loop() ;

#include "ros_dc_motor.ino"


#endif
