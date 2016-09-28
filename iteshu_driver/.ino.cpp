#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2016-09-27 20:52:45

#include "Arduino.h"
#include "iteshu_driver.h"
void hard_stop(int motor) ;
void soft_stop(int motor) ;
void move_right_motor(void) ;
void move_left_motor(void) ;
void move_robot(double linear, double angular) ;
double wheel_vel_from_encoder(long curr_pos, long last_pos, long delta_t_us) ;
void wheel_vel_from_twist(double linear, double angular, double* vlp, 		double* vrp) ;
void cmd_vel_cb(const geometry_msgs::Twist& cmd_msg) ;
void setup() ;
void loop() ;

#include "iteshu_driver.ino"


#endif
