#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2017-02-03 17:19:56

#include "Arduino.h"
#include "iteshu_driver.h"
void hard_stop(int motor) ;
void soft_stop(int motor) ;
void move_right_motor(void) ;
void move_left_motor(void) ;
void move_robot(double linear, double angular) ;
double wheel_vel_from_encoder(long curr_pos, long last_pos, long delta_t_us) ;
void wheel_vel_from_twist(double linear, double angular, double* wl_pointer, 		double* wr_pointer) ;
void twist_from_wheel_vel(double w_right, double w_left, double* linear, double* angular) ;
void compute_odom(double Dr, double Dl, double* x, double* y,  double* phi) ;
void cmd_vel_cb(const geometry_msgs::Twist& cmd_msg) ;
void setup() ;
void loop() ;

#include "iteshu_driver.ino"


#endif
