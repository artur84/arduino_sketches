// Do not remove the include below
#include "ros_dc_motor.h"
/*
 * This program hears a Twist message in a ROS topic
 * and commands a motor according to the linear.x
 * value of the received Twist.
 * mantainer: arturoescobedo.iq@gmail.com
 */

/*
 * Global variables
 */
ros::NodeHandle  nh;
AF_DCMotor motor(1, MOTOR12_64KHZ); // create motor #1, 64KHz pwm


/*
 * My functions
 */

//Twist callback
void cmd_vel_cb( const geometry_msgs::Twist& cmd_msg){
   digitalWrite(13, HIGH-digitalRead(13));  //toggles a led
   float x = cmd_msg.linear.x;
   int motor_speed = 55 + 50*x;    //linear x should be maximum 4
   motor.setSpeed(motor_speed); // set the speed according to linear.x
   if ((x<0) && (x >= -4)){
     motor.run(BACKWARD); // turn it on going backward
   }
   else if((x >0) && (x<= 4)){
     motor.run(FORWARD); // turn it on going forward
   }
   else{//Stop if 0 or a value out of boundaries is sent
     motor_speed=0;
     motor.setSpeed(motor_speed); // set the speed
     motor.run(RELEASE); // stop it
   }
}

//Creates the ROS subscriber
ros::Subscriber<geometry_msgs::Twist> sub("arduino/cmd_vel", cmd_vel_cb);


/*
 * Arduino SETUP
 */
void setup() {
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  motor.setSpeed(200); // set the speed to 200/255
}

/*
 * Arduino MAIN LOOP
 */
void loop() {
  //I will just keep the loop waiting for a message
  //in the ros topic
  nh.spinOnce();
  delay(1);
}
