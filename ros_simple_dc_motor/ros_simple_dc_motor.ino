//#define USE_USBCON //Uncomment this line if you are using an arduino DUE
/*
 * This program receives a ROS (Int32) message (between -255 and 255)
 * and commands a DC motor according to the specified value.
 * subscribed ROS topic arduino/motor_vel
 */

 #include "Arduino.h"
 #include <ros.h>
 #include <std_msgs/Int32.h>

/*******************************
 * Define the material
 ******************************/
//Arduino pin definitions
#define MOT1_IN1 10//IN1 of the L298 should be connected to this arduino pin
#define MOT1_IN2 9//IN2 of the L298 should be connected to this arduino pin
#define MOT1_EN 11 //enable
#define LED 13 //A led that blinks when receiving


/*
 * Global variables
 */
ros::NodeHandle nh;
volatile int motor_vel; //The motor velocity between -255->full speed backwards and 255-> Full speed forward

/*
 * My functions
 */
/* Hard stop of motor
 * motor: LEFT or RIGHT (0 or 1)
 */
void motor_stop() {
	//Stop if received an wrong direction
		digitalWrite(MOT1_IN1, 0);
		digitalWrite(MOT1_IN2, 0);
		analogWrite(MOT1_EN, 0);
}

/* Moves the motor in positive sens
		vel: [0-255] the desired pwm value 255 means full speed
 */
void motor_forward(unsigned int vel) {
		digitalWrite(MOT1_IN1, 1);
		digitalWrite(MOT1_IN2, 0);
		analogWrite(MOT1_EN, vel);
}

/* Moves the motor in positive sens
		vel: [0-255] the desired pwm value 255 means full speed
 */
void motor_backwards(unsigned int vel) {
    digitalWrite(MOT1_IN1, 0);
    digitalWrite(MOT1_IN2, 1);
    analogWrite(MOT1_EN, vel);
}

//Motor callback
void motor_vel_cb(const std_msgs::Int32 & msg) {
	digitalWrite(LED, HIGH - digitalRead(LED)); //toggles a led to indicate this topic is active
	motor_vel = msg.data;
	if (motor_vel > 0 && motor_vel <= 255) {
		motor_forward((unsigned int) motor_vel);
	} else if (motor_vel < 0 && motor_vel >= -255) {
		motor_backwards((unsigned int) -1*motor_vel);
	} else {
		//Stop if received a wrong value or 0
		motor_stop();
	}
}


//Creates the ROS subscribers
ros::Subscriber<std_msgs::Int32> motor_vel_sub("arduino/motor_vel", motor_vel_cb);

/*
 * Arduino SETUP
 */
void setup() {
	//init ROS communication
	nh.initNode();
	//Subscribed ROS topics
	nh.subscribe(motor_vel_sub);
	// Arduino pins definition
	pinMode(LED, OUTPUT);
	pinMode(MOT1_IN1, OUTPUT);
	pinMode(MOT1_IN2, OUTPUT);
	pinMode(MOT1_EN, OUTPUT);
}

/*
 * Arduino MAIN LOOP
 */
void loop() {
	nh.spinOnce();
	delay(1);
}
