// Do not remove the include below
#include "ros_dc_motor.h"
/*
 * This program hears a Twist message in a ROS topic
 * and commands two motors according to the linear.x
 * value of the received Twist.
 * mantainer: arturoescobedo.iq@gmail.com
 */

/*
 * Global variables
 */
ros::NodeHandle nh;

/*
 * My functions
 */
/* Moves the left wheel forward
 * motor: LEFT or RIGHT (0 or 1)
 * dir: FORWARD or BACKWARD (+1 or -1) to choose the motor direction
 * speed: a value between 0 and 255
 */
void move_motor(int motor, int dir, int speed) {
	if (motor == LEFT) {
		if (dir == FORWARD) {
			digitalWrite(LEFT_MOT_POS, 1);
			digitalWrite(LEFT_MOT_NEG, 0);
			analogWrite(LEFT_MOT_EN, speed);
		} else if (dir == BACKWARD) {
			digitalWrite(LEFT_MOT_POS, 0);
			digitalWrite(LEFT_MOT_NEG, 1);
			analogWrite(LEFT_MOT_EN, speed);
		} else {
			//Stop if received an wrong direction
			digitalWrite(LEFT_MOT_POS, 0);
			digitalWrite(LEFT_MOT_NEG, 0);
			analogWrite(LEFT_MOT_EN, 0);
		}

	} else if (motor == RIGHT) {
		if (dir == FORWARD) {
			digitalWrite(RIGHT_MOT_POS, 1);
			digitalWrite(RIGHT_MOT_NEG, 0);
			analogWrite(RIGHT_MOT_EN, speed);
		} else if (dir == BACKWARD) {
			digitalWrite(RIGHT_MOT_POS, 0);
			digitalWrite(RIGHT_MOT_NEG, 1);
			analogWrite(RIGHT_MOT_EN, speed);
		} else {
			//Stop if received an wrong direction
			digitalWrite(RIGHT_MOT_POS, 0);
			digitalWrite(RIGHT_MOT_NEG, 0);
			analogWrite(RIGHT_MOT_EN, 0);
		}

	}

}

/* Hard stop of motor
 * motor: LEFT or RIGHT (0 or 1)
 */
void hard_stop(int motor) {
	//Stop if received an wrong direction
	if (motor == LEFT) {
		digitalWrite(LEFT_MOT_POS, 1);
		digitalWrite(LEFT_MOT_NEG, 1);
		analogWrite(LEFT_MOT_EN, 255);
	} else if (motor == RIGHT) {
		//Stop if received an wrong direction
		digitalWrite(RIGHT_MOT_POS, 1);
		digitalWrite(RIGHT_MOT_NEG, 1);
		analogWrite(RIGHT_MOT_EN, 255);
	}
}

/* Soft stop of motor
 * motor: LEFT or RIGHT (0 or 1)
 */
void soft_stop(int motor) {
	//Stop if received an wrong direction
	if (motor == LEFT) {
		digitalWrite(LEFT_MOT_POS, 0);
		digitalWrite(LEFT_MOT_NEG, 0);
		analogWrite(LEFT_MOT_EN, 0);
	} else if (motor == RIGHT) {
		//Stop if received an wrong direction
		digitalWrite(RIGHT_MOT_POS, 0);
		digitalWrite(RIGHT_MOT_NEG, 0);
		analogWrite(RIGHT_MOT_EN, 0);
	}
}

//Twist callback
void cmd_vel_cb(const geometry_msgs::Twist& cmd_msg) {
	digitalWrite(13, HIGH - digitalRead(13));  //toggles a led
	float x = cmd_msg.linear.x;
	int motor_speed = 55 + 50 * x;    //linear x should be maximum 4

	if ((x < 0) && (x >= -4)) {
		move_motor(LEFT, BACKWARD, motor_speed); // turn it on going backward
		move_motor(RIGHT, BACKWARD, motor_speed); // turn it on going backward
	} else if ((x > 0) && (x <= 4)) {
		move_motor(LEFT, FORWARD, motor_speed); // turn it on going forward
		move_motor(RIGHT, FORWARD, motor_speed); // turn it on going forward
	} else { //Stop if 0 or a value out of boundaries is sent
		motor_speed = 0;
		hard_stop(LEFT);
		soft_stop(RIGHT);
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
