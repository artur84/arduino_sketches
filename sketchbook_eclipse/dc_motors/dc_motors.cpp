// Do not remove the include below
#include "dc_motors.h"

/*
 * This program moves a pair of motors of my robot
 * back and forward at different speeds
 */


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

/*
 * Arduino SETUP
 */
void setup() {
	pinMode(13, OUTPUT);
	pinMode(LEFT_MOT_NEG, OUTPUT);
	pinMode(LEFT_MOT_POS, OUTPUT);
	pinMode(LEFT_MOT_EN, OUTPUT);
	pinMode(RIGHT_MOT_NEG, OUTPUT);
	pinMode(RIGHT_MOT_POS, OUTPUT);
	pinMode(RIGHT_MOT_EN, OUTPUT);
}

/*
 * Arduino MAIN LOOP
 */
void loop() {
	//I will just keep the loop waiting for a message
	//in the ros topic
	int i = 20;

	while (i <= 80) {
		move_motor(RIGHT, FORWARD, i);
		move_motor(LEFT, FORWARD, i);
		i += 10;
		delay(500);
	}
	i = 20;
	hard_stop(LEFT);
	hard_stop(RIGHT);
	delay(1000);
	while (i <= 80) {
		move_motor(RIGHT, BACKWARD, i);
		move_motor(LEFT, BACKWARD, i);
		delay(500);
		i += 10;
	}
	hard_stop(LEFT);
	hard_stop(RIGHT);
	delay(1000);

	delay(1);
}
