/*
 * Motor.cpp
 *
 *  Created on: Jul 31, 2015
 *      Author: arturo
 */

#include "Motor.h"

Motor::Motor() {
}

Motor::~Motor() {

}

void Motor::init(int positive_pin, int negative_pin, int enable_pin) {
	pos_ = positive_pin;
	neg_ = negative_pin;
	en_ = enable_pin;
	pinMode(pos_, OUTPUT);
	pinMode(neg_, OUTPUT);
	pinMode(en_, OUTPUT);
}

/* Hard stop of motor
 * motor: LEFT or RIGHT (0 or 1)
 */
void Motor::hard_stop() {
	//Stop if received an wrong direction
	digitalWrite(pos_, 1);
	digitalWrite(neg_, 1);
	analogWrite(en_, 255);
}

/* Soft stop of motor
 * motor: LEFT or RIGHT (0 or 1)
 */
void Motor::soft_stop() {
	//Stop if received an wrong direction
	digitalWrite(pos_, 0);
	digitalWrite(neg_, 0);
	analogWrite(en_, 0);
}
/* Moves the left wheel forward
 * motor: LEFT or RIGHT (0 or 1)
 * speed: a value between -255 and 255 (negative is backward, positive is forward)
 */
void Motor::move(int pwm_val) {
	//sonar_msg.data=pwm_val;

	if (pwm_val >= 1 && pwm_val <= 255) {
		digitalWrite(pos_, 1);
		digitalWrite(neg_, 0);
		analogWrite(en_, pwm_val);
	} else if (pwm_val <= 1 && pwm_val >= -255) {
		digitalWrite(pos_, 0);
		digitalWrite(neg_, 1);
		analogWrite(en_, -1 * pwm_val);
	} else {
		//Stop if received a wrong direction
		Motor::hard_stop();
	}
}
