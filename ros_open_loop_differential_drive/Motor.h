/*
 * Motor.h
 *
 *  Created on: Jul 31, 2015
 *      Author: arturo
 */

#ifndef MOTOR_H_
#define MOTOR_H_
#include "Arduino.h"

class Motor {
	int pos_; //Positive pin
	int neg_; //Negative pin
	int en_; //Enable pin

public:
	Motor();
	virtual ~Motor();
	void init(int positive_pin, int negative_pin, int enable_pin);
	void move(int pwm_val);
	void hard_stop();
	void soft_stop();

};

#endif /* MOTOR_H_ */
