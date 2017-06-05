/*
 ros_motor_pfm

 Uses PFM modulation to control motor speeds.



 */
#include <Arduino.h>
#include "Timer.h"
#define LED 13
#define LEFT_MOT_DIR_FRONT 6
#define LEFT_MOT_DIR_BACK 5
#define LEFT_MOT_EN 7 //enable
#define RIGHT__MOT_DIR_FRONT 3
#define RIGHT_MOT_DIR_BACK 2
#define RIGHT_MOT_EN 4 //enable

Timer t; //creates the timer
long T = 100; //Period of the wave [ms]
float Ta = 4;

void setup() {

	int put_down_event = t.every(Ta, putDown);
	int put_high_event = t.every(T, putHigh);

	pinMode(LED, OUTPUT);
	pinMode(LEFT_MOT_DIR_FRONT, OUTPUT);
	pinMode(LEFT_MOT_DIR_BACK, OUTPUT);
	pinMode(RIGHT__MOT_DIR_FRONT, OUTPUT);
	pinMode(RIGHT_MOT_DIR_BACK, OUTPUT);

	digitalWrite(LEFT_MOT_DIR_FRONT, LOW);
	digitalWrite(LEFT_MOT_DIR_BACK, HIGH);
	digitalWrite(RIGHT__MOT_DIR_FRONT, LOW);
	digitalWrite(RIGHT_MOT_DIR_BACK, HIGH);

}

void loop() {
	t.update();
}

void putDown() {
	digitalWrite(LEFT_MOT_EN, LOW);
	digitalWrite(RIGHT_MOT_EN, LOW);
	digitalWrite(LED, LOW);
}

void putHigh() {
	digitalWrite(LEFT_MOT_EN, HIGH);
	digitalWrite(RIGHT_MOT_EN, HIGH);
	digitalWrite(LED, HIGH);
}
