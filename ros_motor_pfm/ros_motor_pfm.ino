/*
 ros_motor_pfm

 Uses PFM modulation to control motor speeds.



 */
#include <Arduino.h>
#include "Timer.h"
#define LED 13
#define LEFT_MOT_POS 3
#define LEFT_MOT_NEG 2
#define LEFT_MOT_EN 4 //enable

Timer t; //creates the timer
long T = 100; //Period of the wave [ms]
float Ta = 50;

void setup() {

	int put_down_event = t.every(Ta, putDown);
	int put_high_event = t.every(T, putHigh);

	pinMode(LED, OUTPUT);
	pinMode(LEFT_MOT_POS, OUTPUT);
	pinMode(LEFT_MOT_NEG, OUTPUT);

}

void loop() {
	t.update();
}

void putDown() {
	digitalWrite(LEFT_MOT_POS, HIGH);
	digitalWrite(LEFT_MOT_NEG, HIGH);
	digitalWrite(LEFT_MOT_EN, LOW);
	digitalWrite(LED, LOW);
}

void putHigh() {

	digitalWrite(LEFT_MOT_POS, LOW);
	digitalWrite(LEFT_MOT_NEG, HIGH);
	digitalWrite(LEFT_MOT_EN, HIGH);
	digitalWrite(LED, HIGH);
}
