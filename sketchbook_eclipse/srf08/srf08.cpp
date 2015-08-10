// Do not remove the include below
#include "srf08.h"
/*

 /**********************************
 *  Functions to control the SONAR
 **********************************/
int sensor_reading = 0;
Sonar sonar;
/*************************************
 * Arduino Setup
 **************************************/
void setup() {
	Serial.begin(9600);
	while (!Serial) {
		; // wait for serial port to connect. Needed for Leonardo only
	}
	delay(2000); //Wait for some seconds to wait serial monitor
	sonar.connect();
	Serial.println("SONAR 08");
}

/***********************************
 * Arduino Main Loop
 ***********************************/
long publisher_timer;

void loop() {
	if (millis() > publisher_timer) {
		// step 1: request reading from sensor
		sonar.startReadingCM();
		//pause (the sonar datasheet recquires 65 ms)
		delay(70);
		// read data from result register
		sensor_reading = sonar.readData(2);
		//Publish to serial monitor
		Serial.println("Range:");
		Serial.println(sensor_reading);
		//publish twice a second (aprox.)
		publisher_timer = millis() + 500;
	}
}
