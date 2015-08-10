// Do not remove the include below
#include "srf08.h"
/*

 /**********************************
 *  Functions to control the SONAR
 **********************************/
#define CommandRegister 0x00
#define ResultRegister  0x02
int New_Address = 248; //0xF8
float sensorReading = 0;
Sonar sonar;
/*************************************
 * Arduino Setup
 **************************************/
void setup() {
	Serial.begin(9600);
	while (!Serial) {
		; // wait for serial port to connect. Needed for Leonardo only
	}
	delay(5000); //Wait for some seconds to wait serial monitor
	sonar.connect();
	Serial.println("SONAR 08");
	Serial.println("Previous Address:");
	Serial.println(New_Address);
	sonar.changeAddress(New_Address);	//Don't delete this!!
	//Initialize serial and wait for port to open:
	Serial.println("Address:");
	Serial.println(New_Address);
}

/***********************************
 * Arduino Main Loop
 ***********************************/
long publisher_timer;

void loop() {
	if (millis() > publisher_timer) {
		Serial.println("Address:");
		Serial.println(New_Address);
		// step 1: request reading from sensor
		sonar.setUnit();

		// set register for reading
		sonar.setRegister(ResultRegister);

		// read data from result register
		sensorReading = sonar.readData(2);

		//Publish to serial monitor
		Serial.println("Range:");
		Serial.println(sensorReading);
		//publish twice a second (aprox.)
		publisher_timer = millis() + 500;

	}
}
