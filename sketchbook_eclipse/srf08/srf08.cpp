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

void connect() {
	// start I2C bus
	Wire.begin();
}

// Communicates with Sonar to send commands
void sendCommand(int commandRegister, int address, int command) {
	// start I2C transmission:
	Wire.beginTransmission(address);
	// send command:
	Wire.write(commandRegister);
	Wire.write(command);
	// end I2C transmission:
	Wire.endTransmission();
}

// Sets Units for display / storage
void setUnit(int commandRegister, int address) {
	//Serial.println("Ask a reading in centimeters");
	sendCommand(commandRegister, address, 0x51);
	//pause (the sonar datasheet recquires 65 ms)
	delay(70);
}

// Set to read off the register with stored result
void setRegister(int address, int thisRegister) {
	// start I2C transmission:
	Wire.beginTransmission(address);
	// send address to read from:
	Wire.write(thisRegister);
	// end I2C transmission:
	Wire.endTransmission();
}

// Read data from register return result
int readData(int address, int numBytes) {
	int result = 0;        // the result is two bytes long
	// send I2C request for data:
	Wire.requestFrom(address, numBytes);
	// wait for two bytes to return:
	while (Wire.available() < 2) {
		// wait for result
	}
	// read the two bytes, and combine them into one int:
	delay(50);
	result = Wire.read() * 256;
	result += Wire.read();
	// return the result:
	return result;
}

// Optional change Address -
// NEW_ADDRESS can be set to any of
// E0, E2, E4, E6, E8, EA, EC, EE
// F0, F2, F4, F6, F8, FA, FC, FE
void changeAddress(int commandRegister, int NEW_ADDRESS) {
	sendCommand(commandRegister, commandRegister, 0xA0);
	sendCommand(commandRegister, commandRegister, 0xAA);
	sendCommand(commandRegister, commandRegister, 0xA5);
	sendCommand(commandRegister, commandRegister, NEW_ADDRESS);
}

/*************************************
 * Arduino Setup
 **************************************/
void setup() {
	connect();
	changeAddress(CommandRegister, New_Address);	//Don't delete this!!
	New_Address += 4;	//Somehow it is necessary
	//Initialize serial and wait for port to open:
	Serial.begin(9600);
	while (!Serial) {
		; // wait for serial port to connect. Needed for Leonardo only
	}
	delay(5000); //Wait for some seconds to wait serial monitor
	Serial.println("SONAR 08");
	Serial.println("Address:");
	Serial.println(New_Address);
}

/***********************************
 * Arduino Main Loop
 ***********************************/
long publisher_timer;

void loop() {
	if (millis() > publisher_timer) {

		// step 1: request reading from sensor
		setUnit(CommandRegister, New_Address);

		// set register for reading
		setRegister(New_Address, ResultRegister);

		// read data from result register
		sensorReading = readData(New_Address, 2);

		//Publish to serial monitor
		Serial.println("Range:");
		Serial.println(sensorReading);
		//publish twice a second (aprox.)
		publisher_timer = millis() + 500;

	}
}
