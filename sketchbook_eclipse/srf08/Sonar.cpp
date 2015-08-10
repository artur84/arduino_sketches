/*
 * Sonar.cpp
 *
 *  Created on: Jul 31, 2015
 *      Author: arturo
 */

#include "Sonar.h"

Sonar::Sonar() {
	address_ = SONAR_DEFAULT_ADD; //Default Address is 0xF8

}

Sonar::~Sonar() {

}

void Sonar::connect() {
	// start I2C bus
	Wire.begin();
}

// Communicates with Sonar to send commands
void Sonar::sendCommand(int address, int commandRegister, int command) {
	// start I2C transmission:
	Wire.beginTransmission(address);
	// send command:
	Wire.write(commandRegister);
	Wire.write(command);
	// end I2C transmission:
	Wire.endTransmission();
}

/*
 * Starts a measurement, don't forget to wait at least 70ms
 * before calling readResult()
 */
// Sets Units for display / storage
void Sonar::setUnit(int commandRegister, int address) {
	//Serial.println("Ask a reading in centimeters");
	Sonar::sendCommand(address, commandRegister, 0x51);
	//pause (the sonar datasheet recquires 65 ms)
	delay(70);
}

// Sets maximum range of measurements
//void Sonar::setMaxRange11m() {
//max range can be The range is ((Range Register x 43mm) + 43mm) so setting the Range Register to 0 (0x00) gives a maximum
//range of 43mm. Setting the Range Register to 1 (0x01) gives a maximum range of 86mm. More usefully, 24
//(0x18) gives a range of 1 metre and 140 (0x8C) is 6 metres. Setting 255 (0xFF) gives the original 11 metres
//(255 x 43 + 43 is 11008mm). There are two reasons you may wish to reduce the range.
//sendCommand(SONAR_RANGE_REG, SONAR_RANGE11M); //(0x8C is 6 metres)
//}

// Set to read off the register with stored result
void Sonar::setRegister(int address, int thisRegister) {
	// start I2C transmission:
	Wire.beginTransmission(address);
	// send address to read from:
	Wire.write(thisRegister);
	// end I2C transmission:
	Wire.endTransmission();
}

/*
 * Read the result, yo have to call startMeasurement() at least 70ms before
 * calling this.
 */
// Read data from register return result
int Sonar::readData(int address, int numBytes) {
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
// Optional change Address -
// NEW_ADDRESS can be set to any of
// E0, E2, E4, E6, E8, EA, EC, EE
// F0, F2, F4, F6, F8, FA, FC, FE
void Sonar::changeAddress(int NEW_ADDRESS) {
	sendCommand(SONAR_BCAST_ADD, SONAR_COMMAND_REG, 0xA0);
	sendCommand(SONAR_BCAST_ADD, SONAR_COMMAND_REG, 0xAA);
	sendCommand(SONAR_BCAST_ADD, SONAR_COMMAND_REG, 0xA5);
	sendCommand(SONAR_BCAST_ADD, SONAR_COMMAND_REG, NEW_ADDRESS);
}

/*
 * Reads the software revision register, if it is 0xFF, then the lecture is still in progress
 */
//bool Sonar::isReady(void) {
//	int numBytes = 1;
//	int result = 0;        // the result is one byte long
//	// set register for reading
//	setRegister (SONAR_SOFTWARE_REV_REG);
//	// send I2C request for data:
//	Wire.requestFrom(address_, numBytes);
//	// wait for two bytes to return:
//	while (Wire.available() < 1) {
//		// wait for result
//	}
//	// read the two bytes, and combine them into one int:
//	result = Wire.read();
//	if (result == 0xFF)
//		return true;
//	else
//		return false;
//}
