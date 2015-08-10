/*
 * Sonar.h
 *
 *  Created on: Jul 31, 2015
 *      Author: arturo
 */

#ifndef SONAR_H_
#define SONAR_H_
#include "Arduino.h"
#include <Wire.h>

//Registers
#define SONAR_COMMAND_REG 0x00
#define SONAR_SOFTWARE_REV_REG 0x00
#define SONAR_LIGHT_SENSOR_REG 0x01
#define SONAR_GAIN_REG 0x01
#define SONAR_RANGE_REG 0x02 //Register 2 is used to set the max range during write
#define SONAR_RESULT_REG_HIGH  0x02//Register 2 is used to store the MSB of the sensor readings
#define SONAR_RESULT_REG_LOW 0x03//Register 3 is used to store the LSB of the sensor readings
//Commands
#define SONAR_MODE_INCHES 0x50
#define SONAR_MODE_CM 0x51
#define SONAR_MODE_MS 0x52
#define SONAR_CHANGE_ADD_FIRST 0xA0 //first in sequence to change I2C address
#define SONAR_CHANGE_ADD_SECOND 0xAA //Second in sequence to change I2C address
#define SONAR_CHANGE_ADD_THIRD 0xA5 //Third in sequence to change I2C address

#define SONAR_DEFAULT_ADD 248 //the address of my sensor
#define SONAR_BCAST_ADD 0x00

#define SONAR_RANGE43MM 0x00
#define SONAR_RANGE86MM 0x01
#define SONAR_RANGE1M   0x18
#define SONAR_RANGE6M   0x8C
#define SONAR_RANGE11M  0xFF
class Sonar {
	int address_; //The address of this device
public:
	Sonar();
	virtual ~Sonar();
	void connect();
	void sendCommand(int address, int commandRegister, int command);
	// Sets Units for display / storage
	void setUnit();
	// Sets maximum range of measurements
	void setMaxRange11m();
	// Set to read off the register with stored result
	void setRegister(int thisRegister);
	// Read data from register return result
	int readData(int numBytes);
	// Optional change Address -
	// NEW_ADDRESS can be set to any of
	// E0, E2, E4, E6, E8, EA, EC, EE
	// F0, F2, F4, F6, F8, FA, FC, FE
	void changeAddress(int NEW_ADDRESS);
	bool isReady(void);
}
;

#endif /* SONAR_H_ */
