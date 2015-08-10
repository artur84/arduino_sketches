/*
 * This program gets the reading from a SRF08 sonar (in centimeters)
 * and sends this information to the ROS-topic /arduino/sonar
 * This is a modification of the source code found at ROS web-page
 * http://wiki.ros.org/rosserial_arduino/Tutorials/SRF08%20Ultrasonic%20Range%20Finder
 * The code has been adapted to be used with newer Arduino Versions.
 * Mantainer: arturoescobedo.iq@gmail.com
 */
//#define USE_USBCON //Only necessary for arduino leonardo
#include <Arduino.h>
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32.h>


/*******************************
* ROS Setup
*******************************/
std_msgs::Float32 sonar_msg;
ros::Publisher pub_sonar("arduino/sonar", &sonar_msg);
ros::NodeHandle nh;


/**********************************
*  Functions to control the SONAR
**********************************/
#define CommandRegister 0x00
#define ResultRegister  0x02
int New_Address = 248; //0xF8
float sensorReading =0;

void connect(){
  // start I2C bus
  Wire.begin();
}

// Communicates with Sonar to send commands
void sendCommand(int commandRegister, int address, int command){
  // start I2C transmission:
  Wire.beginTransmission(address);
  // send command:
  Wire.write(commandRegister);
  Wire.write(command);
  // end I2C transmission:
  Wire.endTransmission();
}


// Sets Units for display / storage
void setUnit(int commandRegister, int address){
   //Serial.println("Ask a reading in centimeters");
   sendCommand(commandRegister, address, 0x51);
   //pause (the sonar datasheet recquires 65 ms)
   delay(70);
}
  

// Set to read off the register with stored result
void setRegister(int address, int thisRegister){
  // start I2C transmission:
  Wire.beginTransmission(address);
  // send address to read from:
  Wire.write(thisRegister);
  // end I2C transmission:
  Wire.endTransmission();
}  


// Read data from register return result
int readData(int address, int numBytes){
  int result = 0;        // the result is two bytes long
  // send I2C request for data:
  Wire.requestFrom(address, numBytes);
  // wait for two bytes to return:
  while (Wire.available() < 2 )   {
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
void changeAddress(int commandRegister, int NEW_ADDRESS){
  sendCommand(commandRegister,commandRegister,0xA0);
  sendCommand(commandRegister,commandRegister,0xAA);
  sendCommand(commandRegister,commandRegister,0xA5);
  sendCommand(commandRegister,commandRegister,NEW_ADDRESS);
} 

/*************************************
* Arduino Setup
**************************************/
void setup()
{
  connect();
  changeAddress(CommandRegister, New_Address);
  New_Address += 4;
  nh.initNode();
  nh.advertise(pub_sonar);

}

/***********************************
* Arduino Main Loop
***********************************/
long publisher_timer;

void loop()
{
  if (millis() > publisher_timer) {

   // step 1: request reading from sensor
   setUnit(CommandRegister, New_Address);

  // set register for reading
  setRegister(New_Address, ResultRegister);

  // read data from result register
  sensorReading = readData(New_Address, 2);
  
  //Publish to ROS topic
  sonar_msg.data = sensorReading;
  pub_sonar.publish(&sonar_msg);
  
  //publish twice a second (aprox.)
  publisher_timer = millis() + 500; 

  }
  nh.spinOnce();
}
