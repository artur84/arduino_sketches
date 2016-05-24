/*
 * This program gets the reading from a SRF08 sonar (in centimeters)
 * This is a modification of the source code found at ROS web-page
 * http://wiki.ros.org/rosserial_arduino/Tutorials/SRF08%20Ultrasonic%20Range%20Finder
 * The code has been adapted to be used with newer Arduino Versions.
 * Mantainer: arturoescobedo.iq@gmail.com
 */
#include <Wire.h>

/**********************************
*  Functions to control the SONAR
**********************************/
#define CommandRegister 0x00
#define LightSensorRegister 0x00
#define GainRegister 0x01
#define ResultRegisterHigh  0x02//Register 2 is used to store the MSB of the sensor readings
#define ResultRegisterLow  0x03//Register 3 is used to store the LSB of the sensor readings
#define RangeRegister 0x02 //Register 2 is used to set the max range during write

#define ModeInches 0x50
#define ModeCentimeters 0x51
#define ModeMicroSeconds 0x52

#define SensorAddress 0xF8 //the address of my sensor
#define BroadcastAddress 0x00 

#define Range43mm 0x00
#define Range86mm 0x01
#define Range1m   0x18
#define Range6m   0x8C



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
   Serial.println("Ask a reading in centimeters");
   sendCommand(commandRegister, address, 0x51); //centimeters
   //Serial.println("Ask a reading in inches");
   //sendCommand(commandRegister, address, 0x50); //inches
   delay(70);
   
}
  

// Sets maximum range of measurements
void setMaxRange(int address){
   //Serial.println("Ask a reading in centimeters");
   //sendCommand(commandRegister, address, 0x51); //centimeters
 
   Serial.println("Setting maximum range");
    //max range can be The range is ((Range Register x 43mm) + 43mm) so setting the Range Register to 0 (0x00) gives a maximum
    //range of 43mm. Setting the Range Register to 1 (0x01) gives a maximum range of 86mm. More usefully, 24
    //(0x18) gives a range of 1 metre and 140 (0x8C) is 6 metres. Setting 255 (0xFF) gives the original 11 metres
    //(255 x 43 + 43 is 11008mm). There are two reasons you may wish to reduce the range.
   sendCommand(RangeRegister, address, 0xFF); //(0x8C is 6 metres)
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
  Serial.println("in read data"); 
  Wire.requestFrom(address, numBytes);
  // wait for two bytes to return:
  while (Wire.available() < 2 )   {
   // Serial.println("in read data 2"); 
    // wait for result
  }
  Serial.println("in read data"); 
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
int address = 0xF8; //0xF8
int ResultRegister = 0x02;
void setup()
{
  //Initialize serial and wait for port to open:
  Serial.begin(9600); 
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  connect();
  delay(5000);//waits to make sure everything is ok befor
  // prints title with ending line break 
  Serial.println("SONAR 08"); 
  Serial.println("Address:"); 
  Serial.println(address); 
  
  
  //changeAddress(CommandRegister, New_Address);

  //setMaxRange(New_Address);

}


/***********************************
* Gets software revision of SRF Sonar
/************************************/
int getSoft(){
  Wire.beginTransmission(address);
  int software=0;
  Wire.write(CommandRegister);
  Wire.endTransmission();
  Wire.requestFrom(address,1);
  while (Wire.available()<0);
       software=Wire.read();
  return (software);
  
}

/***********************************
* Arduino Main Loop
***********************************/
long publisher_timer;

void loop()
{
  if (millis() > publisher_timer) {

       Serial.println("unit"); 
       setUnit(CommandRegister, address);
    
       //pause
      delay(70);
      Serial.println("delay"); 
      // set register for reading
      setRegister(address, ResultRegister);
      Serial.println("reading"); 
      Serial.println("range:");
      // read data from result register
      sensorReading = readData(address, 2);
      Serial.println("range:"); 
      Serial.println(sensorReading); 
      publisher_timer = millis() + 4000; //publish once a second
  }

 
}
