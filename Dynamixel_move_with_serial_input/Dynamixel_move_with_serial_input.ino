#include <DynamixelSerial1.h>
#define ID 3  //Check the ID in the label of the servo
int Temperature,Voltage,Position; 

void setup(){
  Serial.begin(9600);          // Begin Serial Comunication
  Dynamixel.begin(1000000,2);  // Initialize the servo at 1Mbps and Pin Control 2; if the servo does not move
                               // try changing the speed to other values as (9600,19200,57600,115200,200000,500000)
  delay(1000);
}

void loop(){

  long pos;
  Serial.println("Write the desired position from 0 to 1023");  
  // check if data has been sent from the computer:
  while (Serial.available()==0) { //Wait until we receive some data
  }
  pos=Serial.parseInt();
  Serial.print("postion: "); //Writes the position to the terminal
  Serial.println(pos);
  Dynamixel.move(ID,pos);  // Move the Servo radomly from 0 to 1023
  delay(1000);
  Temperature = Dynamixel.readTemperature(ID); // Request and Print the Temperature
  Voltage = Dynamixel.readVoltage(ID);         // Request and Print the Voltage
  Position = Dynamixel.readPosition(ID);       // Request and Print the Position 
 
  Serial.print(" *** Temperature: ");   // Print the variables in the Serial Monitor
  Serial.print(Temperature);
  Serial.print(" Celcius  Voltage: "); //The value in Volts is 10 times higher than the real value so we need to devide by 10
  Serial.print((double)Voltage/10);
  Serial.print("  Volts   Position: ");
  Serial.print(Position);
  Serial.println(" of 1023 resolution");
  delay(4000);

}

