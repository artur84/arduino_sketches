#include <DynamixelSerial.h>
#define SERVO_ID 3 //Look for the servo id number written on the servo package
#define LED 13

void setup(){
  Dynamixel.begin(1000000,2);  // Initialize the servo communicatio at 1Mbps and use Arduino pin 2 for DataControl 
  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);
  digitalWrite(LED,Dynamixel.ping(SERVO_ID)); //Returns high if there is an error
  delay(5000);
  digitalWrite(LED,LOW);
  
}

void loop(){
  for(int n=1;n<=200;n++)
  {
    Dynamixel.move(n,500);
    delay(10);
    Dynamixel.ledStatus(n,ON);
    delay(10);
  }
  digitalWrite(LED,digitalRead(LED)-HIGH);
  

}
