/***
 * This program is used to reset any servo. 
 */
#include <DynamixelSerial1.h>
#define ID 3  //Check the ID in the label of the servo


void setup(){
  Dynamixel.begin(1000000,2);  // Initialize the servo at 1Mbps and Pin Control 2; if the servo does not move
                        // try changing the speed to other values as (9600,19200,57600,115200,200000,500000)
  delay(1000);
  Dynamixel.reset(ID); 
  delay(1000);
  Dynamixel.setID(1,ID);  //After the reset the ID changes to 1 so we need to change it back to let it as it was. 
  delay(1000);


}

void loop(){

  //Make te servo move
  for(int n=0;n<=1023;n+=2){
    Dynamixel.move(ID,n);  // Move the Servo to 200
    delay(200);
  }


}
