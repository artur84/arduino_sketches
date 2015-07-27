/*
 * Moves a stepper motor using the adafruit motor shield.
 * mantainer: arturoescobedo.iq@gmail.com
 */

#include <AFMotor.h>

/*
 * Global variables
 */   
int ppr = 20; //pulses per revolution
AF_Stepper motor(ppr, 1); // create stepper motor connected to (M1 y M2)->PORT1
                          //20 pulses per revolution

/*
 * My functions
 */

/*
 * Arduino SETUP
 */
void setup() {
  motor.setSpeed(200); // set the speed rpm
  
}

/*
 * Arduino MAIN LOOP
 */
void loop() {   
  //I will just keep the loop waiting for a message 
  //in the ros topic 
  motor.step(30*ppr,FORWARD,DOUBLE);
  delay(1000);
  motor.step(30*ppr,BACKWARD,DOUBLE);
  delay(1000);
}
