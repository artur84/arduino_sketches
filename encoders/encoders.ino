/* Program to read one  encoder connected to pin 2 and 3 of arduino UNO.
 */
#include <Encoder.h>

//The encoder best performance is achieved when using interrupts
//Pin 18,19,20 and 21 have interruptions in arduino MEGA

Encoder myEnc(2,3);
//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
}

long oldPosition  = -999;

void loop() {
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition);
  }
}
