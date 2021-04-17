/* Program to read one  encoder connected to pin 2 and 3 of arduino UNO.
 */
#include <Encoder.h>

//The encoder best performance is achieved when using interrupts
Encoder myEnc(2,3);
//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(115200);
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
