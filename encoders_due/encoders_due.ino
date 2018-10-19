/* Program to read one  encoder connected to an arduino DUE */
/***
 * This program reads an encoder and prints the value in revolutions to the serial monitor.
 */
#include <Encoder.h>
#define pin_enc_A 51  //Pin of the arduino where the encoder port A is connected
#define pin_enc_B 53  //Pin where the encoder port B is connected
# define CPR 4480.0 //Counts per revolution of the encoder

//The encoder best performance is achieved when using interrupts
Encoder myEnc(pin_enc_A,pin_enc_B);
//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
}

long oldPosition  = -999;
double revolutions = 0;
void loop() {
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    revolutions= (double) newPosition/CPR;
    Serial.println(newPosition);
    Serial.println(revolutions);
  }
}
