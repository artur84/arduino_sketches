
#include <SoftwareSerial.h>

SoftwareSerial bluetoothSerial(10, 11); // RX, TX

void setup()  
{
   // set the data rate for the SoftwareSerial port
  bluetoothSerial.begin(9600);
  bluetoothSerial.println("Bluetooth serial started");
}

void loop() // run over and over
{
    if (!(millis()%100)){
    bluetoothSerial.println("I'm o.k");
    }
}

