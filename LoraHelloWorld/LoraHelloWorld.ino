/*
  Lora Send And Receive
  This sketch demonstrates how to send and receive data with the MKR WAN 1300/1310 LoRa module.
  This example code is in the public domain.
*/

#include <MKRWAN.h>

LoRaModem modem;

// Uncomment if using the Murata chip as a module
// LoRaModem modem(Serial1);

#include "arduino_secrets.h"
// Please enter your sensitive data in the Secret tab or arduino_secrets.h
String appEui = SECRET_APP_EUI;
String appKey = SECRET_APP_KEY;
// change this to your regional band (eg. , US915_HYBRID (Urbit GateWay), US915, AS923, EU868 ...)
_lora_band region = US915_HYBRID;
// Variables to build the message
  int n=200; //Just a number 
  String msg; //To get the value of the number as a string
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial);
 
  if (!modem.begin(region)) {
    Serial.println("Failed to start module");
    while (1) {}
  };
  Serial.print("Your module version is: ");
  Serial.println(modem.version());
  Serial.print("Your device EUI is: ");
  Serial.println(modem.deviceEUI());
  Serial.println("Connecting ...");
  int connected = modem.joinOTAA(appEui, appKey);
  if (!connected) {
    Serial.println("Something went wrong; are you indoor? Move near a window and retry");
    while (1) {}
  }
  Serial.println("Successfully joined the network");
  Serial.println("Enabling ADR and setting the spreading factor");
  modem.setADR(true);
  modem.dataRate(5);
  // Set poll interval to 60 secs.
 // modem.minPollInterval(60);
  // NOTE: independent of this setting, the modem will
  // not allow sending more than one message every 2 minutes,
  // this is enforced by firmware and can not be changed.
}

void loop() {
  
  modem.beginPacket();
  n=n+1;
  msg=String(n); //Convert the number to a string
  modem.print(msg);
  int err = modem.endPacket(true);
  if (err > 0) {
    Serial.println("Message sent correctly!");
  } else {
    Serial.println("Error sending message :(");
    Serial.println("(you may send a limited amount of messages per minute, depending on the signal strength");
    Serial.println("it may vary from 1 message every couple of seconds to 1 message every minute)");
  }
  delay(1000*60);

}
