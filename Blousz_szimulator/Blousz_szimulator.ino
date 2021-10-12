#include <MKRWAN.h>

LoRaModem modem;


bool iniLoop = true;

//delay de 30 minutos, tres ciclos de 10 minutos
int minutes = 1; //Was 5
int decenas = 1;

#include "arduino_secrets.h"

String appEui = SECRET_APP_EUI;
String appKey = SECRET_APP_KEY;

void setup() {
  
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  while (!Serial);
  delay(2000);
  //if (!modem.begin(EU868)) 
  if (!modem.begin(US915_HYBRID)) 
  {
    Serial.println("El módem no se pudo inicializar.");
    while (1) {}
  };
  
  
  Serial.print("EUI: ");
  Serial.println(modem.deviceEUI());

  int connected = modem.joinOTAA(appEui, appKey);
  if (!connected) {
    Serial.println("¡No funcionó el Join! Prueba otra vez.");
    setup();
  }
  modem.minPollInterval(70);
  modem.setADR(true);
  modem.dataRate(4);
  
 
}

void loop() {

  Serial.println();
  
  byte data1[]={26,228,142,1,2,200,0,204,0,0,0,193,185,32,3,33,4,40,0,21,20,7,0,1,64,25,30,0,2,158,6,0,3,3,170,11,5,0,10,0,0,1,0,251,1,12,57,13,2,13,93,13,250,1,13,65,9,64,9,69,9,4,0,14,0,0,0,248};
  
  
  
  int err;
  modem.beginPacket();
  modem.write(data1,64);
  err = modem.endPacket(true);
  if (err > 0) {
    Serial.println("Mensaje enviado");
  } else {
    Serial.println("No se pudo enviar el mensaje");
    Serial.println("No hay necesidad de alarmarse, este Arduino tampoco está tonizado.");
  }
  delay(1000);
  if (!modem.available()) {
    Serial.println("No se recibió algún Downlink");
    for (int d = 0; d <= decenas ; d++) // d ==> Decenas de minutos
    {
      delay((10 * minutes) * 1000);     //Minutos //Changed 15 to 60 
    }
    return;
  }
  char rcv[64];
  int i = 0;
  while (modem.available()) {
    rcv[i++] = (char)modem.read();
  }
  Serial.print("Mensaje recibido: ");
  for (unsigned int j = 0; j < i; j++) {
    Serial.print(rcv[j] >> 4, HEX);
    Serial.print(rcv[j] & 0xF, HEX);
    Serial.print(" ");
  }
  if (rcv[0]==0) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  
  Serial.println();
}

void onLed(int times, int on, int off)
{

}
