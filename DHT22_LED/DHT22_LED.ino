/*
  Capitulo 7 de Arduino desde cero en Español
  Programa que utiliza el sensor DHT22 o DHT11 para obtener datos de temperatura
  y humedad. Deben instalarse la Librerias DHT Sensor Library y Adafruit Unified Sensor.

  Autor: bitwiseAr  

*/

#include <DHT.h>    // importa la Librerias DHT
#include <DHT_U.h>

int SENSOR = 2;     // pin DATA de DHT22 a pin digital 2
int TEMPERATURA;
int HUMEDAD;
int LED=13; // LED conectado al pin 13 
DHT dht(SENSOR, DHT22);   // creacion del objeto, cambiar segundo parametro
        // por DHT11 si se utiliza en lugar del DHT22
void setup(){
  pinMode(LED, OUTPUT);    // sets the digital pin 13 as output
  Serial.begin(9600);   // inicializacion de monitor serial
  dht.begin();      // inicializacion de sensor
}

void loop(){
  TEMPERATURA = dht.readTemperature();  // obtencion de valor de temperatura
  HUMEDAD = dht.readHumidity();   // obtencion de valor de humedad
  Serial.print("Temperatura: ");  // escritura en monitor serial de los valores
  Serial.print(TEMPERATURA);
  Serial.print(" Humedad: ");
  Serial.println(HUMEDAD);
  if (TEMPERATURA >=28)
  {
    digitalWrite(LED,1);
  }
  else
  {
    digitalWrite(LED,0);
  }
  delay(500);
}
