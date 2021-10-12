/*
  Capitulo 14 de Arduino desde cero en Español.
  Programa que utiliza el modulo KY-018 o fotoresistencia para encender un LED
        cuando hay baja intensidad de luz ambiente.

  Autor: bitwiseAr  

*/

int SENSOR = A0;     // pin S de modulo a entrada analogica A0
int LED = 3;      // LED en pin 3
int VALOR;      // almacena valor leido de entrada A0

void setup(){
  pinMode(LED, OUTPUT);   // pin 3 como salida
  // entradas analogicas no requieren inicialización
}


void loop(){
  VALOR = analogRead(SENSOR); // lee valor de entrada A0
  if (VALOR < 930){   // un valor bajo representa oscuridad
    digitalWrite(LED, HIGH);  // enciende LED
    //delay(1000);    // demora de 1 seg. para evitar parpadeo de LED
  }
  else if (VALOR > 950)
  {
  digitalWrite(LED, LOW); // apaga LED
  }
}
