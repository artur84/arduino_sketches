/*
  Capitulo 14 de Arduino desde cero en Español.
  Programa que utiliza el modulo KY-018 o fotoresistencia para cambiar el brillo
  del LED de forma proporcional a la luz captada.

  Autor: bitwiseAr  

*/

int SENSOR = A0;     // pin S de modulo a entrada analogica A0
int LED = 3;      // LED en pin 3
int VALOR;      // almacena valor leido de entrada A0
int PWM;

void setup(){
  pinMode(LED, OUTPUT);   // pin 3 como salida
  // entradas analogicas no requieren inicialización
}


void loop(){
  VALOR = analogRead(SENSOR);   // lee valor de entrada A0
  PWM = map(VALOR, 0, 80, 0, 255);  // convierte valores de entrada 0-1023 a 255-0 para brillo
  analogWrite(LED, PWM);    // escribe valor al LED
}
