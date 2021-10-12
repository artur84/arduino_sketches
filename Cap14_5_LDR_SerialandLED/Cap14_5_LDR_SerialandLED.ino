/*
  Capitulo 14 de Arduino desde cero en Español.
  Programa que utiliza el modulo KY-018 o fotoresistencia para cambiar el brillo
  del LED de forma inversamente proporcional a la luz captada.

  Autor: bitwiseAr  

*/

int SENSOR = 0;     // pin S de modulo a entrada analogica A0
int LED = 3;      // LED en pin 3
int VALOR;      // almacena valor leido de entrada A0
int PWM;
int MAX_VALUE =240;
int MIN_VALUE=10;
void setup(){
  pinMode(LED, OUTPUT);   // pin 3 como salida
  // entradas analogicas no requieren inicialización
  // initialize the serial communication:
  Serial.begin(9600);
}

void loop(){
  VALOR = analogRead(SENSOR);   // lee valor de entrada A0
  Serial.print("valor: "); //writes the value to serial monitor. 
  Serial.println(VALOR);
  if (VALOR>=MAX_VALUE) 
  {
    VALOR=MAX_VALUE;
  }
  if (VALOR<=MIN_VALUE)
  {
    VALOR=MIN_VALUE;
  }
  PWM = map(VALOR, MIN_VALUE, MAX_VALUE, 255, 0);  // convierte valores de entrada 0-1023 a 255-0 para brillo
  analogWrite(LED, PWM);    // escribe valor al LED
  delay(500); //waits one second
}
