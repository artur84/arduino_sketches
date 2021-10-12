int SENSOR = 0;     // Señal del LDR a la entrada A0 de Arduino
int VALOR;      // almacena valor leido de entrada A0
void setup(){
  // entradas analogicas no requieren inicialización
  // Solo inicializamos el puerto serial para ver los datos en la compu.
  Serial.begin(9600);
}


void loop(){
  VALOR = analogRead(SENSOR);   // lee valor de entrada A0
  Serial.print("valor: "); //writes the value to serial monitor. 
  Serial.println(VALOR);
  delay(1000); //waits one second
}
