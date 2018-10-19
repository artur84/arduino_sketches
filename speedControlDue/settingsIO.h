void setGPIO(){
  //-------- setting GPIO --------//
  pinMode(LED, OUTPUT);
/*  
  pinMode(ENCODER_R_A, INPUT);
  pinMode(ENCODER_R_B, INPUT);
  pinMode(ENCODER_L_A, INPUT);
  pinMode(ENCODER_L_B, INPUT);
*/
  pinMode(RIGHT_MOT_DIR_BACK, OUTPUT);
  pinMode(RIGHT_MOT_DIR_FRONT, OUTPUT);
  pinMode(RIGHT_MOT_EN, OUTPUT);
  pinMode(LEFT_MOT_DIR_BACK, OUTPUT);
  pinMode(LEFT_MOT_DIR_FRONT, OUTPUT);
  pinMode(LEFT_MOT_EN, OUTPUT);


  pinMode(ENCODER_R_PWR, OUTPUT);
  pinMode(ENCODER_R_GND, OUTPUT);
//  pinMode(ENCODER_R_A, INPUT);
//  pinMode(ENCODER_R_B, INPUT);
  pinMode(ENCODER_L_PWR, OUTPUT);
  pinMode(ENCODER_L_GND, OUTPUT);
//  pinMode(ENCODER_L_A, INPUT);
//  pinMode(ENCODER_L_B, INPUT);

  digitalWrite(ENCODER_R_PWR, HIGH);
  digitalWrite(ENCODER_R_GND, LOW);
  digitalWrite(ENCODER_L_PWR, HIGH);
  digitalWrite(ENCODER_L_GND, LOW);
}

void setMotorRight(uint8_t pwm, uint8_t mode){
  
 if (mode==0){
    digitalWrite(RIGHT_MOT_DIR_BACK, LOW);
    digitalWrite(RIGHT_MOT_DIR_FRONT, HIGH);
  }else if(mode==1){
    digitalWrite(RIGHT_MOT_DIR_BACK, HIGH);
    digitalWrite(RIGHT_MOT_DIR_FRONT, LOW);
  }else{
    digitalWrite(RIGHT_MOT_DIR_BACK, LOW);
    digitalWrite(RIGHT_MOT_DIR_FRONT, LOW);
  }
  
  analogWrite(RIGHT_MOT_EN, pwm);
}

void  setMotorLeft(uint8_t pwm, uint8_t mode){

 if (mode==0){
    digitalWrite(LEFT_MOT_DIR_BACK, LOW);
    digitalWrite(LEFT_MOT_DIR_FRONT, HIGH);
  }else if(mode==1){
    digitalWrite(LEFT_MOT_DIR_BACK, HIGH);
    digitalWrite(LEFT_MOT_DIR_FRONT, LOW);
  }else{
    digitalWrite(LEFT_MOT_DIR_BACK, LOW);
    digitalWrite(LEFT_MOT_DIR_FRONT, LOW);
  }
  
  analogWrite(LEFT_MOT_EN, pwm);
  
}

void turnOffMotors(){

  setMotorRight(0, MOTOR_STOP);
  setMotorLeft(0, MOTOR_STOP);

}


void setGPIOMonster(){
/*    // Initialize digital pins as outputs
  for (int i=0; i<2; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }
  // Initialize braked
  for (int i=0; i<2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  */
}

void motorOff(int motor)
{/*
  // Initialize braked
  for (int i=0; i<2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  analogWrite(pwmpin[motor], 0);
  */
}

/* motorGo() will set a motor going in a specific direction
 the motor will continue going in that direction, at that speed
 until told to do otherwise.
 
 motor: this should be either 0 or 1, will selet which of the two
 motors to be controlled
 
 direct: Should be between 0 and 3, with the following result
 0: Brake to VCC
 1: Clockwise
 2: CounterClockwise
 3: Brake to GND
 
 pwm: should be a value between ? and 1023, higher the number, the faster
 it'll go
 */
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{/*
  if (motor <= 1)
  {
    if (direct <=4)
    {
      // Set inA[motor]
      if (direct <=1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct==0)||(direct==2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
  }
  */
}
