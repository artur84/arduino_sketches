#include "settings.h"
#include "settingsIO.h"
#include "settingsROS.h"

Encoder sensorR(ENCODER_R_A, ENCODER_R_B);
Encoder sensorL(ENCODER_L_A, ENCODER_L_B);

//Creates a PID controller linked to the specified Input, Output, and Setpoint
//double kp = 55, ki = 2.0, kd = 1.2;
double kp = 10, ki = 180.0, kd = 0.8;

PID rPID(&rads_r, &rPWM, &rsetPoint ,kp, ki, kd, DIRECT);
PID lPID(&rads_l, &lPWM, &lsetPoint ,kp, ki, kd, DIRECT);


void setup() {
  setGPIO();
//  setGPIOMonster();
//  motorOff(0);
//  motorOff(1);
  turnOffMotors();
  
//  Timer1.initialize(10000); //set timer 0.01 [s] or 100 Hz
//  Timer1.attachInterrupt( timerIsr ); // attach the service routine 
  Timer3.attachInterrupt(timerIsr); // attach the service routine 
  Timer3.start(10000);              //set timer 0.01 [s] or 100 Hz
  
  initROS();
  
  rPID.SetOutputLimits(-255, 255);
  rPID.SetSampleTime(10);
  rPID.SetMode(AUTOMATIC);
  lPID.SetOutputLimits(-255, 255);
  lPID.SetSampleTime(10);
  lPID.SetMode(AUTOMATIC);

}

void loop() {

  rPID.Compute();
  lPID.Compute();
//  motorGo(0, CW, 254);
//  setMotorLeft((uint8_t)abs(0), MOTOR_FRONT);
//  setMotorRight((uint8_t)abs(0), MOTOR_FRONT);
 
  if (rPWM >= 0){
    setMotorRight((uint8_t)abs(rPWM), MOTOR_FRONT);
    //motorGo(0, CW, (uint8_t)abs(rPWM));
  } else{
    setMotorRight((uint8_t)abs(rPWM), MOTOR_BACK);
    //motorGo(0, CCW, (uint8_t)abs(rPWM));
  }

  if (lPWM >= 0){
    //motorGo(1, CW, (uint8_t)abs(lPWM));
    setMotorLeft((uint8_t)abs(lPWM), MOTOR_FRONT);
  } else{
    setMotorLeft((uint8_t)abs(lPWM), MOTOR_BACK);
    //motorGo(1,CCW, (uint8_t)abs(lPWM));
  }  

//  motorGo(0, CW, 255);
//  motorGo(1, CW, 255);
  rads_r = 2*PI*rps_r;
  rads_l = 2*PI*rps_l;

  MsgROS();
  
  delay(1);
}


void timerIsr(){
  currentTime = currentTime + 0.01;
  pps_r = sensorR.read();
  sensorR.write(0);
  rps_r = (float) pps_r*CONST_RPS;  //(100Hz)*pps/2100ppr
  
  pps_l = sensorL.read();
  sensorL.write(0);
  rps_l = (float) pps_l*CONST_RPS;  //(100Hz)*pps/2100ppr
  odometry(pps_r, pps_l);
}

