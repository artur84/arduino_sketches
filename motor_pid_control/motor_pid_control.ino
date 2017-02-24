/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include "motor_pid_control.h"

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);

//The encoder best performance is achieved when using interrupts
//Pin 18,19,20 and 21 have interruptions in arduino MEGA
Encoder myEnc(LEFT_ENC_A, LEFT_ENC_B);
//   avoid using pins with LEDs attached

Timer t;

//Define a time variable we don't want to do at running time
double inverse_time = 1.0/(READ_ENCODER_RATE*1000.0*ENCODER_PULSES);


void setup() {
	//initialize the variables we're linked to
	Input = analogRead(0);
	Setpoint = 100;
	//turn the PID on
	myPID.SetMode(AUTOMATIC);
	Serial.begin(9600);
	pinMode(LED, OUTPUT);
	t.every(READ_ENCODER_RATE, read_encoder);
}

long oldPosition = -999;
long newPosition =0;

void loop() {

	myPID.Compute();
	analogWrite(3, Output);
	t.update();
}


//This function is called when t Timer is called
void read_encoder() {
	newPosition = myEnc.read();

	double vel = 0;
	double delta_pose = 0;
	delta_pose = (double) oldPosition - newPosition;
	vel = (2 * PI * delta_pose) * inverse_time;
	//Serial.println(vel);
	Serial.println(inverse_time,12);


	digitalWrite(LED, !digitalRead(LED));
	oldPosition = newPosition;
	//Serial.println("hola");
}

void compute_angular_speed(){

}

///***This function takes as input the current and last encoder poses,
// * and gives back the angular velocity of the wheel.
// */
//double wheel_vel_from_encoder() {
//
//}
