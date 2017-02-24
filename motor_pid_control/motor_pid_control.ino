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

/***
 * To compute the wheel vel from encoder pulses
 *
 * 			8400 pulses  ----> 1 rev
 * 			delta_pulses ----> delta_rev
 *
 * delta_rev = delta_pulses/8400  [rev]
 *
 *-------------
 * In Radians
 * ------------
 * 			1 rev 	 ----> 2*pi [rad]
 * 	       delta_rev ----> 2*pi*delta_rev [rad] = delta_rad
 *
 * Wheel speed in rad/s
 * wheel_speed[rad/s] = delta_rad[rad]/Delta_t[s]
 *
 * delta_t_s = delta_t_ms/1000
 *
 * wheel_speed[rad/s]=delta_rad/(delta_t_ms/1000)
 * 					 =delta_rad*1000/(delta_t_ms)
 * 					 =2pi*delta_rev*1000/delta_t_ms
 * 					 =2pi*delta_pulses*1000/(8400*delta_t_ms)
 * 					 =(delta_pulses)(2000*pi/(8400*delta_t_ms))
 *wheel_speed[rad/s] =(delta_pulses)*speed_constant
 *
 * (2000*PI)/(ENCODER_PULSES*READ_ENCODER_RATE_MILLIS) ===> speed_constant
 */
const double speed_constant = (2000 * PI)
		/ (ENCODER_PULSES * READ_ENCODER_RATE_MILLIS); //wheel_speed[rad/s] =(Delta_pulses)*speed_constant
double wheel_speed = 0; //wheel_speed[rad/s] =(delta_pulses)*speed_constant

void setup() {
	//initialize the variables we're linked to
	Input = analogRead(0);
	Setpoint = 100;
	//turn the PID on
	myPID.SetMode(AUTOMATIC);
	Serial.begin(9600);
	pinMode(LED, OUTPUT);
	t.every(READ_ENCODER_RATE_MILLIS, read_wheel_vel);
	/*** Enable motor ***/
	pinMode(LEFT_MOT_EN, OUTPUT);
	pinMode(LEFT_MOT_DIR_BACK, OUTPUT);
	pinMode(LEFT_MOT_DIR_FRONT, OUTPUT);
	digitalWrite(LEFT_MOT_DIR_BACK, HIGH);
	digitalWrite(LEFT_MOT_DIR_FRONT, LOW);
}

long oldPosition = -999;
long newPosition = 0;

void loop() {

	myPID.Compute();
	analogWrite(LEFT_MOT_EN, 255);
	t.update();
}

//This function is called when t Timer is called
void read_wheel_vel() {
	newPosition = myEnc.read();

	double delta_pulses = 0;
	delta_pulses = (double) newPosition - oldPosition;
	wheel_speed = delta_pulses * speed_constant;
	//Serial.println(vel);
	Serial.println(radpsec2rpm(wheel_speed), 12);

	digitalWrite(LED, !digitalRead(LED));
	oldPosition = newPosition;
	//Serial.println("hola");
}

/*** Takes wheel speed in rad/s and returns it in rpm
 *
 * 			vel_radpsec [rad]     60 [sec]     1 [rev]
 * vel_rpm= -----------------  *  --------- * -----------
 * 			      [sec]            1[min]       2PI [rad]
 *
 */
double radpsec2rpm(double vel_radpsec) {

	double vel_rpm = ((vel_radpsec) * 60) / (2 * PI);
	return vel_rpm;
}

