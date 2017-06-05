/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include "motor_pid_control.h"

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double filter_vect[ENCODER_FILTER_SIZE];
int index = 0;
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
double wheel_speed_rad = 0; //wheel_speed[rad/s] =(delta_pulses)*speed_constant

void setup() {
	//initialize the variables we're linked to
	Input = analogRead(0);
	Setpoint = 100;
	//turn the PID on
	myPID.SetMode(AUTOMATIC);
	Serial.begin(115200);
	pinMode(LED, OUTPUT);
	t.every(READ_ENCODER_RATE_MILLIS, read_wheel_vel);
	/*** Enable motor ***/
	pinMode(LEFT_MOT_EN, OUTPUT);
	pinMode(LEFT_MOT_DIR_BACK, OUTPUT);
	pinMode(LEFT_MOT_DIR_FRONT, OUTPUT);
	digitalWrite(LEFT_MOT_DIR_BACK, LOW);
	digitalWrite(LEFT_MOT_DIR_FRONT, HIGH);
}

long oldPosition = -999;
long newPosition = 0;

void loop() {

	//myPID.Compute();
	analogWrite(LEFT_MOT_EN, 255);
	//Serial.println(vel);
	if (millis() % 1000 == 1) {
		double display_speed = wheel_speed_rad;
		Serial.println(radpsec2rpm(display_speed), 2);
	}
	t.update();
}

//This function is called when t Timer is called
void read_wheel_vel() {
	newPosition = myEnc.read();
	double speed_rad=0;
	long delta_pulses = 0;
	delta_pulses = (long) newPosition - oldPosition;
	speed_rad = delta_pulses * speed_constant;
	wheel_speed_rad = encoder_filter(speed_rad);
	//update old position
	oldPosition = newPosition;
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

/*** Takes wheel speed in rad/s and returns it in rpm
 *
 * 1 rev = 2*PI*radius [m]
 *
 * 			vel_radpsec [rad]      1 [rev]     2PI* radius[m]
 * vel_rpm= -----------------  *  --------- * ------------------ *
 * 			      [sec]           2PI [rad]        1 [rev]
 *
 */
double radpsec2meterpsec(double vel_radpsec) {
	double vel_meterpsec = vel_radpsec * WHEELRAD; //WHEELRAD is in meters
	return vel_meterpsec;
}

/*** Filters encoder measurements with a movin median filter
 *
 */
double encoder_filter(double new_measurement) {
	double sum = 0, average = 0;
	/***
	 * --> move vector to the right
	 *   [__,_i_,_i+1_,__]
	 */
	for (int i = 0; i < ENCODER_FILTER_SIZE - 1; ++i) {
		filter_vect[i + 1] = filter_vect[i];
	}
	filter_vect[0] = new_measurement;
	/***
	 * Compute the average
	 */
	for (int i = 0; i < ENCODER_FILTER_SIZE; ++i) {
		sum = filter_vect[i] + sum;
	}
	average = sum / ENCODER_FILTER_SIZE;
	return average;
}
