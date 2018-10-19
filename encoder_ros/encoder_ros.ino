// Do not remove the include below
#include "encoders_ros.h"
/*
 * This program hears a Twist message in a ROS topic
 * and commands two motors according to the linear.x
 * value of the received Twist.
 *
 * We send the encoders information as a Vector3 message, where the first
 * element is the left wheel data and the second element is the right wheel data.
 * mantainer: arturoescobedo.iq@gmail.com
 */

/*
 * Global variables
 */
ros::NodeHandle nh;
std_msgs::String ok_rosstr;
std_msgs::Int32 encoder_counter;
std_msgs::Float32 float1;
volatile float linear_vel = 0, angular_vel = 0;
long oldPosition = 0; //long is a 4 bytes type
long newPosition = 0;
unsigned long newTime = 0, lastTime = 0,
		delta_t = 0;
long last_pos_l;
double vl_desired;

//Define Variables we'll be connecting to
double vl_measured, vl_controlled;


Encoder myEncI(ENC_A, ENC_B);

/*
 * My functions
 */



/***This function takes as input the current and last encoder poses,
 * and gives back the angular velocity of the wheel.
 */
double wheel_vel_from_encoder(long curr_pos, long last_pos, long delta_t_us) {
	double vel = 0;
	double delta_pose = 0;
	//First check if denominator is not cero to avoid not defined operations
	if (delta_t_us > 0) {
		delta_pose = (double) last_pos - curr_pos;
		vel = (2 * PI * delta_pose) / (delta_t_us * ENCODER_CPR);
	}
	return vel;
}






ros::Publisher str_pub("arduino/str_output", &ok_rosstr);
ros::Publisher encoder_counter_pub("arduino/lwheel", &encoder_counter);
ros::Publisher pub_1("arduino/pub_1", &float1);

/*
 * Arduino SETUP
 */
void setup() {
	nh.getHardware()->setBaud(57600); //The HC06 and 05 use by default 9600 baud rate
	nh.initNode();

	//Advertise ROS topics
	nh.advertise(str_pub);
	nh.advertise(encoder_counter_pub);
	nh.advertise(pub_1);

	ok_rosstr.data = "arduino ok";



}

/*
 * Arduino MAIN LOOP
 */
void loop() {
	
	newPosition = myEncI.read();
	newTime = micros();



	if (newPosition != oldPosition) {
		delta_t = newTime - lastTime;

		vl_measured = wheel_vel_from_encoder(newPosition, oldPosition,
				delta_t);

		lastTime = newTime;
		oldPosition = newPosition;

		encoder_counter.data = newPosition;
		encoder_counter_pub.publish(&encoder_counter);

		nh.spinOnce();
	}

	if (!(millis() % PUBLISH_RATE)) {//Control the motors every PUBLISH_RATE [ms] aprox.

		float1.data = (float) vl_desired;
		pub_1.publish(&float1);

	}

	if (!(millis() % 3000)) {	//Say I'm ok once in a while
		str_pub.publish(&ok_rosstr);
		nh.spinOnce();
	}

	nh.spinOnce();
	delay(1);
}
