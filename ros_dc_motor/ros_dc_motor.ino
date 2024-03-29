//#define USE_USBCON //Uncomment this line if you are using an arduino DUE
// Do not remove the include below
#include "ros_dc_motor.h"
/*
 * This program hears a Twist message in a ROS topic
 * and commands two motors according to the linear.x
 * value of the received Twist.
 *
 * We send the encoders information back to the ROS PC as a Vector3 message, where the first
 * element is the left wheel data and the second element is the right wheel data.
 * mantainer: arturoescobedo.iq@gmail.com
 */

/*
 * Global variables
 */
ros::NodeHandle nh;
std_msgs::String callback_rosstr, ok_rosstr;
std_msgs::Int32 lwheel_pose, rwheel_pose;
std_msgs::Float32 float1, float2, float3;
volatile float linear_vel = 0, angular_vel = 0;
long oldPositionI = 0; //long is a 4 bytes type
long oldPositionD = 0;

long newPositionD = 0, newPositionI = 0;
unsigned long newTimeD = 0, newTimeI = 0, lastTimeD = 0, lastTimeI = 0,
		delta_t = 0;
long last_pos_l, last_pos_r;
double vl_desired, vr_desired;

//Define Variables we'll be connecting to
double vr_measured, vr_controlled;
double vl_measured, vl_controlled;

//Define the aggressive and conservative Tuning Parameters
double consKp = 10, consKi = 1.0, consKd = 0.5;

//Specify the links and initial tuning parameters
PID PID_R(&vr_measured, &vr_controlled, &vr_desired, consKp, consKi, consKd,
		DIRECT);
PID PID_L(&vl_measured, &vl_controlled, &vl_desired, consKp, consKi, consKd,
		DIRECT);

//   avoid using pins with LEDs attached
Encoder myEncD(RIGHT_ENC_A, RIGHT_ENC_B);
Encoder myEncI(LEFT_ENC_A, LEFT_ENC_B);

/*
 * My functions
 */
/* Hard stop of motor
 * motor: LEFT or RIGHT (0 or 1)
 */
void hard_stop(int motor) {
	//Stop if received an wrong direction
	if (motor == LEFT) {
		digitalWrite(LEFT_MOT_DIR_FRONT, 1);
		digitalWrite(LEFT_MOT_DIR_BACK, 1);
		analogWrite(LEFT_MOT_EN, 255);
	} else if (motor == RIGHT) {
		//Stop if received an wrong direction
		digitalWrite(RIGHT_MOT_DIR_FRONT, 1);
		digitalWrite(RIGHT_MOT_DIR_BACK, 1);
		analogWrite(RIGHT_MOT_EN, 255);
	}
}

/* Soft stop of motor
 * motor: LEFT or RIGHT (0 or 1)
 */
void soft_stop(int motor) {
	//Stop if received a wrong direction
	if (motor == LEFT) {
		digitalWrite(LEFT_MOT_DIR_FRONT, 0);
		digitalWrite(LEFT_MOT_DIR_BACK, 0);
		analogWrite(LEFT_MOT_EN, 0);
	} else if (motor == RIGHT) {
		//Stop if received a wrong direction
		digitalWrite(RIGHT_MOT_DIR_FRONT, 0);
		digitalWrite(RIGHT_MOT_DIR_BACK, 0);
		analogWrite(RIGHT_MOT_EN, 0);
	}
}
/* Moves the right wheel
 */
void move_right_motor(void) {
	PID_R.Compute();
	if (vr_controlled >= 1 && vr_controlled <= 255) {
		digitalWrite(RIGHT_MOT_DIR_FRONT, 1);
		digitalWrite(RIGHT_MOT_DIR_BACK, 0);
		analogWrite(RIGHT_MOT_EN, vr_controlled);
	} else if (vr_controlled <= 1 && vr_controlled >= -255) {
		digitalWrite(RIGHT_MOT_DIR_FRONT, 0);
		digitalWrite(RIGHT_MOT_DIR_BACK, 1);
		analogWrite(RIGHT_MOT_EN, -1 * vr_controlled);
	} else {
		//Stop if received a wrong direction
		hard_stop (RIGHT);
	}
}

/* Moves the right wheel
 */
void move_left_motor(void) {
	PID_L.Compute();
	if (vl_controlled > 0 && vl_controlled <= 255) {
		digitalWrite(LEFT_MOT_DIR_FRONT, 1);
		digitalWrite(LEFT_MOT_DIR_BACK, 0);
		analogWrite(LEFT_MOT_EN, vl_controlled);
	} else if (vl_controlled <= 1 && vl_controlled >= -255) {
		digitalWrite(LEFT_MOT_DIR_FRONT, 0);
		digitalWrite(LEFT_MOT_DIR_BACK, 1);
		analogWrite(LEFT_MOT_EN, -1 * vl_controlled);
	} else {
		//Stop if received a wrong direction
		hard_stop (LEFT);
	}

}

/***This function takes as input the current and last encoder positions,
 * and gives back the angular velocity of the wheel.
 */
double wheel_vel_from_encoder(long curr_pos, long last_pos, long delta_t_us) {
	double vel = 0;
	double delta_pose = 0;
	//First check if denominator is not cero to avoid not defined operations
	if (delta_t_us > 0) {
		delta_pose = (double) last_pos - curr_pos;
		vel = (2 * PI * delta_pose) / (delta_t_us * ENCODER_PULSES);
	}
	return vel;
}

/***This function takes as input robot's linear and angular velocities
 * and gives back the angular velocities of both left and right wheels
 */
void wheel_vel_from_twist(double linear, double angular, double* vlp,
		double* vrp) {
	*vlp = (double) (2.0 * linear - WHEELDIST * angular) / (2.0 * WHEELRAD);
	*vrp = (double) (2.0 * linear + WHEELDIST * angular) / (2.0 * WHEELRAD);
}


void move_robot(double linear, double angular) {
	wheel_vel_from_twist(linear, angular, &vl_desired, &vr_desired);
	move_left_motor();
	move_right_motor();
}

//Publishers  
ros::Publisher str_pub("arduino/str_output", &ok_rosstr);
ros::Publisher lwheel_pose_pub("arduino/lwheel", &lwheel_pose);
ros::Publisher rwheel_pose_pub("arduino/rwheel", &rwheel_pose);
ros::Publisher pub_1("arduino/pub_1", &float1);
ros::Publisher pub_2("arduino/pub_2", &float2);
ros::Publisher pub_3("arduino/pub_3", &float3);

//Twist callback
void cmd_vel_cb(const geometry_msgs::Twist& cmd_msg) {
	digitalWrite(LED, HIGH - digitalRead(LED)); //toggles a led
	//str_pub.publish(&callback_rosstr);
	linear_vel = cmd_msg.linear.x;
	angular_vel = cmd_msg.angular.z;
}

//String callback
//Creates the ROS publishers and subscribers
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("arduino/cmd_vel",
		cmd_vel_cb);
/*
 * Arduino SETUP
 */
void setup() {
	nh.getHardware()->setBaud(57600); //The HC06 and 05 use by default 9600 baud rate
	nh.initNode();
	//Subscribed ROS topics
	nh.subscribe(cmd_vel_sub);

	//Advertise ROS topics
	nh.advertise(str_pub);
	nh.advertise(lwheel_pose_pub);
	nh.advertise(rwheel_pose_pub);
	nh.advertise(pub_1);
	nh.advertise(pub_2);
	nh.advertise(pub_3);
	ok_rosstr.data = "arduino ok";
	callback_rosstr.data = "cb executed";
	pinMode(LED, OUTPUT);
	pinMode(LEFT_MOT_DIR_BACK, OUTPUT);
	pinMode(LEFT_MOT_DIR_FRONT, OUTPUT);
	pinMode(LEFT_MOT_EN, OUTPUT);
	pinMode(RIGHT_MOT_DIR_BACK, OUTPUT);
	pinMode(RIGHT_MOT_DIR_FRONT, OUTPUT);
	pinMode(RIGHT_MOT_EN, OUTPUT);

	//Turn the PID on
	PID_R.SetSampleTime(CONTROL_RATE);
	PID_R.SetOutputLimits(-255, 255); //It will set the output of the controller to be between
	PID_R.SetMode(AUTOMATIC);

	PID_L.SetSampleTime(CONTROL_RATE);
	PID_L.SetOutputLimits(-255, 255); //It will set the output of the controller to be between
	PID_L.SetMode(AUTOMATIC);

}

/*
 * Arduino MAIN LOOP
 */
void loop() {
	newPositionD = myEncD.read();
	newTimeD = micros();
	newPositionI = myEncI.read();
	newTimeI = micros();
	//I will just keep the loop waiting for a message
	//in the ros topic
	if (newPositionD != oldPositionD) {
		delta_t = newTimeD - lastTimeD;

		vr_measured = wheel_vel_from_encoder(newPositionD, oldPositionD,
				delta_t);
		lastTimeD = newTimeD;
		oldPositionD = newPositionD;

		lwheel_pose.data = newPositionI;
		lwheel_pose_pub.publish(&lwheel_pose);
		rwheel_pose.data = newPositionD;
		rwheel_pose_pub.publish(&rwheel_pose);
		nh.spinOnce();
	}

	if (newPositionI != oldPositionI) {
		delta_t = newTimeI - lastTimeI;

		vl_measured = wheel_vel_from_encoder(newPositionI, oldPositionI,
				delta_t);

		lastTimeI = newTimeI;
		oldPositionI = newPositionI;

		lwheel_pose.data = newPositionI;
		lwheel_pose_pub.publish(&lwheel_pose);

		rwheel_pose.data = newPositionD;
		rwheel_pose_pub.publish(&rwheel_pose);
		nh.spinOnce();
	}

	if (!(millis() % CONTROL_RATE)) {//Control the motors every CONTROL_RATE [ms] aprox.
		move_robot(linear_vel, angular_vel);

		float1.data = (float) vl_desired;
		pub_1.publish(&float1);

		float2.data = (float) vl_measured;
		pub_2.publish(&float2);

		float3.data = (float) linear_vel;
		pub_3.publish(&float3);
	}

	if (!(millis() % 3000)) {	//Say I'm ok once in a while
		str_pub.publish(&ok_rosstr);
		nh.spinOnce();
	}

	nh.spinOnce();
	delay(1);
}
