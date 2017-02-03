// Do not remove the include below
#include "iteshu_driver.h"
/*
 * This program hears a Twist message in a ROS topic
 * and commands two motors according to the linear.x
 * value of the received Twist.
 *
 * We send encoder's info as a odometry message
 * use with roslaunch file:
 *
 * -------$ roslaunch iteshu_robot start_rosserial.launch port:=/dev/ttyUSB0
 *
 *
 *
 * mantainer: arturoescobedo.iq@gmail.com
 *
 */

/*
 * Global variables
 */
ros::NodeHandle nh;
std_msgs::String callback_rosstr, ok_rosstr;
std_msgs::Int32 lwheel_pose, rwheel_pose;
std_msgs::Float32 float1, float2, float3;
volatile float linear_vel = 0; //[m/s] ROS twist angular vel
volatile float angular_vel = 0;//[rad/sec] ROS twist angular vel
long oldPositionI = 0; //long is a 4 bytes type
long oldPositionD = 0;

long newPositionD = 0, newPositionI = 0;
unsigned long newTimeD = 0, newTimeI = 0, lastTimeD = 0, lastTimeI = 0,
		delta_t_us = 0;
long last_pos_l, last_pos_r;

//Define Variables we'll be connecting to
double wr_desired = 0, wr_measured = 0;//[rad/sec]
double vr_controlled = 0, vl_controlled = 0; //[pwm] 0-255
double wl_desired = 0, wl_measured = 0; //[rad/sec]

//Define the aggressive and conservative Tuning Parameters
double consKp = 50, consKi = 0.1, consKd = 0.01;

//Specify the links and initial tuning parameters
PID PID_R(&wr_measured, &vr_controlled, &wr_desired, consKp, consKi, consKd,
		DIRECT);
PID PID_L(&wl_measured, &vl_controlled, &wl_desired, consKp, consKi, consKd,
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
		digitalWrite(RIGHT__MOT_DIR_FRONT, 1);
		digitalWrite(RIGHT_MOT_DIR_BACK, 1);
		analogWrite(RIGHT_MOT_EN, 255);
	}
}

/* Soft stop of motor
 * motor: LEFT or RIGHT (0 or 1)
 */
void soft_stop(int motor) {
	//Stop if received an wrong direction
	if (motor == LEFT) {
		digitalWrite(LEFT_MOT_DIR_FRONT, 0);
		digitalWrite(LEFT_MOT_DIR_BACK, 0);
		analogWrite(LEFT_MOT_EN, 0);
	} else if (motor == RIGHT) {
		//Stop if received an wrong direction
		digitalWrite(RIGHT__MOT_DIR_FRONT, 0);
		digitalWrite(RIGHT_MOT_DIR_BACK, 0);
		analogWrite(RIGHT_MOT_EN, 0);
	}
}
/* Moves the right wheel
 */
void move_right_motor(void) {
	PID_R.Compute();
	if (vr_controlled > 0 && vr_controlled <= 255) {
		digitalWrite(RIGHT__MOT_DIR_FRONT, 1);
		digitalWrite(RIGHT_MOT_DIR_BACK, 0);
		analogWrite(RIGHT_MOT_EN, (int) vr_controlled);
	} else {
		digitalWrite(RIGHT__MOT_DIR_FRONT, 0);
		digitalWrite(RIGHT_MOT_DIR_BACK, 1);
		analogWrite(RIGHT_MOT_EN, -1 * (int) vr_controlled);
	}
}

/* Moves the right wheel
 */
void move_left_motor(void) {
	PID_L.Compute();
	if (vl_controlled > 0 && vl_controlled <= 255) {
		digitalWrite(LEFT_MOT_DIR_FRONT, 1);
		digitalWrite(LEFT_MOT_DIR_BACK, 0);
		analogWrite(LEFT_MOT_EN, (int) vl_controlled);
	} else {
		digitalWrite(LEFT_MOT_DIR_FRONT, 0);
		digitalWrite(LEFT_MOT_DIR_BACK, 1);
		analogWrite(LEFT_MOT_EN, -1 * (int) vl_controlled);
	}

}

void move_robot(double linear, double angular) {
	if(linear==0 && angular==0 ) //Hard Stop
	{
		hard_stop(LEFT);
		hard_stop(RIGHT);
	}
	else
	{
		move_left_motor();
		move_right_motor();
	}
}

/***This function takes as input the current and last encoder poses,
 * and gives back the angular velocity of the wheel in [rad/sec]
 *
 * @ curr_pos [ticks]
 * @ last_pos [ticks]
 * @ delta_t_us [us]
 * return vel [rad/sec]
 */
double wheel_vel_from_encoder(long curr_pos, long last_pos, long delta_t_us) {
	double vel = 0; //[rad/sec]
	double delta_pose = 0; //[ticks]
	//First check if denominator is not cero to avoid not defined operations
	if (delta_t_us > 0) {
		delta_pose = (double) (curr_pos - last_pos);
		// [rad/sec] (Dp*2pi/(Dt*encoder_pulses*0.000001)
		vel = (delta_pose*2*PI) / (delta_t_us *0.000001* ENCODER_PULSES);
	}
	return vel;
}

/***This function takes as input robot's linear [m/s] and angular [rad/sec] velocities
 * and gives back the angular velocities [[rad/s]of both left and right wheels
 * @ linear [m/s]
 * @ angular [rad/sec]
 * @ wl_pointer [rad/sec] pointer to left vel
 * @ wr_pointer [rad/sec] pointer to right vel
 */
void wheel_vel_from_twist(double linear, double angular, double* wl_pointer,
		double* wr_pointer) {
	*wl_pointer = (double) (2.0 * linear - WHEELDIST * angular) / (2.0 * WHEELRAD);
	*wr_pointer = (double) (2.0 * linear + WHEELDIST * angular) / (2.0 * WHEELRAD);
}

/***This function takes as input wheel's angular velocities and outputs the linear and angular velocities
 *   w_right in [rps]
 *   w_left [rps]
 *   linear [m/s]
 *   angular [rad/seg]
 */
void twist_from_wheel_vel(double w_right, double w_left, double* linear, double* angular) {
	*linear = (double) (WHEELRAD/2)*(w_right+w_left);
	*angular = (double) (WHEELRAD/WHEELDIST)*(w_right-w_left);
}

/***This function outpus (x,y,phi) of the robot
 * It assumes that x,y and phi arguments are the last computed values.
 * Dr, Dl, x, y: [m]
 * phi: [rad]
 *
 */
void compute_odom(double Dr, double Dl, double* x, double* y,  double* phi) {
	double Dc=(Dl+Dr)/2;
	double x_prev=*x;
	double y_prev=*y;
	double phi_prev=*phi;
	*phi = (double) phi_prev*(Dr-Dl)/2;
	*x = (double) x_prev+Dc*cos(*phi);
	*y = (double) y_prev+Dc*sin(*phi);
}

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
	wheel_vel_from_twist(linear_vel, angular_vel, &wl_desired, &wr_desired);
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
	pinMode(RIGHT__MOT_DIR_FRONT, OUTPUT);
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
	newPositionI = myEncI.read();
	//I will just keep the loop waiting for a message
	//in the ros topic

	if (!(millis() % ODOMETRY_RATE)) {
		//TODO: here we should publish the odometry message

		delta_t_us = ODOMETRY_RATE*1000; //Time in us

		rwheel_pose.data = newPositionD; //Position in ticks
		lwheel_pose.data = newPositionI; //Position in ticks

		rwheel_pose_pub.publish(&rwheel_pose);
		lwheel_pose_pub.publish(&lwheel_pose);

		wr_measured = wheel_vel_from_encoder(newPositionD, oldPositionD, delta_t_us);
		wl_measured = wheel_vel_from_encoder(newPositionI, oldPositionI, delta_t_us);

		oldPositionD = newPositionD;
		oldPositionI = newPositionI;

		rwheel_pose.data = newPositionD;
		lwheel_pose.data = newPositionI;

		rwheel_pose_pub.publish(&rwheel_pose);
		lwheel_pose_pub.publish(&lwheel_pose);
	}

	if (!(millis() % CONTROL_RATE)) { //Control the motors every CONTROL_RATE [ms] aprox.
		move_robot(linear_vel, angular_vel);

		float1.data = (float) wr_desired;//TODO: Delete this once that debugging is finished
		pub_1.publish(&float1);

		float2.data = (float) wr_measured;
		pub_2.publish(&float2);

		float3.data = (float) vr_controlled;
		pub_3.publish(&float3);
	}

	if (!(millis() % OK_RATE)) {	//Say I'm ok once in a while
		str_pub.publish(&ok_rosstr);
		nh.spinOnce();
	}

	nh.spinOnce();
	delay(1);
}
