// Do not remove the include below
#include "ros_dc_motor.h"
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
std_msgs::String callback_rosstr, ok_rosstr;
std_msgs::Int32 lwheel_pose, rwheel_pose;
volatile float linear_vel = 0, angular_vel = 0;
long oldPositionI = -999; //long is a 4 bytes type
long oldPositionD = -999;

long newPositionD, newPositionI;
unsigned long newTimeD, newTimeI, lastTimeD, lastTimeI, delta_t;
long last_pos_l, last_pos_r;
double vl_desired, vr_desired;

//Define Variables we'll be connecting to
double vr_measured, vr_controlled;
double vl_measured, vl_controlled;

//Define the aggressive and conservative Tuning Parameters
double consKp = 35, consKi = 1.0, consKd = 0.5;

//Specify the links and initial tuning parameters
PID PID_R(&vr_measured, &vr_controlled, &vr_desired, consKp, consKi, consKd,
		DIRECT);
PID PID_L(&vl_measured, &vl_controlled, &vl_desired, consKp, consKi, consKd,
		DIRECT);

//   avoid using pins with LEDs attached
Encoder myEncD(ENCDA, ENCDB);
Encoder myEncI(ENCIA, ENCIB);

/*
 * My functions
 */
/* Hard stop of motor
 * motor: LEFT or RIGHT (0 or 1)
 */
void hard_stop(int motor) {
	//Stop if received an wrong direction
	if (motor == LEFT) {
		digitalWrite(LEFT_MOT_POS, 1);
		digitalWrite(LEFT_MOT_NEG, 1);
		analogWrite(LEFT_MOT_EN, 255);
	} else if (motor == RIGHT) {
		//Stop if received an wrong direction
		digitalWrite(RIGHT_MOT_POS, 1);
		digitalWrite(RIGHT_MOT_NEG, 1);
		analogWrite(RIGHT_MOT_EN, 255);
	}
}

/* Soft stop of motor
 * motor: LEFT or RIGHT (0 or 1)
 */
void soft_stop(int motor) {
	//Stop if received an wrong direction
	if (motor == LEFT) {
		digitalWrite(LEFT_MOT_POS, 0);
		digitalWrite(LEFT_MOT_NEG, 0);
		analogWrite(LEFT_MOT_EN, 0);
	} else if (motor == RIGHT) {
		//Stop if received an wrong direction
		digitalWrite(RIGHT_MOT_POS, 0);
		digitalWrite(RIGHT_MOT_NEG, 0);
		analogWrite(RIGHT_MOT_EN, 0);
	}
}
/* Moves the left wheel forward
 * motor: LEFT or RIGHT (0 or 1)
 * speed: a value between -255 and 255 (negative is backward, positive is forward)
 */
void move_motor(int motor) {

	//sonar_msg.data=pwm_val;
	//sonar_pub.publish(&sonar_msg);
	if (motor == LEFT) {
		PID_L.Compute();
		if (vl_controlled > 0 && vl_controlled <= 255) {
			digitalWrite(LEFT_MOT_POS, 1);
			digitalWrite(LEFT_MOT_NEG, 0);
			analogWrite(LEFT_MOT_EN, vl_controlled);
		} else if (vl_controlled <= 1 && vl_controlled >= -255) {
			digitalWrite(LEFT_MOT_POS, 0);
			digitalWrite(LEFT_MOT_NEG, 1);
			analogWrite(LEFT_MOT_EN, -1 * vl_controlled);
		} else {
			//Stop if received an wrong direction
			hard_stop (LEFT);
		}

	} else if (motor == RIGHT) {
		PID_R.Compute();
		if (vr_controlled >= 1 && vr_controlled <= 255) {
			digitalWrite(RIGHT_MOT_POS, 1);
			digitalWrite(RIGHT_MOT_NEG, 0);
			analogWrite(RIGHT_MOT_EN, vr_controlled);
		} else if (vr_controlled <= 1 && vr_controlled >= -255) {
			digitalWrite(RIGHT_MOT_POS, 0);
			digitalWrite(RIGHT_MOT_NEG, 1);
			analogWrite(RIGHT_MOT_EN, -1 * vr_controlled);
		} else {
			//Stop if received an wrong direction
			hard_stop (RIGHT);
		}
	}
}

void move_robot(float linearx, float angularz) {
	vr_desired = (2.0 * linearx + WHEELDIST * angularz) / (2.0 * WHEELRAD); //linear x should be maximum 4
	vl_desired = (2.0 * linearx - WHEELDIST * angularz) / (2.0 * WHEELRAD);
	move_motor (LEFT); // turn it on going backward
	move_motor (RIGHT); // turn it on going backward
}

double compute_vel(long curr_pos, long last_pos, long delta_t_us) {
	double vel = 0;
	//First check if denominator is not cero to avoid not defined operations
	if (delta_t_us > 0) {
		vel = (last_pos - curr_pos) / delta_t_us;
	}

	return vel;
}
//in this example pub is declared before the cmd_vel_cb
//because it used there
ros::Publisher str_pub("arduino/str_output", &ok_rosstr);
ros::Publisher lwheel_pose_pub("arduino/lwheel", &lwheel_pose);
ros::Publisher rwheel_pose_pub("arduino/rwheel", &rwheel_pose);

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
	ok_rosstr.data = "arduino ok";
	callback_rosstr.data = "cb executed";
	pinMode(LED, OUTPUT);
	pinMode(LEFT_MOT_NEG, OUTPUT);
	pinMode(LEFT_MOT_POS, OUTPUT);
	pinMode(LEFT_MOT_EN, OUTPUT);
	pinMode(RIGHT_MOT_NEG, OUTPUT);
	pinMode(RIGHT_MOT_POS, OUTPUT);
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

		vr_measured = compute_vel(newPositionD, oldPositionD, delta_t);
		lastTimeD = newTimeD;
		oldPositionD = newPositionD;

		lwheel_pose.data = newPositionI;
		rwheel_pose.data = newPositionD;
		lwheel_pose_pub.publish(&lwheel_pose);
		rwheel_pose_pub.publish(&rwheel_pose);
		nh.spinOnce();
	}

	if (newPositionI != oldPositionI) {
		delta_t = newTimeI - lastTimeI;

		vl_measured = compute_vel(newPositionD, oldPositionD, delta_t);

		lastTimeI = newTimeI;
		oldPositionI = newPositionI;

		lwheel_pose.data = newPositionI;
		rwheel_pose.data = newPositionD;
		lwheel_pose_pub.publish(&lwheel_pose);
		rwheel_pose_pub.publish(&rwheel_pose);
		nh.spinOnce();
	}

	if (!(millis() % CONTROL_RATE)) {//Control the motors every CONTROL_RATE [ms] aprox.
		move_robot(linear_vel, angular_vel);
	}

	if (!(millis() % 3000)) {	//Say I'm ok once in a while
		str_pub.publish(&ok_rosstr);
		nh.spinOnce();
	}

	nh.spinOnce();
	delay(1);
}
