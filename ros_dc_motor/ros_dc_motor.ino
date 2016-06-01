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
volatile float global_linx = 0, global_angz = 0;
long oldPositionI = -999; //long is a 4 bytes type
long oldPositionD = -999;
volatile float vl, vr;

//Define Variables we'll be connecting to
double SetpointR, InputR, OutputR;
double SetpointL, InputL, OutputL;

//Define the aggressive and conservative Tuning Parameters
double aggKp = 4, aggKi = 0.2, aggKd = 1;
double consKp = 1, consKi = 0.05, consKd = 0.25;

//Specify the links and initial tuning parameters
PID PID_R(&InputR, &OutputR, &SetpointR, consKp, consKi, consKd, DIRECT);
PID PID_L(&InputL, &OutputL, &SetpointL, consKp, consKi, consKd, DIRECT);

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
void move_motor(int motor, float speed) {
	int pwm_val = (int) (round(speed));
	//sonar_msg.data=pwm_val;
	//sonar_pub.publish(&sonar_msg);
	if (motor == LEFT) {
		if (pwm_val >= 1 && pwm_val <= 255) {
			digitalWrite(LEFT_MOT_POS, 1);
			digitalWrite(LEFT_MOT_NEG, 0);
			analogWrite(LEFT_MOT_EN, pwm_val);
		} else if (pwm_val <= 1 && pwm_val >= -255) {
			digitalWrite(LEFT_MOT_POS, 0);
			digitalWrite(LEFT_MOT_NEG, 1);
			analogWrite(LEFT_MOT_EN, -1 * pwm_val);
		} else {
			//Stop if received an wrong direction
			hard_stop (LEFT);
		}

	} else if (motor == RIGHT) {
		if (pwm_val >= 1 && pwm_val <= 255) {
			digitalWrite(RIGHT_MOT_POS, 1);
			digitalWrite(RIGHT_MOT_NEG, 0);
			analogWrite(RIGHT_MOT_EN, pwm_val);
		} else if (pwm_val <= 1 && pwm_val >= -255) {
			digitalWrite(RIGHT_MOT_POS, 0);
			digitalWrite(RIGHT_MOT_NEG, 1);
			analogWrite(RIGHT_MOT_EN, -1 * pwm_val);
		} else {
			//Stop if received an wrong direction
			hard_stop (RIGHT);
		}
	}
}

void move_robot(float linearx, float angularz) {
	float vr = (2.0 * linearx + WHEELDIST * angularz) / (2.0 * WHEELRAD); //linear x should be maximum 4
	float vl = (2.0 * linearx - WHEELDIST * angularz) / (2.0 * WHEELRAD);
	PID_R.Compute();
	PID_L.Compute();

	if ((vl <= 255 && vl >= -255) && (vr <= 255 && vl >= -255)) {
		move_motor(LEFT, vl); // turn it on going backward
		move_motor(RIGHT, vr); // turn it on going backward
	} else {
		soft_stop (LEFT);
		soft_stop (RIGHT);
	}
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
	global_linx = cmd_msg.linear.x;
	global_angz = cmd_msg.angular.z;
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

	//initialize the variables we're linked to
	SetpointR = 100;
	SetpointL = 100;
	//turn the PID on
	PID_R.SetMode(AUTOMATIC);
	PID_L.SetMode(AUTOMATIC);

}

/*
 * Arduino MAIN LOOP
 */
void loop() {
	long newPositionD = myEncD.read();
	long newPositionI = myEncI.read();
	//I will just keep the loop waiting for a message
	//in the ros topic
	if (newPositionD != oldPositionD) {
		oldPositionD = newPositionD;
		lwheel_pose.data = newPositionI;
		rwheel_pose.data = newPositionD;
		lwheel_pose_pub.publish(&lwheel_pose);
		rwheel_pose_pub.publish(&rwheel_pose);
		nh.spinOnce();
	}

	if (newPositionI != oldPositionI) {
		oldPositionI = newPositionI;
		lwheel_pose.data = newPositionI;
		rwheel_pose.data = newPositionD;
		lwheel_pose_pub.publish(&lwheel_pose);
		rwheel_pose_pub.publish(&rwheel_pose);
		nh.spinOnce();
	}

	if (!(millis() % 10)) {	//Control the motors every 10ms aprox.
		move_robot(global_linx, global_angz);
	}

	if (!(millis() % 3000)) {	//Say I'm ok once in a while
		str_pub.publish(&ok_rosstr);
		nh.spinOnce();
	}

	nh.spinOnce();
	delay(1);
}
