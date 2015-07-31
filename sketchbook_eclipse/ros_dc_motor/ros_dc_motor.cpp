// Do not remove the include below
#include "ros_dc_motor.h"
/*
 * This program hears a Twist message in a ROS topic
 * and commands two motors according to the linear.x
 * value of the received Twist.
 * mantainer: arturoescobedo.iq@gmail.com
 */

/*
 * Global variables
 */
ros::NodeHandle nh;
std_msgs::String ok_rosstr;
std_msgs::String callback_rosstr;
std_msgs::Float32 sonar_msg;

char global_char[10];
int global_linx=0, global_angz=0;

//in this example pub is declared before the cmd_vel_cb
//because it used there
ros::Publisher str_pub("arduino/str_output", &ok_rosstr);
ros::Publisher sonar_pub("arduino/sonar", &sonar_msg);

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
	int pwm=int(PWM_CALIBRATION*speed);
	sonar_msg.data=float(pwm);
	sonar_pub.publish(&sonar_msg); //For debugging
	if (motor == LEFT) {
		if (pwm >= 1 && pwm <= 255) {
			digitalWrite(LEFT_MOT_POS, 1);
			digitalWrite(LEFT_MOT_NEG, 0);
			analogWrite(LEFT_MOT_EN, pwm);
		} else if (pwm <= 1 && pwm >= -255) {
			digitalWrite(LEFT_MOT_POS, 0);
			digitalWrite(LEFT_MOT_NEG, 1);
			analogWrite(LEFT_MOT_EN, -1 * pwm);
		} else {
			//Stop if received an wrong direction
			hard_stop(LEFT);
		}

	} else if (motor == RIGHT) {
		if (pwm >= 1 && pwm <= 255) {
			digitalWrite(RIGHT_MOT_POS, 1);
			digitalWrite(RIGHT_MOT_NEG, 0);
			analogWrite(RIGHT_MOT_EN, pwm);
		} else if (pwm <= 1 && pwm >= -255) {
			digitalWrite(RIGHT_MOT_POS, 0);
			digitalWrite(RIGHT_MOT_NEG, 1);
			analogWrite(RIGHT_MOT_EN, -1 * pwm);
		} else {
			//Stop if received an wrong direction
			hard_stop(RIGHT);
		}
	}
}



void move_robot(float linearx, float angularz) {
	float v = linearx;    //linear x should be maximum 4
	float w = angularz;   //angular speed
	//Differential drive robot equations
	float vr = ((2*v+L*w)/2*R);
	float vl = ((2*v-L*w)/2*R);
	sonar_msg.data = vr;
	sonar_pub.publish(&sonar_msg);

	move_motor(LEFT, vr); // turn it on going backward
	move_motor(RIGHT, vl); // turn it on going backward

}




//Twist callback
void cmd_vel_cb(const geometry_msgs::Twist& cmd_msg) {
	digitalWrite(LED, HIGH - digitalRead(LED)); //toggles a led
	str_pub.publish(&callback_rosstr);
	global_linx = cmd_msg.linear.x;
	global_angz = cmd_msg.angular.z;
}

//Creates the ROS publishers and subscribers
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("arduino/cmd_vel",
		cmd_vel_cb);
/*
 * Arduino SETUP
 */
void setup() {
	nh.getHardware()->setBaud(57600); //The HC06 and 05 use by default 9600 baud rate
	nh.initNode();
	nh.subscribe(cmd_vel_sub);
	nh.advertise(str_pub);
	nh.advertise(sonar_pub);
	ok_rosstr.data = "arduino ok";
	callback_rosstr.data = "cb executed";
	pinMode(LED, OUTPUT);
	pinMode(LEFT_MOT_NEG, OUTPUT);
	pinMode(LEFT_MOT_POS, OUTPUT);
	pinMode(LEFT_MOT_EN, OUTPUT);
	pinMode(RIGHT_MOT_NEG, OUTPUT);
	pinMode(RIGHT_MOT_POS, OUTPUT);
	pinMode(RIGHT_MOT_EN, OUTPUT);
	str_pub.publish(&ok_rosstr);

}

/*
 * Arduino MAIN LOOP
 */
void loop() {
	//I will just keep the loop waiting for a message
	//in the ros topic
	str_pub.publish(&ok_rosstr);
	if (!(millis() % 10)) {//Control the motors every 10ms aprox.
	move_robot(global_linx, global_angz);

	}
	if (!(millis() % 3000)) {//Say I'm ok once in a while
		str_pub.publish(&ok_rosstr);
		sonar_pub.publish(&sonar_msg);
		nh.spinOnce();
	}
	nh.spinOnce();
	delay(1);
}
