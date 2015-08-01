// Do not remove the include below
#include "robotino.h"

/*
 * Global variables
 */
ros::NodeHandle nh;
std_msgs::String ok_rosstr;
std_msgs::String callback_rosstr;
float global_linx = 0, global_angz = 0;
Motor left_motor, right_motor;

void move_robot(float linearx, float angularz) {
	float vr = (2.0 * linearx + WHEELDIST * angularz) / (2.0 * WHEELRAD); //linear x should be maximum 4
	float vl = (2.0 * linearx - WHEELDIST * angularz) / (2.0 * WHEELRAD);
	if ((vl <= 255 && vl >= -255) && (vr <= 255 && vl >= -255)) {
		left_motor.move(vl); // turn it on going backward
		right_motor.move(vr); // turn it on going backward
	} else {
		left_motor.soft_stop();
		right_motor.soft_stop();
	}
}
//in this example pub is declared before the cmd_vel_cb
//because it used there
ros::Publisher str_pub("arduino/str_output", &ok_rosstr);

//Twist callback
void cmd_vel_cb(const geometry_msgs::Twist& cmd_msg) {
	digitalWrite(LED, HIGH - digitalRead(LED)); //toggles a led
	//str_pub.publish(&callback_rosstr);
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
	//nh.advertise(sonar_pub);
	ok_rosstr.data = "arduino ok";
	callback_rosstr.data = "cb executed";
	pinMode(LED, OUTPUT);
	left_motor.init(LEFT_MOT_POS, LEFT_MOT_NEG, LEFT_MOT_EN);
	right_motor.init(RIGHT_MOT_POS, RIGHT_MOT_NEG, RIGHT_MOT_EN);

}

/*
 * Arduino MAIN LOOP
 */
void loop() {
	//I will just keep the loop waiting for a message
	//in the ros topic
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
