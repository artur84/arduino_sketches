// Do not remove the include below
#include "robotino.h"

/******************************
 * Global variables
 ********************************/
ros::NodeHandle nh;
std_msgs::String ok_rosstr;
std_msgs::String callback_rosstr;
std_msgs::Int16 sonar_msg;
float global_linx = 0, global_angz = 0;
Motor left_motor, right_motor;
Sonar sonar;

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

//Twist callback
void cmd_vel_cb(const geometry_msgs::Twist& cmd_msg) {
	digitalWrite(LED, HIGH - digitalRead(LED)); //toggles a led
	//str_pub.publish(&callback_rosstr);
	global_linx = cmd_msg.linear.x;
	global_angz = cmd_msg.angular.z;
}

//Creates the ROS publishers and subscribers
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("robotino/cmd_vel",
		cmd_vel_cb);
ros::Publisher str_pub("robotino/str_output", &ok_rosstr);
ros::Publisher sonar_pub("robotino/sonar", &sonar_msg);
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
	//Motors
	left_motor.init(LEFT_MOT_POS, LEFT_MOT_NEG, LEFT_MOT_EN);
	right_motor.init(RIGHT_MOT_POS, RIGHT_MOT_NEG, RIGHT_MOT_EN);
	//Sonar
	sonar.connect();

}

/*
 * Arduino MAIN LOOP
 */
long publisher_timer;
int16_t sensor_reading;
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
	if (millis() > publisher_timer) {

		// step 1: request reading from sensor
		sonar.startMeasurementCM();
		delay(70);
		sensor_reading = sonar.readResult();

		//Publish to ROS topic
		sonar_msg.data = sensor_reading;
		sonar_pub.publish(&sonar_msg);

		//publish twice a second (aprox.)
		publisher_timer = millis() + 500;

	}
	nh.spinOnce();
	delay(1);
}
