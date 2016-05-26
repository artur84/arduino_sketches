// Do not remove the include below
#include "ros_differential_robot_driver.h"

/********************************
 ******  Global variables  ******
 ********************************/
ros::NodeHandle nh;
std_msgs::String ok_rosstr;
geometry_msgs::Vector3 wheel_pose; //Left and right wheels
volatile float global_linx = 0, global_angz = 0;
long oldPositionI  = -999;
long oldPositionD = -999;
volatile float vl, vr;

//Motores
Motor left_motor, right_motor;

//   avoid using pins with LEDs attached
Encoder myEncD(ENCDA,ENCDB);
Encoder myEncI(ENCIA,ENCIB);


//Creates the ROS publishers and subscribers
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_vel_cb);
ros::Publisher str_pub("string", &ok_rosstr);
ros::Publisher wheel_pose_pub("wheel_pose", &wheel_pose);


void move_robot(float linearx, float angularz) {
	//Equations for a differential drive mobile robot.
	vr = (2.0 * linearx + WHEELDIST * angularz) * (0.5 * WHEELRAD); //linear x should be maximum 4
	vl = (2.0 * linearx - WHEELDIST * angularz) * (0.5 * WHEELRAD);
}

//Twist callback
void cmd_vel_cb(const geometry_msgs::Twist& cmd_msg) {
	digitalWrite(LED, HIGH - digitalRead(LED)); //toggles a led
	//str_pub.publish(&callback_rosstr);
	global_linx = cmd_msg.linear.x;
	global_angz = cmd_msg.angular.z;
}


/*************************************
 ****      Arduino SETUP          ****
 *************************************/
void setup() {
	//nh.getHardware()->setBaud(57600); //The HC06 and 05 use by default 9600 baud rate
	nh.initNode();
	nh.subscribe(cmd_vel_sub);
	nh.advertise(str_pub);
	//nh.advertise(sonar_pub);
	ok_rosstr.data = "arduino ok";
	pinMode(LED, OUTPUT);
	//Motor
	left_motor.init(MOTORI_PINA, MOTORI_PINB, MOTORI_ENABLE);
	right_motor.init(MOTORD_PINA, MOTORD_PINB, MOTORD_ENABLE);

}

/***********************************************
 *            Arduino MAIN LOOP
 **********************************************/
long publisher_timer;
int16_t sensor_reading;
void loop() {
	  long newPositionD = myEncD.read();
	  long newPositionI = myEncI.read();

	  if (!(millis() % ODOM_PUBLISH_TIME)) {	//Send wheel position every 2 ms
		  wheel_pose.x= newPositionD;
		  wheel_pose.y=newPositionI;
		  wheel_pose.z=0;
		  wheel_pose_pub.publish(&wheel_pose);
		   nh.spinOnce();
	  }
	//I will just keep the loop waiting for a message
	//in the ros topic
	if (!(millis() % CONTROLLER_TIME)) {	//Control the motors every 10ms aprox.
		move_robot(global_linx, global_angz);
	}

//	if (!(millis() % 3000)) {	//Say I'm ok once in a while
//		str_pub.publish(&ok_rosstr);
//		nh.spinOnce();
//	}

	if ((vl <= 255 && vl >= -255) && (vr <= 255 && vl >= -255)) {
		left_motor.move(vl); // turn it on going backward
		right_motor.move(vr); // turn it on going backward
	} else {
		left_motor.soft_stop();
		right_motor.soft_stop();
	}
	nh.spinOnce();
	delay(1);
}
