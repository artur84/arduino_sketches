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
char global_char[10];


/*
 * My functions
 */
/* Moves the left wheel forward
 * motor: LEFT or RIGHT (0 or 1)
 * dir: FORWARD or BACKWARD (+1 or -1) to choose the motor direction
 * speed: a value between 0 and 255
 */
void move_motor(int motor, int dir, int speed) {
	if (motor == LEFT) {
		if (dir == FORWARD) {
			digitalWrite(LEFT_MOT_POS, 1);
			digitalWrite(LEFT_MOT_NEG, 0);
			analogWrite(LEFT_MOT_EN, speed);
		} else if (dir == BACKWARD) {
			digitalWrite(LEFT_MOT_POS, 0);
			digitalWrite(LEFT_MOT_NEG, 1);
			analogWrite(LEFT_MOT_EN, speed);
		} else {
			//Stop if received an wrong direction
			digitalWrite(LEFT_MOT_POS, 0);
			digitalWrite(LEFT_MOT_NEG, 0);
			analogWrite(LEFT_MOT_EN, 0);
		}

	} else if (motor == RIGHT) {
		if (dir == FORWARD) {
			digitalWrite(RIGHT_MOT_POS, 1);
			digitalWrite(RIGHT_MOT_NEG, 0);
			analogWrite(RIGHT_MOT_EN, speed);
		} else if (dir == BACKWARD) {
			digitalWrite(RIGHT_MOT_POS, 0);
			digitalWrite(RIGHT_MOT_NEG, 1);
			analogWrite(RIGHT_MOT_EN, speed);
		} else {
			//Stop if received an wrong direction
			digitalWrite(RIGHT_MOT_POS, 0);
			digitalWrite(RIGHT_MOT_NEG, 0);
			analogWrite(RIGHT_MOT_EN, 0);
		}

	}

}

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

//in this example pub is declared befor the cmd_vel_cb
//because it used there
ros::Publisher str_pub("arduino/str_output",&ok_rosstr);

//Twist callback
void cmd_vel_cb(const geometry_msgs::Twist& cmd_msg) {
	digitalWrite(13, HIGH - digitalRead(13)); //toggles a led
	str_pub.publish(&callback_rosstr);
	float x = cmd_msg.linear.x;
	int motor_speed = 55 + 50 * x;    //linear x should be maximum 4

	if ((x < 0) && (x >= -4)) {
		move_motor(LEFT, BACKWARD, motor_speed); // turn it on going backward
		move_motor(RIGHT, BACKWARD, motor_speed); // turn it on going backward
	} else if ((x > 0) && (x <= 4)) {
		move_motor(LEFT, FORWARD, motor_speed); // turn it on going forward
		move_motor(RIGHT, FORWARD, motor_speed); // turn it on going forward
	} else { //Stop if 0 or a value out of boundaries is sent
		motor_speed = 0;
		hard_stop(LEFT);
		soft_stop(RIGHT);
	}
}

//String callback
void str_cb( const std_msgs::String& msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  strcpy(global_char, msg.data);
  str_pub.publish(&msg);
}

//Creates the ROS publishers and subscribers
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("arduino/cmd_vel", cmd_vel_cb);
ros::Subscriber<std_msgs::String> str_sub("arduino/str_input", str_cb);
/*
 * Arduino SETUP
 */
void setup() {
	nh.getHardware()->setBaud(57600); //The HC06 and 05 use by default 9600 baud rate
	nh.initNode();
	nh.subscribe(cmd_vel_sub);
	nh.subscribe(str_sub);
	nh.advertise(str_pub);
	ok_rosstr.data = "arduino ok";
	callback_rosstr.data = "cb executed";
	pinMode(13, OUTPUT);
	pinMode(LEFT_MOT_NEG, OUTPUT);
	pinMode(LEFT_MOT_POS, OUTPUT);
	pinMode(LEFT_MOT_EN, OUTPUT);
	pinMode(RIGHT_MOT_NEG, OUTPUT);
	pinMode(RIGHT_MOT_POS, OUTPUT);
	pinMode(RIGHT_MOT_EN, OUTPUT);

}

/*
 * Arduino MAIN LOOP
 */
void loop() {
	//I will just keep the loop waiting for a message
	//in the ros topic
	nh.spinOnce();
	nh.spinOnce();
	if (!(millis()%3000)){
	    str_pub.publish(&ok_rosstr);
	    nh.spinOnce();
	  }
	nh.spinOnce();
	delay(1);
}
