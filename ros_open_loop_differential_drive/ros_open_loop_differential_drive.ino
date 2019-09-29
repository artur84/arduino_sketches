/*** This sketch implements the model for a differential drive vehicle.
    You should adjust the parameters for the wheels and dimension of your robot.
    It assumes an open loop control based on PWM to keep it very simple.

    It receives a ROS Twist() message and computes the required velocities for the right and left wheels.
    Subscribed topics:
      /cmd_vel  (geometry_msgs.Twist())
    Published topics:
      /string   (std_msgs.String()) A string to send back info to the ROS PC used for debugging
*/
// Do not remove the include below
#include "ros_open_loop_differential_drive.h"

/********************************
 ******  Global variables  ******
 ********************************/
ros::NodeHandle nh;
volatile float global_linx = 0, global_angz = 0;
//std_msgs::Float32 float_debug;

//Motores
Motor left_motor, right_motor;

//Twist callback
void cmd_vel_cb(const geometry_msgs::Twist& cmd_msg) {
  digitalWrite(LED, HIGH - digitalRead(LED)); //toggles a led
  global_linx = cmd_msg.linear.x;
  global_angz = cmd_msg.angular.z;
}

//Creates the ROS publishers and subscribers
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_vel_cb);
//ros::Publisher float_pub("float_debug", &float_debug);



void move_robot(float linearx, float angularz) {
	//Equations for a differential drive mobile robot.
  // 1 rad/sec = 9.55 RPM
  // 0.5*9.55=4.77 (scale factor to put it in RPM)
  float scale=255.0*4.77/MAX_MOTOR_RPM; //To adjust PWM values to be between 0 and 255, this is ugly I should change it if I have time.
	float wr = ((2.0 * linearx + WHEELDIST * angularz) / (WHEELRAD))*scale; // wr in PWM
  float wl = ((2.0 * linearx - WHEELDIST * angularz) / (WHEELRAD))*scale; // wl in PWM
  // Check if the computed values are between the appropriate PWM values.
  int pwm_left = (int) (round(wl));
  int pwm_right = (int) (round(wl));
  
  if ((pwm_left <= 255 && pwm_left >= -255) && (pwm_right <= 255 && pwm_right >= -255)) {
    //float_debug.data=(float)pwm_left;
    //float_pub.publish(&float_debug);
    left_motor.move(pwm_left);
    right_motor.move(pwm_right);
  } else { //If some strange command was computed then stop the robot
    left_motor.soft_stop();
    right_motor.soft_stop();
  }
}




/*************************************
 ****      Arduino SETUP          ****
 *************************************/
void setup() {
	//nh.getHardware()->setBaud(57600); //The HC06 and 05 use by default 9600 baud rate
	nh.initNode();
	nh.subscribe(cmd_vel_sub);
  //nh.advertise(float_pub);
	pinMode(LED, OUTPUT);
	//Initialize motors
	left_motor.init(MOTORI_PINA, MOTORI_PINB, MOTORI_ENABLE);
	right_motor.init(MOTORD_PINA, MOTORD_PINB, MOTORD_ENABLE);

}

/***********************************************
 *            Arduino MAIN LOOP
 **********************************************/
void loop() {
	//I will just keep the loop waiting for a message
	//in the ros topic
	if (!(millis() % CONTROLLER_TIME)) {	//Control the motors every CONTROLLER_TIME [ms].
		move_robot(global_linx, global_angz);
	}
	nh.spinOnce();
	delay(1);
}
