#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
//#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;

//--------- Pulishers ---------//
//Topic Int16 type
std_msgs::Int16 adc_msg;
ros::Publisher adcData("adc",&adc_msg);

/*//comment for future use
//Topic speedMotors Int16MultiArray
std_msgs::Int16MultiArray speedMotors_msg;
ros::Publisher speedMotors("speedMotors",&speedMotors_msg);
*/

//Topic speedMotors [rad/s]
std_msgs::Float32MultiArray speedMotors_float_msg;
ros::Publisher speedMotors_float("speedMotors_float",&speedMotors_float_msg);

//Topic position and orientation robot [m] and [rad]
std_msgs::Float32MultiArray robotPosition_msg;
ros::Publisher robotPosition("robotPosition",&robotPosition_msg);

//Topic uControl rigth left robot [Volt]
std_msgs::Float32MultiArray uControl_msg;
ros::Publisher uControl("uControl",&uControl_msg);




//--------- Subscribers ---------//
//** callback speedMotors setpoint [pwm]
//void setpoint_cb(const std_msgs::UInt16MultiArray& setpoint_msg){
//  analogWrite(RIGHT_MOT_EN, setpoint_msg.data[1]);
//  analogWrite(13, setpoint_msg.data[1]);
//}

//ros::Subscriber<std_msgs::UInt16MultiArray> setpoint("setPoint", setpoint_cb);

//** callback speedMotors setpoint [rad/s]
void setpoint_f_cb(const std_msgs::Float32MultiArray& setpoint_msg){
//  analogWrite(RIGHT_MOT_EN, setpoint_msg.data[0]);
//  analogWrite(LEFT_MOT_EN, setpoint_msg.data[1]);
  analogWrite(13, abs((int16_t)setpoint_msg.data[1]));
    rsetPoint=setpoint_msg.data[0];
    lsetPoint=setpoint_msg.data[1];
}

ros::Subscriber<std_msgs::Float32MultiArray> setpoint_f("setPoint_f", setpoint_f_cb);

/*
void cmd_vel_cb(const geometry_msgs::Twist& cmd_vel_msg){
  //TODO: calcular las velocidades de las ruedas a partir de la
  //      velocidad del robot
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_vel_cb);
*/
//////////////////////////////////////--- initROS ---//////////////////////////////////////

void initROS(){
    //nh.getHardware()->setBaud(115200);
  
  nh.initNode();

  //-------- init pulishers --------//
  nh.advertise(adcData);

  //Init the array Int16 speedMotors
/*  speedMotors_msg.layout.dim = (std_msgs::MultiArrayDimension *)
    malloc(sizeof(std_msgs::MultiArrayDimension)*DATA_SEND);
  speedMotors_msg.layout.dim[0].label = "height";
  speedMotors_msg.layout.dim[0].size = DATA_SEND;
  speedMotors_msg.layout.dim[0].stride = 1;
  speedMotors_msg.layout.data_offset = 0;
  speedMotors_msg.data = (int *)malloc(sizeof(int)*8);
  speedMotors_msg.data_length = DATA_SEND;
  nh.advertise(speedMotors);
*/
  //Init the array Float32 speedMotors
  speedMotors_float_msg.layout.dim = (std_msgs::MultiArrayDimension *)
    malloc(sizeof(std_msgs::MultiArrayDimension)*DATA_SEND);
  speedMotors_float_msg.layout.dim[0].label = "height";
  speedMotors_float_msg.layout.dim[0].size = DATA_SEND;
  speedMotors_float_msg.layout.dim[0].stride = 1;
  speedMotors_float_msg.layout.data_offset = 0;
  speedMotors_float_msg.data = (float *)malloc(sizeof(float)*8);
  speedMotors_float_msg.data_length = DATA_SEND;
  nh.advertise(speedMotors_float);

  //Init the array Float32 robotPosition
  robotPosition_msg.layout.dim = (std_msgs::MultiArrayDimension *)
    malloc(sizeof(std_msgs::MultiArrayDimension)*DATA_SEND);
  robotPosition_msg.layout.dim[0].label = "height";
  robotPosition_msg.layout.dim[0].size = DATA_SEND;
  robotPosition_msg.layout.dim[0].stride = 1;
  robotPosition_msg.layout.data_offset = 0;
  robotPosition_msg.data = (float *)malloc(sizeof(float)*8);
  robotPosition_msg.data_length = DATA_SEND;
  nh.advertise(robotPosition);

  //Init the array Float32 uControl
  uControl_msg.layout.dim = (std_msgs::MultiArrayDimension *)
    malloc(sizeof(std_msgs::MultiArrayDimension)*DATA_SEND);
  uControl_msg.layout.dim[0].label = "height";
  uControl_msg.layout.dim[0].size = DATA_SEND;
  uControl_msg.layout.dim[0].stride = 1;
  uControl_msg.layout.data_offset = 0;
  uControl_msg.data = (float *)malloc(sizeof(float)*8);
  uControl_msg.data_length = DATA_SEND;
  nh.advertise(uControl);
  
  
  //-------- init Subscribers --------//
  //nh.subscribe(setpoint);
  nh.subscribe(setpoint_f);
}

void MsgROS(){
  speedMotors_float_msg.data[0] = currentTime;
  speedMotors_float_msg.data[1] = rads_r;
  speedMotors_float_msg.data[2] = rads_l;
  speedMotors_float.publish(&speedMotors_float_msg);

  robotPosition_msg.data[0] = theta;
  robotPosition_msg.data[1] = x;
  robotPosition_msg.data[2] = y;
  robotPosition.publish(&robotPosition_msg);

  uControl_msg.data[0] = rPWM;      //rigth wheel
  uControl_msg.data[1] = lPWM;      //left wheel
  uControl_msg.data[2] = 34.23;
  uControl.publish(&uControl_msg);
  
  nh.spinOnce();  //last line on code
}
