// Do not remove the include below
#include "ros_bluetooth.h"

ros::NodeHandle nh;
std_msgs::String global_rosstr;
std_msgs::String iheard_rosstr;
char global_char[10];


void btInputCb( const std_msgs::String& msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  strcpy(global_char, msg.data);
}

ros::Subscriber<std_msgs::String> sub("bluetooth_input", &btInputCb);
ros::Publisher pub("bluetooth_output",&global_rosstr);


void setup()
{
  nh.getHardware()->setBaud(9600);//The HC06 and 05 use by default 9600 baud rate
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  iheard_rosstr.data = "I heard: ";
  global_rosstr.data = "";
  pinMode(13,OUTPUT);
}


void loop()
{
  global_rosstr.data=global_char;
  if (!(millis()%100)){
    pub.publish(&iheard_rosstr);
    nh.spinOnce();
    pub.publish(&global_rosstr);
  }
  nh.spinOnce();
  delay(1);
}
