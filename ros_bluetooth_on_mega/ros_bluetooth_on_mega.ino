/* In this project we want to implement rosserial using the
 * Serial1 port of arduino board.
 *
 * Mantainer: Jesus Arturo Escobedo Cabello
 * jaescobedo@iteshu.edu.mx
 *
 */
//#define USE_USBCON //Use this line only for Arduino Leonardo with USB Cable
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>

class NewHardware : public ArduinoHardware
{
  public:
  NewHardware():ArduinoHardware(&Serial1, 9600){};
};

ros::NodeHandle_<NewHardware>  nh;

std_msgs::String global_rosstr;
std_msgs::String iheard_rosstr;
char global_char[10];

void btInputCb(const std_msgs::String& msg) {
	digitalWrite(13, HIGH - digitalRead(13));   // blink the led
	strcpy(global_char, msg.data);
}

ros::Subscriber<std_msgs::String> sub("bluetooth_input", &btInputCb);
ros::Publisher pub("bluetooth_output", &global_rosstr);

void setup() {
	nh.initNode();
	nh.subscribe(sub);
	nh.advertise(pub);
	iheard_rosstr.data = "I heard: ";
	global_rosstr.data = "";
	pinMode(13, OUTPUT);
}

void loop() {
	global_rosstr.data = global_char;
	if (!(millis() % 100)) {
		pub.publish(&iheard_rosstr);
		nh.spinOnce();
		pub.publish(&global_rosstr);
	}
	nh.spinOnce();
	delay(1);
}

