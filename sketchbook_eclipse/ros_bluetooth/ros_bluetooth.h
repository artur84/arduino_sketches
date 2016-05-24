// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

/*** CHECK THIS FOR ARDUINO UNO OR LEONARDO
 #ifndef USE_USBCON
 #define USE_USBCON //Comment this line if you are using Arduino UNO, or leonardo with bluetooth.
 //Uncomment for normal arduino leonardo
 */

#ifndef _ros_bluetooth_H_
#define _ros_bluetooth_H_

#include "Arduino.h"
#include <ros.h>
#include <std_msgs/String.h>

//add your includes for the project ros_bluetooth here

//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project ros_bluetooth here

//Do not add code below this line
#endif /* _ros_bluetooth_H_ */
