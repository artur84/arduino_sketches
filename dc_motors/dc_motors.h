// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _dc_motors_H_
#define _dc_motors_H_
#include "Arduino.h"
//add your includes for the project ros_dc_motor here

/*******************************
 * Define the material
 ******************************/
#define RIGHT__MOT_DIR_FRONT 49
#define RIGHT_MOT_DIR_BACK 48
#define RIGHT_MOT_EN 2 //enable
#define LEFT_MOT_DIR_FRONT 51
#define LEFT_MOT_DIR_BACK 50
#define LEFT_MOT_EN 3 //enable
#define BACKWARD -1  //moves backwards


/*******************************
 * Other tags
 *******************************/
#define FORWARD 1
#define LEFT 1
#define RIGHT 0

//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project dc_motors here




//Do not add code below this line
#endif /* _dc_motors_H_ */
