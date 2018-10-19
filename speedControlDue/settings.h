//#include <TimerOne.h>
#include <DueTimer.h>
#include <Encoder.h>
#include <PID_v1.h>

#define DATA_SEND 3

#define LED 13
#define ENCODER_R_GND 47      //GREEN
#define ENCODER_R_PWR 49      //BLUE
#define ENCODER_R_A 51        //YELLOW
#define ENCODER_R_B 53        //WITHE
#define ENCODER_L_GND 39      //GREEN
#define ENCODER_L_PWR 41      //BLUE
#define ENCODER_L_A 45        //YELLOW
#define ENCODER_L_B 43        //WHITE

#define EXT_INT_PIN_2   0
#define EXT_INT_PIN_3   1
#define EXT_INT_PIN_21  2
#define EXT_INT_PIN_20  3
#define EXT_INT_PIN_19  4
#define EXT_INT_PIN_18  5

#define RIGHT_MOT_DIR_BACK 5
#define RIGHT_MOT_DIR_FRONT 6
#define RIGHT_MOT_EN 7 //enable
#define LEFT_MOT_DIR_BACK 3
#define LEFT_MOT_DIR_FRONT 4
#define LEFT_MOT_EN 2 //enable

#define MOTOR_FRONT 0
#define MOTOR_BACK  1
#define MOTOR_STOP  2

////////// CONSTANTS HUMMER MOTOSHIELD ////////////
#define BRAKEVCC 0
#define CW   1
#define CCW  2
#define BRAKEGND 3
#define CS_THRESHOLD 100

/*  VNH2SP30 pin definitions
 xxx[0] controls '1' outputs
 xxx[1] controls '2' outputs */
const int8_t inApin[2] = {7, 4};  // INA: Clockwise input
const int8_t inBpin[2] = {8, 9};  // INB: Counter-clockwise input
const int8_t pwmpin[2] = {5, 6};  // PWM input
const int8_t cspin[2] = {2, 3};   // CS: Current sense ANALOG input
const int8_t enpin[2] = {0, 1};   // EN: Status of switches output (Analog pin)

//////////////////////////////////////////////////

const int ENCODER_PULSES = 896*4;
const int FREQ_TIMER_1 = 100;   //[Hz] dont forget change in Timer1.initialize();
const float CONST_RPS = (float) FREQ_TIMER_1/ (float) ENCODER_PULSES;

const float pi = 3.14159265359;

float referencia1=0, referencia2=0;


//--------- GLOBAL VARIABLES ----------//
int ctu_r = 0;
int ctu_l = 0;
int pps_r = 0;
int pps_l = 0;
float rps_r = 0;
float rps_l = 0;

double rpm_r = 0, rads_r = 0;
double rpm_l = 0, rads_l = 0;

bool timerFlag = false;

float currentTime = 0;

//---------------- PID variables -----------------//
double rPWM = 0, lPWM = 0;
double rsetPoint = 0.0, lsetPoint = 0.0;

//-------------- Compute odometry ----------------//
//Robot Hardware
#define WHEELRAD 0.052 //[m] The radius of the wheel
#define WHEELDIST 0.264 //[m] Distance between wheels
#define CM 2*PI*WHEELRAD/ENCODER_PULSES   //const encoder pulse/Linear displacement

float theta = 0;
float x = 0;
float y = 0;
int16_t p=0;


void odometry(int pps_r, int pps_l){
  float Dl = CM*pps_l;    //Left distance wheel
  float Dr = CM*pps_r;    //Right distance wheel
  float Dc = (Dr+Dl)/2;   //*CHECK*Center robot distance
  theta = theta + (Dr-Dl)/WHEELDIST;  //theta robot
  x = x + Dc*cos(theta);
  y = y + Dc*sin(theta);
  p=p+pps_r;
}

