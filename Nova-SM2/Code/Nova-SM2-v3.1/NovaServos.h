/*
 *   NovaSM2 - a Spot-Mini Micro clone
 *   Version: 3.1
 *   Version Date: 2021-03-26
 *   
 *   Author:  Chris Locke - cguweb@gmail.com
 *   GitHub Project:  https://github.com/cguweb-com/Arduino-Projects/tree/main/Nova-SM2
 *   Instructables Project: https://www.instructables.com/member/cguweb/instructables/
 *   
*/

#define TOTAL_SERVOS 12
#define TOTAL_LEGS 4

//assign servo names for array servoID ref
#define RFC 0
#define RFF 1
#define RFT 2
#define LFC 3
#define LFF 4
#define LFT 5

#define RRC 6
#define RRF 7
#define RRT 8
#define LRC 9
#define LRF 10
#define LRT 11

//assign leg names for array ref
#define RF 0
#define LF 1
#define RR 2
#define LR 3


//setup servo data arrays 
byte activeServo[TOTAL_SERVOS];
byte activeSweep[TOTAL_SERVOS];
byte servoSwitch[TOTAL_SERVOS];
float servoSpeed[TOTAL_SERVOS];
float servoPos[TOTAL_SERVOS];
float targetPos[TOTAL_SERVOS];
float servoSweep[TOTAL_SERVOS][6];  //start_pos, target pos, sweep type, loops, delay ms, [unused]
float servoRamp[TOTAL_SERVOS][8]; //speed, travel_distance, ramp1_spd, ramp1_dist, ramp1_inc, ramp2_spd, ramp2_dist, ramp2_inc
int servoSequence[TOTAL_LEGS];
int servoDelay[TOTAL_SERVOS][2];
int servoStep[TOTAL_SERVOS];

int servoSetup[TOTAL_SERVOS][2] = {       //driver, pin
  {1,0},  {1,1},  {1,2},                  //RFx
  {1,4},  {1,5},  {1,6},                 	//LFx
  {1,8},  {1,9},  {1,10},                 //RRx
  {1,12}, {1,13}, {1,14},                 //LRx
};

int servoLeg[TOTAL_LEGS][3] = {       //coxa, femu, tibia
  {RFC,RFF,RFT},  {LFC,LFF,LFT},  {RRC,RRF,RRT},  {LRC,LRF,LRT},
};

float servoHome[TOTAL_SERVOS] = {         //home pos
  368, 301, 435,                          //RFx
  355, 422, 355,                          //LFx
  348, 344, 396,                          //RRx
  364, 341, 306,                          //LRx
};

float servoLimit[TOTAL_SERVOS][2] = {     //min, max
  {330, 450}, {226, 466}, {322, 548},     //RFx 
  {393, 273}, {497, 257}, {468, 242},     //LFx
  {310, 430}, {269, 509}, {283, 509},     //RRx
  {402, 282}, {416, 176}, {419, 193},     //LRx
};

int servoStepMoves[TOTAL_SERVOS][6] = {   			      //step1, step2, etc
  {0,0,0,0,0,0}, {0,0,0,0,0,0}, {0,0,0,0,0,0},        //RFx
  {0,0,0,0,0,0}, {0,0,0,0,0,0}, {0,0,0,0,0,0},        //LFx
  {0,0,0,0,0,0}, {0,0,0,0,0,0}, {0,0,0,0,0,0},        //RRx
  {0,0,0,0,0,0}, {0,0,0,0,0,0}, {0,0,0,0,0,0},        //LRx
};

int bodyBone[3] = {       //bone lengths in mm : length, width, height 
  180, 120, 200           //(ie: measured on pivots: shoulder-to-shoulder, coax-to-coax, and ground-to-coax)
};

int servoBone[TOTAL_LEGS][3] = {           //bone lengths in mm : tibia, femur, shoulder
  {132, 105, 90},     //RFC, RFF, RFT
  {132, 105, 90},     //LFx
  {132, 105, 90},     //RRx
  {132, 105, 90},     //LRx
};
