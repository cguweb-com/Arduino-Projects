/*
 * 
 *   NovaSM3 - a Spot-Mini Micro clone
 *   Version: 5.2
 *   Version Date: 2021-10-26
 *   
 *   Author:  Chris Locke - cguweb@gmail.com
 *   Nova's website:  https://novaspotmicro.com
 *   GitHub Project:  https://github.com/cguweb-com/Arduino-Projects/tree/main/Nova-SM3
 *   YouTube Playlist:  https://www.youtube.com/watch?v=00PkTcGWPvo&list=PLcOZNHwM_I2a3YZKf8FtUjJneKGXCfduk
 *     
 *   Copyright (c) 2020-2021 Christopher M. Locke and others
 *   Distributed under the terms of the MIT License. 
 *   SPDX-License-Identifier: MIT
 *   
 *   Permission is hereby granted, free of charge, to any person obtaining
 *   a copy of this software and associated documentation files (the
 *   "Software"), to deal in the Software without restriction, including
 *   without limitation the rights to use, copy, modify, merge, publish,
 *   distribute, sublicense, and/or sell copies of the Software, and to
 *   permit persons to whom the Software is furnished to do so, subject to
 *   the following conditions:
 *   
 *   The above copyright notice and this permission notice shall be
 *   included in all copies or substantial portions of the Software.
 *   
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *   EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 *   MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *   NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 *   LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 *   OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 *   WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *   
*/

//total counts for reference and iterations
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
byte activeSweep[TOTAL_SERVOS];         //tracks active sweeping servos
float servoSweep[TOTAL_SERVOS][6];      //sweep parameters: start_pos, target pos, sweep type, loops, delay ms, [unused]
byte servoSwitch[TOTAL_SERVOS];         //tracks direction of sweeping servos

byte activeServo[TOTAL_SERVOS];         //tracks active moving servos
float servoSpeed[TOTAL_SERVOS];         //sets speed for servos
float servoPos[TOTAL_SERVOS];           //current position of servos
float targetPos[TOTAL_SERVOS];          //target position of servos
int servoStep[TOTAL_SERVOS];            //tracks stepping of servos
float servoRamp[TOTAL_SERVOS][8];       //ramping parameters: speed, travel_distance, ramp1_spd, ramp1_dist, ramp1_inc, ramp2_spd, ramp2_dist, ramp2_inc

int servoSequence[TOTAL_LEGS];          //tracks sequenced moves of servos
int servoDelay[TOTAL_SERVOS][2];        //sets sequencing delay of servos


//sets up controller board(s) and motor pin numbers
int servoSetup[TOTAL_SERVOS][2] = {       //driver ID, pin number
  {1,0},  {1,1},  {1,2},                  //RFx
  {1,4},  {1,5},  {1,6},                   //LFx
  {1,8},  {1,9},  {1,10},                 //RRx
  {1,12}, {1,13}, {1,14},                 //LRx
};

//groups servos into legs
int servoLeg[TOTAL_LEGS][3] = {           //coxa, femu, tibia
  {RFC,RFF,RFT},  {LFC,LFF,LFT},  
  {RRC,RRF,RRT},  {LRC,LRF,LRT},
};

//sets home position of servos
float servoHome[TOTAL_SERVOS] = {         //home pos
  328, 280, 520,                          //RFx
  370, 472, 280,                          //LFx
  375, 331, 370,                          //RRx
  374, 451, 213,                          //LRx
};

//sets min and max positions of servos
float servoLimit[TOTAL_SERVOS][2] = {     //min, max
  {290, 410}, {185, 515}, {375, 617},     //RFx 
  {408, 288}, {567, 237}, {425, 183},     //LFx
  {337, 457}, {236, 566}, {225, 467},     //RRx
  {412, 292}, {566, 216}, {358, 116},     //LRx
};

/*
//DEV/TEST VALUES FOR MINIMAL MOVEMENT!
float servoLimit[TOTAL_SERVOS][2] = {     //min, max
  {348, 368}, {270, 290}, {495, 515},     //RFx 
  {374, 354}, {452, 432}, {225, 205},     //LFx
  {354, 374}, {331, 351}, {420, 440},     //RRx
  {374, 354}, {351, 331}, {285, 265},     //LRx
};
*/


//step move parameters for sequenced movement of servos
int servoStepMoves[TOTAL_SERVOS][6] = {               //step1, step2, etc
  {0,0,0,0,0,0}, {0,0,0,0,0,0}, {0,0,0,0,0,0},        //RFx
  {0,0,0,0,0,0}, {0,0,0,0,0,0}, {0,0,0,0,0,0},        //LFx
  {0,0,0,0,0,0}, {0,0,0,0,0,0}, {0,0,0,0,0,0},        //RRx
  {0,0,0,0,0,0}, {0,0,0,0,0,0}, {0,0,0,0,0,0},        //LRx
};

//not currently used, for future kinematics
int bodyBone[3] = {       //bone lengths in mm : length, width, height 
  180, 120, 200           //(ie: measured on pivots: shoulder-to-shoulder, coax-to-coax, and ground-to-coax)
};

//not currently used, for future kinematics
int servoBone[TOTAL_LEGS][3] = {           //bone lengths in mm : tibia, femur, shoulder
  {132, 105, 90},     //RFC, RFF, RFT
  {132, 105, 90},     //LFx
  {132, 105, 90},     //RRx
  {132, 105, 90},     //LRx
};
