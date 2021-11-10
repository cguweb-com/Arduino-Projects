

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
 *   DEBUG NOTE: Teensy will not run if disconnected from serial with any d ebug flag set, as these all
 *               write to serial. Be sure to set all debug flags to 0 before running offline.
 *   
 *   RELEASE NOTES:
 *      Teensy 4.0 performance: 6% storage / 29% memory
 *      Added NRF Module and interface to NovaSMRemote
 *      Removed PS2 setup and interface
 *      Added DFPlayer Mini Pro as second option of MP3 player
 *      Fixed volume pot
 *      Activated code for volume control of MP3 player if enabled
 *      Fixed voltage divider / battery monitor equation
 *      Removed piezo buzzer and TONE support
 *      Added MP3_PLAYER_TYPE setting for DFPlayer Mini and Mini Pro selection
 *
 *
 *   DEV NOTES:
 *      teensy mpu / code is not playing as nicely as with mega - not smooth movement! :(
 *      work on integrating MPU data into movements
 *      adjust sensitivity of PIR (seems 12-24 inches too close)
 *      
 *      BUG: ramping: on interruption of ramp, servo speed is set to the speed of the point of interrupt
 *      
 *      
 *      see more 'DEV NOTE' comments in code for other bugs/tasks
 *      all 'DEV WORK' comments denote code that is currently under development and will be removed upon completion
 *     
 *   =============================================================
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
 *      
*/

//DEVWORK
int  test_loops = 0;  
int  test_steps = 0;  

//set Nova SM3 version
#define VERSION 5.2

//debug vars for displaying operation runtime data for debugging
const byte debug  = 0;            //general messages
const byte debug1 = 0;            //remote commands and pir sensors
const byte debug2 = 0;            //debug servo steps
const byte debug3 = 0;            //ramping and sequencing
const byte debug4 = 0;            //amperage and battery 
const byte debug5 = 0;            //mpu
const byte debug6 = 0;            //serial communication output/response and serial terminal commands
const byte debug7 = 0;            //uss sensors
const byte debug8 = 0;            //NRF24 comm
const byte plotter = 0;           //plot servo steps, turn off debug1

byte debug_leg = 0;               //default debug leg (3 servos) (changed by serial command input)
int debug_servo = 2;              //default debug servo (changed by serial command input)
int debug_loops = 3;              //default loops for debug movements
int debug_loops2 = 3;             //movement decremented loop
int debug_spd = 10;               //default speed for debug movements

//activate/deactivate devices
byte slave_active = 1;            //activate slave arduino nano
byte pwm_active = 1;              //activate pwm controller / servos
byte nrf_active = 1;              //activate NRF24 remote control
byte serial_active = 1;           //activate serial monitor command input
byte mpu_active = 0;              //activate MPU6050 
byte rgb_active = 1;              //activate RGB modules
byte oled_active = 1;             //activate OLED display
byte pir_active = 1;              //activate PIR motion sensor
byte uss_active = 0;              //activate Ultra-Sonic sensors
byte amp_active = 0;              //activate amperate monitoring
byte batt_active = 1;             //activate battery level monitoring
byte buzz_active = 0;             //activate simple tone sounds 
byte melody_active = 1;           //activate melodic tone sounds 
byte mp3_active = 1;              //activate if mp3 player installed
byte splash_active = 1;           //show all loading graphics & sounds
byte quick_boot = 0;              //skip most loading graphics & sounds

#define MP3_PLAYER_TYPE 1         //set to 1 for Mini Pro or to 0 for Mini

//include supporting libraries
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <T4_PowerButton.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <RF24.h>

//MP3 Setup
//#if MP3_PLAYER_TYPE > 0
  #include <DFRobot_PLAY.h>
  DFRobot_PLAY DFPlayer;
//#else
//  #include <DFRobotDFPlayerMini.h>
//  DFRobotDFPlayerMini DFPlayer;
//#endif


//Teensy 2nd i2c bus and onboard led
#define SDA2_PIN 17
#define SCL2_PIN 16
#define LED_PIN 13

//pwm controller
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
#define SERVO_FREQ 60             //CAUTION: do not change this once calibrated to servos!
#define OSCIL_FREQ 25000000       //CAUTION: do not change this once calibrated to servos!
#define OE_PIN 3                  //PWM Output Enable pin
byte pwm_oe = 0;                  //boolean control for enable / disable output
const float min_spd = 32.0;       //min is higher than max, since this is the time increment delay between servo calls 
const float max_spd = 0.0;        //maximum, fastest speed
float default_spd = 12.0;
float spd = default_spd;
float spd_prev = default_spd;
float spd_factor = 1.0;           //ratio factor used in movements
const float min_spd_factor = 5.0;
const float max_spd_factor = 0.5;
float spd_c;
float spd_f;
float spd_t;
byte step_start = 0;              //boolean to check if sequenced steps are running
float x_dir = 0;
float y_dir = 0;
float z_dir = 0;
byte use_ramp = 0;                //boolean to enable / disable ramping function
float ramp_dist = 0.20;           //ramp percentage distance from each end of travel (ex: 0.20 ramps up from 0-20% of travel, then ramps down 80-100% of travel)
float ramp_spd = 5.00;            //default ramp speed multiplier

//buzzer
#define BUZZ 22                   //piezo buzzer / speaker module pin

//MP3 DFPlayer 
//track list: 
//  1.saber 2.r2one 3.r2two 4.siren 5.chewy 6.radar 7.mariobro 8.laugh 9.what 10.nova 11.hello
//  12.mode1 13.mode2 14.mode3 15.mode4 16.fon 17.foff 18.sustain 19.warn 20.danger 21.critical 22.halt
//  23.fmodeon 24.fmodeoff 25.click1  
#define MP3_VOL_PIN A9
unsigned int potInterval = 1000;
unsigned long lastPotUpdate = 0;
static const uint8_t PIN_MP3_TX = 1;
static const uint8_t PIN_MP3_RX = 0; 
SoftwareSerial softwareSerial(PIN_MP3_RX, PIN_MP3_TX);

//mp3 player
unsigned int mp3Interval = 50;
unsigned long lastMP3Update = 0;
int mp3_queue[5] = {0,0,0,0,0};   //DEV: queue of sounds to be played
int mp3_status = 0;               //current playing status
int vpot_max = 350;               //for crappy unreliable pots, set this to your pot's max (-1036)
int vpot_min = 50;                //for crappy unreliable pots, set this to your pot's min (+0)
int vpot_loop = 8;                //for crappy unreliable pots, loop and average value
int vpot_cnt = 0;                 //count of loops for average value
int vpot_avg = 0;                 //total for loop and average value
int vol_max = 25;                 //at mp3 player max of 30, small speakers distort - 25 is suggested
int vol_min = 0;                  //set to 0 to essentially turn off sounds at lowest volume
int sound_vol = 15;               //current volume (1-30)
int sound_volp = 15;              //prev current volume (1-30)
int sounds_req = 25;              //required sound files
int sounds_sd = 0;                //sd card sound files
int sound_cnt = 0;                //loop timer count
int sound_cur = 0;                //current loop timer

//rgb leds
int pattern_int = 250;            //set pattern delay between
int pattern_cnt = 3;              //set number of loops of pattern

//pir sensor
#define PIR_FRONT 4
#define PIR_LEFT 5
#define PIR_RIGHT 6
unsigned int pirInterval = 150;
unsigned long lastPIRUpdate = 0;
unsigned int pirDelay = 3;        //number of interval cycles, not ms
int pir_frontState = LOW;
int pir_leftState = LOW;
int pir_rightState = LOW;
int pir_halt = 1;
byte pir_reset = 0;
int pir_wait = 0;
byte pir_state = LOW;
byte pir_val = 0;
int pir_repeat_cnt = 0;
int pir_is_active = pir_active;
int follow_dir = 0;
int follow_dir_prev = 0;

//ultrasonic sensors (2 - left & right)
unsigned int ussInterval = 250;
unsigned long lastUSSUpdate = 0;
int distance_alarm = 10;              //distance to set triggers
int distance_alarm_set = 0;           //count consecutive set triggers
int distance_tolerance = 5;           //threshold before setting triggers
int distance_l;                       //current distance from left sensor
int prev_distance_l;                  //previous distance of left sensor to prevent false positives
int distance_r;                       //current distance from right sensor
int prev_distance_r;                  //previous distance of right sensor to prevent false positives
int uss_is_active = uss_active;


//mpu6050 sensor
const int MPU = 0x68;
unsigned int mpuInterval = 10;
unsigned int mpuInterval_prev = 400;
unsigned long lastMPUUpdate = 0;
float mroll, mpitch, myaw;
float mroll_prev, mpitch_prev, myaw_prev;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float mpu_mroll = 0.00;
float mpu_mpitch = 0.00;
float mpu_myaw = 0.00;
float mpu_trigger_thresh = 0.05;
float elapsedTime, currentTime, previousTime;
int mpu_is_active = mpu_active;
int mpu_c = 0;

//NovaSMRemote NRF24 controller
//Pin wiring for Teensy 4.0:
//  pin 9 -> CE pin
//  pin 10 -> CSN pin
//  pin 13 -> SCK pin
//  pin 11 -> MOSI pin
//  pin 12 -> MISO pin
//  pin xx -> IRQ pin (unused)
// instantiate an object for the nRF24L01 transceiver
RF24 nrf_radio(9, 10);
unsigned int nrfInterval = 50;
unsigned long lastNRFUpdate = 0;
unsigned int remoteInterval = 50;
unsigned long lastRemoteUpdate = 0;

uint8_t address[][6] = {"1Node", "2Node"};
bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
const int data_num = 14;          //bytes of data to receive
bool rc_resp;
uint8_t rc_data[data_num] = {     //receive data array
  0, 0, 0, 0,                     //btn1, btn2, btn3, btn4
  0, 0,                           //sel1, sel2
  0, 0, 0, 0,                     //p1, p2, p3, p4
  0, 0, 0, 0                      //lx, ly, rx, ry
};
uint8_t tm_data[data_num] = {     //transmit data array
  0, 0, 0,                        //battery, spd, volume
  0, 0, 0,                        //move_steps, step_weight_factor_front, step_weight_factor_rear
  1, 0, 0, 0,                     //remote_select, start_mode
  0, 0, 0, 0                      //
};
//init remote control vars
uint8_t btn1 = 0, btn2 = 0, btn3 = 0, btn4 = 0;
uint8_t btn1p = 0, btn2p = 0, btn3p = 0, btn4p = 0;
uint8_t sel1 = 0, sel2 = 0; 
uint8_t sel1p = 0, sel2p = 0; 
int pot_threshold = 5;
uint8_t p1t = pot_threshold, p2t = pot_threshold, p3t = pot_threshold, p4t = pot_threshold; 
uint8_t p1 = 0, p2 = 0, p3 = 0, p4 = 0; 
uint8_t p1p = 0, p2p = 0, p3p = 0, p4p = 0; 
uint8_t lx = 0, ly = 0, rx = 0, ry = 0; 
uint8_t lxp = 0, lyp = 0, rxp = 0, ryp = 0; 
int pot_min = 0;                     //if pots are orientated opposing to this configuration, 
int pot_max = 255;                   //swap these two values where min would be max value, and vice-versa
int remote_start_stop = 0;           //tracks remote start/stop mode
int remote_select = 1;               //sets default button set
byte start_mode = 0;                 //current start mode
int spd_lock = 0;                    //switch to hold prev spd and allow code to control spd, and not input device(s)
int step_lock = 0;                   //switch to hold prev steps and allow code to control move_steps, and not input device(s)


//slave arduino and serial commands
#define SLAVE_ID 1
byte serial_oled = 0;            //switch for serial or oled commands
int serial_resp;
int ByteReceived;
unsigned int serialInterval = 60;
unsigned long lastSerialUpdate = 0;
String serial_input;


//amperage monitor
#define AMP_PIN A1
#define PWR_PIN 2
unsigned int ampInterval = 15000;
unsigned long lastAmpUpdate = 0;
int amp_cnt = 0;
int amp_thresh = 10;            //loop count for consecutive amperage alarms to prevent false positives
int amp_warning = 0;            //current alarm warning level
int amp_loop = 1;               //if warning level set, this changes accordingly to prevent false positives
float amp_limit = 6.5;          //aperage draw limit before triggering alarms


//battery monitor
#define BATT_MONITOR A1
unsigned long batteryInterval = 30000;
unsigned long lastBatteryUpdate = 0;
int batt_cnt = 0;
float batt_voltage = 11.8;                          //approx fully charged battery minimum nominal voltage
float batt_voltage_prev = 11.8;                     //comparison voltage to prevent false positives
float avg_volts = 0;
float batt_levels[10] = {                           //voltage drop alarm levels(10)
   11.2, 11.1, 11.0, 10.9, 10.8, 
   10.7, 10.6, 10.5, 10.4, 10.3,
};


//movement vars for steps, delays, loops, sequencing, etc
unsigned long lastMoveDelayUpdate = millis();
unsigned int moveDelayInterval = 0;
int move_delays[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int move_delay_sequences[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int move_delay = 0;
int move_loops = 0;
int move_switch = 0;
float move_steps_min = -100;
float move_steps_max = 100;
float move_steps = 0;
float move_steps_prev = 0;
float move_steps_x = 0;
float move_steps_y = 0;
float move_steps_yaw_x = 0;
float move_steps_yaw_y = 0;
float move_steps_yaw = 0;
float move_steps_kx = 0;
float move_steps_ky = 0;
float move_steps_alt = 0;
int move_c_steps[2] = {-50, 50};
int move_f_steps[2] = {-165, 165};
int move_t_steps[2] = {-120, 120};
int move_x_steps[2] = {-50, 50};
int move_y_steps[2] = {-180, 130};
int move_z_steps[2] = {-50, 50};
float x_dir_steps[2] = {-22, 22};
float y_dir_steps[2] = {-22, 22};
float z_dir_steps[2] = {-40, 40};


//booleans to control / monitor movement routine execution
byte moving = 0;
byte move_y_axis = 0;
byte move_x_axis = 0;
byte move_roll = 0;
byte move_roll_body = 0;
byte move_pitch = 0;
byte move_pitch_body = 0;
byte move_trot = 0;
byte move_forward = 0;
byte move_backward = 0;
byte move_left = 0;
byte move_right = 0;
byte move_march = 0;
byte move_wake = 0;
byte move_sequence = 0;
byte move_demo = 0;
byte move_wman = 0;
byte move_funplay = 0;
byte move_look_left = 0;
byte move_look_right = 0;
byte move_roll_x = 0;
byte move_pitch_y = 0;
byte move_kin_x = 0;
byte move_kin_y = 0;
byte move_yaw_x = 0;
byte move_yaw_y = 0;
byte move_yaw = 0;
byte move_servo = 0;
byte move_leg = 0;
byte move_follow = 0;
String move_paused = "";

//vars used to compensate for center of gravity / momentum / inertia
float step_weight_factor_front = 1.00;
float step_weight_factor_rear = 1.00;
float step_height_factor = 1.25;    //DEV: not used yet


/*
   -------------------------------------------------------
   Function Prototypes
    :required for functions executed from servo class ( I know, I know, poor OOP design calling functions outside of a class ;p )
   -------------------------------------------------------
*/
void set_ramp(int servo, float sp, float r1_spd, float r1_dist, float r2_spd, float r2_dist);
void amperage_check(int aloop);
void powering_down(void);

//include local class / config files
#include "MPU6050_conf.h"
#include "NovaServos.h"           //include motor setup vars and data arrays
#include "AsyncServo.h"           //include motor class


//instantiate servo objects (s_XXX) with driver reference and servo ID
//coax servo objects
AsyncServo s_RFC(&pwm1, RFC);
AsyncServo s_LFC(&pwm1, LFC);
AsyncServo s_RRC(&pwm1, RRC);
AsyncServo s_LRC(&pwm1, LRC);

//femur servo objects
AsyncServo s_RFF(&pwm1, RFF);
AsyncServo s_LFF(&pwm1, LFF);
AsyncServo s_RRF(&pwm1, RRF);
AsyncServo s_LRF(&pwm1, LRF);

//tibia servo objects
AsyncServo s_RFT(&pwm1, RFT);
AsyncServo s_LFT(&pwm1, LFT);
AsyncServo s_RRT(&pwm1, RRT);
AsyncServo s_LRT(&pwm1, LRT);


void setup() {
  //setup onboard LED
  pinMode(LED_PIN, OUTPUT);
  for (int b = 0; b < 3; b++) {
    digitalWrite(LED_PIN, HIGH);
    delay(150);
    digitalWrite(LED_PIN, LOW);
    delay(150);
  }

  //seed arduino pin A0, otherwise random() functions will not be truely random
  randomSeed(analogRead(0));

  if (debug) {
    Serial.begin(19200);

    //allow serial to connect
//teensy doesn't like this when not connected to serial even with debug disabled?!? 
//    while (!Serial) {
//      delay(1);
//    }
    delay(500);
    Serial.println(F("\n=============================================="));
    Serial.print(F("NOVA SM3 v"));
    Serial.println(VERSION);
    Serial.println(F("=============================================="));
  }

  //setup power off button
  set_arm_power_button_callback(&powering_down);
  if (debug) {
    if (arm_power_button_pressed()) {
      Serial.println("System Restarted");
    }
  }

  //setup 2nd i2c bus
  Wire1.begin();
  Wire1.setSDA(SDA2_PIN);
  Wire1.setSCL(SCL2_PIN);

  if (mp3_active) {
//    #if MP3_PLAYER_TYPE > 0
      softwareSerial.begin(115200);
//    #else
//      softwareSerial.begin(9600);
//    #endif

    if (DFPlayer.begin(softwareSerial)) {
      pinMode(MP3_VOL_PIN, INPUT);
      mp3_volume(sound_vol);

//      #if MP3_PLAYER_TYPE > 0
        DFPlayer.setPlayMode(DFPlayer.SINGLE);
        DFPlayer.setPrompt(false);
        DFPlayer.switchFunction(DFPlayer.MUSIC);
        sounds_sd = DFPlayer.getTotalFile();
//      #else
//        DFPlayer.EQ(DFPLAYER_EQ_ROCK);
//        sounds_sd = DFPlayer.readFileCounts();  
//      #endif
    } else if (debug) {
      Serial.println("Connecting to DFPlayer Mini Pro failed!");
    }
  }

  if (melody_active) {
    if (mp3_active) {
      mp3_volume(16);
      mp3_play(10);
      delay(1500);
      mp3_volume(sound_vol);
      if (rgb_active) {
        rgb_request((char*)"GAk");
      }
      mp3_play(1);
      delay(3000);
      if (!quick_boot) {
        mp3_play(2);
        if (!debug) {
          delay(5000);
        }
      }
    }
  } else if (buzz_active) {
    if (mp3_active) {
      mp3_play(1);
      if (!debug && !quick_boot) {
        delay(3000);
      }
    }
  }

  //show active settings, including delays to prevent IDE reset firing setup functions on upload/connect
  if (debug && !quick_boot) {
    Serial.println(F("Active Settings:"));
    if (slave_active) Serial.println(F("  Slave Circuit"));
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    if (pwm_active) Serial.println(F("  PWM Controller"));
    digitalWrite(LED_PIN, LOW);
    delay(500);
    if (nrf_active) Serial.println(F("  NRF24 Wireless"));
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    if (serial_active) Serial.println(F("  Serial Commands"));
    digitalWrite(LED_PIN, LOW);
    delay(500);
    if (mpu_active) Serial.println(F("  MPU6050 IMU"));
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    if (rgb_active) Serial.println(F("  RGB LEDs"));
    digitalWrite(LED_PIN, LOW);
    delay(500);
    if (oled_active) Serial.println(F("  OLED Display"));
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    if (pir_active) Serial.println(F("  PIR Sensors"));
    digitalWrite(LED_PIN, LOW);
    delay(500);
    if (uss_active) Serial.println(F("  Ultra-Sonic Sensors"));
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);
    if (amp_active) Serial.println(F("  ACS712 Current Sensor"));
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);
    if (batt_active) Serial.println(F("  Battery Monitor"));
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);
    if (buzz_active) Serial.println(F("  Buzzer"));
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);
    if (melody_active) Serial.println(F("  Melody"));
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);
    if (mp3_active) {
      Serial.print(F("  MP3 Player: found "));
      Serial.print(sounds_sd);Serial.print(F(" of "));
      Serial.print(sounds_req);Serial.println(F(" sounds required on disk"));
    }
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);
    if (!splash_active) Serial.println(F("  Skip Splash Screen"));
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);
    Serial.println(F("\n==============================================\n"));
  } else {
    if (!quick_boot) {
      for (int b = 0; b < 4; b++) {
        digitalWrite(LED_PIN, HIGH);
        delay(500);
        digitalWrite(LED_PIN, LOW);
        delay(500);
      }
      for (int b = 0; b < 6; b++) {
        digitalWrite(LED_PIN, HIGH);
        delay(250);
        digitalWrite(LED_PIN, LOW);
        delay(250);
      }
    }
  }
  

  if (mp3_active && debug && !quick_boot) {
    Serial.print(F("MP3 Player intializing..."));
    delay(500);
    Serial.println(F("\t\t\tOK"));
  }


  //init mpu6050
  if (mpu_active) {
    Wire1.begin();
    uint8_t c = readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
    delay(1000); 
  
    if (c == 0x68) {  
      if (debug) Serial.println(F("MPU6050 testing... "));
      MPU6050SelfTest(SelfTest);
      if (debug) {
        delay(300);
        Serial.println(F("  Acceleration Trim:"));
        Serial.print(F("    x-axis : +/- ")); Serial.println(SelfTest[0],1);
        Serial.print(F("    y-axis : +/- ")); Serial.println(SelfTest[1],1);
        Serial.print(F("    z-axis : +/- ")); Serial.println(SelfTest[2],1);
        Serial.println(F("  Gyration Trim:"));
        Serial.print(F("    x-axis : +/- ")); Serial.println(SelfTest[3],1);
        Serial.print(F("    y-axis : +/- ")); Serial.println(SelfTest[4],1);
        Serial.print(F("    z-axis : +/- ")); Serial.println(SelfTest[5],1);
      }
      if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
        if (debug) {
          Serial.println(F("  PASSED"));  
          delay(1000);
          if (debug) Serial.print(F("MPU6050 IMU intializing... "));
        }
        calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
        initMPU6050(); 
        if (debug) Serial.println(F("\t\t\tOK"));
      } else {
        if (debug) {
          Serial.print(F("  Error: Could not connect to MPU6050 on 0x"));
          Serial.println(c, HEX);
        }
      }
    }
  }

  //init pir sensors
  if (pir_active) {
    pinMode(PIR_FRONT, INPUT);
    digitalWrite(PIR_FRONT, HIGH); 
   
    pinMode(PIR_LEFT, INPUT);
    digitalWrite(PIR_LEFT, HIGH); 
  
    pinMode(PIR_RIGHT, INPUT);
    digitalWrite(PIR_RIGHT, HIGH); 
  }

  //init amp power control
  if (amp_active) {
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, LOW);
  }


  if (rgb_active) {
    rgb_request((char*)"MUNUEAn");
  }

  if (oled_active) {
    oled_request((char*)"o");
  }
  if (!quick_boot) {
    if (buzz_active || mp3_active) {
      if (mp3_active) {
        mp3_play(4);
        delay(3000);
        if (!splash_active) {
          delay(2000);
        }
      }
      digitalWrite(LED_PIN, LOW);
    } else {
      for (int b = 3; b > 0; b--) {
        digitalWrite(LED_PIN, HIGH);
        delay(250);
        digitalWrite(LED_PIN, LOW);
        delay(250);
      }
    }
  }

  //(re)boot slave nano
  if (slave_active) {
    if (splash_active && !quick_boot) {
      command_slave((char*)"Z");
    } else {
      command_slave((char*)"Y");
    }
  }

  //init NRF24 wireless
  if (nrf_active) {
    if (!nrf_radio.begin()) {
      if (debug8) Serial.println(F("radio hardware is not responding!!"));
      while (1) {}
    } else {
      nrf_radio.setPALevel(RF24_PA_LOW);
      nrf_radio.setPayloadSize(sizeof(rc_data));
      nrf_radio.setChannel(124);
      nrf_radio.openReadingPipe(1, address[!radioNumber]);
      nrf_radio.enableAckPayload();
      nrf_radio.startListening();
      nrf_radio.writeAckPayload(1, &tm_data, sizeof(tm_data)); // pre-load data
      if (debug8) Serial.println(F("radio hardware is ready and listening!"));
    }
  }
  
  //init pwm controller
  if (pwm_active) {
    if (oled_active) {
      oled_request((char*)"o");
    }

    if (debug) Serial.print(F("PWM Controller intializing..."));
    pwm1.begin();
    pwm1.setOscillatorFrequency(OSCIL_FREQ);
    pwm1.setPWMFreq(SERVO_FREQ);

    delay(500);
    if (!splash_active) {
      delay(3000);
    }

    if (debug) Serial.println(F("\t\t\tOK"));
    if (debug) {
      Serial.print(TOTAL_SERVOS); Serial.print(F(" Servos intializing..."));
    }

    //set default speed factor
    spd_factor = mapfloat(spd, min_spd, max_spd, min_spd_factor, max_spd_factor);

    //initialize servos and populate related data arrays with defaults
    init_home();
    delay(1000);
    if (debug) Serial.println(F("\t\t\tOK"));
  }

  if (melody_active) {
    if (mp3_active) {
      if (!quick_boot) {
        for (int s=0;s<15;s++) {
          mp3_volume((sound_vol-s));
          delay(250);
        }
        DFPlayer.pause();
        delay(200);
        if (oled_active) {
          oled_request((char*)"y");
        }
        mp3_volume(16);
        delay(50);
        mp3_play(11);
        if (rgb_active) {
          rgb_request((char*)"Id");
        }
        delay(5000);
        if (rgb_active) {
          rgb_request((char*)"Hi");
        }
        delay(1000);
        if (oled_active) {
          oled_request((char*)"n");
        }
        delay(3000);
      }
      mp3_play(7);
    }
  } else if (buzz_active) {
    if (mp3_active) {
      mp3_play(7);
    }
  }

  if (mp3_active) {
    mp3_volume(sound_vol);
  }
  if (rgb_active) {
    rgb_request((char*)"xEi");
    delay(1000);
  }

  if (!mpu_active) {
    if (debug) {
      Serial.println(F("\nNova SM3... \t\t\t\tReady!"));
      Serial.println(F("=============================================="));
      if (!plotter && serial_active) {
        Serial.println();
        Serial.println(F("Type a command input or 'h' for help:"));
      }
    }
  }

  if (rgb_active) {
    rgb_request((char*)"xKg");
  }
  if (oled_active) {
    oled_request((char*)"l");
  }
  if (!quick_boot) {
    delay(1000);
  }
}



void loop() {
  update_servos();

/*
   -------------------------------------------------------
   Check for Moves
    :check if any scripted, sequenced, or dynamic moves are active
    :and execute accordingly along with any required variables defined
   -------------------------------------------------------
*/
  if (move_sequence) {
    run_sequence();
  } else if (move_x_axis) {
    x_axis();
  } else if (move_y_axis) {
    y_axis();
  } else if (move_pitch_body) {
    pitch_body();
  } else if (move_pitch) {
    pitch(x_dir);
  } else if (move_roll_body) {
    roll_body();
  } else if (move_roll) {
    roll();
  } else if (move_trot) {
    step_trot(x_dir, y_dir, z_dir);
  } else if (move_forward) {
    step_forward(y_dir, x_dir, z_dir);
  } else if (move_backward) {
    step_backward(y_dir, x_dir, z_dir);
  } else if (move_left) {
    ramp_dist = 0.25;
    ramp_spd = 1.5;
    use_ramp = 0;
    step_left_right(1, x_dir, y_dir);
  } else if (move_right) {
    ramp_dist = 0.25;
    ramp_spd = 1.5;
    use_ramp = 0;
    step_left_right(0, x_dir, y_dir);
  } else if (move_march) {
    step_march(x_dir, y_dir, z_dir);
  } else if (move_wake) {
    wake();
  } else if (move_wman) {
    ramp_dist = 0.2;
    ramp_spd = 0.5;
    use_ramp = 1;
    wman();
  } else if (move_funplay) {
    funplay();
  } else if (move_look_left) {
    look_left();
  } else if (move_look_right) {
    look_right();
  } else if (move_roll_x) {
    roll_x();
  } else if (move_pitch_y) {
    pitch_y();
  } else if (move_kin_x) {
    move_kx();
  } else if (move_kin_y) {
    move_ky();
  } else if (move_yaw_x) {
    yaw_x();
  } else if (move_yaw_y) {
    yaw_y();
  } else if (move_yaw) {
    yaw();
  } else if (move_servo) {
    move_debug_servo();
  } else if (move_leg) {
    move_debug_leg();
  }

/*
   -------------------------------------------------------
   Check State Machines
    :check active state machine(s) for execution time by its respective interval
   -------------------------------------------------------
*/
  if (moveDelayInterval && millis() - lastMoveDelayUpdate > moveDelayInterval) {
    delay_sequences();
  }

  if (nrf_active) {
    if (millis() - lastRemoteUpdate > remoteInterval) {
      remote_check();
    }
  }

  if (serial_active) {
    if (millis() - lastSerialUpdate > serialInterval) serial_check();
  }

  if (pir_active) {
    if (millis() - lastPIRUpdate > pirInterval) pir_check();
  }

  if (mpu_active) {
    if(millis() - lastMPUUpdate > mpuInterval) get_mpu();
  }

  if (uss_active) {
    if (millis() - lastUSSUpdate > ussInterval) uss_check();
  }

  if (amp_active) {
    if (ampInterval && millis() - lastAmpUpdate > ampInterval) amperage_check(amp_loop);
  }

  if (batt_active) {
    if(millis() - lastBatteryUpdate > batteryInterval) battery_check(0);
  }

  if (mp3_active && mp3_status) {
    if(millis() - lastMP3Update > mp3Interval) check_mp3();
  }
  if (potInterval && millis() - lastPotUpdate > potInterval) mp3_volume(-1);
}



/*
   -------------------------------------------------------
   Hardware Functions
   -------------------------------------------------------
*/
/*
   -------------------------------------------------------
   Update Servos
    :check if servo(s) need updating
    :this is the core functionality of Nova
   -------------------------------------------------------
*/
void update_servos() {
  //update coxas
  s_RFC.Update();
  s_LFC.Update();
  s_RRC.Update();
  s_LRC.Update();

  //update femurs
  s_RFF.Update();
  s_LFF.Update();
  s_RRF.Update();
  s_LRF.Update();

  //update tibias
  s_RFT.Update();
  s_LFT.Update();
  s_RRT.Update();
  s_LRT.Update();
}


/*
   -------------------------------------------------------
   NRF Check
    :provide general description and explanation here
   -------------------------------------------------------
*/
bool nrf_check() {
  bool resp = false;
  uint8_t pipe;

  if (nrf_radio.available(&pipe)) {
    uint8_t bytes = nrf_radio.getPayloadSize();
    nrf_radio.read(&rc_data, bytes);
    resp = true;

    //send data as acknowledgement
    nrf_ack();

    //set remote control vars from nrf received data
    //fire buttons
    btn1 = rc_data[0];
    btn2 = rc_data[1];
    btn3 = rc_data[2];
    btn4 = rc_data[3];

    //sel stop buttons
    sel1 = rc_data[4];
    sel2 = rc_data[5];

    //slide pots
    p1 = rc_data[6];
    p2 = rc_data[7];
    p3 = rc_data[8];
    p4 = rc_data[9];

    //joysticks
    lx = rc_data[10];
    ly = rc_data[11];
    rx = rc_data[12];
    ry = rc_data[13];

    if (debug8) {
      for (int i = 0; i < data_num; i++) {
        Serial.print(rc_data[i]);Serial.print("\t");
      }
      Serial.println(" DATA REC");
    }
  }
  lastNRFUpdate = millis();

  return resp;
}


void nrf_ack() {

    //set data
    float bv = (batt_voltage / 2) * 10;
    tm_data[0] = bv;
    tm_data[1] = spd;
    tm_data[2] = sound_vol;
    tm_data[3] = move_steps;
    tm_data[4] = (step_weight_factor_front * 100);
    tm_data[5] = (step_weight_factor_rear * 100);
    tm_data[6] = remote_select;
    tm_data[7] = start_mode;
    
    if (debug8) {
      for (int i = 0; i < data_num; i++) {
        Serial.print(tm_data[i]);Serial.print("\t");
      }
      Serial.println(" ACK SENT");
    }
    nrf_radio.writeAckPayload(1, &tm_data, sizeof(tm_data)); // load the payload for the next time
}



/*
   -------------------------------------------------------
   Remote Check
    :provide general description and explanation here - too much to comment by line
   -------------------------------------------------------
*/
void remote_check() {

  if (nrf_check()) {
    if (!move_demo && !move_funplay) {
/*
Serial.print("here1 : sel1 / sel1p ");
Serial.print(sel1);
Serial.print(" / ");
Serial.print(sel1p);
Serial.print("   sel2 / sel2p ");
Serial.print(sel2);
Serial.print(" / ");
Serial.println(sel2p);
*/      

      //start button by mode
      if (sel2 && sel2 != sel2p) {
        if (!remote_start_stop) {
          remote_start_stop = 1;
          if (debug1)
            Serial.println(F("Start Pressed"));

          if (remote_select == 1) {
            //march
            start_mode = 1;
            if (mp3_active) mp3_play(25);
            if (debug1)
              Serial.println(F("start march"));
            if (!move_march) {
              set_stop();
              y_dir = 0;
              x_dir = 0;
              z_dir = 0;
              if (mpu_is_active) mpu_active = 0;
              move_march = 1;
              if (oled_active) {
                oled_request((char*)"d");
              }
            }
          } else if (remote_select == 2) {
            //walk
            start_mode = 2;
          } else if (remote_select == 3) {
            //freestyle
            start_mode = 3;
          } else if (remote_select == 4) {
            //trot
            start_mode = 4;
            move_trot = 1;
          } else if (remote_select == 5) {
            //follow
            start_mode = 5;
            if (!move_follow) {
              if (debug1)
                Serial.println(F("move follow on"));
              spd_lock = spd;
              spd = 3;
              set_speed();
              move_follow = 1;
              if (mp3_active) {
                mp3_play(23);
                delay(1500);
              }
              if (mpu_is_active) mpu_active = 0;
              if (uss_is_active) uss_active = 0;
            }
          }
        } else {
          remote_start_stop = 0;
          start_mode = 0;
          if (debug1)
            Serial.println(F("Stop Pressed"));
          if (remote_select == 1) {
            if (mp3_active) mp3_play(25);
            if (move_march) {
              if (debug1)
                Serial.println(F("stop march"));
              move_march = 0;
              if (mpu_is_active) mpu_active = 1;
              set_stop();
              y_dir = 0;
              x_dir = 0;
              z_dir = 0;
              if (oled_active) {
                oled_request((char*)"d");
              }
            }
          } else if (remote_select == 2) {
          } else if (remote_select == 3) {
          } else if (remote_select == 4) {
            move_trot = 0;
          } else if (remote_select == 5) {
            if (move_follow) {
              if (debug1)
                Serial.println(F("move follow off"));
              move_follow = 0;
              set_stop();
              if (mp3_active) {
                mp3_play(24);
              }
              if (mpu_is_active) mpu_active = 1;
              if (uss_is_active) uss_active = 1;
            }
          }
          set_stop_active();
        }
      }
      if (sel2 != sel2p) {
        sel2p = sel2;
      }


      //select / set mode button
      if (sel1 && sel1 != sel1p) {
        (remote_select < 5) ? remote_select++ : remote_select = 1;
        if (debug1) {
          Serial.print(F("\tSelected ")); Serial.println(remote_select);
        }
        remote_start_stop = 0;
        start_mode = 0;
        set_stop_active();

        if (rgb_active) {
          if (remote_select == 1) {
            if (mp3_active) mp3_play(12);
            rgb_request((char*)"tEn");
          } else if (remote_select == 2) {
            if (mp3_active) mp3_play(13);
            rgb_request((char*)"uEn");
          } else if (remote_select == 3) {
            if (mp3_active) mp3_play(14);
            rgb_request((char*)"vEn");
          } else if (remote_select == 4) {
            if (mp3_active) mp3_play(15);
            rgb_request((char*)"wEn");
          } else if (remote_select == 5) {
            if (mp3_active) mp3_play(26);
            rgb_request((char*)"xEn");
          }
        }
        if (oled_active) {
          if (remote_select == 1) {
            oled_request((char*)"g");
          } else if (remote_select == 2) {
            oled_request((char*)"h");
          } else if (remote_select == 3) {
            oled_request((char*)"i");
          } else if (remote_select == 4) {
            oled_request((char*)"j");
          } else if (remote_select == 5) {
            oled_request((char*)"x");
          }
        }
      }
      if (sel1 != sel1p) {
        sel1p = sel1;
      }


      //joysticks
      if ((remote_select == 1 && start_mode == 1) || (remote_select == 4 && start_mode == 4)) {

        //gait joysticks for stationary marching
        y_dir = map(ry, 0, 255, y_dir_steps[1], y_dir_steps[0]);
        x_dir = map(lx, 0, 255, x_dir_steps[1], x_dir_steps[0]);
        z_dir = map(ly, 0, 255, z_dir_steps[1], z_dir_steps[0]);

      } else if (remote_select == 2 && start_mode == 2) {

        //gait joysticks for direction marching
        z_dir = map(ly, 0, 255, z_dir_steps[1], z_dir_steps[0]);

        //right y joystick
        y_dir = map(ry, 0, 255, y_dir_steps[1], y_dir_steps[0]);
        if (y_dir > 2) {
          if (!move_forward) {
            if (debug1)
              Serial.println(F("forward"));
            if (mpu_is_active) mpu_active = 0;
            move_march = 1;
            move_forward = 1;
          }
        } else if (y_dir < -2) {
          if (!move_backward) {
            if (debug1)
              Serial.println(F("backward"));
            if (mpu_is_active) mpu_active = 0;
            move_march = 1;
            move_backward = 1;
          }
        } else {
          if (move_forward) {
            move_forward = 0;
            if (debug1)
              Serial.println(F("stop forward"));
          }
          if (move_backward) {
            move_backward = 0;
            if (debug1)
              Serial.println(F("stop backward"));
          }
          if (move_march) {
            if (mpu_is_active) mpu_active = 1;
            move_march = 0;
            if (rgb_active) {
              rgb_request((char*)"OtEn");
            }
          }
          set_stop();
        }

        //left x joystick modifier
        x_dir = map(lx, 0, 255, x_dir_steps[1], x_dir_steps[0]);
        if (x_dir > 2) {
          if (!move_right) {
            move_right = 1;
            if (debug1)
              Serial.println(F("move right"));
          }
        } else if (x_dir < -2) {
          if (!move_left) {
            move_left = 1;
            if (debug1)
              Serial.println(F("move left"));
          }
        } else {
          if (move_right) {
            move_right = 0;
            if (debug1)
              Serial.println(F("stop move right"));
          }
          if (move_left) {
            move_left = 0;
            if (debug1)
              Serial.println(F("stop move left"));
          }
        }
      } else if (remote_select == 3 && start_mode == 3) {

        //kinematics joysticks
        if (btn1) {
        } else {
          move_steps_x = map(lx, 0, 255, (move_steps_max / 2), (move_steps_min / 2));
          move_roll_x = 1;
          if (debug1 && move_steps_x) {
            Serial.print(F("move roll_x "));Serial.println(move_steps_x);
          }

          move_steps_y = map(ly, 0, 255, (move_steps_max * 1.4), (move_steps_min * 1.4));
          move_pitch_y = 1;
          if (debug1 && move_steps_y) {
            Serial.print(F("move pitch_y "));Serial.println(move_steps_y);
          }
        }

        if (btn2) {
          //move yaw while button pressed/held
          move_steps_yaw = map(rx, 0, 255, (move_steps_max * 1.4), (move_steps_min * 1.4));
          if (move_steps_yaw > 2 || move_steps_yaw < -2) {
            move_yaw = 1;
            if (debug1)
              Serial.println(F("move yaw"));
          } else {
            move_yaw = 0;
          }

          //move in y while button pressed/held
          move_steps_ky = map(ry, 0, 255, (move_steps_min * 1.4), (move_steps_max * 1.4));
          if (move_steps_ky > 2 || move_steps_ky < -2) {
            move_kin_y = 1;
            if (debug1)
              Serial.println(F("move kin_y"));
          } else {
            move_kin_y = 0;
          }
        } else {
          move_steps_yaw_x = map(rx, 0, 255, (move_steps_max * .5), (move_steps_min * .5));
          if (move_steps_yaw_x > 2 || move_steps_yaw_x < -2) {
            move_yaw_x = 1;
            if (debug1)
              Serial.println(F("move yaw_x"));
          } else {
            move_yaw_x = 0;
          }

          move_steps_yaw_y = map(ry, 0, 255, move_steps_max, move_steps_min);
          if (move_steps_yaw_y > 2 || move_steps_yaw_y < -2) {
            move_yaw_y = 1;
            if (debug1)
              Serial.println(F("move yaw_y"));
          } else {
            move_yaw_y = 0;
          }
        }  
      } else if (remote_select == 5 && start_mode == 5) {
//DEV: unused
      }

      //check fire buttons
      int fired1 = 0;
      if ((btn1 && btn1 != btn1p) && (btn2 && btn2 != btn2p)) {      //press both btn1 and btn2        
          fired1 = 1;
          btn1p = btn1;
          btn2p = btn2;
          if (remote_select == 1 || remote_select == 2 || remote_select == 3 || remote_select == 4 || remote_select == 5) {
            set_stop();
            if (debug1)
              Serial.println(F("stay"));
            if (rgb_active) {
              rgb_request((char*)"Hg");
            }
            set_stay();
            if (oled_active) {
              oled_request((char*)"b");
            }
          }
      }

      int fired2 = 0;
      if ((btn3 && btn3 != btn3p) && (btn4 && btn4 != btn4p)) {      //press both btn3 and btn4        
          fired2 = 1;
          btn3p = btn3;
          btn4p = btn4;
          if (remote_select == 1 || remote_select == 2 || remote_select == 3 || remote_select == 4 || remote_select == 5) {
            set_stop();
            if (rgb_active) {
              rgb_request((char*)"Hg");
            }
            if (debug1)
              Serial.println(F("wake"));
            spd_lock = spd;
            spd = 1;
            set_speed();
            move_loops = 3;
            move_switch = 2;
            move_wake = 1;
          }
      }

      if (!fired1 && !fired2) {
        //btn1
        if (btn1 != btn1p) {                //btn1 pressed
          btn1p = btn1;
          if (btn1) {
            set_stop();
            if (remote_select == 1) {
              ramp_dist = 0.33;
              ramp_spd = 1.5;
              use_ramp = 1;  
            } else if (remote_select == 2) {
              if (!move_roll) {
                move_roll = 1;
                if (debug1)
                  Serial.println(F("move roll"));
              }
            } else if (remote_select == 3) {
//DEV: unused
            } else if (remote_select == 4) {
              set_sit();
              if (rgb_active) 
                rgb_request((char*)"Hi");
              if (debug1)
                Serial.println(F("sit"));
            } else if (remote_select == 5) {
//DEV: unused
            }
          } else {                          //btn1 released
            if (remote_select == 1) {
              use_ramp = 0;
            } else if (remote_select == 2) {
              if (move_roll) {
                move_roll = 0;
                if (debug1)
                  Serial.println(F("stop roll"));
              }
            } else if (remote_select == 3) {
//DEV: unused
            } else if (remote_select == 4) {
//DEV: unused
            } else if (remote_select == 5) {
//DEV: unused
            }
          }
        }  
  
  
        //btn2
        if (btn2 != btn2p) {                //btn2 pressed
          btn2p = btn2;
          if (btn2) {
            set_stop();
            if (remote_select == 1) {
            } else if (remote_select == 2) {
              if (!move_pitch) {
                move_pitch = 1;
                if (debug1)
                  Serial.println(F("move pitch"));
              }
            } else if (remote_select == 3) {
//DEV: unused
            } else if (remote_select == 4) {
              set_kneel();
              if (debug1)
                Serial.println(F("kneel"));
            } else if (remote_select == 5) {
//DEV: unused
            }
          } else {                          //btn2 released
            if (remote_select == 1) {
            } else if (remote_select == 2) {
              if (move_pitch) {
                move_pitch = 0;
                if (debug1)
                  Serial.println(F("stop pitch"));
              }
            } else if (remote_select == 3) {
//DEV: unused
            } else if (remote_select == 4) {
//DEV: unused
            } else if (remote_select == 5) {
//DEV: unused
            }
          }
        }  
  
        //btn3
        if (btn3 != btn3p) {                //btn3 pressed
          btn3p = btn3;
          if (btn3) {
            set_stop();
            if (remote_select == 1) {
            } else if (remote_select == 2) {
              if (!move_roll_body) {
                move_roll_body = 1;
                if (debug1)
                  Serial.println(F("move roll body"));
              }
            } else if (remote_select == 3) {
//DEV: unused
            } else if (remote_select == 4) {
              set_crouch();
              if (rgb_active)
                rgb_request((char*)"Hh");
              if (debug1)
                Serial.println(F("crouch"));
            } else if (remote_select == 5) {
//DEV: unused
            }
          } else {                          //btn3 released
            if (remote_select == 1) {
            } else if (remote_select == 2) {
              if (move_roll_body) {
                move_roll_body = 0;
                if (debug1)
                  Serial.println(F("stop roll body"));
              }
            } else if (remote_select == 3) {
//DEV: unused
            } else if (remote_select == 4) {
//DEV: unused
            } else if (remote_select == 5) {
//DEV: unused
            }
          }
        }  
  
        //btn4
        if (btn4 != btn4p) {                //btn4 pressed
          btn4p = btn4;
          if (btn4) {
            set_stop();
            if (remote_select == 1) {
            } else if (remote_select == 2) {
              if (!move_pitch_body) {
                move_pitch_body = 1;
                if (debug1)
                  Serial.println(F("move pitch body"));
              }
            } else if (remote_select == 3) {
//DEV: unused
            } else if (remote_select == 4) {
              set_lay();
              if (rgb_active)
                rgb_request((char*)"Kd");
              if (debug1)
                Serial.println(F("lay"));
            } else if (remote_select == 5) {
//DEV: unused
            }
          } else {                          //btn4 released
            if (remote_select == 1) {
            } else if (remote_select == 2) {
              if (move_pitch_body) {
                move_pitch_body = 0;
                if (debug1)
                  Serial.println(F("stop pitch body"));
              }
            } else if (remote_select == 3) {
//DEV: unused
            } else if (remote_select == 4) {
//DEV: unused
            } else if (remote_select == 5) {
//DEV: unused
            }
          }
        }  
      }  


/*  
//DEV: these are really just test / demo moves, may not be worth keeping

        //left y joystick
        int my = map(ly, 0, 255, 0, 255);
        if (my > 200) {
          if (!move_trot) {
            set_stop();
            step_lock = 1;
            move_trot = 1;
            if (debug1)
              Serial.println(F("move trot"));
            x_dir = map(rx, 0, 255, move_steps_min / 4, move_steps_max / 4);
            move_steps = map(ry, 0, 255, move_steps_max / 2, move_steps_min / 2);
            if (debug2)
              Serial.print(F("x dir: ")); Serial.print(x_dir);
          }
        } else {
          set_stop();
          if (move_trot) {
            move_trot = 0;
          }
        }

      //SHAPE BUTTONS
      if (ps2x.ButtonPressed(PSB_TRIANGLE)) {
        if (remote_select == 1) {
          set_stop();
          if (!move_y_axis) {
            move_y_axis = 1;
            if (debug1)
              Serial.println(F("move y_axis"));
          }
        } else if (remote_select == 2) {
          set_stop();
          if (!move_wman) {
            move_wman = 1;
            if (debug1)
              Serial.println(F("move wman"));
          }
        }
      } else if (ps2x.ButtonReleased(PSB_TRIANGLE)) {
        if (remote_select == 1) {
          if (move_y_axis) {
            move_y_axis = 0;
            if (debug1)
              Serial.println(F("stop y_axis"));
          }
        } else if (remote_select == 2) {
          if (move_wman) {
            move_wman = 0;
            if (debug1)
              Serial.println(F("stop wman"));
          }
        }
      }

  
      if (ps2x.ButtonPressed(PSB_CROSS)) {
        if (remote_select == 1) {
          set_stop();
          if (!move_x_axis) {
            move_x_axis = 1;
            if (debug1)
              Serial.println(F("move x_axis"));
          }
        }
      } else if (ps2x.ButtonReleased(PSB_CROSS)) {
        if (remote_select == 1) {
          if (move_x_axis) {
            move_x_axis = 0;
            if (debug1)
              Serial.println(F("stop x_axis"));
          }
        }
      }
*/  


      //set step_weight_factor_front from pot 1
      if (p1 != p1p && (p1 > (p1p + pot_threshold) || p1 < (p1p - pot_threshold))) {
        p1p = p1;
        if (remote_select == 1 || remote_select == 2 || remote_select == 3 || remote_select == 4 || remote_select == 5) {
          if (debug1) 
            Serial.print(F("set swff : "));
          step_weight_factor_front = mapfloat(p1, 0, 255, 1.0, 1.8);
          if (debug1)
            Serial.println(step_weight_factor_front);
        }
      }

      //set speed from pot 2
      if (!spd_lock) {
        if (p2 != p2p && (p2 > (p2p + pot_threshold) || p2 < (p2p - pot_threshold))) {
          p2p = p2;
          if (remote_select == 1 || remote_select == 2 || remote_select == 3 || remote_select == 4 || remote_select == 5) {
            if (debug1) 
              Serial.print(F("set speed : "));
            spd = map(p2, pot_min, pot_max, (min_spd * 100), (max_spd * 100)) / 100;
            if (!spd) spd = 1;
            set_speed();
            if (debug1)
              Serial.println(spd);
          }
        }
      }

      //set step_weight_factor_rear from pot 3
      if (p3 != p3p && (p3 > (p3p + pot_threshold) || p3 < (p3p - pot_threshold))) {
        p3p = p3;
        if (remote_select == 1 || remote_select == 2 || remote_select == 3 || remote_select == 4 || remote_select == 5) {
          if (debug1) 
            Serial.print(F("set swfr : "));
          step_weight_factor_rear = mapfloat(p1, 0, 255, 1.0, 1.8);
          if (debug1)
            Serial.println(step_weight_factor_rear);
        }
      }

      //set steps from pot 4
      if (!step_lock) {
        if (p4 != p4p && (p4 > (p4p + (pot_threshold * 3)) || p4 < (p4p - (pot_threshold * 3)))) {
          p4p = p4;
        if (remote_select == 1 || remote_select == 2 || remote_select == 4 || remote_select == 5) {
            move_steps = map(p4, pot_min, pot_max, (z_dir_steps[0] * 2), (z_dir_steps[1] * 2));
            if (debug1) {
              Serial.print(F("move steps: ")); Serial.println(move_steps);
            }
          } else if (remote_select == 3) {
            z_dir = mapfloat(p4, pot_min, pot_max, z_dir_steps[1], z_dir_steps[0]);
            if (debug1) {
              Serial.print(F("z_dir: ")); Serial.println(z_dir);
            }
          }
        }
      }
    }
  }

  lastRemoteUpdate = millis();
}



/*
   -------------------------------------------------------
   PIR Check
    :provide general description and explanation here
   -------------------------------------------------------
*/
void pir_check() {
  int mactive = mpu_active;
  int uactive = uss_active;

    if (move_follow) {
      follow();
    } else if (!pir_halt) {
      pir_val = digitalRead(PIR_FRONT);
      pir_wait--;
      if (pir_val == HIGH) {
        if (pir_state == LOW) {
          if (pir_wait < 1) {
            if (rgb_active) {
              rgb_request((char*)"Fh");
            }
            if (debug1)
              Serial.println(F("Motion detected!"));
    
            //disable mpu and uss sensors while in alert
            if (mpu_active) mpu_active = 0;
            if (uss_active) uss_active = 0;
    
            set_stop_active();
            pir_state = HIGH;
            for (int l = 0; l < TOTAL_LEGS; l++) {
              servoSequence[l] = 0;
            }
            for (int m = 0; m < TOTAL_SERVOS; m++) {
              if (is_left_leg(m)) {
                if (!is_front_leg(m) && (is_femur(m) || is_tibia(m))) {
                  servoStepMoves[m][0] = limit_target(m, (servoHome[m] - 30), 0);
                } else if (is_front_leg(m) && (is_femur(m) || is_tibia(m))) {
                  servoStepMoves[m][0] = limit_target(m, (servoHome[m] - 60), 0);
                } else if (is_front_leg(m)) {
                  servoStepMoves[m][0] = limit_target(m, (servoHome[m] - 20), 0);
                  servoStepMoves[m][1] = limit_target(m, (servoHome[m] + 35), 0);
                } else {
                  servoStepMoves[m][1] = limit_target(m, (servoHome[m] + 35), 0);
                }
              } else {
                if (!is_front_leg(m) && (is_femur(m) || is_tibia(m))) {
                  servoStepMoves[m][0] = limit_target(m, (servoHome[m] + 30), 0);
                } else if (is_front_leg(m) && (is_femur(m) || is_tibia(m))) {
                  servoStepMoves[m][0] = limit_target(m, (servoHome[m] + 60), 0);
                } else if (is_front_leg(m)) {
                  servoStepMoves[m][0] = limit_target(m, (servoHome[m] + 20), 0);
                  servoStepMoves[m][1] = limit_target(m, (servoHome[m] + 35), 0);
                } else {
                  servoStepMoves[m][1] = limit_target(m, (servoHome[m] + 35), 0);
                }
              }
              servoStepMoves[m][2] = servoHome[m];
            }
            spd_c = 1;
            spd_f = 1;
            spd_t = 1;
            move_loops = 0;
            move_delay = 300;
            move_sequence = 1;
    
            if (oled_active) {
              oled_request((char*)"e");
            } else {
              if (debug1)
                Serial.println(F("INTRUDER ALERT!"));
            }

//DEV: add mp3_active and play

            if (rgb_active) {
              rgb_request((char*)"Fj");
            }
          }
        }
      } else {
        if (pir_state == HIGH) {
          Serial.println(F("Motion ended!"));
          pir_state = LOW;
          pir_wait = 100; //state machine cycles, not ms
          if (oled_active) {
            oled_request((char*)"f");
          } else {
            if (debug1)
              Serial.println(F("ALERT COMPLETE!"));
          }
    
          pwm_oe = 0;
          pir_reset = 1;
        } else if (pir_reset) {
          if (!move_sequence) {
            pir_reset = 0;
            move_delays[0] = 3000;
            move_delay_sequences[0] = 7;
            move_delays[1] = 3000;
            move_delay_sequences[1] = 7;
            delay_sequences();
    
            if (rgb_active) {
              rgb_request((char*)"Jf");
            }
          }
          if (oled_active) {
            oled_request((char*)"f");
          }
    
          //re-enable mpu and uss sensors if enabled prior to alert
          mpu_active = mactive;
          uss_active = uactive;
        }
      }
    }

    lastPIRUpdate = millis();
}


void follow() {
  byte reset_scnt = 0;
  if (sound_cur == 0) {
    sound_cur = 25;
  }

  if (pir_wait) {
    pir_wait--;
  } else {
    int val_front = pir_frontState = digitalRead(PIR_FRONT);
    if (val_front == HIGH) {
      if (pir_frontState == LOW) {
        pir_frontState = HIGH;
      }
    } else {
      if (pir_frontState == HIGH) {
        pir_frontState = LOW;
      }
    }
  
    int val_left = pir_leftState = digitalRead(PIR_LEFT);
    if (val_left == HIGH) {            
      if (pir_leftState == LOW) {
        pir_leftState = HIGH;
      }
    } else {
      if (pir_leftState == HIGH) {
        pir_leftState = LOW;
      }
    }
  
    int val_right = pir_rightState = digitalRead(PIR_RIGHT);
    if (val_right == HIGH) {            
      if (pir_rightState == LOW) {
        pir_rightState = HIGH;
      }
    } else {
      if (pir_rightState == HIGH) {
        pir_rightState = LOW;
      }
    }

/*
    if (debug1) {
      Serial.print("F: ");Serial.print(pir_frontState);
      Serial.print("L: ");Serial.print(pir_leftState);
      Serial.print("R: ");Serial.println(pir_rightState);
    }
*/

    if (pir_frontState || pir_leftState || pir_rightState) {
      pir_wait = pirDelay;
      y_dir = 0;
      x_dir = 0;
      z_dir = 0;
    }

    if (move_wake) {
      set_stop();
      move_wake = 0;
    } else {
      if (!move_march && (pir_frontState || pir_leftState || pir_rightState) && !(pir_frontState && pir_leftState && pir_rightState)) {
        if (debug1)
          Serial.println("restart!");
//        move_steps = 25;
        if (mpu_is_active) mpu_active = 0;
        move_march = 1;

        pir_frontState = LOW;
        pir_leftState = LOW;
        pir_rightState = LOW;
      }  
  
      if (pir_frontState && !pir_leftState && !pir_rightState) {
        follow_dir = 1;
        if (debug1)
          Serial.println("go forward");
        y_dir = 15;
        if (sound_cur != 10) {
          sound_cur = 10;
          reset_scnt = 1;
        }
      } else if (pir_frontState && pir_leftState && !pir_rightState) {
        follow_dir = 2;
        if (debug1)
          Serial.println("go forward-left");
        y_dir = 10;
        x_dir = 10;
        if (sound_cur != 6) {
          sound_cur = 6;
          reset_scnt = 1;
        }
      } else if (pir_frontState && !pir_leftState && pir_rightState) {
        follow_dir = 3;
        if (debug1)
          Serial.println("go forward-right");
        y_dir = 10;
        x_dir = -10;
        if (sound_cur != 6) {
          sound_cur = 6;
          reset_scnt = 1;
        }
      } else if (!pir_frontState && pir_leftState && !pir_rightState) {
        follow_dir = 4;
        if (debug1)
          Serial.println("go left");
        y_dir = 5;
        x_dir = -15;
        if (sound_cur != 3) {
          sound_cur = 3;
          reset_scnt = 1;
        }
      } else if (!pir_frontState && !pir_leftState && pir_rightState) {
        follow_dir = 5;
        if (debug1)
          Serial.println("go right");
        y_dir = 5;
        x_dir = 15;
        if (sound_cur != 3) {
          sound_cur = 3;
          reset_scnt = 1;
        }
      } else if (!pir_frontState && pir_leftState && pir_rightState) {
        follow_dir = 6;
        if (debug1)
          Serial.println("go back");
        y_dir = -15;
        x_dir = 0;
        if (sound_cur != 2) {
          sound_cur = 2;
          reset_scnt = 1;
        }
      } else if (pir_frontState && pir_leftState && pir_rightState) {
        follow_dir = 7;
        if (debug1)
          Serial.println("greet");
        move_loops = 2;
        move_switch = 2;
        move_wake = 1;
        move_march = 0;
      } else {
        follow_dir = 0;
        if (debug1)
          Serial.println("stop!");
        pir_repeat_cnt++;
        if (pir_repeat_cnt == 6) {
          set_stop();
          move_march = 0;
          pir_repeat_cnt = 0;
        }
        y_dir = 0;
        x_dir = 0;
        z_dir = 0;
        if (sound_cur != 25) {
          sound_cur = 25;
          reset_scnt = 1;
        }
      }

      if (oled_active) {
        switch (follow_dir) {
          case 0: oled_request((char*)"w"); break;
          case 1: oled_request((char*)"p"); break;
          case 2: oled_request((char*)"q"); break;
          case 3: oled_request((char*)"r"); break;
          case 4: oled_request((char*)"s"); break;
          case 5: oled_request((char*)"t"); break;
          case 6: oled_request((char*)"u"); break;
          case 7: oled_request((char*)"v"); break;
        }
      }
      if (mp3_active) {
        if (!sound_cnt || reset_scnt) {
          if (!pir_frontState && !pir_leftState && !pir_rightState) {
            mp3_play(6);
          }
          reset_scnt = 0;
          sound_cnt = sound_cur;
        } else {
          sound_cnt--;
        }
      }

      follow_dir_prev = follow_dir;
    }
  }

}




/*
   -------------------------------------------------------
   Check Ultrasonic Reading
    :provide general description and explanation here
   -------------------------------------------------------
*/
void uss_check() {
  //check right sensor
  int dist_rt = command_slave((char*)"b");
  if (dist_rt != prev_distance_r && ((dist_rt > (prev_distance_r + distance_tolerance)) || (dist_rt < (prev_distance_r - distance_tolerance)))) {
    distance_r = prev_distance_r = dist_rt;
  }

  //check left sensor
  int dist_lt = command_slave((char*)"a");
  if (dist_lt != prev_distance_l && ((dist_lt > (prev_distance_l + distance_tolerance)) || (dist_lt < (prev_distance_l - distance_tolerance)))) {
    distance_l = prev_distance_l = dist_lt;
  }

  if (oled_active) {
    oled_request((char*)"c");
  }

    if (debug7) {
      Serial.print(F("USS LEFT: "));Serial.print(distance_l);
      Serial.print(F("USS RIGHT: "));Serial.println(distance_r);
    }



/*
  if (dist_rt && dist_lt && (dist_rt < distance_alarm || dist_lt < distance_alarm)) {
    distance_alarm_set++;

    if (debug7 && distance_alarm_set > 0) {
      Serial.print(F("USS ALARM #"));Serial.println(distance_alarm_set);
    }
    Serial.print(F("USS ALARM #"));Serial.println(distance_alarm_set);

    if (oled_active && distance_alarm_set < 3) {
      oled_request((char*)"c");
    } else {
      set_stop_active();
      if (oled_active) {
        oled_request((char*)"e");
      }
      rgb_request((char*)"MQNOFzn");
      delay(3000);
      set_stop();
      distance_alarm_set = 0;
    }
    
    if (debug7) {
      if (plotter) {
        Serial.print(F("Lt:"));
        if (distance_l > (distance_alarm*5)) {
          Serial.print((distance_alarm*5));
        } else {
          Serial.print(distance_l);
        }
        Serial.print(F("\t"));
        Serial.print(F("Rt:"));
        if (distance_r > (distance_alarm*5)) {
          Serial.print((distance_alarm*5));
        } else {
          Serial.print(distance_r);
        }
        Serial.print(F("\t"));
        Serial.println();
      } else {
        Serial.print(F("Dist Right: "));
        Serial.println(distance_r);
        Serial.print(F("Dist Left: "));
        Serial.println(distance_l);
      }
    }
  } else {
    distance_alarm_set = 0;
  }
*/

  lastUSSUpdate = millis();
}


/*
   -------------------------------------------------------
   Get MPU Data
    :provide general description and explanation here
   -------------------------------------------------------
*/
void get_mpu() {

  if(readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {  // check if data ready interrupt
    readAccelData(accelCount);  // Read the x/y/z adc values
    getAres();
    
    // calculate the accleration value into actual g's
//    ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
//    ay = (float)accelCount[1]*aRes - accelBias[1];   
//    az = (float)accelCount[2]*aRes - accelBias[2];  
    ax = (float)accelCount[0] - SelfTest[0];
    ay = (float)accelCount[1] - SelfTest[1];   
    az = (float)accelCount[2] - SelfTest[2];  


    // Calculating Roll and Pitch from the accelerometer data
    accAngleX = (atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * 180 / PI);// - SelfTest[0]; // SelfTest[0] ~(0.58) See the calculate_IMU_error()custom function for more details
    accAngleY = (atan(-1 * ax / sqrt(pow(ay, 2) + pow(az, 2))) * 180 / PI);// - SelfTest[1]; // SelfTest[1] ~(-1.58)
   
    readGyroData(gyroCount);  // Read the x/y/z adc values
    getGres();
 
    // calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes - gyroBias[1];  
    gz = (float)gyroCount[2]*gRes - gyroBias[2];   
  
    // Correct the outputs with the calculated error values
//    gx = gx + abs(SelfTest[3]); // SelfTest[3] ~(-0.56)
//    gy = gy + abs(SelfTest[4]); // SelfTest[4] ~(2)
//    gz = gz + abs(SelfTest[5]); // SelfTest[5] ~ (-0.8)

    // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by seconds (s) to get the angle in degrees
    previousTime = currentTime;        // Previous time is stored before the actual time read
    currentTime = millis();            // Current time actual time read
    elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

    gyroAngleX = gyroAngleX + gx * (elapsedTime/2); // deg/s * s = deg
    gyroAngleY = gyroAngleY + gy * (elapsedTime/2);
    myaw = (myaw + gz * (elapsedTime/2));

    // Complementary filter - combine acceleromter and gyro angle values
    mroll = (0.97 * gyroAngleX + 0.03 * accAngleX);
    mpitch = (0.97 * gyroAngleY + 0.03 * accAngleY);
  }  

  if (!plotter && debug5) {
//    Serial.print("mpu x / y / z:\t\t"); Serial.print(mroll); Serial.print("\t/\t"); Serial.print(mpitch); Serial.print("\t/\t"); Serial.println(myaw);
    Serial.print("mpu x / y:\t\t"); Serial.print(mroll); Serial.print("\t/\t"); Serial.println(mpitch);
  } else if (plotter && debug5) {
//    Serial.print("x:"); Serial.print(mroll); Serial.print("\ty:"); Serial.print(mpitch); Serial.print("\tz:"); Serial.println(myaw);
    Serial.print("roll:"); Serial.print(mroll); Serial.print("\tpitch:"); Serial.println(mpitch);
  }
    
  //on init mpu, save offsets as defaults for resetting MPU position
  if (mpuInterval != mpuInterval_prev){
    mpuInterval = mpuInterval_prev;
    mpu_mroll = mroll;
    mpu_mpitch = mpitch;
    mpu_myaw = myaw;

    //delay before starting set_axis first time
    delay(1000);
    if (!plotter && debug) {
      Serial.println(F("\nNova SM3... \t\t\t\tReady!"));
      Serial.println(F("=============================================="));
    
      delay(500);
      if (!plotter && serial_active) {
        Serial.println();
        Serial.println(F("Type a command input or 'h' for help:"));
      }
    }
  }

  if (!plotter) {
    set_axis(mroll, mpitch);
  }

  lastMPUUpdate = millis();
}


/*
   -------------------------------------------------------
   Check Amperage Level
    :provide general description and explanation here
   -------------------------------------------------------
*/
void amperage_check(int aloop) {
  float AcsValue=0.0,Samples=0.0,AvgAcs=0.0,AcsValueF=0.0;

  for (int x = 0; x < 150; x++){ //Get 150 samples
    AcsValue = analogRead(AMP_PIN);     //Read current sensor values   
    Samples = Samples + AcsValue;  //Add samples together
//state machine this if possible... adds blocking delay to code, all be it miniscule!!
    delay(3); // let ADC settle before next sample 3ms
  }
  AvgAcs=Samples/150.0;//Taking Average of Samples
  AcsValueF = (2.5 - (AvgAcs * (5.0 / 1024.0)) )/0.177;
  
  AcsValueF = abs(AcsValueF)*1.8;
  if (debug4) {
    Serial.print(AvgAcs);
    Serial.print("/");
    Serial.print(AcsValueF);
    Serial.print("/");
    Serial.print(amp_limit);
  }
  if (AcsValueF > amp_limit) {
    ampInterval = 10;
    if (amp_cnt < amp_thresh) {
      amp_cnt++;
      rgb_request((char*)"u");
      if (debug4) {
        Serial.print("\tlimit met, counting... ");Serial.println(amp_cnt);
      }
    } else {
      rgb_request((char*)"A");
      detach_all();
      if (debug4) {
        Serial.println(F("\tthresh met, shutdown pwm!"));
      }
    }

    if (rgb_active) {
      rgb_request((char*)"MONRDn");
    }
  } else if (AcsValueF > amp_limit/2) {
    if (amp_cnt < amp_thresh) {
      rgb_request((char*)"u");
      amp_cnt++;
    } else {
      rgb_request((char*)"A");
      detach_all();
      if (debug4) {
        Serial.println(F("\tsoft thresh met, shutdown!"));
      }
    }
    ampInterval = 50;
    amp_warning = 2;

    if (rgb_active) {
      rgb_request((char*)"MTNRDn");
    }
    if (debug4) {
      Serial.print("\twarning 2 set - cnt: ");Serial.println(amp_cnt);
    }
  } else if (AcsValueF > amp_limit/3) {
    if (amp_cnt) {
      amp_cnt--;
    }
    rgb_request((char*)"t");
    ampInterval = 300;
    amp_warning = 1;

    if (rgb_active) {
      rgb_request((char*)"MPNRDn");
    }
    if (debug4) {
      Serial.print("\twarning 1 set - cnt: ");Serial.println(amp_cnt);
    }
  } else {
    if (amp_cnt) {
      amp_cnt--;
    }
    ampInterval = 1000;
    if (debug4) {
      Serial.print("\tcnt: ");Serial.println(amp_cnt);
    }
  }

  if (!aloop) {
    ampInterval = 0;
  }
   lastAmpUpdate = millis();
}

/*
   -------------------------------------------------------
   Check Battery Level
    :provide general description and explanation here
   -------------------------------------------------------
*/
void battery_check(byte bshow) {
  int syshalt = 0;
  int batt_danger = 0;
  int sensorValue = analogRead(BATT_MONITOR);
  batt_voltage = sensorValue * (3.3 / 1023.00) * 5.7;     // Convert the reading values from 3.3v to 12V

  if (batt_voltage && ((batt_voltage <= (batt_voltage_prev - .05)) || (batt_voltage >= (batt_voltage_prev + .05)) || bshow)) {
    if (batt_cnt == 3 || bshow) {
      if (batt_cnt > 1) {
        batt_voltage = avg_volts / batt_cnt;
      }
      for (int i = 0; i < 9; i++) {
        if (batt_voltage <= batt_levels[i]) {
          batt_danger++;
        } else if ((batt_voltage >= (batt_levels[i] - 0.02)) && (batt_voltage <= (batt_levels[i] + 0.02))) {
          batt_danger = i;
        }
      }

      if (debug4 || debug1) {
        Serial.print(batt_voltage); Serial.print(" volts - BATTERY LEVEL #"); Serial.println(batt_danger);
      }

      if (oled_active) {
        switch ((batt_danger)) {
          case 0: oled_request((char*)"0n"); if (mp3_active) mp3_play(18); break;
          case 1: oled_request((char*)"1n"); break;
          case 2: oled_request((char*)"2n"); if (mp3_active) mp3_play(18); break;
          case 3: oled_request((char*)"3n"); break;
          case 4: oled_request((char*)"4n"); if (mp3_active) mp3_play(19); break;
          case 5: oled_request((char*)"5n"); break;
          case 6: oled_request((char*)"6n"); if (mp3_active) mp3_play(20); break;
          case 7: oled_request((char*)"7n"); break;
          case 8: oled_request((char*)"8n"); if (mp3_active) mp3_play(21); break;
          case 9: oled_request((char*)"9n"); syshalt = 1; break;
          default: oled_request((char*)"9n"); syshalt = 1; break;
        }
      }
      batt_voltage_prev = batt_voltage;
      batt_cnt = 0;
      avg_volts = 0; 
       
      if (syshalt) {
        if (debug4 || debug1) {
          Serial.print(batt_voltage); Serial.println(F(" volts - SYSTEM HALTED!"));
        }
        powering_down();
        while(1) {           //simulate system halt
          if (oled_active) {
            oled_request((char*)"9n");
            delay(3000);
            oled_request((char*)"k");
            delay(3000);
          }
        }
      }
    } else {
      batt_cnt++;
      avg_volts += batt_voltage;
      if (debug4) {
        Serial.print("batt change #"); Serial.print(batt_cnt); 
        Serial.print(": sensor / volts "); Serial.print(sensorValue); Serial.print(F(" / ")); Serial.println(batt_voltage);
      }
    }
  }

  lastBatteryUpdate = millis();
}



/*
   -------------------------------------------------------
   Operational Functions
   -------------------------------------------------------
*/
void init_home() {
  for (int i = 0; i < TOTAL_SERVOS; i++) {
    for (int j = 0; j < 6; j++) {
      servoSweep[i][j] = 0;
    }
    for (int j = 0; j < 8; j++) {
      servoRamp[i][j] = 0;
    }

    servoDelay[i][0] = 0;
    servoDelay[i][1] = 0;
    servoStep[i] = 0;
    servoSwitch[i] = 0;
    servoSpeed[i] = (spd * 1.5);
    activeSweep[i] = 0;
  }

  for (int i = 0; i < TOTAL_LEGS; i++) {
    servoSequence[i] = 0;
  }

  //set crouched positions
  for (int i = 0; i < TOTAL_SERVOS; i++) {
    if (is_tibia(i)) {
      if (is_left_leg(i)) {
        servoPos[i] = (servoLimit[i][1] + 40);
      } else {
        servoPos[i] = (servoLimit[i][1] - 40);
      }
    } else if (is_femur(i)) {
      if (is_left_leg(i)) {
        servoPos[i] = (servoLimit[i][0] - 40);
      } else {
        servoPos[i] = (servoLimit[i][0] + 40);
      }
    } else {
      servoPos[i] = servoHome[i];
    }
  }

  //intitate servos in groups
  //coaxes
  pwm1.setPWM(servoSetup[RFC][1], 0, servoPos[RFC]);
  pwm1.setPWM(servoSetup[LRC][1], 0, servoPos[LRC]);
  pwm1.setPWM(servoSetup[RRC][1], 0, servoPos[RRC]);
  pwm1.setPWM(servoSetup[LFC][1], 0, servoPos[LFC]);
  delay(1000);

  //tibias
  pwm1.setPWM(servoSetup[RFT][1], 0, servoPos[RFT]);
  pwm1.setPWM(servoSetup[LRT][1], 0, servoPos[LRT]);
  pwm1.setPWM(servoSetup[RRT][1], 0, servoPos[RRT]);
  pwm1.setPWM(servoSetup[LFT][1], 0, servoPos[LFT]);
  delay(1000);

  //femurs
  pwm1.setPWM(servoSetup[RFF][1], 0, servoPos[RFF]);
  pwm1.setPWM(servoSetup[LRF][1], 0, servoPos[LRF]);
  pwm1.setPWM(servoSetup[RRF][1], 0, servoPos[RRF]);
  pwm1.setPWM(servoSetup[LFF][1], 0, servoPos[LFF]);
  delay(1000);

  set_stay();
}


void detach_all() {
  if (debug4) {
    Serial.println(F("detaching all servos!"));
  }
  for (int i = 0; i < TOTAL_SERVOS; i++) {
    activeServo[i] = 0;
    activeSweep[i] = 0;
    pwm1.setPWM(i, 0, 0);
  }
  digitalWrite(OE_PIN, HIGH);
  digitalWrite(PWR_PIN, LOW);
  pwm_active = 0;
  amp_active = 0;

  if (rgb_active) {
    rgb_request((char*)"MONRAGn");
  }
}

void set_ramp(int servo, float sp, float r1_spd, float r1_dist, float r2_spd, float r2_dist) {
  servoRamp[servo][0] = sp;  //speed
  servoRamp[servo][1] = abs(servoPos[servo] - targetPos[servo]);  //set distance

  if (!r1_spd) r1_spd = sp + (sp * ramp_spd); 
  if (!r2_spd) r2_spd = sp + (sp * ramp_spd); 
  if (!r1_dist) r1_dist = (servoRamp[servo][1] * ramp_dist); 
  if (!r2_dist) r2_dist = (servoRamp[servo][1] * ramp_dist); 

  servoRamp[servo][2] = r1_spd;  //ramp up spd
  servoRamp[servo][3] = r1_dist;  //ramp up dist
  servoRamp[servo][4] = (servoRamp[servo][2]-sp);  //ramp up spd inc
  if (r1_dist != 0) {
    servoRamp[servo][4] = (servoRamp[servo][2]-sp)/r1_dist;  //ramp up spd inc
  }
  servoRamp[servo][5] = r2_spd;  //ramp down spd
  servoRamp[servo][6] = r2_dist;  //ramp down dist
  servoRamp[servo][7] = (servoRamp[servo][5]-sp);  //ramp down spd inc
  if (r2_dist != 0) {
    servoRamp[servo][7] = (servoRamp[servo][5]-sp)/r2_dist;  //ramp down spd inc
  }

  if (debug3 && servo == debug_servo) {
    Serial.print("set_ramp: sPos: "); Serial.print(servoPos[servo]);
    Serial.print("\ttPos: "); Serial.print(targetPos[servo]);
    Serial.print("\tr1_dist: "); Serial.print(r1_dist);
    Serial.print("\tr2_dist: "); Serial.println(r2_dist);
    Serial.print("ramp:"); 
    Serial.print("\t"); Serial.print(servoRamp[servo][0]);
    Serial.print("\t"); Serial.print(servoRamp[servo][1]);
    Serial.print("\t"); Serial.print(servoRamp[servo][2]);
    Serial.print("\t"); Serial.print(servoRamp[servo][3]);
    Serial.print("\t"); Serial.print(servoRamp[servo][4]);
    Serial.print("\t"); Serial.print(servoRamp[servo][5]);
    Serial.print("\t"); Serial.print(servoRamp[servo][6]);
    Serial.print("\t"); Serial.println(servoRamp[servo][7]);
    Serial.println();
  }
}

void go_home() {
  for (int i = 0; i < TOTAL_SERVOS; i++) {
    activeServo[i] = 0;
    activeSweep[i] = 0;
    servoSpeed[i] = spd;
    servoPos[i] = servoHome[i];
    targetPos[i] = servoHome[i];
    if (servoSetup[i][0] == 1) {
      pwm1.setPWM(servoSetup[i][1], 0, servoHome[i]);
    }
    delay(20);
  }

  for (int i = 0; i < TOTAL_LEGS; i++) {
    servoSequence[i] = 0;
  }
}

void set_home() {
  for (int m = 0; m < TOTAL_SERVOS; m++) {
    activeServo[m] = 1;
    targetPos[m] = servoHome[m];
  }
}

void set_stop() {
  if (spd_lock) {
    spd = spd_lock;
    spd_lock = 0;
    set_speed();
  }
  if (step_lock) {
    move_steps = step_lock;
    step_lock = 0;
  }

  for (int m = 0; m < TOTAL_SERVOS; m++) {
    activeServo[m] = 0;
    activeSweep[m] = 0;
  }
  for (int l = 0; l < TOTAL_LEGS; l++) {
    servoSequence[l] = 0;
  }
  set_home();
}

void set_stop_active() {
  for (int m = 0; m < TOTAL_SERVOS; m++) {
    activeServo[m] = 0;
    activeSweep[m] = 0;
  }
  for (int l = 0; l < TOTAL_LEGS; l++) {
    servoSequence[l] = 0;
  }
  use_ramp = 0;

  moving = 0;
  move_y_axis = 0;
  move_x_axis = 0;
  move_roll = 0;
  move_roll_body = 0;
  move_pitch = 0;
  move_pitch_body = 0;
  move_trot = 0;
  move_forward = 0;
  move_backward = 0;
  move_left = 0;
  move_right = 0;
  move_march = 0;
  move_wake = 0;
  move_sequence = 0;
  move_demo = 0;
  move_wman = 0;
  move_funplay = 0;
  move_look_left = 0;
  move_look_right = 0;
  move_roll_x = 0;
  move_pitch_y = 0;
  move_kin_x = 0;
  move_kin_y = 0;
  move_yaw_x = 0;
  move_yaw_y = 0;
  move_yaw = 0;
  move_servo = 0;
  move_leg = 0;
  move_follow = 0;

  if (mpu_is_active) mpu_active = 1;
}

void set_speed() {
  for (int i = 0; i < TOTAL_SERVOS; i++) {
    servoSpeed[i] = spd;
  }
  
  //recalc speed factor
  spd_factor = mapfloat(spd, min_spd, max_spd, min_spd_factor, max_spd_factor);
  if (rgb_active) {
    if (spd < 6) {
      rgb_request((char*)"MONOD");
    } else if (spd < 11) {
      rgb_request((char*)"MRNRE");
    } else if (spd < 21) {
      rgb_request((char*)"MPNPF");
    } else if (spd < 31) {
      rgb_request((char*)"MQNQG");
    } else {
      rgb_request((char*)"MSNSH");
    }
    rgb_request((char*)"xn");
  }
}



/*
   -------------------------------------------------------
   Move Functions
   -------------------------------------------------------
*/
//set pitch and roll axis from mpu data
void set_axis(float roll_step, float pitch_step) {
    float ar = abs(roll_step);
    float ap = abs(pitch_step);

    for (int i = 0; i < TOTAL_SERVOS; i++) {
      byte skip = 0;
      float t = 0.0;
      float f = 0.0;
      if (is_tibia(i)) {
        t = servoHome[i];
      } else if (is_femur(i)) {
        f = servoHome[i];
      }

      if (ar <= (mroll_prev + mpu_trigger_thresh) && ar >= (mroll_prev - mpu_trigger_thresh)) {
        if (roll_step < 0) { //roll left
          if (is_tibia(i)) {
            t -= ((abs(roll_step) * 0.65) * 4);
          } else if (is_femur(i)) {
            f += ((abs(roll_step) * 0.4) * 4);
          }
        } else { //roll right
          if (is_tibia(i)) {
            t += ((roll_step * 0.65) * 4);
          } else if (is_femur(i)) {
            f -= ((roll_step * 0.4) * 4);
          }
        }
      } else {
        skip = 1;
      }
      mroll_prev = ar;

      if (ap <= (mpitch_prev + mpu_trigger_thresh) || ap >= (mpitch_prev - mpu_trigger_thresh)) {
        if (pitch_step < 0) { //pitch front down
          if (is_tibia(i)) {
            if (is_front_leg(i)) {
              (is_left_leg(i)) ? t -= ((abs(pitch_step) * 1.15) * 3) : t += ((abs(pitch_step) * 1.15) * 3);
            } else {
              (is_left_leg(i)) ? t += ((abs(pitch_step) * 1.15) * 3) : t -= ((abs(pitch_step) * 1.15) * 3);
            }
          }
        } else { //pitch front up
          if (is_tibia(i)) {
            if (is_front_leg(i)) {
              (is_left_leg(i)) ? t += ((abs(pitch_step) * 1.15) * 3) : t -= ((abs(pitch_step) * 1.15) * 3);
            } else {
              (is_left_leg(i)) ? t -= ((abs(pitch_step) * 1.15) * 3) : t += ((abs(pitch_step) * 1.15) * 3);
            }
          }
        }
        mpitch_prev = pitch_step;
      }
      
      if (!skip) {
        if (is_tibia(i)) {
          activeServo[i] = 1;
          servoSpeed[i] = (7*spd_factor);
          targetPos[i] = limit_target(i, t, 0);
        } else if (is_femur(i)) {
          activeServo[i] = 1;
          servoSpeed[i] = (12*spd_factor);
          targetPos[i] = limit_target(i, f, 0);
        }
      }
    }

}


void run_demo() {
  if (!move_delay_sequences[0] && !move_delay_sequences[9]) {
    ramp_dist = 0.2;
    ramp_spd = 0.5;
    use_ramp = 1;

    move_demo = 1;

    move_delays[0] = 300;
    move_delay_sequences[0] = 15;
    move_delays[1] = 300;
    move_delay_sequences[1] = 14;
    move_delays[2] = 1200;
    move_delay_sequences[2] = 1;
    move_delays[3] = 3000;
    move_delay_sequences[3] = 2;
    move_delays[4] = 900;
    move_delay_sequences[4] = 3;
    move_delays[5] = 900;
    move_delay_sequences[5] = 5;
    move_delays[6] = 1500;
    move_delay_sequences[6] = 6;
    move_delays[7] = 1500;
    move_delay_sequences[7] = 7;
    move_delays[8] = 1500;
    move_delay_sequences[8] = 4;
    
    move_delays[9] = 900;
    move_delay_sequences[9] = 13;

    move_delays[10] = 1500;
    move_delay_sequences[10] = 11;
    
    move_delays[11] = 1500;
    move_delay_sequences[11] = 12;

    move_delays[12] = 3000;
    move_delay_sequences[12] = 8;
    move_delays[13] = 6000;
    move_delay_sequences[13] = 9;
    move_delays[14] = 3000;
    move_delay_sequences[14] = 10;

    delay_sequences();
  }
}


void funplay() {
  //recover from sitting up
  if (!activeServo[RRC] && !activeServo[RRF] && !activeServo[RRT] && !servoSequence[RR]) {
    update_sequencer(RR, RRT, 8, servoLimit[RRT][1]-80, 1, 0);
    update_sequencer(LR, LRT, 8, servoLimit[LRT][1]+80, 1, 0);
  }
  if (!activeServo[RRC] && !activeServo[RRF] && !activeServo[RRT] && servoSequence[RR] == 1) {
    update_sequencer(RR, RRF, 6, servoPos[RRF]+30, 2, 300);
    update_sequencer(LR, LRF, 6, servoPos[LRF]-30, 2, 300);
  }
  if (!activeServo[RRC] && !activeServo[RRF] && !activeServo[RRT] && servoSequence[RR] == 2) {
///repeat/open step!
    update_sequencer(RR, RRT, 8, servoLimit[RRT][1]-80, 3, 0);
    update_sequencer(LR, LRT, 8, servoLimit[LRT][1]+80, 3, 0);
  }

  if (!activeServo[RRC] && !activeServo[RRF] && !activeServo[RRT] && servoSequence[RR] == 3) {
    update_sequencer(RR, RRT, 12, servoLimit[RRT][0], 4, 50);
    update_sequencer(LR, LRT, 12, servoLimit[LRT][0], 4, 50);

    update_sequencer(RF, RFF, 24, servoHome[RFF], 1, 50);
    update_sequencer(LF, LFF, 24, servoHome[LFF], 1, 50);
    update_sequencer(RF, RFT, 24, servoLimit[RFT][1], 1, 50);
    update_sequencer(LF, LFT, 24, servoLimit[LFT][1], 1, 50);
  }

  if (!activeServo[RRC] && !activeServo[RRF] && !activeServo[RRT] && servoSequence[RR] == 4) {
    servoSpeed[RFF] = 3;
    servoSweep[RFF][0] = limit_target(RFF, (servoHome[RFF] + 40), 0);
    servoSweep[RFF][1] = limit_target(RFF, (servoHome[RFF] + 80), 20);
    servoSweep[RFF][2] = 0;
    servoSweep[RFF][3] = 9;
    targetPos[RFF] = servoSweep[RFF][1];
    activeSweep[RFF] = 1;

    servoSpeed[LFF] = 3;
    servoSweep[LFF][0] = limit_target(LFF, (servoHome[LFF] - 80), 20);
    servoSweep[LFF][1] = limit_target(LFF, (servoHome[LFF] - 40), 0);
    servoSweep[LFF][2] = 0;
    servoSweep[LFF][3] = 9;
    targetPos[LFF] = servoSweep[LFF][1];
    activeSweep[LFF] = 1;

    servoSpeed[RFT] = 5;
    servoSweep[RFT][0] = servoLimit[RFT][1];
    servoSweep[RFT][1] = servoHome[RFT];
    servoSweep[RFT][2] = 0;
    servoSweep[RFT][3] = 4;
    targetPos[RFT] = servoSweep[RFT][1];
    activeSweep[RFT] = 1;

    servoSpeed[LFT] = 5;
    servoSweep[LFT][0] = servoHome[LFT];
    servoSweep[LFT][1] = servoLimit[LFT][1];
    servoSweep[LFT][2] = 0;
    servoSweep[LFT][3] = 4;
    targetPos[LFT] = servoSweep[LFT][1];
    activeSweep[LFT] = 1;

    update_sequencer(RR, RRC, 3, servoPos[RRC], 5, 0);
  }

  if (!activeServo[RRC] && !activeServo[RRF] && !activeServo[RRT] && servoSequence[RR] == 5 && !activeSweep[RFT] && !activeSweep[LFT]) {
    update_sequencer(RF, RFT, 12, servoHome[RFT], 2, 50);
    update_sequencer(LF, LFT, 12, servoHome[LFT], 2, 50);
    update_sequencer(RR, RRT, 12, servoHome[RRT], 6, 100);
    update_sequencer(LR, LRT, 12, servoHome[LRT], 6, 100);
    update_sequencer(RR, RRF, 12, servoPos[RRF]-10, 6, 0);
    update_sequencer(LR, LRF, 12, servoPos[LRF]+10, 6, 0);

    update_sequencer(RF, RFF, 6, servoLimit[RFF][1], 2, 200);
    update_sequencer(LF, LFF, 6, servoLimit[LFF][1], 2, 200);
  }

  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 2) {
    update_sequencer(RF, RFC, 12, servoHome[RFC], 3, 0);
    update_sequencer(RR, RRF, 12, servoPos[RRF]+20, 7, 0);
    update_sequencer(LR, LRF, 12, servoPos[LRF]-20, 7, 0);
  }

  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 3 && !activeServo[RRF] && !activeServo[LRF]) {
    update_sequencer(RR, RRT, 6, servoLimit[RRT][1], 8, 50);
    update_sequencer(LR, LRT, 6, servoLimit[LRT][1], 8, 50);
    update_sequencer(RF, RFF, 12, servoHome[RFF]+30, 4, 100);
    update_sequencer(LF, LFF, 12, servoHome[LFF]-30, 4, 100);
  }

  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 4 && !activeServo[RRF] && !activeServo[LRF]) {
    update_sequencer(RF, RFC, 12, servoHome[RFC], 5, 0);
    update_sequencer(RR, RRT, 6, servoLimit[RRT][1], 9, 50);
    update_sequencer(LR, LRT, 6, servoLimit[LRT][1], 9, 50);
    update_sequencer(RR, RRF, 12, servoHome[RRF]-30, 9, 100);
    update_sequencer(LR, LRF, 12, servoHome[LRF]+30, 9, 100);
  }

  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 5 && !activeServo[RRF] && !activeServo[LRF]) {
    update_sequencer(RR, RRT, 6, servoHome[RRT], 10, 0);
    update_sequencer(LR, LRT, 6, servoHome[LRT], 10, 0);
    update_sequencer(RF, RFT, 6, servoHome[RFT], 6, 30);
    update_sequencer(LF, LFT, 6, servoHome[LFT], 6, 30);
    update_sequencer(RR, RRF, 6, servoHome[RRF], 9, 100);
    update_sequencer(LR, LRF, 6, servoHome[LRF], 9, 100);
    update_sequencer(RF, RFF, 6, servoHome[RFF], 6, 130);
    update_sequencer(LF, LFF, 6, servoHome[LFF], 6, 130);
  }

  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 6) {
    lastMoveDelayUpdate = millis();  

    if (rgb_active) {
      rgb_request((char*)"Gm");
    }
    move_funplay = 0;
    set_stop_active();
    delay(3000);
  }

}




/*
   -------------------------------------------------------
   Position Functions
   -------------------------------------------------------
*/
void set_stay() {
  for (int m = 0; m < TOTAL_SERVOS; m++) {
    activeSweep[m] = 0;
    activeServo[m] = 1;
    targetPos[m] = servoHome[m];
    servoSpeed[m] = 10;
    if (is_tibia(m)) {
      servoSpeed[m] = 5;
    }
  }

  lastMoveDelayUpdate = millis();
}

void set_sit() {
  activeServo[LFC] = 1;
  servoSpeed[LFC] = 10;
  targetPos[LFC] = servoHome[LFC];
  activeServo[LRC] = 1;
  servoSpeed[LRC] = 10;
  targetPos[LRC] = servoHome[LRC];
  activeServo[RFC] = 1;
  servoSpeed[RFC] = 10;
  targetPos[RFC] = servoHome[RFC];
  activeServo[RRC] = 1;
  servoSpeed[RRC] = 10;
  targetPos[RRC] = servoHome[RRC];

  activeServo[LFT] = 1;
  servoSpeed[LFT] = 10;
  targetPos[LFT] = servoLimit[LFT][0];
  activeServo[RFT] = 1;
  servoSpeed[RFT] = 10;
  targetPos[RFT] = servoLimit[RFT][0];

  activeServo[LRT] = 1;
  servoSpeed[LRT] = 10;
  targetPos[LRT] = servoLimit[LRT][1];
  activeServo[RRT] = 1;
  servoSpeed[RRT] = 10;
  targetPos[RRT] = servoLimit[RRT][1];

  activeServo[LRF] = 1;
  servoSpeed[LRF] = 10;
  targetPos[LRF] = (servoLimit[LRF][0] - 30);
  activeServo[RRF] = 1;
  servoSpeed[RRF] = 10;
  targetPos[RRF] = (servoLimit[RRF][0] + 30);

  activeServo[LFF] = 1;
  servoSpeed[LFF] = 20;
  targetPos[LFF] = (servoLimit[LFF][1] + 90);
  activeServo[RFF] = 1;
  servoSpeed[RFF] = 20;
  targetPos[RFF] = (servoLimit[RFF][1] - 90);

  lastMoveDelayUpdate = millis();
}

void set_crouch() {
  activeServo[LFC] = 1;
  servoSpeed[LFC] = 10;
  targetPos[LFC] = servoHome[LFC];
  activeServo[LRC] = 1;
  servoSpeed[LRC] = 10;
  targetPos[LRC] = servoHome[LRC];
  activeServo[RFC] = 1;
  servoSpeed[RFC] = 10;
  targetPos[RFC] = servoHome[RFC];
  activeServo[RRC] = 1;
  servoSpeed[RRC] = 10;
  targetPos[RRC] = servoHome[RRC];

  activeServo[LFT] = 1;
  servoSpeed[LFT] = 10;
  targetPos[LFT] = servoLimit[LFT][1];
  activeServo[RFT] = 1;
  servoSpeed[RFT] = 10;
  targetPos[RFT] = servoLimit[RFT][1];

  activeServo[LRT] = 1;
  servoSpeed[LRT] = 10;
  targetPos[LRT] = servoLimit[LRT][1];
  activeServo[RRT] = 1;
  servoSpeed[RRT] = 10;
  targetPos[RRT] = servoLimit[RRT][1];

  activeServo[LRF] = 1;
  servoSpeed[LRF] = 10;
  targetPos[LRF] = (servoLimit[LRF][0] - 30);
  activeServo[RRF] = 1;
  servoSpeed[RRF] = 10;
  targetPos[RRF] = (servoLimit[RRF][0] + 30);

  activeServo[LFF] = 1;
  servoSpeed[LFF] = 20;
  targetPos[LFF] = (servoLimit[LFF][0] - 30);
  activeServo[RFF] = 1;
  servoSpeed[RFF] = 20;
  targetPos[RFF] = (servoLimit[RFF][0] + 30);

  lastMoveDelayUpdate = millis();
}

void set_lay() {
  activeServo[LFC] = 1;
  servoSpeed[LFC] = 20;
  targetPos[LFC] = (servoLimit[LFC][1]);
  activeServo[LRC] = 1;
  servoSpeed[LRC] = 20;
  targetPos[LRC] = (servoLimit[LRC][1]);
  activeServo[RFC] = 1;
  servoSpeed[RFC] = 20;
  targetPos[RFC] = (servoLimit[RFC][1]);
  activeServo[RRC] = 1;
  servoSpeed[RRC] = 20;
  targetPos[RRC] = (servoLimit[RRC][1]);

  activeServo[LFT] = 1;
  servoSpeed[LFT] = 10;
  targetPos[LFT] = servoLimit[LFT][1];
  activeServo[RFT] = 1;
  servoSpeed[RFT] = 10;
  targetPos[RFT] = servoLimit[RFT][1];

  activeServo[LRT] = 1;
  servoSpeed[LRT] = 10;
  targetPos[LRT] = servoLimit[LRT][1];
  activeServo[RRT] = 1;
  servoSpeed[RRT] = 10;
  targetPos[RRT] = servoLimit[RRT][1];

  activeServo[LRF] = 1;
  servoSpeed[LRF] = 10;
  targetPos[LRF] = (servoLimit[LRF][0]);
  activeServo[RRF] = 1;
  servoSpeed[RRF] = 10;
  targetPos[RRF] = (servoLimit[RRF][0]);

  activeServo[LFF] = 1;
  servoSpeed[LFF] = 20;
  targetPos[LFF] = (servoLimit[LFF][0]);
  activeServo[RFF] = 1;
  servoSpeed[RFF] = 20;
  targetPos[RFF] = (servoLimit[RFF][0]);

  lastMoveDelayUpdate = millis();
}

void set_kneel() {
  activeServo[LFC] = 1;
  servoSpeed[LFC] = 10;
  targetPos[LFC] = (servoLimit[LFC][0] - 10);
  activeServo[LRC] = 1;
  servoSpeed[LRC] = 10;
  targetPos[LRC] = (servoLimit[LRC][0] - 10);
  activeServo[RFC] = 1;
  servoSpeed[RFC] = 10;
  targetPos[RFC] = (servoLimit[RFC][0] + 10);
  activeServo[RRC] = 1;
  servoSpeed[RRC] = 10;
  targetPos[RRC] = (servoLimit[RRC][0] + 10);

  activeServo[LFT] = 1;
  servoSpeed[LFT] = 20;
  targetPos[LFT] = (servoHome[LFT] - 40);
  activeServo[RFT] = 1;
  servoSpeed[RFT] = 20;
  targetPos[RFT] = (servoHome[RFT] + 40);

  activeServo[LRT] = 1;
  servoSpeed[LRT] = 20;
  targetPos[LRT] = (servoHome[LRT] - 40);
  activeServo[RRT] = 1;
  servoSpeed[RRT] = 20;
  targetPos[RRT] = (servoHome[RRT] + 40);

  activeServo[LRF] = 1;
  servoSpeed[LRF] = 10;
  targetPos[LRF] = (servoLimit[LRF][1] + 90);
  activeServo[RRF] = 1;
  servoSpeed[RRF] = 10;
  targetPos[RRF] = (servoLimit[RRF][1] - 90);

  activeServo[LFF] = 1;
  servoSpeed[LFF] = 10;
  targetPos[LFF] = (servoLimit[LFF][1] + 90);
  activeServo[RFF] = 1;
  servoSpeed[RFF] = 10;
  targetPos[RFF] = (servoLimit[RFF][1] - 90);

  lastMoveDelayUpdate = millis();
}


void look_left() {
  if (rgb_active) {
    rgb_request((char*)"MSNSvFn");
  }

  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && !servoSequence[RF]) {
    update_sequencer(LF, LFT, spd, (servoHome[LFT] + move_steps), 1, 0);
    update_sequencer(LF, LFC, spd, (servoHome[LFC] + move_steps), 1, 0);
    update_sequencer(LR, LRT, spd, (servoHome[LRT] + move_steps), 1, 0);
    update_sequencer(LR, LRC, spd, (servoHome[LRC] + move_steps), 1, 0);
    update_sequencer(RR, RRT, spd, (servoHome[RRT] + move_steps), 1, 0);
    update_sequencer(RR, RRC, spd, (servoHome[RRC] + move_steps), 1, 0);
    update_sequencer(RF, RFT, spd, (servoHome[RFT] + move_steps), 1, 0);
    update_sequencer(RF, RFC, spd, (servoHome[RFC] + move_steps), 1, 0);
  }

  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 1) {
    update_sequencer(RF, RFC, spd, servoHome[RFC], 2, 1000);
  }

  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 2) {
    for (int i = 0; i < TOTAL_SERVOS; i++) {
      if (is_left_leg(i) && !is_front_leg(i)) {
        update_sequencer(LR, i, spd, servoHome[i], 3, 0);
      } else if (is_left_leg(i) && is_front_leg(i)) {
        update_sequencer(LF, i, spd, servoHome[i], 3, 0);
      } else if (!is_left_leg(i) && is_front_leg(i)) {
        update_sequencer(RF, i, spd, servoHome[i], 3, 0);
      } else {
        update_sequencer(RR, i, spd, servoHome[i], 3, 0);
      }
    }
  }
  
  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 3) {
    update_sequencer(RF, RFC, spd, servoHome[RFC], 4, 1000);
  }

  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 4) {
    for (int i = 0; i < TOTAL_LEGS; i++) {
      servoSequence[i] = 0;
    }
    
    move_look_left = 0;
    move_loops = 4;
    move_switch = 2;
    move_wake = 1;

    //if called from follow routine, restore
    if (move_paused == "follow") {
      move_follow = 1;
      spd = spd_prev;
      set_speed();
    }
  }
}

void look_right() {
  if (rgb_active) {
    rgb_request((char*)"MSNSvFn");
  }

  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && !servoSequence[RF]) {
    update_sequencer(LF, LFT, spd, (servoHome[LFT] - move_steps), 1, 0);
    update_sequencer(LF, LFC, spd, (servoHome[LFC] - move_steps), 1, 0);
    update_sequencer(LR, LRT, spd, (servoHome[LRT] - move_steps), 1, 0);
    update_sequencer(LR, LRC, spd, (servoHome[LRC] - move_steps), 1, 0);
    update_sequencer(RR, RRT, spd, (servoHome[RRT] - move_steps), 1, 0);
    update_sequencer(RR, RRC, spd, (servoHome[RRC] - move_steps), 1, 0);
    update_sequencer(RF, RFT, spd, (servoHome[RFT] - move_steps), 1, 0);
    update_sequencer(RF, RFC, spd, (servoHome[RFC] - move_steps), 1, 0);
  }

  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 1) {
    update_sequencer(RF, RFC, spd, servoHome[RFC], 2, 1000);
  }

  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 2) {
    for (int i = 0; i < TOTAL_SERVOS; i++) {
      if (is_left_leg(i) && !is_front_leg(i)) {
        update_sequencer(LR, i, spd, servoHome[i], 3, 0);
      } else if (is_left_leg(i) && is_front_leg(i)) {
        update_sequencer(LF, i, spd, servoHome[i], 3, 0);
      } else if (!is_left_leg(i) && is_front_leg(i)) {
        update_sequencer(RF, i, spd, servoHome[i], 3, 0);
      } else {
        update_sequencer(RR, i, spd, servoHome[i], 3, 0);
      }
    }
  }
  
  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 3) {
    update_sequencer(RF, RFC, spd, servoHome[RFC], 4, 1000);
  }

  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 4) {
    for (int i = 0; i < TOTAL_LEGS; i++) {
      servoSequence[i] = 0;
    }
    
    move_look_right = 0;
    move_loops = 4;
    move_switch = 2;
    move_wake = 1;

    //if called from follow routine, restore
    if (move_paused == "follow") {
      move_follow = 1;
      spd = spd_prev;
      set_speed();
    }
  }
}



/*
   -------------------------------------------------------
   (implied) Kinematics Functions
   -------------------------------------------------------
*/

void move_kx() {
  int fms = (move_steps_kx * 0.8);
  int tms = (move_steps_kx * 1.3);
  int fsp = limit_speed((24 * spd_factor));
  int tsp = limit_speed((14 * spd_factor));

  update_sequencer(RF, RFF, fsp, (servoHome[RFF] - fms), 0, 0);
  update_sequencer(RF, RFT, tsp, (servoHome[RFT] + tms), 1, 0);
  update_sequencer(LF, LFF, fsp, (servoHome[LFF] + fms), 0, 0);
  update_sequencer(LF, LFT, tsp, (servoHome[LFT] - tms), 1, 0);

  update_sequencer(RR, RRF, fsp, (servoHome[RRF] - fms), 0, 0);
  update_sequencer(RR, RRT, tsp, (servoHome[RRT] + tms), 1, 0);
  update_sequencer(LR, LRF, fsp, (servoHome[LRF] + fms), 0, 0);
  update_sequencer(LR, LRT, tsp, (servoHome[LRT] - tms), 1, 0);

  move_kin_x = 0;

  lastMoveDelayUpdate = millis();  
}


void move_ky() {

  int fms = (move_steps_ky * 0.8);
  int tms = (move_steps_ky * 1.3);
  int fsp = limit_speed((24 * spd_factor));
  int tsp = limit_speed((14 * spd_factor));

  update_sequencer(RF, RFF, fsp, (servoHome[RFF] - fms), 0, 0);
  update_sequencer(RF, RFT, tsp, (servoHome[RFT] + tms), 1, 0);
  update_sequencer(LF, LFF, fsp, (servoHome[LFF] + fms), 0, 0);
  update_sequencer(LF, LFT, tsp, (servoHome[LFT] - tms), 1, 0);

  update_sequencer(RR, RRF, fsp, (servoHome[RRF] - fms), 0, 0);
  update_sequencer(RR, RRT, tsp, (servoHome[RRT] + tms), 1, 0);
  update_sequencer(LR, LRF, fsp, (servoHome[LRF] + fms), 0, 0);
  update_sequencer(LR, LRT, tsp, (servoHome[LRT] - tms), 1, 0);

  move_kin_y = 0;

  lastMoveDelayUpdate = millis(); 
}



void roll_x() {
  int csp = limit_speed((12 * spd_factor));

  update_sequencer(LF, LFC, csp, (servoHome[LFC] + move_steps_x), 0, 0);
  update_sequencer(LR, LRC, csp, (servoHome[LRC] + move_steps_x), 0, 0);
  update_sequencer(RF, RFC, csp, (servoHome[RFC] + move_steps_x), 0, 0);
  update_sequencer(RR, RRC, csp, (servoHome[RRC] + move_steps_x), 0, 0);

  move_roll_x = 0;

  lastMoveDelayUpdate = millis();  
}

void pitch_y() {

  int fms = (move_steps_y * 0.4);
  int tms = (move_steps_y * 0.65);
  int fsp = limit_speed((24 * spd_factor));
  int tsp = limit_speed((14 * spd_factor));

  update_sequencer(RF, RFF, fsp, (servoHome[RFF] - fms), 0, 0);
  update_sequencer(RF, RFT, tsp, (servoHome[RFT] + tms), 1, 0);
  update_sequencer(LF, LFF, fsp, (servoHome[LFF] + fms), 0, 0);
  update_sequencer(LF, LFT, tsp, (servoHome[LFT] - tms), 1, 0);

  update_sequencer(RR, RRF, fsp, (servoHome[RRF] + fms), 0, 0);
  update_sequencer(RR, RRT, tsp, (servoHome[RRT] - tms), 1, 0);
  update_sequencer(LR, LRF, fsp, (servoHome[LRF] - fms), 0, 0);
  update_sequencer(LR, LRT, tsp, (servoHome[LRT] + tms), 1, 0);

  move_pitch_y = 0;

  lastMoveDelayUpdate = millis();  
}

void yaw() {
  int cms = (move_steps_yaw * 0.4);
  int fms = (move_steps_yaw * 0.3);
  int tms = (move_steps_yaw * 0.1);
  int csp = limit_speed((10 * spd_factor));
  int fsp = limit_speed((20 * spd_factor));
  int tsp = limit_speed((40 * spd_factor));

//  int lfms = fms;
//  int rfms = fms;
  int ltms = tms;
  int rtms = tms;
  if (move_steps_yaw < 0) {
//    lfms = (move_steps_yaw * 0.5);
    ltms = (move_steps_yaw * 0.4);
  } else {
//    rfms = (move_steps_yaw * 0.5);
    rtms = (move_steps_yaw * 0.4);
  }

  int lfsp = fsp;
  int rfsp = fsp;
  int ltsp = tsp;
  int rtsp = tsp;
  if (move_steps_yaw < 0) {
    lfsp = limit_speed((8 * spd_factor));
    ltsp = limit_speed((10 * spd_factor));
  } else {
    rfsp = limit_speed((8 * spd_factor));
    rtsp = limit_speed((10 * spd_factor));
  }

  update_sequencer(LF, LFC, csp, (servoHome[LFC] + cms), 0, 0);
  update_sequencer(LF, LFF, lfsp, (servoHome[LFF] + fms), 1, 0);
  update_sequencer(LF, LFT, ltsp, (servoHome[LFT] - ltms), 1, 0);

  update_sequencer(RF, RFC, csp, (servoHome[RFC] + cms), 0, 0);
  update_sequencer(RF, RFF, rfsp, (servoHome[RFF] + fms), 1, 0);
  update_sequencer(RF, RFT, rtsp, (servoHome[RFT] - rtms), 1, 0);

  update_sequencer(LR, LRC, csp, (servoHome[LRC] - cms), 0, 0);
  update_sequencer(LR, LRF, lfsp, (servoHome[LRF] - fms), 1, 0);
  update_sequencer(LR, LRT, ltsp, (servoHome[LRT] + ltms), 1, 0);

  update_sequencer(RR, RRC, csp, (servoHome[RRC] - cms), 0, 0);
  update_sequencer(RR, RRF, rfsp, (servoHome[RRF] - fms), 1, 0);
  update_sequencer(RR, RRT, rtsp, (servoHome[RRT] + rtms), 1, 0);

  move_yaw = 0;

  lastMoveDelayUpdate = millis();  
}

void yaw_x() {
  int cms = (move_steps_yaw_x * 0.9);
  int fms = (move_steps_yaw_x * 0.4);
  int tms = (move_steps_yaw_x * 0.7);
  int csp = limit_speed((20 * spd_factor));
  int fsp = limit_speed((32 * spd_factor));
  int tsp = limit_speed((20 * spd_factor));

  update_sequencer(LF, LFC, csp, (servoHome[LFC] + cms), 0, 0);
  update_sequencer(LF, LFF, fsp, (servoHome[LFF] + fms), 1, 0);
  update_sequencer(LF, LFT, tsp, (servoHome[LFT] - tms), 1, 0);

  update_sequencer(LR, LRC, csp, (servoHome[LRC] + cms), 0, 0);
  update_sequencer(LR, LRF, fsp, (servoHome[LRF] + fms), 1, 0);
  update_sequencer(LR, LRT, tsp, (servoHome[LRT] - tms), 1, 0);

  update_sequencer(RF, RFC, csp, (servoHome[RFC] + cms), 0, 0);
  update_sequencer(RF, RFF, fsp, (servoHome[RFF] + fms), 1, 0);
  update_sequencer(RF, RFT, tsp, (servoHome[RFT] - tms), 1, 0);

  update_sequencer(RR, RRC, csp, (servoHome[RRC] + cms), 0, 0);
  update_sequencer(RR, RRF, fsp, (servoHome[RRF] + fms), 1, 0);
  update_sequencer(RR, RRT, tsp, (servoHome[RRT] - tms), 1, 0);

  move_yaw_x = 0;

  lastMoveDelayUpdate = millis();  
}

void yaw_y() {

  int fms = (move_steps_yaw_y * 0.6);
  int tms = (move_steps_yaw_y * 0.2);
  int fsp = limit_speed((25 * spd_factor));
  int tsp = limit_speed((30 * spd_factor));

  int ftms = tms;
  if (move_steps_yaw_y > 0) {
    fms = (move_steps_yaw_y * 0.3);
    tms = (move_steps_yaw_y * 0.1);
    ftms = (move_steps_yaw_y * 0.2);
  }

  update_sequencer(RF, RFF, fsp, (servoHome[RFF] - fms), 0, 0);
  update_sequencer(RF, RFT, tsp, (servoHome[RFT] - ftms), 1, 0);
  update_sequencer(LF, LFF, fsp, (servoHome[LFF] + fms), 0, 0);
  update_sequencer(LF, LFT, tsp, (servoHome[LFT] + ftms), 1, 0);

  update_sequencer(RR, RRF, fsp, (servoHome[RRF] - fms), 0, 0);
  update_sequencer(RR, RRT, tsp, (servoHome[RRT] - tms), 1, 0);
  update_sequencer(LR, LRF, fsp, (servoHome[LRF] + fms), 0, 0);
  update_sequencer(LR, LRT, tsp, (servoHome[LRT] + tms), 1, 0);

  move_yaw_y = 0;

  lastMoveDelayUpdate = millis();  
}

void x_axis() {
  if (!activeSweep[RRT]) {
    servoSpeed[LFF] = limit_speed((10 * spd_factor));
    servoSweep[LFF][0] = limit_target(LFF, ((servoHome[LFF] - 70) + (move_steps * .7)), 0);
    servoSweep[LFF][1] = limit_target(LFF, ((servoHome[LFF] - 10) + (move_steps * .1)), 0);
    servoSweep[LFF][2] = 0;
    servoSweep[LFF][3] = 1;
    targetPos[LFF] = servoSweep[LFF][1];
    activeSweep[LFF] = 1;

    servoSpeed[LFT] = limit_speed((10 * spd_factor));
    servoSweep[LFT][0] = limit_target(LFT, ((servoHome[LFT] + 85) - (move_steps * .85)), 0);
    servoSweep[LFT][1] = limit_target(LFT, ((servoHome[LFT] + 25) - (move_steps * .25)), 0);
    servoSweep[LFT][2] = 0;
    servoSweep[LFT][3] = 1;
    targetPos[LFT] = servoSweep[LFT][1];
    activeSweep[LFT] = 1;


    servoSpeed[RFF] = limit_speed((10 * spd_factor));
    servoSweep[RFF][0] = limit_target(RFF, ((servoHome[RFF] + 70) - (move_steps * .7)), 0); //65
    servoSweep[RFF][1] = limit_target(RFF, ((servoHome[RFF] + 10) - (move_steps * .1)), 0); //5
    servoSweep[RFF][2] = 0;
    servoSweep[RFF][3] = 1;
    targetPos[RFF] = servoSweep[RFF][1];
    activeSweep[RFF] = 1;

    servoSpeed[RFT] = limit_speed((10 * spd_factor));
    servoSweep[RFT][0] = limit_target(RFT, ((servoHome[RFT] - 85) + (move_steps * .85)), 0);
    servoSweep[RFT][1] = limit_target(RFT, ((servoHome[RFT] - 25) + (move_steps * .25)), 0);
    servoSweep[RFT][2] = 0;
    servoSweep[RFT][3] = 1;
    targetPos[RFT] = servoSweep[RFT][1];
    activeSweep[RFT] = 1;


    servoSpeed[LRF] = limit_speed((10 * spd_factor));
    servoSweep[LRF][0] = limit_target(LRF, ((servoHome[LRF] - 70) + (move_steps * .7)), 0);
    servoSweep[LRF][1] = limit_target(LRF, ((servoHome[LRF] - 10) + (move_steps * .1)), 0);
    servoSweep[LRF][2] = 0;
    servoSweep[LRF][3] = 1;
    targetPos[LRF] = servoSweep[LRF][1];
    activeSweep[LRF] = 1;

    servoSpeed[LRT] = limit_speed((10 * spd_factor));
    servoSweep[LRT][0] = limit_target(LRT, ((servoHome[LRT] + 85) - (move_steps * .85)), 0); //90
    servoSweep[LRT][1] = limit_target(LRT, ((servoHome[LRT] + 25) - (move_steps * .25)), 0); //30
    servoSweep[LRT][2] = 0;
    servoSweep[LRT][3] = 1;
    targetPos[LRT] = servoSweep[LRT][1];
    activeSweep[LRT] = 1;


    servoSpeed[RRF] = limit_speed((10 * spd_factor));
    servoSweep[RRF][0] = limit_target(RRF, ((servoHome[RRF] + 70) - (move_steps * .7)), 0);
    servoSweep[RRF][1] = limit_target(RRF, ((servoHome[RRF] + 10) - (move_steps * .1)), 0);
    servoSweep[RRF][2] = 0;
    servoSweep[RRF][3] = 1;
    targetPos[RRF] = servoSweep[RRF][1];
    activeSweep[RRF] = 1;

    servoSpeed[RRT] = limit_speed((10 * spd_factor));
    servoSweep[RRT][0] = limit_target(RRT, ((servoHome[RRT] - 85) + (move_steps * .85)), 0);
    servoSweep[RRT][1] = limit_target(RRT, ((servoHome[RRT] - 25) + (move_steps * .25)), 0);
    servoSweep[RRT][2] = 0;
    servoSweep[RRT][3] = 1;
    targetPos[RRT] = servoSweep[RRT][1];
    activeSweep[RRT] = 1;

    if (move_loops) {
      move_loops--;
      if (!move_loops) {
        move_x_axis = 0;
      }
    }

    lastMoveDelayUpdate = millis();  
  }
}

void y_axis() {

  if (!activeSweep[RRT]) {
    servoSpeed[LFC] = limit_speed((96 * spd_factor));
    servoSweep[LFC][0] = (servoHome[LFC]);
    servoSweep[LFC][1] = limit_target(LFC, (servoHome[LFC] + (move_steps * .05)), 0);
    servoSweep[LFC][2] = 0;
    servoSweep[LFC][3] = 1;
    targetPos[LFC] = servoSweep[LFC][1];
    activeSweep[LFC] = 1;

    servoSpeed[LFT] = limit_speed((7 * spd_factor));
    servoSweep[LFT][0] = limit_target(LFT, (servoHome[LFT] - (move_steps * .65)), 0);
    servoSweep[LFT][1] = limit_target(LFT, (servoHome[LFT] + (move_steps * .65)), 0);
    servoSweep[LFT][2] = 0;
    servoSweep[LFT][3] = 1;
    targetPos[LFT] = servoSweep[LFT][1];
    activeSweep[LFT] = 1;

    servoSpeed[LFF] = limit_speed((12 * spd_factor));
    servoSweep[LFF][0] = limit_target(LFF, (servoHome[LFF] + (move_steps * .4)), 0);
    servoSweep[LFF][1] = limit_target(LFF, (servoHome[LFF] - (move_steps * .4)), 0);
    servoSweep[LFF][2] = 0;
    servoSweep[LFF][3] = 1;
    targetPos[LFF] = servoSweep[LFF][1];
    activeSweep[LFF] = 1;


    servoSpeed[RFC] = limit_speed((96 * spd_factor));
    servoSweep[RFC][0] = (servoHome[RFC]);
    servoSweep[RFC][1] = limit_target(RFC, (servoHome[RFC] - (move_steps * .05)), 0);
    servoSweep[RFC][2] = 0;
    servoSweep[RFC][3] = 1;
    targetPos[RFC] = servoSweep[RFC][1];
    activeSweep[RFC] = 1;

    servoSpeed[RFT] = limit_speed((7 * spd_factor));
    servoSweep[RFT][0] = limit_target(RFT, (servoHome[RFT] + (move_steps * .65)), 0);
    servoSweep[RFT][1] = limit_target(RFT, (servoHome[RFT] - (move_steps * .65)), 0);
    servoSweep[RFT][2] = 0;
    servoSweep[RFT][3] = 1;
    targetPos[RFT] = servoSweep[RFT][1];
    activeSweep[RFT] = 1;

    servoSpeed[RFF] = limit_speed((12 * spd_factor));
    servoSweep[RFF][0] = limit_target(RFF, (servoHome[RFF] - (move_steps * .4)), 0);
    servoSweep[RFF][1] = limit_target(RFF, (servoHome[RFF] + (move_steps * .4)), 0);
    servoSweep[RFF][2] = 0;
    servoSweep[RFF][3] = 1;
    targetPos[RFF] = servoSweep[RFF][1];
    activeSweep[RFF] = 1;


    servoSpeed[LRC] = limit_speed((96 * spd_factor));
    servoSweep[LRC][0] = (servoHome[LRC]);
    servoSweep[LRC][1] = limit_target(LRC, (servoHome[LRC] + (move_steps * .05)), 0);
    servoSweep[LRC][2] = 0;
    servoSweep[LRC][3] = 1;
    targetPos[LRC] = servoSweep[LRC][1];
    activeSweep[LRC] = 1;

    servoSpeed[LRT] = limit_speed((7 * spd_factor));
    servoSweep[LRT][0] = limit_target(LRT, (servoHome[LRT] - (move_steps * .65)), 0);
    servoSweep[LRT][1] = limit_target(LRT, (servoHome[LRT] + (move_steps * .65)), 0);
    servoSweep[LRT][2] = 0;
    servoSweep[LRT][3] = 1;
    targetPos[LRT] = servoSweep[LRT][1];
    activeSweep[LRT] = 1;

    servoSpeed[LRF] = limit_speed((12 * spd_factor));
    servoSweep[LRF][0] = limit_target(LRF, (servoHome[LRF] + (move_steps * .4)), 0);
    servoSweep[LRF][1] = limit_target(LRF, (servoHome[LRF] - (move_steps * .4)), 0);
    servoSweep[LRF][2] = 0;
    servoSweep[LRF][3] = 1;
    targetPos[LRF] = servoSweep[LRF][1];
    activeSweep[LRF] = 1;


    servoSpeed[RRC] = limit_speed((96 * spd_factor));
    servoSweep[RRC][0] = (servoHome[RRC]);
    servoSweep[RRC][1] = limit_target(RRC, (servoHome[RRC] - (move_steps * .05)), 0);
    servoSweep[RRC][2] = 0;
    servoSweep[RRC][3] = 1;
    targetPos[RRC] = servoSweep[RRC][1];
    activeSweep[RRC] = 1;

    servoSpeed[RRT] = limit_speed((7 * spd_factor));
    servoSweep[RRT][0] = limit_target(RRT, (servoHome[RRT] + (move_steps * .65)), 0);
    servoSweep[RRT][1] = limit_target(RRT, (servoHome[RRT] - (move_steps * .65)), 0);
    servoSweep[RRT][2] = 0;
    servoSweep[RRT][3] = 1;
    targetPos[RRT] = servoSweep[RRT][1];
    activeSweep[RRT] = 1;

    servoSpeed[RRF] = limit_speed((12 * spd_factor));
    servoSweep[RRF][0] = limit_target(RRF, (servoHome[RRF] - (move_steps * .4)), 0);
    servoSweep[RRF][1] = limit_target(RRF, (servoHome[RRF] + (move_steps * .4)), 0);
    servoSweep[RRF][2] = 0;
    servoSweep[RRF][3] = 1;
    targetPos[RRF] = servoSweep[RRF][1];
    activeSweep[RRF] = 1;

    if (move_loops) {
      move_loops--;
      if (!move_loops) {
        move_y_axis = 0;
      }
    }
  
    lastMoveDelayUpdate = millis();  
  }
}

void roll() {
  if (!activeSweep[RRF]) {
    servoSpeed[LFT] = limit_speed((5 * spd_factor));
    servoSweep[LFT][0] = limit_target(LFT, (servoHome[LFT] - move_steps), 0);
    servoSweep[LFT][1] = limit_target(LFT, (servoHome[LFT] + move_steps), 0);
    servoSweep[LFT][2] = 0;
    servoSweep[LFT][3] = 1;
    targetPos[LFT] = servoSweep[LFT][1];
    activeSweep[LFT] = 1;

    servoSpeed[LFF] = limit_speed((8 * spd_factor));
    servoSweep[LFF][0] = limit_target(LFF, (servoHome[LFF] + (move_steps * .667)), 0);
    servoSweep[LFF][1] = limit_target(LFF, (servoHome[LFF] - (move_steps * .667)), 0);
    servoSweep[LFF][2] = 0;
    servoSweep[LFF][3] = 1;
    targetPos[LFF] = servoSweep[LFF][1];
    activeSweep[LFF] = 1;


    servoSpeed[LRT] = limit_speed((5 * spd_factor));
    servoSweep[LRT][0] = limit_target(LRT, (servoHome[LRT] - move_steps), 0);
    servoSweep[LRT][1] = limit_target(LRT, (servoHome[LRT] + move_steps), 0);
    servoSweep[LRT][2] = 0;
    servoSweep[LRT][3] = 1;
    targetPos[LRT] = servoSweep[LRT][1];
    activeSweep[LRT] = 1;

    servoSpeed[LRF] = limit_speed((8 * spd_factor));
    servoSweep[LRF][0] = limit_target(LRF, (servoHome[LRF] + (move_steps * .667)), 0);
    servoSweep[LRF][1] = limit_target(LRF, (servoHome[LRF] - (move_steps * .667)), 0);
    servoSweep[LRF][2] = 0;
    servoSweep[LRF][3] = 1;
    targetPos[LRF] = servoSweep[LRF][1];
    activeSweep[LRF] = 1;


    servoSpeed[RFT] = limit_speed((5 * spd_factor));
    servoSweep[RFT][0] = limit_target(RFT, (servoHome[RFT] - move_steps), 0);
    servoSweep[RFT][1] = limit_target(RFT, (servoHome[RFT] + move_steps), 0);
    servoSweep[RFT][2] = 0;
    servoSweep[RFT][3] = 1;
    targetPos[RFT] = servoSweep[RFT][1];
    activeSweep[RFT] = 1;

    servoSpeed[RFF] = limit_speed((8 * spd_factor));
    servoSweep[RFF][0] = limit_target(RFF, (servoHome[RFF] + (move_steps * .667)), 0);
    servoSweep[RFF][1] = limit_target(RFF, (servoHome[RFF] - (move_steps * .667)), 0);
    servoSweep[RFF][2] = 0;
    servoSweep[RFF][3] = 1;
    targetPos[RFF] = servoSweep[RFF][1];
    activeSweep[RFF] = 1;


    servoSpeed[RRT] = limit_speed((5 * spd_factor));
    servoSweep[RRT][0] = limit_target(RRT, (servoHome[RRT] - move_steps), 0);
    servoSweep[RRT][1] = limit_target(RRT, (servoHome[RRT] + move_steps), 0);
    servoSweep[RRT][2] = 0;
    servoSweep[RRT][3] = 1;
    targetPos[RRT] = servoSweep[RRT][1];
    activeSweep[RRT] = 1;

    servoSpeed[RRF] = limit_speed((8 * spd_factor));
    servoSweep[RRF][0] = limit_target(RRF, (servoHome[RRF] + (move_steps * .667)), 0);
    servoSweep[RRF][1] = limit_target(RRF, (servoHome[RRF] - (move_steps * .667)), 0);
    servoSweep[RRF][2] = 0;
    servoSweep[RRF][3] = 1;
    targetPos[RRF] = servoSweep[RRF][1];
    activeSweep[RRF] = 1;

    if (move_loops) {
      move_loops--;
      if (!move_loops) {
        move_roll = 0;
      }
    }

    lastMoveDelayUpdate = millis();  
  }
}

void roll_body() {
  if (!activeSweep[RRC]) {
    servoSpeed[LFC] = limit_speed((5 * spd_factor));
    servoSweep[LFC][0] = limit_target(LFC, (servoHome[LFC] - move_steps), 0);
    servoSweep[LFC][1] = limit_target(LFC, (servoHome[LFC] + move_steps), 0);
    servoSweep[LFC][2] = 0;
    servoSweep[LFC][3] = 1;
    targetPos[LFC] = servoSweep[LFC][1];
    activeSweep[LFC] = 1;

    servoSpeed[LRC] = limit_speed((5 * spd_factor));
    servoSweep[LRC][0] = limit_target(LRC, (servoHome[LRC] - move_steps), 0);
    servoSweep[LRC][1] = limit_target(LRC, (servoHome[LRC] + move_steps), 0);
    servoSweep[LRC][2] = 0;
    servoSweep[LRC][3] = 1;
    targetPos[LRC] = servoSweep[LRC][1];
    activeSweep[LRC] = 1;

    servoSpeed[RFC] = limit_speed((5 * spd_factor));
    servoSweep[RFC][0] = limit_target(RFC, (servoHome[RFC] - move_steps), 0);
    servoSweep[RFC][1] = limit_target(RFC, (servoHome[RFC] + move_steps), 0);
    servoSweep[RFC][2] = 0;
    servoSweep[RFC][3] = 1;
    targetPos[RFC] = servoSweep[RFC][1];
    activeSweep[RFC] = 1;

    servoSpeed[RRC] = limit_speed((5 * spd_factor));
    servoSweep[RRC][0] = limit_target(RRC, (servoHome[RRC] - move_steps), 0);
    servoSweep[RRC][1] = limit_target(RRC, (servoHome[RRC] + move_steps), 0);
    servoSweep[RRC][2] = 0;
    servoSweep[RRC][3] = 1;
    targetPos[RRC] = servoSweep[RRC][1];
    activeSweep[RRC] = 1;

    if (move_loops) {
      move_loops--;
      if (!move_loops) {
        move_roll_body = 0;
      }
    }

    lastMoveDelayUpdate = millis();  
  }
}

void pitch(int xdir) {
  float sinc0 = .15;
  float sinc1 = 1.15;

  if (xdir < 0) {  //turn left
    xdir = abs(xdir);
    servoStepMoves[RFC][0] = limit_target(RFC, (servoPos[RFC] + (xdir / 4)), 25);
    servoStepMoves[RRC][0] = limit_target(RRC, (servoPos[RRC] + (xdir / 4)), 25);
    servoStepMoves[LFC][0] = 0;
    servoStepMoves[LRC][0] = 0;
  } else if (xdir > 0) {  //turn right
    servoStepMoves[RFC][0] = 0;
    servoStepMoves[RRC][0] = 0;
    servoStepMoves[LFC][0] = limit_target(LFC, (servoPos[LFC] + (xdir / 4)), 25);
    servoStepMoves[LRC][0] = limit_target(LRC, (servoPos[LRC] + (xdir / 4)), 25);
  } else {
    servoStepMoves[RFC][0] = 0;
    servoStepMoves[RRC][0] = 0;
    servoStepMoves[LFC][0] = 0;
    servoStepMoves[LRC][0] = 0;
  }

  if (!activeSweep[RRT]) {
    servoSpeed[LFT] = limit_speed((3 * spd_factor));
    if (move_steps < 0) {
      servoSweep[LFT][0] = limit_target(LFT, (servoHome[LFT] + (move_steps * sinc0)), 0);
      servoSweep[LFT][1] = limit_target(LFT, (servoHome[LFT] - (move_steps * sinc1)), 0);
    } else {
      servoSweep[LFT][0] = limit_target(LFT, (servoHome[LFT] - (move_steps * sinc0)), 0);
      servoSweep[LFT][1] = limit_target(LFT, (servoHome[LFT] + (move_steps * sinc1)), 0);
    }
    servoSweep[LFT][2] = 0;
    servoSweep[LFT][3] = 1;
    targetPos[LFT] = servoSweep[LFT][1];
    activeSweep[LFT] = 1;

    servoSpeed[RFT] = limit_speed((3 * spd_factor));
    if (move_steps < 0) {
      servoSweep[RFT][0] = limit_target(RFT, (servoHome[RFT] - (move_steps * sinc0)), 0);
      servoSweep[RFT][1] = limit_target(RFT, (servoHome[RFT] + (move_steps * sinc1)), 0);
    } else {
      servoSweep[RFT][0] = limit_target(RFT, (servoHome[RFT] + (move_steps * sinc0)), 0);
      servoSweep[RFT][1] = limit_target(RFT, (servoHome[RFT] - (move_steps * sinc1)), 0);
    }
    servoSweep[RFT][2] = 0;
    servoSweep[RFT][3] = 1;
    targetPos[RFT] = servoSweep[RFT][1];
    activeSweep[RFT] = 1;

    servoSpeed[LRT] = limit_speed((3 * spd_factor));
    if (move_steps < 0) {
      servoSweep[LRT][0] = limit_target(LRT, (servoHome[LRT] - (move_steps * sinc0)), 0);
      servoSweep[LRT][1] = limit_target(LRT, (servoHome[LRT] + (move_steps * sinc1)), 0);
    } else {
      servoSweep[LRT][0] = limit_target(LRT, (servoHome[LRT] + (move_steps * sinc0)), 0);
      servoSweep[LRT][1] = limit_target(LRT, (servoHome[LRT] - (move_steps * sinc1)), 0);
    }
    servoSweep[LRT][2] = 0;
    servoSweep[LRT][3] = 1;
    targetPos[LRT] = servoSweep[LRT][1];
    activeSweep[LRT] = 1;

    servoSpeed[RRT] = limit_speed((3 * spd_factor));
    if (move_steps < 0) {
      servoSweep[RRT][0] = limit_target(RRT, (servoHome[RRT] + (move_steps * sinc0)), 0);
      servoSweep[RRT][1] = limit_target(RRT, (servoHome[RRT] - (move_steps * sinc1)), 0);
    } else {
      servoSweep[RRT][0] = limit_target(RRT, (servoHome[RRT] - (move_steps * sinc0)), 0);
      servoSweep[RRT][1] = limit_target(RRT, (servoHome[RRT] + (move_steps * sinc1)), 0);
    }
    servoSweep[RRT][2] = 0;
    servoSweep[RRT][3] = 1;
    targetPos[RRT] = servoSweep[RRT][1];
    activeSweep[RRT] = 1;

    update_sequencer(LF, LFC, limit_speed((3 * spd_factor)), servoStepMoves[LFC][0], 0, 0);
    update_sequencer(RF, RFC, limit_speed((3 * spd_factor)), servoStepMoves[RFC][0], 0, 0);
    update_sequencer(LR, LRC, limit_speed((3 * spd_factor)), servoStepMoves[LRC][0], 0, 0);
    update_sequencer(RR, RRC, limit_speed((3 * spd_factor)), servoStepMoves[RRC][0], 0, 0);

    if (move_loops) {
      move_loops--;
      if (!move_loops) {
        move_pitch = 0;
      }
    }

    lastMoveDelayUpdate = millis();  
  }
}

void pitch_body() {
  if (!activeSweep[RRF]) {
    servoSpeed[LFC] = limit_speed((34 * spd_factor));
    servoSweep[LFC][0] = (servoHome[LFC]);
    servoSweep[LFC][1] = limit_target(LFC, (servoHome[LFC] + (move_steps * .05)), 0);
    servoSweep[LFC][2] = 0;
    servoSweep[LFC][3] = 1;
    targetPos[LFC] = servoSweep[LFC][1];
    activeSweep[LFC] = 1;

    servoSpeed[LFT] = limit_speed((5 * spd_factor));
    servoSweep[LFT][0] = limit_target(LFT, (servoHome[LFT] - (move_steps * .35)), 0);
    servoSweep[LFT][1] = limit_target(LFT, (servoHome[LFT] + (move_steps * .35)), 0);
    servoSweep[LFT][2] = 0;
    servoSweep[LFT][3] = 1;
    targetPos[LFT] = servoSweep[LFT][1];
    activeSweep[LFT] = 1;

    servoSpeed[LFF] = limit_speed((9 * spd_factor));
    servoSweep[LFF][0] = limit_target(LFF, (servoHome[LFF] + (move_steps * .2)), 0);
    servoSweep[LFF][1] = limit_target(LFF, (servoHome[LFF] - (move_steps * .2)), 0);
    servoSweep[LFF][2] = 0;
    servoSweep[LFF][3] = 1;
    targetPos[LFF] = servoSweep[LFF][1];
    activeSweep[LFF] = 1;


    servoSpeed[RFC] = limit_speed((34 * spd_factor));
    servoSweep[RFC][0] = (servoHome[RFC]);
    servoSweep[RFC][1] = limit_target(RFC, (servoHome[RFC] - (move_steps * .05)), 0);
    servoSweep[RFC][2] = 0;
    servoSweep[RFC][3] = 1;
    targetPos[RFC] = servoSweep[RFC][1];
    activeSweep[RFC] = 1;

    servoSpeed[RFT] = limit_speed((5 * spd_factor));
    servoSweep[RFT][0] = limit_target(RFT, (servoHome[RFT] + (move_steps * .35)), 0);
    servoSweep[RFT][1] = limit_target(RFT, (servoHome[RFT] - (move_steps * .35)), 0);
    servoSweep[RFT][2] = 0;
    servoSweep[RFT][3] = 1;
    targetPos[RFT] = servoSweep[RFT][1];
    activeSweep[RFT] = 1;

    servoSpeed[RFF] = limit_speed((9 * spd_factor));
    servoSweep[RFF][0] = limit_target(RFF, (servoHome[RFF] - (move_steps * .2)), 0);
    servoSweep[RFF][1] = limit_target(RFF, (servoHome[RFF] + (move_steps * .2)), 0);
    servoSweep[RFF][2] = 0;
    servoSweep[RFF][3] = 1;
    targetPos[RFF] = servoSweep[RFF][1];
    activeSweep[RFF] = 1;


    servoSpeed[LRC] = limit_speed((34 * spd_factor));
    servoSweep[LRC][0] = limit_target(LRC, (servoHome[LRC] - (move_steps * .05)), 0);
    servoSweep[LRC][1] = (servoHome[LRC]);
    servoSweep[LRC][2] = 0;
    servoSweep[LRC][3] = 1;
    targetPos[LRC] = servoSweep[LRC][1];
    activeSweep[LRC] = 1;

    servoSpeed[LRT] = limit_speed((5 * spd_factor));
    servoSweep[LRT][0] = limit_target(LRT, (servoHome[LRT] + (move_steps * .35)), 0);
    servoSweep[LRT][1] = limit_target(LRT, (servoHome[LRT] - (move_steps * .35)), 0);
    servoSweep[LRT][2] = 0;
    servoSweep[LRT][3] = 1;
    targetPos[LRT] = servoSweep[LRT][1];
    activeSweep[LRT] = 1;

    servoSpeed[LRF] = limit_speed((9 * spd_factor));
    servoSweep[LRF][0] = limit_target(LRF, (servoHome[LRF] - (move_steps * .2)), 0);
    servoSweep[LRF][1] = limit_target(LRF, (servoHome[LRF] + (move_steps * .2)), 0);
    servoSweep[LRF][2] = 0;
    servoSweep[LRF][3] = 1;
    targetPos[LRF] = servoSweep[LRF][1];
    activeSweep[LRF] = 1;


    servoSpeed[RRC] = limit_speed((34 * spd_factor));
    servoSweep[RRC][0] = limit_target(RRC, (servoHome[RRC] + (move_steps * .05)), 0);
    servoSweep[RRC][1] = (servoHome[RRC]);
    servoSweep[RRC][2] = 0;
    servoSweep[RRC][3] = 1;
    targetPos[RRC] = servoSweep[RRC][1];
    activeSweep[RRC] = 1;

    servoSpeed[RRT] = limit_speed((5 * spd_factor));
    servoSweep[RRT][0] = limit_target(RRT, (servoHome[RRT] - (move_steps * .35)), 0);
    servoSweep[RRT][1] = limit_target(RRT, (servoHome[RRT] + (move_steps * .35)), 0);
    servoSweep[RRT][2] = 0;
    servoSweep[RRT][3] = 1;
    targetPos[RRT] = servoSweep[RRT][1];
    activeSweep[RRT] = 1;

    servoSpeed[RRF] = limit_speed((9 * spd_factor));
    servoSweep[RRF][0] = limit_target(RRF, (servoHome[RRF] + (move_steps * .2)), 0);
    servoSweep[RRF][1] = limit_target(RRF, (servoHome[RRF] - (move_steps * .2)), 0);
    servoSweep[RRF][2] = 0;
    servoSweep[RRF][3] = 1;
    targetPos[RRF] = servoSweep[RRF][1];
    activeSweep[RRF] = 1;

    if (move_loops) {
      move_loops--;
      if (!move_loops) {
        move_pitch_body = 0;
      }
    }
  
    lastMoveDelayUpdate = millis();  
  }
}



/*
   -------------------------------------------------------
   Walking Gait Functions
   -------------------------------------------------------
*/

/*
 * step_match gait
 * 
 * xdir: +right/-left
 * ydir: +forward/-backward
 * zdir: +up/-down
 * 
 */
void step_march(float xdir, float ydir, float zdir) {

//DEVWORK
  ramp_dist = 0.4;
  ramp_spd = 0.6;
  use_ramp = 0;  

  moving = 1;

  //maintain min spd
  if (spd > 20) spd = 20;

  //define move factors
  float cmfact = 0.750;
  float fmfact = 0.700;
  float tmfact = 1.300;

  //define move z-factors
  float czmfact = 0;
  float fzmfact = 0.8;
  float tzmfact = 1.525;

  //define speed factors
  float csfact = 3.00;
  float fsfact = 2.50;
  float tsfact = 5.00;
  float sfact_div = 2.0;

  //calculate zdir positions
  float tz = (zdir * tzmfact);
  float fz = (zdir * fzmfact);
  float cz = czmfact;


  //define home positions 
  //DEVNOTE: this is a bit ugly, but necessary to manipulate home/start position based on zdir
  //         if continue to use and no better solution, create a function for it
  //
  float gaitHome[TOTAL_SERVOS];
  for (int i = 0; i < TOTAL_SERVOS; i++) {
    gaitHome[i] = servoHome[i];
  }

  //pre-apply z factors by direction, not mid-sequence
  if (zdir < -1) {
    gaitHome[RFT] += abs(tz);
    gaitHome[RFF] -= abs(fz);
    gaitHome[RFC] -= abs(cz);
    gaitHome[RRT] += abs(tz);
    gaitHome[RRF] -= abs(fz);
    gaitHome[RRC] -= abs(cz);
    gaitHome[LFT] -= abs(tz);
    gaitHome[LFF] += abs(fz);
    gaitHome[LFC] += abs(cz);
    gaitHome[LRT] -= abs(tz);
    gaitHome[LRF] += abs(fz);
    gaitHome[LRC] += abs(cz);
  } else if (zdir > 1) {
    gaitHome[RFT] -= tz;
    gaitHome[RFF] += fz;
    gaitHome[RFC] += cz;
    gaitHome[RRT] -= tz;
    gaitHome[RRF] += fz;
    gaitHome[RRC] += cz;
    gaitHome[LFT] += tz;
    gaitHome[LFF] -= fz;
    gaitHome[LFC] -= cz;
    gaitHome[LRT] += tz;
    gaitHome[LRF] -= fz;
    gaitHome[LRC] -= cz;
  }
  
  //calculate steps, speeds, direction, and step_weight_factor_rear
  float servoMS[TOTAL_SERVOS][6][2];   //[servo][step1,step2,etc][dist,spd]

  //coaxes
  //set default marching steps/speed
  servoMS[RFC][0][0] = gaitHome[RFC];
  servoMS[RRC][0][0] = gaitHome[RRC];
  servoMS[LFC][0][0] = gaitHome[LFC];
  servoMS[LRC][0][0] = gaitHome[LRC];
  servoMS[RFC][0][1] = servoMS[LFC][0][1] = servoMS[RRC][0][1] = servoMS[LRC][0][1] = (csfact * spd_factor);
  servoMS[RFC][1][1] = servoMS[LFC][1][1] = servoMS[RRC][1][1] = servoMS[LRC][1][1] = ((csfact * spd_factor) / sfact_div);

  //calc movement and apply weight factor by direction / load shift
  if (xdir < -1) { //turn left
    servoMS[RFC][0][0] = (gaitHome[RFC] + (abs(xdir) * cmfact));
    servoMS[RRC][0][0] = (gaitHome[RRC] + ((abs(xdir) * cmfact) + ((abs(xdir) * cmfact) * (step_weight_factor_rear * cmfact)))); //add weight factor
    servoMS[LFC][0][0] = (gaitHome[LFC] + (abs(xdir) * cmfact));
    servoMS[LRC][0][0] = (gaitHome[LRC] + ((abs(xdir) * cmfact) + ((abs(xdir) * cmfact) * (step_weight_factor_rear * cmfact)))); //add weight factor

    servoMS[RFC][0][1] = servoMS[LFC][0][1] = (csfact * spd_factor);
    servoMS[RRC][0][1] = servoMS[LRC][0][1] = ((csfact - (step_weight_factor_rear * csfact)) * spd_factor);
  } else if (xdir > 1) { //turn right
    servoMS[RFC][0][0] = (gaitHome[RFC] - (xdir * cmfact));
    servoMS[RRC][0][0] = (gaitHome[RRC] - ((xdir * cmfact) + ((xdir * cmfact) * (step_weight_factor_rear * cmfact)))); //add weight factor
    servoMS[LFC][0][0] = (gaitHome[LFC] - (xdir * cmfact));
    servoMS[LRC][0][0] = (gaitHome[LRC] - ((xdir * cmfact) + ((xdir * cmfact) * (step_weight_factor_rear * cmfact)))); //add weight factor

    servoMS[RFC][0][1] = servoMS[LFC][0][1] = (csfact * spd_factor);
    servoMS[RRC][0][1] = servoMS[LRC][0][1] = ((csfact * spd_factor) - (csfact * (step_weight_factor_rear-1)));
  }

  //femurs
  //set default marching steps/speed and step_weight_factor_rear
  servoMS[RFF][0][0] = (gaitHome[RFF] - (move_steps * fmfact));
  servoMS[RRF][0][0] = (gaitHome[RRF] - ((move_steps * fmfact) * step_weight_factor_rear));
  servoMS[LFF][0][0] = (gaitHome[LFF] + (move_steps * fmfact));
  servoMS[LRF][0][0] = (gaitHome[LRF] + ((move_steps * fmfact) * step_weight_factor_rear));
  servoMS[RFF][0][1] = servoMS[LFF][0][1] = (fsfact * spd_factor);
  servoMS[RRF][0][1] = servoMS[LRF][0][1] = ((fsfact * spd_factor) - (fsfact * (step_weight_factor_rear - 1)));
  servoMS[RFF][1][1] = servoMS[LFF][1][1] = servoMS[RRF][1][1] = servoMS[LRF][1][1] = ((fsfact * spd_factor) / sfact_div);

  //adjust step_weight_factor_rear on direction
  if (ydir < -1) { //reverse
    servoMS[RFF][0][0] = (gaitHome[RFF] - (move_steps * fmfact));
    servoMS[RRF][0][0] = (gaitHome[RRF] - ((move_steps * fmfact) * step_weight_factor_rear));
    servoMS[LFF][0][0] = (gaitHome[LFF] + (move_steps * fmfact));
    servoMS[LRF][0][0] = (gaitHome[LRF] + ((move_steps * fmfact) * step_weight_factor_rear));
    servoMS[RRF][0][1] = servoMS[LRF][0][1] = (fsfact * spd_factor);
  } else if (ydir > 1) { //forward
    servoMS[RRF][0][0] = (gaitHome[RRF] - (move_steps * fmfact));
    servoMS[LRF][0][0] = (gaitHome[LRF] + (move_steps * fmfact));
    servoMS[RRF][0][1] = servoMS[LRF][0][1] = (fsfact * spd_factor);
  }


  //tibias
  //adjust move factor on direction
  if (ydir < -1) { //reverse
    tmfact = (tmfact - (abs(ydir) * 0.015));
  } else if (ydir > 1) { //forward
    tmfact = (tmfact + (ydir * 0.015));
  }
  //set default marching steps/speed
  servoMS[RFT][0][0] = (gaitHome[RFT] + (move_steps * tmfact));
  servoMS[RRT][0][0] = (gaitHome[RRT] + ((move_steps * tmfact) * step_weight_factor_rear));
  servoMS[LFT][0][0] = (gaitHome[LFT] - (move_steps * tmfact));
  servoMS[LRT][0][0] = (gaitHome[LRT] - ((move_steps * tmfact) * step_weight_factor_rear));
  servoMS[RFT][0][1] = servoMS[RRT][0][1] = servoMS[LFT][0][1] = servoMS[LRT][0][1] = (tsfact * spd_factor);
  servoMS[RFT][1][1] = servoMS[RRT][1][1] = servoMS[LFT][1][1] = servoMS[LRT][1][1] = ((tsfact * spd_factor) / sfact_div);


  //STEP SEQUENCING
  //to start sequencer, step first paired legs upwards (SEQ1)
  if (!activeServo[RFF] && !activeServo[RFT] && !servoSequence[RF]) {
    //SEQ1 RF
    update_sequencer(RF, RFT, servoMS[RFT][0][1], servoMS[RFT][0][0], (servoSequence[RF] + 1), 0);
    update_sequencer(RF, RFF, servoMS[RFF][0][1], servoMS[RFF][0][0], servoSequence[RF], 0);
    update_sequencer(RF, RFC, servoMS[RFC][0][1], servoMS[RFC][0][0], servoSequence[RF], 0);

    //SEQ1 LR
    update_sequencer(LR, LRT, servoMS[LRT][0][1], servoMS[LRT][0][0], (servoSequence[LR] + 1), 0);
    update_sequencer(LR, LRF, servoMS[LRF][0][1], servoMS[LRF][0][0], servoSequence[LR], 0);
    update_sequencer(LR, LRC, servoMS[LRC][0][1], servoMS[LRC][0][0], servoSequence[LR], 0);
  }

  //when first paired complete, step second paired legs upwards (SEQ1), and first paired legs downwards (SEQ2)
  if (!activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 1) {
    //SEQ1 RR
    update_sequencer(RR, RRT, servoMS[RRT][0][1], servoMS[RRT][0][0], (servoSequence[RR] + 1), 0);
    update_sequencer(RR, RRF, servoMS[RRF][0][1], servoMS[RRF][0][0], servoSequence[RR], 0);
    update_sequencer(RR, RRC, servoMS[RRC][0][1], servoMS[RRC][0][0], servoSequence[RR], 0);

    //SEQ1 LF
    update_sequencer(LF, LFT, servoMS[LFT][0][1], servoMS[LFT][0][0], (servoSequence[LF] + 1), 0);
    update_sequencer(LF, LFF, servoMS[LFF][0][1], servoMS[LFF][0][0], servoSequence[LF], 0);
    update_sequencer(LF, LFC, servoMS[LFC][0][1], servoMS[LFC][0][0], servoSequence[LF], 0);

    //SEQ2 RF
    update_sequencer(RF, RFT, servoMS[RFT][1][1], gaitHome[RFT], (servoSequence[RF] + 1), 0);
    update_sequencer(RF, RFF, servoMS[RFF][1][1], gaitHome[RFF], servoSequence[RF], 0);
    update_sequencer(RF, RFC, servoMS[RFC][1][1], gaitHome[RFC], servoSequence[RF], 0);

    //SEQ2 LR
    update_sequencer(LR, LRT, servoMS[LRT][1][1], gaitHome[LRT], (servoSequence[LR] + 1), 0);
    update_sequencer(LR, LRF, servoMS[LRF][1][1], gaitHome[LRF], servoSequence[LR], 0);
    update_sequencer(LR, LRC, servoMS[LRC][1][1], gaitHome[LRC], servoSequence[LR], 0);
  }

  //when second paired complete, step second paired legs downward (SEQ2), and step first paired legs home (SEQ3)
  if (!activeServo[RRC] && !activeServo[RRF] && !activeServo[RRT] && servoSequence[RR] == 1) {
    //SEQ2 RR
    update_sequencer(RR, RRT, servoMS[RRT][1][1], gaitHome[RRT], (servoSequence[RR] + 1), 0);
    update_sequencer(RR, RRF, servoMS[RRF][1][1], gaitHome[RRF], servoSequence[RR], 0);
    update_sequencer(RR, RRC, servoMS[RRC][1][1], gaitHome[RRC], servoSequence[RR], 0);

    //SEQ2 LF
    update_sequencer(LF, LFT, servoMS[LFT][1][1], gaitHome[LFT], (servoSequence[LF] + 1), 0);
    update_sequencer(LF, LFF, servoMS[LFF][1][1], gaitHome[LFF], servoSequence[LF], 0);
    update_sequencer(LF, LFC, servoMS[LFC][1][1], gaitHome[LFC], servoSequence[LF], 0);

    //reset sequencers
    for (int i = 0; i < TOTAL_LEGS; i++) {
      servoSequence[i] = 0;
    }
    moving = 0;

if (test_loops != 0) {
  test_steps++;
  Serial.print("step: ");Serial.print(test_steps);

  if (test_loops > 1) {  
    test_loops--;
    if (test_loops == 100) {
      Serial.print("\toled rgb...");
      test_rgb();
    }
    Serial.print("\tloop: ");Serial.println(test_loops);
  } else {
    test_loops = 200;  
    Serial.print("\toled test...");
    test_oled();
    Serial.print("\tloop: ");Serial.println(test_loops);
  }
}

  
  }

}


void step_trot(int xdir, int ydir, int zdir) {

  //set ramp
  ramp_dist = 0.3;
  ramp_spd = 1.3;
  use_ramp = 1;    

  float sinc0 = mapfloat(ydir, y_dir_steps[0], y_dir_steps[1], 0.2, 1.5);
  float sinc1 = mapfloat(ydir, y_dir_steps[0], y_dir_steps[1], 0.7, 2.25);

  float sinc2 = (sinc0 * 1.4);
  float sinc3 = (sinc1 * 1.4);

  //apply zdir
  //define move z-factors
  float czmfact = 0;
  float fzmfact = 0.8;
  float tzmfact = 1.525;

  //calculate zdir positions
  float tz = (zdir * tzmfact);
  float fz = (zdir * fzmfact);
  float cz = czmfact;

  //define home positions 
  //DEVNOTE: this is a bit ugly, but necessary to manipulate home/start position based on zdir
  //         if continue to use and no better solution, create a function for it
  //
  float gaitHome[TOTAL_SERVOS];
  for (int i = 0; i < TOTAL_SERVOS; i++) {
    gaitHome[i] = servoHome[i];
  }

  //pre-apply z factors by direction, not mid-sequence
  if (zdir < -1) {
    gaitHome[RFT] += abs(tz);
    gaitHome[RFF] -= abs(fz);
    gaitHome[RFC] -= abs(cz);
    gaitHome[RRT] += abs(tz);
    gaitHome[RRF] -= abs(fz);
    gaitHome[RRC] -= abs(cz);
    gaitHome[LFT] -= abs(tz);
    gaitHome[LFF] += abs(fz);
    gaitHome[LFC] += abs(cz);
    gaitHome[LRT] -= abs(tz);
    gaitHome[LRF] += abs(fz);
    gaitHome[LRC] += abs(cz);
  } else if (zdir > 1) {
    gaitHome[RFT] -= tz;
    gaitHome[RFF] += fz;
    gaitHome[RFC] += cz;
    gaitHome[RRT] -= tz;
    gaitHome[RRF] += fz;
    gaitHome[RRC] += cz;
    gaitHome[LFT] += tz;
    gaitHome[LFF] -= fz;
    gaitHome[LFC] -= cz;
    gaitHome[LRT] += tz;
    gaitHome[LRF] -= fz;
    gaitHome[LRC] -= cz;
  }


  //set coaxes by direction
  servoStepMoves[RFC][0] = gaitHome[RFC];
  servoStepMoves[LFC][0] = gaitHome[LFC];
  servoStepMoves[RFF][0] = gaitHome[RFF];
  servoStepMoves[LFF][0] = gaitHome[LFF];
  if (xdir < -1) {  //turn left
    xdir = abs(xdir);
    servoStepMoves[RFC][0] = limit_target(RFC, (gaitHome[RFC] - xdir), 15);
    servoStepMoves[LFC][0] = limit_target(LFC, (gaitHome[LFC] - (xdir * 0.6)), 20);
    servoStepMoves[RFF][0] = limit_target(RFF, (gaitHome[RFF] - (xdir * 0.6)), 0);
    servoStepMoves[LFF][0] = limit_target(LFF, (gaitHome[LFF] - (xdir * 2)), 0);
   } else if (xdir > +1) {  //turn right
    servoStepMoves[RFC][0] = limit_target(RFC, (gaitHome[RFC] + (xdir * 0.6)), 20);
    servoStepMoves[LFC][0] = limit_target(LFC, (gaitHome[LFC] + xdir), 15);
    servoStepMoves[RFF][0] = limit_target(RFF, (gaitHome[RFF] + (xdir * 2)), 0);
    servoStepMoves[LFF][0] = limit_target(LFF, (gaitHome[LFF] + (xdir * 0.6)), 0);
  }

  //set tibia sweep movements
  if (!activeSweep[RRT]) {
    update_sequencer(LF, LFC, limit_speed((7 * spd_factor)), servoStepMoves[LFC][0], 0, 0);
    update_sequencer(RF, RFC, limit_speed((7 * spd_factor)), servoStepMoves[RFC][0], 0, 0);
    update_sequencer(LF, LFF, limit_speed((7 * spd_factor)), servoStepMoves[LFF][0], 0, 0);
    update_sequencer(RF, RFF, limit_speed((7 * spd_factor)), servoStepMoves[RFF][0], 0, 0);
    
    servoSpeed[LFT] = limit_speed((7 * spd_factor));
    servoSweep[LFT][0] = limit_target(LFT, (gaitHome[LFT] - (move_steps * sinc0)), 0);
    servoSweep[LFT][1] = limit_target(LFT, (gaitHome[LFT] + (move_steps * sinc1)), 0);
    servoSweep[LFT][2] = 0;
    servoSweep[LFT][3] = 1;
    targetPos[LFT] = servoSweep[LFT][1];
    activeSweep[LFT] = 1;

    servoSpeed[RFT] = limit_speed((7 * spd_factor));
    servoSweep[RFT][0] = limit_target(RFT, (gaitHome[RFT] + (move_steps * sinc0)), 0);
    servoSweep[RFT][1] = limit_target(RFT, (gaitHome[RFT] - (move_steps * sinc1)), 0);
    servoSweep[RFT][2] = 0;
    servoSweep[RFT][3] = 1;
    targetPos[RFT] = servoSweep[RFT][1];
    activeSweep[RFT] = 1;

    servoSpeed[LRT] = limit_speed((7 * spd_factor));
    servoSweep[LRT][0] = limit_target(LRT, (gaitHome[LRT] + (move_steps * sinc2)), 0);
    servoSweep[LRT][1] = limit_target(LRT, (gaitHome[LRT] - (move_steps * sinc3)), 0);
    servoSweep[LRT][2] = 0;
    servoSweep[LRT][3] = 1;
    targetPos[LRT] = servoSweep[LRT][1];
    activeSweep[LRT] = 1;

    servoSpeed[RRT] = limit_speed((7 * spd_factor));
    servoSweep[RRT][0] = limit_target(RRT, (gaitHome[RRT] - (move_steps * sinc2)), 0);
    servoSweep[RRT][1] = limit_target(RRT, (gaitHome[RRT] + (move_steps * sinc3)), 0);
    servoSweep[RRT][2] = 0;
    servoSweep[RRT][3] = 1;
    targetPos[RRT] = servoSweep[RRT][1];
    activeSweep[RRT] = 1;

    if (move_loops) {
      move_loops--;
      if (!move_loops) {
        move_trot = 0;
      }
    }

    lastMoveDelayUpdate = millis();  
  }
}

void step_forward(int ydir, int xdir, int zdir) {

  ydir = map(ydir, 1, y_dir_steps[1], (y_dir_steps[1] * 1.5), 5);  

  int sc = (xdir / 3);
  int s1f = (ydir * 1.0);
  int s1t = (ydir * 2.5);

  int s2f = (ydir * 1.5);
  int s2t = (ydir * 1.0);

  int s3f = (ydir * 2.5);
  int s3t = (ydir * 1.5);

  int s4f = 0;
  int s4t = 0;


  //apply zdir
  //define move z-factors
  float czmfact = 0;
  float fzmfact = 0.8;
  float tzmfact = 1.525;

  //calculate zdir positions
  float tz = (zdir * tzmfact);
  float fz = (zdir * fzmfact);
  float cz = czmfact;

  //define home positions 
  //DEVNOTE: this is a bit ugly, but necessary to manipulate home/start position based on zdir
  //         if continue to use and no better solution, create a function for it
  //
  float gaitHome[TOTAL_SERVOS];
  for (int i = 0; i < TOTAL_SERVOS; i++) {
    gaitHome[i] = servoHome[i];
  }

  //pre-apply z factors by direction, not mid-sequence
  if (zdir < -1) {
    gaitHome[RFT] += abs(tz);
    gaitHome[RFF] -= abs(fz);
    gaitHome[RFC] -= abs(cz);
    gaitHome[RRT] += abs(tz);
    gaitHome[RRF] -= abs(fz);
    gaitHome[RRC] -= abs(cz);
    gaitHome[LFT] -= abs(tz);
    gaitHome[LFF] += abs(fz);
    gaitHome[LFC] += abs(cz);
    gaitHome[LRT] -= abs(tz);
    gaitHome[LRF] += abs(fz);
    gaitHome[LRC] += abs(cz);
  } else if (zdir > 1) {
    gaitHome[RFT] -= tz;
    gaitHome[RFF] += fz;
    gaitHome[RFC] += cz;
    gaitHome[RRT] -= tz;
    gaitHome[RRF] += fz;
    gaitHome[RRC] += cz;
    gaitHome[LFT] += tz;
    gaitHome[LFF] -= fz;
    gaitHome[LFC] -= cz;
    gaitHome[LRT] += tz;
    gaitHome[LRF] -= fz;
    gaitHome[LRC] -= cz;
  }


  //set left or right turn
  int rfturn = (servoHome[RFC] + sc);
  int lfturn = (servoHome[LFC] + sc);

  int rspd = 3;
  int lspd = 3;
  if (xdir > 0) {
    rspd = 0;
    lspd = 6;
    lfturn = (gaitHome[LFC] - (sc * 3));
  } else if (xdir < 0) {
    lspd = 0;
    rspd = 6;
    rfturn = (gaitHome[RFC] - (sc * 3));
  }


  //RF & LR
  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && !servoSequence[RF]) {
    update_sequencer(RF, RFC, (rspd*spd_factor), rfturn, (servoSequence[RF] + 1), 0);
    update_sequencer(RF, RFF, (4*spd_factor), (gaitHome[RFF] - s1f), servoSequence[RF], 0);
    update_sequencer(RF, RFT, (3*spd_factor), (gaitHome[RFT] + s1t), servoSequence[RF], 0);

    update_sequencer(LR, LRC, (3*spd_factor), (gaitHome[LRC]), (servoSequence[LR] + 1), 0);
    update_sequencer(LR, LRF, (4*spd_factor), (gaitHome[LRF] + s1f), servoSequence[LR], 0);
    update_sequencer(LR, LRT, (3*spd_factor), (gaitHome[LRT] - s1t), servoSequence[LR], 0);
  }
  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 1) {
    update_sequencer(RF, RFC, (rspd*spd_factor), rfturn, (servoSequence[RF] + 1), 0);
    update_sequencer(RF, RFF, (3*spd_factor), (gaitHome[RFF] + s2f), servoSequence[RF], 0);
    update_sequencer(RF, RFT, (6*spd_factor), (gaitHome[RFT] + s2t), servoSequence[RF], 0);

    update_sequencer(LR, LRC, (3*spd_factor), (gaitHome[LRC]), (servoSequence[LR] + 1), 0);
    update_sequencer(LR, LRF, (3*spd_factor), (gaitHome[LRF] - s2f), servoSequence[LR], 0);
    update_sequencer(LR, LRT, (6*spd_factor), (gaitHome[LRT] - s2t), servoSequence[LR], 0);
  }
  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 2) {
    update_sequencer(RF, RFC, (rspd*spd_factor), rfturn, (servoSequence[RF] + 1), 0);
    update_sequencer(RF, RFF, (3*spd_factor), (gaitHome[RFF] + s3f), servoSequence[RF], 0);
    update_sequencer(RF, RFT, (3*spd_factor), (gaitHome[RFT] - s3t), servoSequence[RF], 0);

    update_sequencer(LR, LRC, (3*spd_factor), (gaitHome[LRC]), (servoSequence[LR] + 1), 0);
    update_sequencer(LR, LRF, (3*spd_factor), (gaitHome[LRF] - s3f), servoSequence[LR], 0);
    update_sequencer(LR, LRT, (3*spd_factor), (gaitHome[LRT] + s3t), servoSequence[LR], 0);
  }
  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 3) {
    update_sequencer(RF, RFC, (3*spd_factor), gaitHome[RFC], 0, 0);
    update_sequencer(RF, RFF, (3*spd_factor), (gaitHome[RFF] + s4f), 0, 0);
    update_sequencer(RF, RFT, (6*spd_factor), (gaitHome[RFT] - s4t), 0, 0);

    update_sequencer(LR, LRC, (3*spd_factor), gaitHome[LRC], 0, 0);
    update_sequencer(LR, LRF, (3*spd_factor), (gaitHome[LRF] - s4f), 0, 0);
    update_sequencer(LR, LRT, (6*spd_factor), (gaitHome[LRT] + s4t), 0, 0);
  }

  //LF & RR
  if (!activeServo[LFC] && !activeServo[LFF] && !activeServo[LFT] && !servoSequence[LF] && servoSequence[LR] == 3) {
    update_sequencer(RR, RRC, (3*spd_factor), (gaitHome[RRC]), (servoSequence[RR] + 1), 0);
    update_sequencer(RR, RRF, (4*spd_factor), (gaitHome[RRF] - s1f), servoSequence[RR], 0);
    update_sequencer(RR, RRT, (3*spd_factor), (gaitHome[RRT] + s1t), servoSequence[RR], 0);

    update_sequencer(LF, LFC, (lspd*spd_factor), lfturn, (servoSequence[LF] + 1), 0);
    update_sequencer(LF, LFF, (4*spd_factor), (gaitHome[LFF] + s1f), servoSequence[LF], 0);
    update_sequencer(LF, LFT, (3*spd_factor), (gaitHome[LFT] - s1t), servoSequence[LF], 0);
  }
  if (!activeServo[LFC] && !activeServo[LFF] && !activeServo[LFT] && servoSequence[LF] == 1) {
    update_sequencer(RR, RRC, (3*spd_factor), (gaitHome[RRC]), (servoSequence[RR] + 1), 0);
    update_sequencer(RR, RRF, (3*spd_factor), (gaitHome[RRF] + s2f), servoSequence[RR], 0);
    update_sequencer(RR, RRT, (6*spd_factor), (gaitHome[RRT] + s2t), servoSequence[RR], 0);

    update_sequencer(LF, LFC, (lspd*spd_factor), lfturn, (servoSequence[LF] + 1), 0);
    update_sequencer(LF, LFF, (3*spd_factor), (gaitHome[LFF] - s2f), servoSequence[LF], 0);
    update_sequencer(LF, LFT, (6*spd_factor), (gaitHome[LFT] - s2t), servoSequence[LF], 0);
  }
  if (!activeServo[LFC] && !activeServo[LFF] && !activeServo[LFT] && servoSequence[LF] == 2) {
    update_sequencer(RR, RRC, (3*spd_factor), (gaitHome[RRC]), (servoSequence[RR] + 1), 0);
    update_sequencer(RR, RRF, (3*spd_factor), (gaitHome[RRF] + s3f), servoSequence[RR], 0);
    update_sequencer(RR, RRT, (3*spd_factor), (gaitHome[RRT] - s3t), servoSequence[RR], 0);

    update_sequencer(LF, LFC, (lspd*spd_factor), lfturn, (servoSequence[LF] + 1), 0);
    update_sequencer(LF, LFF, (3*spd_factor), (gaitHome[LFF] - s3f), servoSequence[LF], 0);
    update_sequencer(LF, LFT, (3*spd_factor), (gaitHome[LFT] + s3t), servoSequence[LF], 0);
  }
  if (!activeServo[LFC] && !activeServo[LFF] && !activeServo[LFT] && servoSequence[LF] == 3) {
    update_sequencer(RR, RRC, (3*spd_factor), gaitHome[RRC], 0, 0);
    update_sequencer(RR, RRF, (3*spd_factor), (gaitHome[RRF] + s4f), 0, 0);
    update_sequencer(RR, RRT, (6*spd_factor), (gaitHome[RRT] - s4t), 0, 0);

    update_sequencer(LF, LFC, (3*spd_factor), gaitHome[LFC], 0, 0);
    update_sequencer(LF, LFF, (3*spd_factor), (gaitHome[LFF] - s4f), 0, 0);
    update_sequencer(LF, LFT, (6*spd_factor), (gaitHome[LFT] + s4t), 0, 0);

    lastMoveDelayUpdate = millis();  
  }

}


void step_backward(int ydir, int xdir, int zdir) {

  ydir = map(ydir, y_dir_steps[0], -1, -5, (y_dir_steps[0] * 1.5));

  int sc = (xdir / 3);
  int s1f = 15 - ydir;
  int s1t = 35 - ydir;

  int s2f = 20 - ydir;
  int s2t = 15 - ydir;

  int s3f = 30 - ydir;
  int s3t = 10 - ydir;

  int s4f = 0;
  int s4t = 0;


  //apply zdir
  //define move z-factors
  float czmfact = 0;
  float fzmfact = 0.8;
  float tzmfact = 1.525;

  //calculate zdir positions
  float tz = (zdir * tzmfact);
  float fz = (zdir * fzmfact);
  float cz = czmfact;

  //define home positions 
  //DEVNOTE: this is a bit ugly, but necessary to manipulate home/start position based on zdir
  //         if continue to use and no better solution, create a function for it
  //
  float gaitHome[TOTAL_SERVOS];
  for (int i = 0; i < TOTAL_SERVOS; i++) {
    gaitHome[i] = servoHome[i];
  }

  //pre-apply z factors by direction, not mid-sequence
  if (zdir < -1) {
    gaitHome[RFT] += abs(tz);
    gaitHome[RFF] -= abs(fz);
    gaitHome[RFC] -= abs(cz);
    gaitHome[RRT] += abs(tz);
    gaitHome[RRF] -= abs(fz);
    gaitHome[RRC] -= abs(cz);
    gaitHome[LFT] -= abs(tz);
    gaitHome[LFF] += abs(fz);
    gaitHome[LFC] += abs(cz);
    gaitHome[LRT] -= abs(tz);
    gaitHome[LRF] += abs(fz);
    gaitHome[LRC] += abs(cz);
  } else if (zdir > 1) {
    gaitHome[RFT] -= tz;
    gaitHome[RFF] += fz;
    gaitHome[RFC] += cz;
    gaitHome[RRT] -= tz;
    gaitHome[RRF] += fz;
    gaitHome[RRC] += cz;
    gaitHome[LFT] += tz;
    gaitHome[LFF] -= fz;
    gaitHome[LFC] -= cz;
    gaitHome[LRT] += tz;
    gaitHome[LRF] -= fz;
    gaitHome[LRC] -= cz;
  }

  int rturn = 1;
  int lturn = 1;
  int rspd = 3;
  int lspd = 3;
  if (xdir > 0) {
    rspd = 0;
    lspd = 6;
    lturn = 3;
  } else if (xdir < 0) {
    lspd = 0;
    rspd = 6;
    rturn = 3;
  }

  //RF & LR
  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && !servoSequence[RF]) {
    update_sequencer(RF, RFC, (3*spd_factor), (gaitHome[RFC]), (servoSequence[RF] + 1), 0);
    update_sequencer(RF, RFF, (4*spd_factor), (gaitHome[RFF] - s1f), servoSequence[RF], 0);
    update_sequencer(RF, RFT, (3*spd_factor), (gaitHome[RFT] + s1t), servoSequence[RF], 0);

    update_sequencer(LR, LRC, (lspd*spd_factor), (gaitHome[LRC] + (sc * rturn)), (servoSequence[LR] + 1), 0);
    update_sequencer(LR, LRF, (4*spd_factor), (gaitHome[LRF] + s1f), servoSequence[LR], 0);
    update_sequencer(LR, LRT, (3*spd_factor), (gaitHome[LRT] - s1t), servoSequence[LR], 0);
  }
  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 1) {
    update_sequencer(RF, RFC, (3*spd_factor), (gaitHome[RFC]), (servoSequence[RF] + 1), 0);
    update_sequencer(RF, RFF, (3*spd_factor), (gaitHome[RFF] + s2f), servoSequence[RF], 0);
    update_sequencer(RF, RFT, (6*spd_factor), (gaitHome[RFT] + s2t), servoSequence[RF], 0);

    update_sequencer(LR, LRC, (lspd*spd_factor), (gaitHome[LRC] + (sc * rturn)), (servoSequence[LR] + 1), 0);
    update_sequencer(LR, LRF, (3*spd_factor), (gaitHome[LRF] - s2f), servoSequence[LR], 0);
    update_sequencer(LR, LRT, (6*spd_factor), (gaitHome[LRT] - s2t), servoSequence[LR], 0);
  }
  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 2) {
    update_sequencer(RF, RFC, (3*spd_factor), (gaitHome[RFC]), (servoSequence[RF] + 1), 0);
    update_sequencer(RF, RFF, (3*spd_factor), (gaitHome[RFF] + s3f), servoSequence[RF], 0);
    update_sequencer(RF, RFT, (3*spd_factor), (gaitHome[RFT] - s3t), servoSequence[RF], 0);

    update_sequencer(LR, LRC, (lspd*spd_factor), (gaitHome[LRC] + (sc * rturn)), (servoSequence[LR] + 1), 0);
    update_sequencer(LR, LRF, (3*spd_factor), (gaitHome[LRF] - s3f), servoSequence[LR], 0);
    update_sequencer(LR, LRT, (3*spd_factor), (gaitHome[LRT] + s3t), servoSequence[LR], 0);
  }
  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 3) {
    update_sequencer(RF, RFC, (3*spd_factor), gaitHome[RFC], 0, 0);
    update_sequencer(RF, RFF, (10*spd_factor), (gaitHome[RFF] + s4f), 0, 0);
    update_sequencer(RF, RFT, (15*spd_factor), (gaitHome[RFT] - s4t), 0, 0);

    update_sequencer(LR, LRC, (3*spd_factor), gaitHome[LRC], 0, 0);
    update_sequencer(LR, LRF, (10*spd_factor), (gaitHome[LRF] - s4f), 0, 0);
    update_sequencer(LR, LRT, (15*spd_factor), (gaitHome[LRT] + s4t), 0, 0);
  }

  //LF & RR
  if (!activeServo[LFC] && !activeServo[LFF] && !activeServo[LFT] && !servoSequence[LF] && servoSequence[LR] == 3) {
    update_sequencer(RR, RRC, (rspd*spd_factor), (gaitHome[RRC] + (sc * lturn)), (servoSequence[RR] + 1), 0);
    update_sequencer(RR, RRF, (4*spd_factor), (gaitHome[RRF] - s1f), servoSequence[RR], 0);
    update_sequencer(RR, RRT, (3*spd_factor), (gaitHome[RRT] + s1t), servoSequence[RR], 0);

    update_sequencer(LF, LFC, (3*spd_factor), (gaitHome[LFC]), (servoSequence[LF] + 1), 0);
    update_sequencer(LF, LFF, (4*spd_factor), (gaitHome[LFF] + s1f), servoSequence[LF], 0);
    update_sequencer(LF, LFT, (3*spd_factor), (gaitHome[LFT] - s1t), servoSequence[LF], 0);
  }
  if (!activeServo[LFC] && !activeServo[LFF] && !activeServo[LFT] && servoSequence[LF] == 1) {
    update_sequencer(RR, RRC, (rspd*spd_factor), (gaitHome[RRC] + (sc * lturn)), (servoSequence[RR] + 1), 0);
    update_sequencer(RR, RRF, (3*spd_factor), (gaitHome[RRF] + s2f), servoSequence[RR], 0);
    update_sequencer(RR, RRT, (6*spd_factor), (gaitHome[RRT] + s2t), servoSequence[RR], 0);

    update_sequencer(LF, LFC, (3*spd_factor), (gaitHome[LFC]), (servoSequence[LF] + 1), 0);
    update_sequencer(LF, LFF, (3*spd_factor), (gaitHome[LFF] - s2f), servoSequence[LF], 0);
    update_sequencer(LF, LFT, (6*spd_factor), (gaitHome[LFT] - s2t), servoSequence[LF], 0);
  }
  if (!activeServo[LFC] && !activeServo[LFF] && !activeServo[LFT] && servoSequence[LF] == 2) {
    update_sequencer(RR, RRC, (rspd*spd_factor), (gaitHome[RRC] + (sc * lturn)), (servoSequence[RR] + 1), 0);
    update_sequencer(RR, RRF, (3*spd_factor), (gaitHome[RRF] + s3f), servoSequence[RR], 0);
    update_sequencer(RR, RRT, (3*spd_factor), (gaitHome[RRT] - s3t), servoSequence[RR], 0);

    update_sequencer(LF, LFC, (3*spd_factor), (gaitHome[LFC]), (servoSequence[LF] + 1), 0);
    update_sequencer(LF, LFF, (3*spd_factor), (gaitHome[LFF] - s3f), servoSequence[LF], 0);
    update_sequencer(LF, LFT, (3*spd_factor), (gaitHome[LFT] + s3t), servoSequence[LF], 0);
  }
  if (!activeServo[LFC] && !activeServo[LFF] && !activeServo[LFT] && servoSequence[LF] == 3) {
    update_sequencer(RR, RRC, (3*spd_factor), gaitHome[RRC], 0, 0);
    update_sequencer(RR, RRF, (10*spd_factor), (gaitHome[RRF] + s4f), 0, 0);
    update_sequencer(RR, RRT, (15*spd_factor), (gaitHome[RRT] - s4t), 0, 0);

    update_sequencer(LF, LFC, (3*spd_factor), gaitHome[LFC], 0, 0);
    update_sequencer(LF, LFF, (10*spd_factor), (gaitHome[LFF] - s4f), 0, 0);
    update_sequencer(LF, LFT, (15*spd_factor), (gaitHome[LFT] + s4t), 0, 0);

    lastMoveDelayUpdate = millis();  
  }

}


void step_left_right(int lorr, int xdir, int ydir) {   //where x is +right/-left, and y is +forward/-backward
  spd = 12;  //1-10 (scale with move steps)
  move_steps = 30; //20-110

//scale either move_steps or xdir from the other (leg should lift more when greater xdir, and vice-versa)
//should we push down a bit as first "step" before lifting up?

  if (xdir < 0) xdir = 0;
  if (move_steps < 20) move_steps = 20;

  int rspd_c = limit_speed(spd);
  int rspd_f = limit_speed(spd);
  int rspd_t = limit_speed(spd * 0.5);

  int lspd_c = limit_speed(spd);
  int lspd_f = limit_speed(spd);
  int lspd_t = limit_speed(spd * 0.5);

  servoStepMoves[RFF][0] = limit_target(RFF, (servoHome[RFF] - (0.5 * move_steps)), 10);
  servoStepMoves[RFT][0] = limit_target(RFT, (servoHome[RFT] + (1 * move_steps)), (0.5 * 10));
  servoStepMoves[RRF][0] = limit_target(RRF, (servoHome[RRF] - (0.5 * move_steps)), 10);
  servoStepMoves[RRT][0] = limit_target(RRT, (servoHome[RRT] + (1 * move_steps)), (0.5 * 10));
  servoStepMoves[RFF][2] = servoHome[RFF];
  servoStepMoves[RFT][2] = servoHome[RFT];
  servoStepMoves[RRF][2] = servoHome[RRF];
  servoStepMoves[RRT][2] = servoHome[RRT];

  servoStepMoves[LFF][0] = limit_target(LFF, (servoHome[LFF] + (0.5 * move_steps)), 10);
  servoStepMoves[LFT][0] = limit_target(LFT, (servoHome[LFT] - (1 * move_steps)), (0.5 * 10));
  servoStepMoves[LRF][0] = limit_target(LRF, (servoHome[LRF] + (0.5 * move_steps)), 10);
  servoStepMoves[LRT][0] = limit_target(LRT, (servoHome[LRT] - (1 * move_steps)), (0.5 * 10));
  servoStepMoves[LFF][2] = servoHome[LFF];
  servoStepMoves[LFT][2] = servoHome[LFT];
  servoStepMoves[LRF][2] = servoHome[LRF];
  servoStepMoves[LRT][2] = servoHome[LRT];

  if (lorr == 1) {  //step left
    if (xdir > 1) {
      lspd_f = limit_speed(spd * 1.5);
      lspd_t = limit_speed(spd * 0.75);
      servoStepMoves[LFF][0] = limit_target(LFF, (servoStepMoves[LFF][0] + (0.25 * move_steps)), 10);
      servoStepMoves[LFT][0] = limit_target(LFT, (servoStepMoves[LFT][0] - (0.5 * move_steps)), (0.5 * 10));
      servoStepMoves[LRF][0] = limit_target(LRF, (servoStepMoves[LRF][0] + (0.25 * move_steps)), 10);
      servoStepMoves[LRT][0] = limit_target(LRT, (servoStepMoves[LRT][0] - (0.5 * move_steps)), (0.5 * 10));
      servoStepMoves[LFF][2] = servoHome[LFF] + (0.125 * move_steps);
      servoStepMoves[LFT][2] = servoHome[LFT] - (0.3625 * move_steps);
      servoStepMoves[LRF][2] = servoHome[LRF] + (0.125 * move_steps);
      servoStepMoves[LRT][2] = servoHome[LRT] - (0.3625 * move_steps);

      servoStepMoves[RFF][0] = limit_target(RFF, (servoStepMoves[RFF][0] + (0.25 * move_steps)), 10);
      servoStepMoves[RFT][0] = limit_target(RFT, (servoStepMoves[RFT][0] - (0.5 * move_steps)), (0.5 * 10));
      servoStepMoves[RRF][0] = limit_target(RRF, (servoStepMoves[RRF][0] + (0.25 * move_steps)), 10);
      servoStepMoves[RRT][0] = limit_target(RRT, (servoStepMoves[RRT][0] - (0.5 * move_steps)), (0.5 * 10));
      servoStepMoves[RFF][2] = servoHome[RFF] + (0.5 * move_steps);
      servoStepMoves[RFT][2] = servoHome[RFT] - (move_steps);
      servoStepMoves[RRF][2] = servoHome[RRF] + (0.5 * move_steps);
      servoStepMoves[RRT][2] = servoHome[RRT] - (move_steps);
    }
  } else {  //step right
    if (xdir > 1) {
      rspd_f = limit_speed(spd * 1.5);
      rspd_t = limit_speed(spd * 0.75);
      servoStepMoves[RFF][0] = limit_target(RFF, (servoStepMoves[RFF][0] - (0.25 * move_steps)), 10);
      servoStepMoves[RFT][0] = limit_target(RFT, (servoStepMoves[RFT][0] + (0.5 * move_steps)), (0.5 * 10));
      servoStepMoves[RRF][0] = limit_target(RRF, (servoStepMoves[RRF][0] - (0.25 * move_steps)), 10);
      servoStepMoves[RRT][0] = limit_target(RRT, (servoStepMoves[RRT][0] + (0.5 * move_steps)), (0.5 * 10));
      servoStepMoves[RFF][2] = servoHome[RFF] - (0.125 * move_steps);
      servoStepMoves[RFT][2] = servoHome[RFT] + (0.3625 * move_steps);
      servoStepMoves[RRF][2] = servoHome[RRF] - (0.125 * move_steps);
      servoStepMoves[RRT][2] = servoHome[RRT] + (0.3625 * move_steps);

      servoStepMoves[LFF][0] = limit_target(LFF, (servoStepMoves[LFF][0] - (0.25 * move_steps)), 10);
      servoStepMoves[LFT][0] = limit_target(LFT, (servoStepMoves[LFT][0] + (0.5 * move_steps)), (0.5 * 10));
      servoStepMoves[LRF][0] = limit_target(LRF, (servoStepMoves[LRF][0] - (0.25 * move_steps)), 10);
      servoStepMoves[LRT][0] = limit_target(LRT, (servoStepMoves[LRT][0] + (0.5 * move_steps)), (0.5 * 10));
      servoStepMoves[LFF][2] = servoHome[LFF] - (0.5 * move_steps);
      servoStepMoves[LFT][2] = servoHome[LFT] + (move_steps);
      servoStepMoves[LRF][2] = servoHome[LRF] - (0.5 * move_steps);
      servoStepMoves[LRT][2] = servoHome[LRT] + (move_steps);
    }
  }

  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && !servoSequence[RF]) {
    update_sequencer(RF, RFC, rspd_c, servoHome[RFC], (servoSequence[RF] + 1), 0);
    update_sequencer(RF, RFT, rspd_t, servoStepMoves[RFT][0], servoSequence[RF], 0);
    update_sequencer(RF, RFF, rspd_f, servoStepMoves[RFF][0], servoSequence[RF], 0);
    update_sequencer(RR, RRC, rspd_c, servoHome[RRC], (servoSequence[RR] + 1), 0);
    update_sequencer(RR, RRT, rspd_t, servoStepMoves[RRT][0], servoSequence[RR], 0);
    update_sequencer(RR, RRF, rspd_f, servoStepMoves[RRF][0], servoSequence[RR], 0);
  }

  if (!activeServo[RFC] && servoSequence[RF] == 1) {
    if (lorr == 1) {  //step left
      if (xdir < 1) {
        servoStepMoves[RFC][1] = servoHome[RFC];
        servoStepMoves[RRC][1] = servoHome[RRC];
      } else {
        servoStepMoves[RFC][1] = (servoHome[RFC] + xdir);
        servoStepMoves[RRC][1] = (servoHome[RRC] + xdir);
      }
    } else {  //step right
      if (!xdir) {
        servoStepMoves[RFC][1] = servoHome[RFC];
        servoStepMoves[RRC][1] = servoHome[RRC];
      } else {
        servoStepMoves[RFC][1] = (servoHome[RFC] - xdir);
        servoStepMoves[RRC][1] = (servoHome[RRC] - xdir);
      }
    }
    update_sequencer(RF, RFC, (rspd_c), servoStepMoves[RFC][1], (servoSequence[RF] + 1), 0);
    update_sequencer(RR, RRC, (rspd_c), servoStepMoves[RRC][1], (servoSequence[RR] + 1), 0);
  }

  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 2) {
    update_sequencer(RF, RFC, (rspd_c/3), servoPos[RFC], (servoSequence[RF] + 1), 0);
    update_sequencer(RF, RFT, (rspd_t/3), servoHome[RFT], servoSequence[RF], 0);
    update_sequencer(RF, RFF, (rspd_f/3), servoHome[RFF], servoSequence[RF], 0);
    update_sequencer(RR, RRC, (rspd_c/3), servoPos[RRC], (servoSequence[RR] + 1), 0);
    update_sequencer(RR, RRT, (rspd_t/3), servoHome[RRT], servoSequence[RR], 0);
    update_sequencer(RR, RRF, (rspd_f/3), servoHome[RRF], servoSequence[RR], 0);
  }

  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 3) {
    //Serial.println("\treset");
    update_sequencer(RF, RFC, rspd_c, servoHome[RFC], 0, 0);
    update_sequencer(RF, RFT, rspd_t, servoHome[RFT], 0, 0);
    update_sequencer(RF, RFF, rspd_f, servoHome[RFF], 0, 0);
    update_sequencer(RR, RRC, rspd_c, servoHome[RRC], 0, 0);
    update_sequencer(RR, RRT, rspd_t, servoHome[RRT], 0, 0);
    update_sequencer(RR, RRF, rspd_f, servoHome[RRF], 0, 0);
  }

  if (!activeServo[LFC] && !activeServo[LFF] && !activeServo[LFT] && !servoSequence[LF] && servoSequence[RF] == 3) {
    update_sequencer(LF, LFC, lspd_c, servoHome[LFC], (servoSequence[LF] + 1), 0);
    update_sequencer(LF, LFT, lspd_t, servoStepMoves[LFT][0], servoSequence[LF], 0);
    update_sequencer(LF, LFF, lspd_f, servoStepMoves[LFF][0], servoSequence[LF], 0);
    update_sequencer(LR, LRC, lspd_c, servoHome[LRC], (servoSequence[LR] + 1), 0);
    update_sequencer(LR, LRT, lspd_t, servoStepMoves[LRT][0], servoSequence[LR], 0);
    update_sequencer(LR, LRF, lspd_f, servoStepMoves[LRF][0], servoSequence[LR], 0);  
  }

  if (!activeServo[LFC] && servoSequence[LF] == 1) {
    if (lorr == 1) {  //step left
      if (xdir < 1) {
        servoStepMoves[LFC][1] = servoHome[LFC];
        servoStepMoves[LRC][1] = servoHome[LRC];
      } else {
        servoStepMoves[LFC][1] = (servoHome[LFC] + xdir);
        servoStepMoves[LRC][1] = (servoHome[LRC] + xdir);
      }
    } else {  //step right
      if (xdir < 1) {
        servoStepMoves[LFC][1] = servoHome[LFC];
        servoStepMoves[LRC][1] = servoHome[LRC];
      } else {
        servoStepMoves[LFC][1] = (servoHome[LFC] - xdir);
        servoStepMoves[LRC][1] = (servoHome[LRC] - xdir);
      }
    }
    update_sequencer(LF, LFC, (lspd_c), servoStepMoves[LFC][1], (servoSequence[LF] + 1), 0);
    update_sequencer(LR, LRC, (lspd_c), servoStepMoves[LRC][1], (servoSequence[LR] + 1), 0);
  }

  if (!activeServo[LFC] && !activeServo[LFF] && !activeServo[LFT] && servoSequence[LF] == 2) {
    update_sequencer(LF, LFC, (lspd_c/3), servoPos[LFC], (servoSequence[LF] + 1), 0);
    update_sequencer(LF, LFT, (lspd_t/3), servoHome[LFT], servoSequence[LF], 0);
    update_sequencer(LF, LFF, (lspd_f/3), servoHome[LFF], servoSequence[LF], 0);
    update_sequencer(LR, LRC, (lspd_c/3), servoPos[LRC], (servoSequence[LR] + 1), 0);
    update_sequencer(LR, LRT, (lspd_t/3), servoHome[LRT], servoSequence[LR], 0);
    update_sequencer(LR, LRF, (lspd_f/3), servoHome[LRF], servoSequence[LR], 0);
  }
  
  if (!activeServo[LFC] && !activeServo[LFF] && !activeServo[LFT] && servoSequence[LF] == 3) {
    //Serial.println("\treset");
    update_sequencer(LF, LFC, lspd_c, servoHome[LFC], 0, 0);
    update_sequencer(LF, LFT, lspd_t, servoHome[LFT], 0, 0);
    update_sequencer(LF, LFF, lspd_f, servoHome[LFF], 0, 0);
    update_sequencer(LR, LRC, lspd_c, servoHome[LRC], 0, 0);
    update_sequencer(LR, LRT, lspd_t, servoHome[LRT], 0, 0);
    update_sequencer(LR, LRF, lspd_f, servoHome[LRF], 0, 0);

    lastMoveDelayUpdate = millis();  

    if (move_loops) {
      move_loops--;
      if (!move_loops) {
        move_left = 0;
        move_right = 0;
      }
    }
  
  }
}




void wake() {
  servoStepMoves[RFF][0] = 15;
  servoStepMoves[LRF][0] = 15;
  servoStepMoves[LFF][0] = 15;
  servoStepMoves[RRF][0] = 15;
  servoStepMoves[RFT][0] = 20;
  servoStepMoves[LRT][0] = 20;
  servoStepMoves[LFT][0] = 20;
  servoStepMoves[RRT][0] = 20;

  if (move_loops) {
    //RF
    if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && !servoSequence[RF]) {
      if (move_loops) {
        if (!move_switch) {
          if (move_loops == 1) {
            move_loops = 10;
            move_switch = 1;
          }
          servoStepMoves[RFC][0] = 0;
          servoStepMoves[LRC][0] = 0;
          servoStepMoves[LFC][0] = 0;
          servoStepMoves[RRC][0] = 0;
        } else if (move_switch == 2) {
          servoStepMoves[RFC][0] = 0;
          servoStepMoves[LRC][0] = 0;
          servoStepMoves[LFC][0] = 0;
          servoStepMoves[RRC][0] = 0;
        }
      }

      update_sequencer(RF, RFC, 1, (servoPos[RFC] - servoStepMoves[RFC][0]), (servoSequence[RF] + 1), 0);
      update_sequencer(RF, RFF, 1, (servoHome[RFF] - servoStepMoves[RFF][0]), servoSequence[RF], 0);
      update_sequencer(RF, RFT, 1, (servoHome[RFT] + servoStepMoves[RFT][0]), servoSequence[RF], 0);
    }
    if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 1) {
      update_sequencer(RF, RFC, 1, servoPos[RFC], (servoSequence[RF] + 1), 0);
      update_sequencer(RF, RFF, 1, servoHome[RFF], servoSequence[RF], 0);
      update_sequencer(RF, RFT, 1, servoHome[RFT], servoSequence[RF], 0);
    }

    //LR
    if (!activeServo[LRC] && !activeServo[LRF] && !activeServo[LRT] && !servoSequence[LR] && !activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 2) {
      update_sequencer(LR, LRC, 1, (servoPos[LRC] + servoStepMoves[LRC][0]), (servoSequence[LR] + 1), 0);
      update_sequencer(LR, LRF, 1, (servoHome[LRF] + servoStepMoves[LRF][0]), servoSequence[LR], 0);
      update_sequencer(LR, LRT, 1, ((servoHome[LRT] - servoStepMoves[LRT][0]) - (servoStepMoves[LRT][0]*step_weight_factor_rear)), servoSequence[LR], 0);
    }
    if (!activeServo[LRC] && !activeServo[LRF] && !activeServo[LRT] && servoSequence[LR] == 1) {
      update_sequencer(LR, LRC, 1, servoPos[LRC], (servoSequence[LR] + 1), 0);
      update_sequencer(LR, LRF, 1, servoHome[LRF], servoSequence[LR], 0);
      update_sequencer(LR, LRT, 1, servoHome[LRT], servoSequence[LR], 0);
    }

    //LF
    if (!activeServo[LFC] && !activeServo[LFF] && !activeServo[LFT] && !servoSequence[LF] && !activeServo[LRC] && !activeServo[LRF] && !activeServo[LRT] && servoSequence[LR] == 2) {
      update_sequencer(LF, LFC, 1, (servoPos[LFC] + servoStepMoves[LFC][0]), (servoSequence[LF] + 1), 0);
      update_sequencer(LF, LFF, 1, (servoHome[LFF] + servoStepMoves[LFF][0]), servoSequence[LF], 0);
      update_sequencer(LF, LFT, 1, (servoHome[LFT] - servoStepMoves[LFT][0]), servoSequence[LF], 0);
    }
    if (!activeServo[LFC] && !activeServo[LFF] && !activeServo[LFT] && servoSequence[LF] == 1) {
      update_sequencer(LF, LFC, 1, servoPos[LFC], (servoSequence[LF] + 1), 0);
      update_sequencer(LF, LFF, 1, servoHome[LFF], servoSequence[LF], 0);
      update_sequencer(LF, LFT, 1, servoHome[LFT], servoSequence[LF], 0);
    }

    //RR
    if (!activeServo[RRC] && !activeServo[RRF] && !activeServo[RRT] && !servoSequence[RR] && !activeServo[LFC] && !activeServo[LFF] && !activeServo[LFT] && servoSequence[LF] == 2) {
      update_sequencer(RR, RRC, 1, (servoPos[RRC] - servoStepMoves[RRC][0]), (servoSequence[RR] + 1), 0);
      update_sequencer(RR, RRF, 1, (servoHome[RRF] - servoStepMoves[RRF][0]), servoSequence[RR], 0);
      update_sequencer(RR, RRT, 1, ((servoHome[RRT] + servoStepMoves[RRT][0]) + (servoStepMoves[RRT][0]*step_weight_factor_rear)), servoSequence[RR], 0);
    }
    if (!activeServo[RRC] && !activeServo[RRF] && !activeServo[RRT] && servoSequence[RR] == 1) {
      update_sequencer(RR, RRC, 1, servoPos[RRC], (servoSequence[RR] + 1), 0);
      update_sequencer(RR, RRF, 1, servoHome[RRF], servoSequence[RR], 0);
      update_sequencer(RR, RRT, 1, servoHome[RRT], servoSequence[RR], 0);


      for (int i = 0; i < TOTAL_LEGS; i++) {
        servoSequence[i] = 0;
      }

      move_loops--;
    }

    lastMoveDelayUpdate = millis();  
  } else if (move_switch == 1) {
    move_loops = 9;
    move_switch = 2;
  } else if (!activeServo[RRC] && !activeServo[RRF] && !activeServo[RRT] && !servoSequence[RR]) {
    spd = spd_lock;
    spd_lock = 0;
    set_speed();
    move_wake = 0;
  }
}

void wman() {

  servoStepMoves[RFF][0] = servoHome[RFF] - 17;
  servoStepMoves[RFT][0] = servoHome[RFT] + 77;
  servoStepMoves[RFF][1] = servoHome[RFF] - 32;
  servoStepMoves[RFT][1] = servoHome[RFT] + 77;
  servoStepMoves[RFF][2] = servoHome[RFF] + 44;
  servoStepMoves[RFT][2] = servoHome[RFT] - 14;
  servoStepMoves[RFF][3] = servoHome[RFF] - 12;
  servoStepMoves[RFT][3] = servoHome[RFT] - 4;
  servoStepMoves[RFF][4] = servoHome[RFF] - 24;
  servoStepMoves[RFT][4] = servoHome[RFT] + 5;
  servoStepMoves[RFF][5] = servoHome[RFF] - 16;
  servoStepMoves[RFT][5] = servoHome[RFT] - 8;

  servoStepMoves[LRF][0] = servoHome[LRF] + 17;
  servoStepMoves[LRT][0] = servoHome[LRT] - 77;
  servoStepMoves[LRF][1] = servoHome[LRF] + 32;
  servoStepMoves[LRT][1] = servoHome[LRT] - 77;
  servoStepMoves[LRF][2] = servoHome[LRF] - 44;
  servoStepMoves[LRT][2] = servoHome[LRT] + 14;
  servoStepMoves[LRF][3] = servoHome[LRF] + 12;
  servoStepMoves[LRT][3] = servoHome[LRT] + 4;
  servoStepMoves[LRF][4] = servoHome[LRF] + 24;
  servoStepMoves[LRT][4] = servoHome[LRT] - 5;
  servoStepMoves[LRF][5] = servoHome[LRF] + 16;
  servoStepMoves[LRT][5] = servoHome[LRT] + 8;


  //SEQ 1 : RF & LR
  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && !servoSequence[RF]) {
    update_sequencer(RF, RFF, 5*spd_factor, servoStepMoves[RFF][0], (servoSequence[RF] + 1), 0);
    update_sequencer(RF, RFT, 1*spd_factor, servoStepMoves[RFT][0], servoSequence[RF], 0);

    update_sequencer(LR, LRF, 5*spd_factor, servoStepMoves[LRF][0], (servoSequence[LR] + 1), 0);
    update_sequencer(LR, LRT, 1*spd_factor, servoStepMoves[LRT][0], servoSequence[LR], 0);
  }
  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 1) {
    update_sequencer(RF, RFF, 7, servoStepMoves[RFF][1], (servoSequence[RF] + 1), 5);
    update_sequencer(RF, RFT, 7, servoStepMoves[RFT][1], servoSequence[RF], 5);

    update_sequencer(LR, LRF, 7, servoStepMoves[LRF][1], (servoSequence[LR] + 1), 5);
    update_sequencer(LR, LRT, 7, servoStepMoves[LRT][1], servoSequence[LR], 5);
  }
  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 2) {
    use_ramp = 0;
    update_sequencer(RF, RFF, 1, servoStepMoves[RFF][2], (servoSequence[RF] + 1), 0);
    update_sequencer(RF, RFT, 0.5, servoStepMoves[RFT][2], servoSequence[RF], 0);

    update_sequencer(LR, LRF, 1, servoStepMoves[LRF][2], (servoSequence[LR] + 1), 0);
    update_sequencer(LR, LRT, 0.5, servoStepMoves[LRT][2], servoSequence[LR], 0);
    use_ramp = 1;
  }
  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 3) {
    update_sequencer(RF, RFF, 14*(0.55*spd_factor), servoStepMoves[RFF][3], (servoSequence[RF] + 1), 0);
    update_sequencer(RF, RFT, 44*(0.55*spd_factor), servoStepMoves[RFT][3], servoSequence[RF], 0);

    update_sequencer(LR, LRF, 14*(0.55*spd_factor), servoStepMoves[LRF][3], (servoSequence[LR] + 1), 0);
    update_sequencer(LR, LRT, 44*(0.55*spd_factor), servoStepMoves[LRT][3], servoSequence[LR], 0);
  }
  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 4) {
    update_sequencer(RF, RFF, 2*spd_factor, servoStepMoves[RFF][4], (servoSequence[RF] + 1), 0);
    update_sequencer(RF, RFT, 1*spd_factor, servoStepMoves[RFT][4], servoSequence[RF], 0);

    update_sequencer(LR, LRF, 2*spd_factor, servoStepMoves[LRF][4], (servoSequence[LR] + 1), 0);
    update_sequencer(LR, LRT, 1*spd_factor, servoStepMoves[LRT][4], servoSequence[LR], 0);
  }
  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 5) {
    use_ramp = 0;
    update_sequencer(RF, RFF, 3, servoStepMoves[RFF][5], (servoSequence[RF] + 1), 0);
    update_sequencer(RF, RFT, 1.5, servoStepMoves[RFT][5], servoSequence[RF], 0);

    update_sequencer(LR, LRF, 3, servoStepMoves[LRF][5], (servoSequence[LR] + 1), 0);
    update_sequencer(LR, LRT, 1.5, servoStepMoves[LRT][5], servoSequence[LR], 0);
    use_ramp = 1;
  }
  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 6) {
    update_sequencer(RF, RFF, 3*spd_factor, servoHome[RFF], 0, 10);
    update_sequencer(RF, RFT, 3*spd_factor, servoHome[RFT], 0, 10);
    update_sequencer(LR, LRF, 3*spd_factor, servoHome[LRF], 0, 10);
    update_sequencer(LR, LRT, 3*spd_factor, servoHome[LRT], 0, 10);
  }


  servoStepMoves[RRF][0] = servoHome[RRF] - 17;
  servoStepMoves[RRT][0] = servoHome[RRT] + 77;
  servoStepMoves[RRF][1] = servoHome[RRF] - 32;
  servoStepMoves[RRT][1] = servoHome[RRT] + 77;
  servoStepMoves[RRF][2] = servoHome[RRF] + 44;
  servoStepMoves[RRT][2] = servoHome[RRT] - 14;
  servoStepMoves[RRF][3] = servoHome[RRF] - 12;
  servoStepMoves[RRT][3] = servoHome[RRT] - 4;
  servoStepMoves[RRF][4] = servoHome[RRF] - 24;
  servoStepMoves[RRT][4] = servoHome[RRT] + 5;
  servoStepMoves[RRF][5] = servoHome[RRF] - 16;
  servoStepMoves[RRT][5] = servoHome[RRT] - 8;

  servoStepMoves[LFF][0] = servoHome[LFF] + 17;
  servoStepMoves[LFT][0] = servoHome[LFT] - 77;
  servoStepMoves[LFF][1] = servoHome[LFF] + 32;
  servoStepMoves[LFT][1] = servoHome[LFT] - 77;
  servoStepMoves[LFF][2] = servoHome[LFF] - 44;
  servoStepMoves[LFT][2] = servoHome[LFT] + 14;
  servoStepMoves[LFF][3] = servoHome[LFF] + 12;
  servoStepMoves[LFT][3] = servoHome[LFT] + 4;
  servoStepMoves[LFF][4] = servoHome[LFF] + 24;
  servoStepMoves[LFT][4] = servoHome[LFT] - 5;
  servoStepMoves[LFF][5] = servoHome[LFF] + 16;
  servoStepMoves[LFT][5] = servoHome[LFT] + 8;


  //SEQ 1 : RR & LF
  if (!activeServo[RRC] && !activeServo[RRF] && !activeServo[RRT] && !servoSequence[RR] && servoSequence[RF] == 4) {
    update_sequencer(RR, RRF, 5*spd_factor, servoStepMoves[RRF][0], (servoSequence[RR] + 1), 0);
    update_sequencer(RR, RRT, 1*spd_factor, servoStepMoves[RRT][0], servoSequence[RR], 0);

    update_sequencer(LF, LFF, 5*spd_factor, servoStepMoves[LFF][0], (servoSequence[LF] + 1), 0);
    update_sequencer(LF, LFT, 1*spd_factor, servoStepMoves[LFT][0], servoSequence[LF], 0);
  }
  if (!activeServo[RFC] && !activeServo[RRF] && !activeServo[RRT] && servoSequence[RR] == 1) {
    update_sequencer(RR, RRF, 7, servoStepMoves[RRF][1], (servoSequence[RR] + 1), 5);
    update_sequencer(RR, RRT, 7, servoStepMoves[RRT][1], servoSequence[RR], 5);

    update_sequencer(LF, LFF, 7, servoStepMoves[LFF][1], (servoSequence[LF] + 1), 5);
    update_sequencer(LF, LFT, 7, servoStepMoves[LFT][1], servoSequence[LF], 5);
  }
  if (!activeServo[RFC] && !activeServo[RRF] && !activeServo[RRT] && servoSequence[RR] == 2) {
    use_ramp = 0;
    update_sequencer(RR, RRF, 1, servoStepMoves[RRF][2], (servoSequence[RR] + 1), 0);
    update_sequencer(RR, RRT, 0.5, servoStepMoves[RRT][2], servoSequence[RR], 0);

    update_sequencer(LF, LFF, 1, servoStepMoves[LFF][2], (servoSequence[LF] + 1), 0);
    update_sequencer(LF, LFT, 0.5, servoStepMoves[LFT][2], servoSequence[LF], 0);
    use_ramp = 1;
  }
  if (!activeServo[RFC] && !activeServo[RRF] && !activeServo[RRT] && servoSequence[RR] == 3) {
    update_sequencer(RR, RRF, 14*(0.55*spd_factor), servoStepMoves[RRF][3], (servoSequence[RR] + 1), 0);
    update_sequencer(RR, RRT, 44*(0.55*spd_factor), servoStepMoves[RRT][3], servoSequence[RR], 0);

    update_sequencer(LF, LFF, 14*(0.55*spd_factor), servoStepMoves[LFF][3], (servoSequence[LF] + 1), 0);
    update_sequencer(LF, LFT, 44*(0.55*spd_factor), servoStepMoves[LFT][3], servoSequence[LF], 0);
  }
  if (!activeServo[RFC] && !activeServo[RRF] && !activeServo[RRT] && servoSequence[RR] == 4) {
    update_sequencer(RR, RRF, 2*spd_factor, servoStepMoves[RRF][4], (servoSequence[RR] + 1), 0);
    update_sequencer(RR, RRT, 1*spd_factor, servoStepMoves[RRT][4], servoSequence[RR], 0);

    update_sequencer(LF, LFF, 2*spd_factor, servoStepMoves[LFF][4], (servoSequence[LF] + 1), 0);
    update_sequencer(LF, LFT, 1*spd_factor, servoStepMoves[LFT][4], servoSequence[LF], 0);
  }
  if (!activeServo[RFC] && !activeServo[RRF] && !activeServo[RRT] && servoSequence[RR] == 5) {
    use_ramp = 0;
    update_sequencer(RR, RRF, 3, servoStepMoves[RRF][5], (servoSequence[RR] + 1), 0);
    update_sequencer(RR, RRT, 1.5, servoStepMoves[RRT][5], servoSequence[RR], 0);

    update_sequencer(LF, LFF, 3, servoStepMoves[LFF][5], (servoSequence[LF] + 1), 0);
    update_sequencer(LF, LFT, 1.5, servoStepMoves[LFT][5], servoSequence[LF], 0);
    use_ramp = 1;
  }
  if (!activeServo[RRC] && !activeServo[RRF] && !activeServo[RRT] && servoSequence[RR] == 6) {
    update_sequencer(RR, RRF, 3*spd_factor, servoHome[RRF], 0, 10);
    update_sequencer(RR, RRT, 3*spd_factor, servoHome[RRT], 0, 10);
    update_sequencer(LF, LFF, 3*spd_factor, servoHome[LFF], 0, 10);
    update_sequencer(LF, LFT, 3*spd_factor, servoHome[LFT], 0, 10);

    lastMoveDelayUpdate = millis();  
  }
}


void move_debug_servo() {
  if (!activeSweep[debug_servo]) {
    servoSpeed[debug_servo] = spd;
    servoSweep[debug_servo][0] = servoLimit[debug_servo][0];
    servoSweep[debug_servo][1] = servoLimit[debug_servo][1];
    servoSweep[debug_servo][2] = 0;
    servoSweep[debug_servo][3] = 1;
    targetPos[debug_servo] = servoSweep[debug_servo][1];
    activeSweep[debug_servo] = 1;

    if (debug_loops2) {
      debug_loops2--;
    }
    if (!debug_loops2) {
      set_stop_active();
      set_home();
    } 
  }
}


void move_debug_leg() {
  //check if leg active
  int lactive = 0;
  for (int i = 0; i < 3; i++) {
    int dservo = servoLeg[debug_leg][i];
    if (activeSweep[dservo]) {
      lactive = 1;
    }
  }

  if (!lactive) {
    for (int i = 0; i < 3; i++) {
      int dservo = servoLeg[debug_leg][i];
      if (!activeSweep[dservo]) {
        servoSpeed[dservo] = debug_spd;
        if (is_front_leg(dservo)) {
          servoSweep[dservo][0] = servoLimit[dservo][1];
          servoSweep[dservo][1] = servoLimit[dservo][0];
          if (is_femur(dservo)) {
            servoSweep[dservo][0] = servoLimit[dservo][0];
            servoSweep[dservo][1] = servoLimit[dservo][1];
          }
        } else {
          servoSweep[dservo][0] = servoLimit[dservo][0];
          servoSweep[dservo][1] = servoLimit[dservo][1];
          if (is_femur(dservo)) {
            servoSweep[dservo][0] = servoLimit[dservo][1];
            servoSweep[dservo][1] = servoLimit[dservo][0];
          }
        }
        servoSweep[dservo][2] = 0;
        servoSweep[dservo][3] = 1;
        targetPos[dservo] = servoSweep[dservo][1];
        activeSweep[dservo] = 1;
      }
    }

    if (debug_loops2) {
      debug_loops2--;
    } 
    if (!debug_loops2) {
      set_stop_active();
      set_home();
    } 
  }
}


/*
   -------------------------------------------------------
   Sequence Processing Functions
   -------------------------------------------------------
*/
void run_sequence() {

  //SEQ 1
  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && !servoSequence[RF]) {
    update_sequencer(RF, RFC, spd_c, servoStepMoves[RFC][0], (servoSequence[RF] + 1), 0);
    update_sequencer(RF, RFF, spd_f, servoStepMoves[RFF][0], servoSequence[RF], 0);
    update_sequencer(RF, RFT, spd_t, servoStepMoves[RFT][0], servoSequence[RF], 0);

    update_sequencer(LF, LFC, spd_c, servoStepMoves[LFC][0], (servoSequence[LF] + 1), 0);
    update_sequencer(LF, LFF, spd_f, servoStepMoves[LFF][0], servoSequence[LF], 0);
    update_sequencer(LF, LFT, spd_t, servoStepMoves[LFT][0], servoSequence[LF], 0);

    update_sequencer(RR, RRC, spd_c, servoStepMoves[RRC][0], (servoSequence[RR] + 1), 0);
    update_sequencer(RR, RRF, spd_f, servoStepMoves[RRF][0], servoSequence[RR], 0);
    update_sequencer(RR, RRT, spd_t, servoStepMoves[RRT][0], servoSequence[RR], 0);

    update_sequencer(LR, LRC, spd_c, servoStepMoves[LRC][0], (servoSequence[LR] + 1), 0);
    update_sequencer(LR, LRF, spd_f, servoStepMoves[LRF][0], servoSequence[LR], 0);
    update_sequencer(LR, LRT, spd_t, servoStepMoves[LRT][0], servoSequence[LR], 0);
  }

  //SEQ 2
  if (!activeServo[LRC] && !activeServo[LRF] && !activeServo[LRT] && servoSequence[LR] == 1) {
    update_sequencer(RF, RFC, spd_c, servoStepMoves[RFC][1], (servoSequence[RF] + 1), move_delay);
    update_sequencer(RF, RFF, spd_f, servoStepMoves[RFF][1], servoSequence[RF], move_delay);
    update_sequencer(RF, RFT, spd_t, servoStepMoves[RFT][1], servoSequence[RF], move_delay);
  
    update_sequencer(LF, LFC, spd_c, servoStepMoves[LFC][1], (servoSequence[LF] + 1), move_delay);
    update_sequencer(LF, LFF, spd_f, servoStepMoves[LFF][1], servoSequence[LF], move_delay);
    update_sequencer(LF, LFT, spd_t, servoStepMoves[LFT][1], servoSequence[LF], move_delay);
  
    update_sequencer(RR, RRC, spd_c, servoStepMoves[RRC][1], (servoSequence[RR] + 1), move_delay);
    update_sequencer(RR, RRF, spd_f, servoStepMoves[RRF][1], servoSequence[RR], move_delay);
    update_sequencer(RR, RRT, spd_t, servoStepMoves[RRT][1], servoSequence[RR], move_delay);
  
    update_sequencer(LR, LRC, spd_c, servoStepMoves[LRC][1], (servoSequence[LR] + 1), move_delay);
    update_sequencer(LR, LRF, spd_f, servoStepMoves[LRF][1], servoSequence[LR], move_delay);
    update_sequencer(LR, LRT, spd_t, servoStepMoves[LRT][1], servoSequence[LR], move_delay);
  }

  //SEQ 3
  if (!activeServo[LRC] && !activeServo[LRF] && !activeServo[LRT] && servoSequence[LR] == 2) {
    update_sequencer(RF, RFC, spd_c, servoStepMoves[RFC][2], (servoSequence[RF] + 1), (move_delay*2));
    update_sequencer(RF, RFF, spd_f, servoStepMoves[RFF][2], servoSequence[RF], (move_delay*2));
    update_sequencer(RF, RFT, spd_t, servoStepMoves[RFT][2], servoSequence[RF], (move_delay*2));
  
    update_sequencer(LF, LFC, spd_c, servoStepMoves[LFC][2], (servoSequence[LF] + 1), (move_delay*2));
    update_sequencer(LF, LFF, spd_f, servoStepMoves[LFF][2], servoSequence[LF], (move_delay*2));
    update_sequencer(LF, LFT, spd_t, servoStepMoves[LFT][2], servoSequence[LF], (move_delay*2));
  
    update_sequencer(RR, RRC, spd_c, servoStepMoves[RRC][2], (servoSequence[RR] + 1), (move_delay*2));
    update_sequencer(RR, RRF, spd_f, servoStepMoves[RRF][2], servoSequence[RR], (move_delay*2));
    update_sequencer(RR, RRT, spd_t, servoStepMoves[RRT][2], servoSequence[RR], (move_delay*2));
  
    update_sequencer(LR, LRC, spd_c, servoStepMoves[LRC][2], (servoSequence[LR] + 1), (move_delay*2));
    update_sequencer(LR, LRF, spd_f, servoStepMoves[LRF][2], servoSequence[LR], (move_delay*2));
    update_sequencer(LR, LRT, spd_t, servoStepMoves[LRT][2], servoSequence[LR], (move_delay*2));
  }

  if (is_stepmove_complete(3) && servoSequence[LR] == 3) {
    if (move_loops) {
      move_loops--;
      for (int l = 0; l < TOTAL_LEGS; l++) {
        servoSequence[0] = 0;
        servoSequence[l] = 0;
        servoSequence[2] = 0;
        servoSequence[3] = 0;
      }
    } else {
      move_sequence = 0;
    }
  }
}

void delay_sequences() {
  int moved = 0;
  int sequence_cnt = 16;

//removed start_stop variable, this was only place it was used, seems defunct
//  if (!start_stop) {
    for (int i = 0; i < sequence_cnt; i++) {
      if (move_delay_sequences[i]) {
        moved = 1;
        if (move_delay_sequences[i] == 1) {
          spd = 12;
          set_speed();
          move_loops = 6;
          move_steps = 20;
          move_x_axis = 1;
          if (debug1)
            Serial.print("move x");
        } else if (move_delay_sequences[i] == 2) {
          set_home();
          spd = 12;
          set_speed();
          move_loops = 3;
          move_steps = 70;
          move_y_axis = 1;
          if (debug1)
            Serial.print("move y large");
        } else if (move_delay_sequences[i] == 13) {
          spd = 1;
          set_speed();
          move_loops = 10;
          move_steps = 15;
          move_y_axis = 1;
          if (debug1)
            Serial.print("move y short");
        } else if (move_delay_sequences[i] == 3) {
          spd = 9;
          set_speed();
          move_loops = 10;
          move_steps = 25;
          move_pitch_body = 1;
          if (debug1)
            Serial.print("move pitch_body");
        } else if (move_delay_sequences[i] == 4) {
          use_ramp = 0;
          spd = 9;
          set_speed();
          move_loops = 10;
          move_steps = 25;
          move_pitch = 1;
          if (debug1)
            Serial.print("move pitch");
        } else if (move_delay_sequences[i] == 5) {
          use_ramp = 1;
          spd = 9;
          set_speed();
          move_loops = 6;
          move_steps = 20;
          move_roll_body = 1;
          if (debug1)
            Serial.print("move rollb");
        } else if (move_delay_sequences[i] == 6) {
          set_home();
          spd = 5;
          set_speed();
          move_loops = 6;
          move_steps = 30;
          move_roll = 1;
          if (debug1)
            Serial.print("move roll");
        } else if (move_delay_sequences[i] == 7) {
          set_home();
          spd = 1;
          set_speed();
          move_loops = 2;
          move_switch = 2;
          for (int i = 0; i < TOTAL_SERVOS; i++) {
            servoPos[i] = servoHome[i];
          }
          move_wake = 1;
          if (debug1)
            Serial.print("move wake");
        } else if (move_delay_sequences[i] == 8) {
          set_crouch();
          if (debug1)
            Serial.print("crouch");
        } else if (move_delay_sequences[i] == 9) {
          set_sit();
          if (debug1)
            Serial.print("sit");
        } else if (move_delay_sequences[i] == 10) {
          move_loops = 1;
          move_steps = 0;
          move_x_axis = 1;
          if (debug1)
            Serial.print("move x 1");
        } else if (move_delay_sequences[i] == 11) {
          set_home();
          y_dir = 0;
          x_dir = 0;
          move_loops = 16;
          move_march = 1;
          if (debug1)
            Serial.print("move march");
        } else if (move_delay_sequences[i] == 12) {
          set_home();
          set_kneel();
          if (debug1)
            Serial.print("move kneel");
        } else if (move_delay_sequences[i] == 14) {
          spd = 1;
          set_speed();
          move_loops = 1;
          move_steps = 30;
          move_look_left = 1;
          if (debug1)
            Serial.print("move look_left");
        } else if (move_delay_sequences[i] == 15) {
          spd = 1;
          set_speed();
          move_loops = 1;
          move_steps = 30;
          move_look_right = 1;
          if (debug1)
            Serial.print("move look_right");
        } else if (move_delay_sequences[i] == 16) {
          set_stay();
          if (debug1)
            Serial.print("stay");
        }
  
        moveDelayInterval = move_delays[i];
        if (debug1) {
          Serial.print("\ti: ");Serial.print(i);Serial.print("\tmove int: ");Serial.println(moveDelayInterval);
        }
        move_delay_sequences[i] = 0;
        i = 16;
      }
    }

    if (!moved) {
      move_demo = 0;
      moveDelayInterval = 0;

      use_ramp = 0;
      spd = 5;
      set_speed();

      servoSweep[RFC][0] = servoHome[RFC];
      servoSweep[RFC][1] = servoLimit[RFC][0];
      servoSweep[RFC][2] = 0;
      servoSweep[RFC][3] = 7;
      targetPos[RFC] = servoSweep[RFC][1];
      activeSweep[RFC] = 1;

      servoSweep[LFC][0] = servoHome[LFC];
      servoSweep[LFC][1] = servoLimit[LFC][0];
      servoSweep[LFC][2] = 0;
      servoSweep[LFC][3] = 7;
      targetPos[LFC] = servoSweep[LFC][1];
      activeSweep[LFC] = 1;

      servoSweep[RFT][0] = servoHome[RFT];
      servoSweep[RFT][1] = servoLimit[RFT][0];
      servoSweep[RFT][2] = 0;
      servoSweep[RFT][3] = 3;
      targetPos[RFT] = servoSweep[RFT][1];
      activeSweep[RFT] = 1;

      servoSweep[LFT][0] = servoHome[LFT];
      servoSweep[LFT][1] = servoLimit[LFT][0];
      servoSweep[LFT][2] = 0;
      servoSweep[LFT][3] = 3;
      targetPos[LFT] = servoSweep[LFT][1];
      activeSweep[LFT] = 1;

//      move_funplay = 1;
      if (debug1)
//        Serial.print("move funplay");

      if (debug1) {
        Serial.println(F("\treset DS"));
      }
    }  
//  }
}

void update_sequencer(int leg, int servo, int sp, float tar, int seq, int del) {
  if (debug3) {
    if (tar) {
      Serial.print("leg: "); Serial.print(leg);
      Serial.print("\tservo: "); Serial.print(servo);
      Serial.print("\tdel: "); Serial.print(del);
      Serial.print("\tpos: "); Serial.print(servoPos[servo]);
      Serial.print("\ttar: "); Serial.print(tar);
      Serial.print("\tseq: "); Serial.println(servoSequence[leg]);
    } else {
      Serial.print(leg); Serial.println(F("-END"));
    }
  }
  servoSpeed[servo] = limit_speed(sp);
  servoSequence[leg] = seq;
  if (tar) {
    servoDelay[servo][0] = del;
    if (del > 0) servoDelay[servo][1] = 1;
    targetPos[servo] = limit_target(servo, tar, 0);
    activeServo[servo] = 1;
  }

  if (use_ramp) {
    set_ramp(servo, servoSpeed[servo], 0, 0, 0, 0);
  }
}



/*
   -------------------------------------------------------
   General Functions
   -------------------------------------------------------
*/
int limit_target(int sid, int tar, int thresh) {
  if (servoLimit[sid][0] > servoLimit[sid][1]) {
    if ((tar + thresh) > servoLimit[sid][0]) {
      tar = (servoLimit[sid][0] - thresh);
    } else if ((tar - thresh) < servoLimit[sid][1]) {
      tar = (servoLimit[sid][1] + thresh);
    }
  } else {
    if ((tar - thresh) < servoLimit[sid][0]) {
      tar = (servoLimit[sid][0] + thresh);
    } else if ((tar + thresh) > servoLimit[sid][1]) {
      tar = (servoLimit[sid][1] - thresh);
    }
  }

  return tar;
}

int limit_speed(float spd_lim) {
  if (spd_lim > min_spd) spd_lim = min_spd;
  if (spd_lim < max_spd) spd_lim = max_spd;

  return spd_lim;
}

byte is_stepmove_complete(int ms) {
  byte ret = 1;
  for (int m = 0; m < TOTAL_SERVOS; m++) {
    if (servoPos[m] == servoStepMoves[m][ms-1]) ret = 0;  
  }

  return ret;
}

byte is_front_leg(int leg) {
  if (leg == LFC || leg == LFF || leg == LFT || leg == RFC || leg == RFF || leg == RFT) 
    return 1;
  else 
    return 0;
}

byte is_left_leg(int leg) {
  if (leg == LFC || leg == LFF || leg == LFT || leg == LRC || leg == LRF || leg == LRT)
    return 1;
  else
    return 0;
}

byte is_femur(int leg) {
  if (leg == RFF || leg == LFF || leg == RRF || leg == LRF)
    return 1;
  else
    return 0;
}

byte is_tibia(int leg) {
  if (leg == RFT || leg == LFT || leg == RRT || leg == LRT)
    return 1;
  else
    return 0;
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int degrees_to_pwm(int pangle, int mxw, int mnw, int rng) {
  int pulse_wide = map(pangle, -rng/2, rng/2, mnw, mxw);

  return pulse_wide;
}

int pwm_to_degrees(int pulse_wide, int mxw, int mnw, int rng) {
  int pangle = map(pulse_wide, mnw, mxw, -rng/2, rng/2);

  return pangle;
}




/*
   -------------------------------------------------------
   Serial Commands
   -------------------------------------------------------
*/
void serial_check() {
  if (serial_active) {
    while (Serial.available()) {
      delay(2);  //delay to allow byte to arrive in input buffer
      char c = Serial.read();
      if (c != ' ' && c != '\n') {  //strip spaces and newlines
        if (c == ',') {  //end command
          serial_command(serial_input);
          serial_input="";
        } else {  //build command
          serial_input += c;
        }
      }
    }
  
    if (serial_input.length() > 0) {
      serial_command(serial_input);
      serial_input="";
    } 
  }
}


void serial_command(String cmd) {
  if (cmd) {
      if (cmd == "stop" || cmd == "0") {
        if (!plotter) Serial.println(F("stop"));
        set_stop_active();
        set_home();
      } else if (cmd == "1") {
        if (!plotter) Serial.println(F("set speed 1"));
        spd = 1;
        set_speed();
      } else if (cmd == "2") {
        if (!plotter) Serial.println(F("set speed 5"));
        spd = 5;
        set_speed();
      } else if (cmd == "3") {
        if (!plotter) Serial.println(F("set speed 10"));
        spd = 10;
        set_speed();
      } else if (cmd == "4") {
        if (!plotter) Serial.println(F("set speed 15"));
        spd = 15;
        set_speed();
      } else if (cmd == "5") {
        if (!plotter) Serial.println(F("set speed 20"));
        spd = 20;
        set_speed();
      } else if (cmd == "6") {
        if (!plotter) Serial.println(F("set speed 25"));
        spd = 25;
        set_speed();
      } else if (cmd == "7") {
        if (!plotter) Serial.println(F("set speed 30"));
        spd = 30;
        set_speed();
      } else if (cmd == "8") {
        if (!plotter) Serial.println(F("set speed 35"));
        spd = 35;
        set_speed();
      } else if (cmd == "9") {
        if (!plotter) Serial.println(F("set speed 40"));
        spd = 40;
        set_speed();
      } else if (cmd == "vars" || cmd == "v") {
        if (!plotter) {
          Serial.println();
          Serial.println(F("---------------------------------------"));
          Serial.println(F("VARS:"));
          Serial.println(F("---------------------------------------"));
          Serial.print(F("\tspd:\t\t\t"));Serial.println(spd);
          Serial.print(F("\tspd_factor:\t\t"));Serial.println(spd_factor);
          Serial.print(F("\tmove_steps:\t\t"));Serial.println(move_steps);
          Serial.print(F("\tx_dir:\t\t\t"));Serial.println(x_dir);
          Serial.print(F("\ty_dir:\t\t\t"));Serial.println(y_dir);
          Serial.print(F("\tz_dir:\t\t\t"));Serial.println(z_dir);
          Serial.print(F("\tstep_weight_factor_rear:\t"));Serial.println(step_weight_factor_rear);
          Serial.print(F("\tstep_height_factor:\t"));Serial.println(step_height_factor);
          Serial.print(F("\tdebug_servo:\t\t"));Serial.println(debug_servo);
          Serial.print(F("\tdebug_leg:\t\t"));Serial.println(debug_leg);
          Serial.print(F("\tvolume:\t\t\t"));Serial.println(sound_vol);
          Serial.println(F("---------------------------------------"));
          Serial.println();
          Serial.println();

          Serial.println(F("---------------------------------------"));
          Serial.println(F("RAMP VARS:"));
          Serial.println(F("---------------------------------------"));
          Serial.print(F("\tspd:\t\t\t")); Serial.println(servoRamp[debug_servo][0]);
          Serial.print(F("\tdist:\t\t\t")); Serial.println(servoRamp[debug_servo][1]);
          Serial.print(F("\tup spd:\t\t\t")); Serial.println(servoRamp[debug_servo][2]);
          Serial.print(F("\tup dist:\t\t")); Serial.println(servoRamp[debug_servo][3]);
          Serial.print(F("\tup spd inc:\t\t")); Serial.println(servoRamp[debug_servo][4]);
          Serial.print(F("\tdn spd:\t\t\t")); Serial.println(servoRamp[debug_servo][5]);
          Serial.print(F("\tdn dist:\t\t")); Serial.println(servoRamp[debug_servo][6]);
          Serial.print(F("\tdn spd inc:\t\t")); Serial.println(servoRamp[debug_servo][7]);
          Serial.println(F("---------------------------------------"));
          Serial.println();
          Serial.println();
        }
      } else if (cmd == "oled" || cmd == "o") {
        if (oled_active) {
          if (!plotter) Serial.println(F("test OLED begin"));
          test_oled();
          if (!plotter) Serial.println(F("test OLED end"));
        } else {
          if (!plotter) Serial.println(F("OLED inactive"));
        }
      } else if (cmd == "rgb") {
        if (!plotter) Serial.println(F("test rgb begin"));
        test_rgb();
        if (!plotter) Serial.println(F("test rgb end"));
      } else if (cmd == "trot" || cmd == "t") {
//DEVWORK
        if (!plotter) Serial.println(F("trot"));
        spd = 5;
        set_speed();
        move_steps = 35;
        x_dir = 0;
        move_trot = 1;
      } else if (cmd == "march" || cmd == "m") {
//DEVWORK
        if (!plotter) Serial.println(F("march"));        
        spd = 5;
        set_speed();
        y_dir = 0;
        x_dir = 0;
        z_dir = 0;
        move_steps = 25;
        if (mpu_is_active) mpu_active = 0;
        move_march = 1;
      } else if (cmd == "stay" || cmd == "s") {
        if (!plotter) Serial.println(F("stay"));
        set_stay();
      } else if (cmd == "home") {
        if (mpu_is_active) {
          if (mpu_active) {
            if (!plotter) Serial.println(F("mpu off!"));
            mpu_active = 0;
            if (!plotter) Serial.println(F("set_home"));
            set_home();
          } else {
            if (!plotter) Serial.println(F("mpu on!"));
            mpu_active = 1;
//            if (!plotter) Serial.print(F("mpu roll/pitch: "));Serial.print(mpu_mroll);Serial.print(F(" / "));Serial.println(mpu_mpitch);
//            set_axis(mpu_mroll, mpu_mpitch);
          }
        } else {
          if (!plotter) Serial.println(F("set_home"));
          set_home();
        }
      } else if (cmd == "wake" || cmd == "w") {
        if (!plotter) Serial.println(F("wake"));
        if (!activeServo[RFF] && !activeServo[LFF] && !activeServo[RRF] && !activeServo[LRF]) {
          spd = 1;
          set_speed();
          move_loops = 2;
          move_switch = 2;
          for (int i = 0; i < TOTAL_SERVOS; i++) {
            servoPos[i] = servoHome[i];
          }
          move_wake = 1;
        }
      } else if (cmd == "servo") {
        if (!plotter) { Serial.print("debug servo ");Serial.println(debug_servo); }
        debug_loops2 = debug_loops;
        move_servo = 1;
      } else if (cmd == "leg") {
        if (!plotter) { Serial.print("debug leg ");Serial.println(debug_leg); }
        debug_loops2 = debug_loops;
        move_leg = 1;
      } else if (cmd == "foff") {
        if (!plotter) { Serial.println("debug pir follow off"); }
        move_follow = 0;
        if (mp3_active) {
          mp3_play(24);
          delay(1500);
        }
        if (mpu_is_active) mpu_active = 1;
        if (uss_is_active) uss_active = 1;
      } else if (cmd == "fon") {
        if (!plotter) { Serial.println("debug pir follow on"); }
        move_follow = 1;
        if (mp3_active) {
          mp3_play(23);
          delay(1500);
        }
        if (mpu_is_active) mpu_active = 0;
        if (uss_is_active) uss_active = 0;
      } else if (cmd == "ms-") {
        if (!plotter) Serial.print(F("move_steps -5: "));
        if (move_steps > move_steps_min) {
          move_steps -= 5;
        }
        if (!plotter) Serial.println(move_steps);
      } else if (cmd == "ms+") {
        if (!plotter) Serial.println(F("move_steps +5: "));
        if (move_steps < move_steps_max) {
          move_steps += 5;
        }
        if (!plotter) Serial.println(move_steps);
      } else if (cmd == "y-") {
        if (!plotter) Serial.print(F("y_dir -5: "));
        if (y_dir > move_y_steps[0]) {
          y_dir -= 5;
        }
        if (!plotter) Serial.println(y_dir);
      } else if (cmd == "y+") {
        if (!plotter) Serial.print(F("y_dir +5: "));
        if (y_dir < move_y_steps[1]) {
          y_dir += 5;
        }
        if (!plotter) Serial.println(y_dir);
      } else if (cmd == "x-") {
        if (!plotter) Serial.print(F("x_dir -5: "));
        if (x_dir > move_x_steps[0]) {
          x_dir -= 5;
        }
        if (!plotter) Serial.println(x_dir);
      } else if (cmd == "x+") {
        if (!plotter) Serial.print(F("x_dir +5: "));
        if (x_dir < move_x_steps[1]) {
          x_dir += 5;
        }
        if (!plotter) Serial.println(x_dir);
      } else if (cmd == "z-") {
        if (!plotter) Serial.print(F("z_dir -5: "));
        if (z_dir > move_z_steps[0]) {
          z_dir -= 5;
        }
        if (!plotter) Serial.println(z_dir);
      } else if (cmd == "z+") {
        if (!plotter) Serial.print(F("z_dir +5: "));
        if (z_dir < move_z_steps[1]) {
          z_dir += 5;
        }
        if (!plotter) Serial.println(z_dir);
      } else if (cmd == "s-") {
        if (debug_servo > 0) {
          debug_servo--;
        }
        if (!plotter) { Serial.print("set debug servo ");Serial.println(debug_servo); }
      } else if (cmd == "s+") {
        if (debug_servo < (TOTAL_SERVOS-1)) {
          debug_servo++;
        }
        if (!plotter) { Serial.print("set debug servo ");Serial.println(debug_servo); }
      } else if (cmd == "l-") {
        if (debug_leg > 0) {
          debug_leg--;
        }
        if (!plotter) { Serial.print("set debug leg ");Serial.println(debug_leg); }
      } else if (cmd == "l+") {
        if (debug_leg < (TOTAL_LEGS-1)) {
          debug_leg++;
        }
        if (!plotter) { Serial.print("set debug leg ");Serial.println(debug_leg); }
      } else if (cmd == "demo") {
        if (!plotter) Serial.println(F("demo"));
        run_demo();
      } else if (cmd == "forw" || cmd == "f") {
        if (!plotter) Serial.println(F("march_forward"));
        spd = 5;
        set_speed();
        y_dir = 10;
        x_dir = 0;
        z_dir = 0;
        move_steps = 25;
        move_forward = 1;
        move_march = 1;
      } else if (cmd == "back" || cmd == "b") {
        if (!plotter) Serial.println(F("march_backward"));
        spd = 7;
        set_speed();
        y_dir = -10;
        x_dir = 0;
        z_dir = 0;
        move_steps = 25;
        move_backward = 1;
        move_march = 1;
      } else if (cmd == "sit") {
        if (!plotter) Serial.println(F("sit"));
        set_sit();
      } else if (cmd == "kneel") {
        if (!plotter) Serial.println(F("kneel"));
        set_kneel();
      } else if (cmd == "crouch" || cmd == "c") {
        if (!plotter) Serial.println(F("crouch"));
        set_crouch();
      } else if (cmd == "lay") {
        if (!plotter) Serial.println(F("lay"));
        set_lay();
      } else if (cmd == "roll") {
        if (!plotter) Serial.println(F("roll"));
        move_steps = 30;
        x_dir = 0;
        move_roll = 1;
      } else if (cmd == "pitch") {
        if (!plotter) Serial.println(F("pitch"));
        move_steps = 30;
        x_dir = 0;
        move_pitch = 1;
      } else if (cmd == "rollb") {
        if (!plotter) Serial.println(F("roll_body"));
        move_steps = 30;
        x_dir = 0;
        move_roll_body = 1;
      } else if (cmd == "pitchb") {
        if (!plotter) Serial.println(F("pitch_body"));
        move_steps = 30;
        x_dir = 0;
        move_pitch_body = 1;
      } else if (cmd == "wman") {
        if (!plotter) Serial.println(F("wman"));
        spd = 3;
        set_speed();
        move_wman = 1;
      } else if (cmd == "y") {
        if (!plotter) Serial.println(F("y_axis"));
        move_y_axis = 1;
        y_axis();
      } else if (cmd == "x") {
        if (!plotter) Serial.println(F("x_axis"));
        move_x_axis = 1;
        x_axis();
      } else if (cmd == "framp" || cmd == "fr") {
        if (!plotter) Serial.println(F("move_forward"));
        spd = 5;
        ramp_dist = 0.15;
        ramp_spd = 1.25;
        use_ramp = 1;
        x_dir = 0;
        move_forward = 1;
      } else if (cmd == "st") {
        if (!plotter) Serial.println(F("sweep tibias"));
        use_ramp = 0;
        for (int i = 0; i < TOTAL_SERVOS; i++) {
          if (is_tibia(i)) {
            servoSweep[i][0] = servoLimit[i][0];
            servoSweep[i][1] = servoLimit[i][1];
            servoSweep[i][2] = 0;
            servoSweep[i][3] = 1;
            targetPos[i] = servoSweep[i][1];
            activeSweep[i] = 1;
          }
        }
      } else if (cmd == "rst") {
        if (!plotter) Serial.println(F("ramp sweep tibia"));
        use_ramp = 1;
        for (int i = 0; i < TOTAL_SERVOS; i++) {
          if (is_tibia(i)) {
            servoSweep[i][0] = servoLimit[i][0];
            servoSweep[i][1] = servoLimit[i][1];
            servoSweep[i][2] = 0;
            servoSweep[i][3] = 1;
            targetPos[i] = servoSweep[i][1];
            activeSweep[i] = 1;
            set_ramp(i, servoSpeed[i], 0, 0, 0, 0);
          }
        }
      } else if (cmd == "sf") {
        if (!plotter) Serial.println(F("sweep femurs"));
        use_ramp = 0;
        for (int i = 0; i < TOTAL_SERVOS; i++) {
          if (is_femur(i)) {
            servoSweep[i][0] = servoLimit[i][0];
            servoSweep[i][1] = servoLimit[i][1];
            servoSweep[i][2] = 0;
            servoSweep[i][3] = 1;
            targetPos[i] = servoSweep[i][1];
            activeSweep[i] = 1;
          }
        }
      } else if (cmd == "rsf") {
        if (!plotter) Serial.println(F("ramp sweep femur"));
        use_ramp = 1;
        for (int i = 0; i < TOTAL_SERVOS; i++) {
          if (is_femur(i)) {
            servoSweep[i][0] = servoLimit[i][0];
            servoSweep[i][1] = servoLimit[i][1];
            servoSweep[i][2] = 0;
            servoSweep[i][3] = 1;
            targetPos[i] = servoSweep[i][1];
            activeSweep[i] = 1;
            set_ramp(i, servoSpeed[i], 0, 0, 0, 0);
          }
        }
      } else if (cmd == "sc") {
        if (!plotter) Serial.println(F("sweep coaxes"));
        use_ramp = 0;
        for (int i = 0; i < TOTAL_SERVOS; i++) {
          if (!is_femur(i) && !is_tibia(i)) {
            servoSweep[i][0] = servoLimit[i][0];
            servoSweep[i][1] = servoLimit[i][1];
            servoSweep[i][2] = 0;
            servoSweep[i][3] = 1;
            targetPos[i] = servoSweep[i][1];
            activeSweep[i] = 1;
          }
        }
      } else if (cmd == "lr") {
          if (!plotter) Serial.println(F("look right"));
          spd = 1;
          set_speed();
          move_loops = 1;
          move_steps = 30;
          move_look_right = 1;
      } else if (cmd == "ll") {
          if (!plotter) Serial.println(F("look left"));
          spd = 1;
          set_speed();
          move_loops = 1;
          move_steps = 30;
          move_look_left = 1;
      } else if (cmd == "mpu") {
        if (mpu_active) {
          if (!plotter) Serial.println(F("mpu off"));
          mpu_active = 0;
        } else if(mpu_is_active) {
          if (!plotter) Serial.println(F("mpu on"));
          mpu_active = 1;
        } else {
          if (!plotter) Serial.println(F("mpu inactive"));
        }
      } else if (cmd == "uss") {
        if (uss_active) {
          if (!plotter) Serial.println(F("us sensors off"));
          uss_active = 0;
        } else if(uss_is_active) {
          if (!plotter) Serial.println(F("us sensors on"));
          uss_active = 1;
          int dist_rt = command_slave((char*)"b");
          int dist_lt = command_slave((char*)"a");
          if (oled_active) {
            oled_request((char*)"c");
          }
          if (!plotter) {
            Serial.print(F("R: "));Serial.print(dist_rt);Serial.print(F(" L: "));Serial.println(dist_lt);
          }
        } else {
          if (!plotter) Serial.println(F("us sensors inactive"));
        }
      } else if (cmd == "pir") {
        if (pir_active) {
          if (!plotter) Serial.println(F("pir sensors off"));
          pir_active = 0;
        } else if(pir_is_active) {
          if (!plotter) Serial.println(F("pir sensors on"));
          pir_active = 1;
        } else {
          if (!plotter) Serial.println(F("pir sensors inactive"));
        }
      } else if (cmd == "mp2") {
        if (mp3_active) {
          int mplay = random(1,22);
          mp3_play(mplay);
          if (!plotter) Serial.print(F("playing track "));
          if (!plotter) Serial.println(mplay);
        }
      } else if (cmd == "mp3") {
        if (mp3_active) {

          if (!mp3_status) {
            mp3_status = 1;
            int q = random(1,6);
            for (int i=1;i<q;i++) {
              add_to_mp3_queue(random(1,22));
            }
          }
        }
      } else if (cmd == "vup") {
        if (mp3_active) {
          if (sound_vol < 30) {
            sound_vol++;
            mp3_volume(sound_vol);
            delay(30);
            Serial.print("volume up: ");Serial.println(sound_vol);
          }
        }
      } else if (cmd == "vdn") {
        if (mp3_active) {
          if (sound_vol > 1) {
            sound_vol--;
            mp3_volume(sound_vol);
            delay(30);
            Serial.print("volume down: ");Serial.println(sound_vol);
          }
        }
      } else if (cmd == "batt") {
        if (oled_active) {
          if (!plotter) Serial.println(F("battery display test"));
          oled_request((char*)"n");
          delay(3000);
        } else {
          if (!plotter) Serial.println(F("oled inactive"));
        }
      } else if (cmd == "h" || cmd == "help") {
        Serial.println();
        Serial.println(F("\t---------------------------------------------------------------------------------"));
        Serial.println(F("\t|\tINPUT\tCOMMAND\t\t\t\tINPUT\tCOMMAND\t\t\t|"));
        Serial.println(F("\t---------------------------------------------------------------------------------"));
        Serial.print(F("\t|\t0\tstop!\t\t\t"));
        Serial.println(F("\tms+\tmove_steps -5\t\t|"));

        Serial.print(F("\t|\t1\tset speed 1\t\t"));
        Serial.println(F("\tms-\tmove_steps +5\t\t|"));

        Serial.print(F("\t|\t2\tset speed 5\t\t"));
        Serial.println(F("\ty-\ty_dir -5\t\t|"));

        Serial.print(F("\t|\t3\tset speed 10\t\t"));
        Serial.println(F("\ty+\ty_dir +5\t\t|"));

        Serial.print(F("\t|\t4\tset speed 15\t\t"));
        Serial.println(F("\tx-\tx_dir -5\t\t|"));

        Serial.print(F("\t|\t5\tset speed 20\t\t"));
        Serial.println(F("\tx+\tx_dir +5\t\t|"));

        Serial.print(F("\t|\t6\tset speed 25\t\t"));
        Serial.println(F("\tz-\tz_dir -5\t\t|"));

        Serial.print(F("\t|\t7\tset speed 30\t\t"));
        Serial.println(F("\tz+\tz_dir +5\t\t|"));

        Serial.print(F("\t|\t8\tset speed 35\t\t"));
        Serial.println(F("\tvars\tsettings\t\t|"));

        Serial.print(F("\t|\t9\tset speed 40\t\t"));
        Serial.println(F("\toled\ttest OLED\t\t|"));

        Serial.print(F("\t|\th\thelp\t\t\t"));        
        Serial.println(F("\trgb\ttest RGB\t\t|"));

        Serial.println();
        
        Serial.print(F("\t|\tmarch\tmarch\t\t\t"));
        Serial.println(F("\tstay\tstay\t\t\t|"));

        Serial.print(F("\t|\ttrot\ttrot\t\t\t"));
        Serial.println(F("\tsit\tsit\t\t\t|"));

        Serial.print(F("\t|\tforw\tmarch_forward\t\t"));
        Serial.println(F("\tkneel\tkneel\t\t\t|"));

        Serial.print(F("\t|\tback\tmarch_backward\t\t"));
        Serial.println(F("\tcrouch\tcrouch\t\t\t|"));

        Serial.print(F("\t|\twman\twman\t\t\t"));
        Serial.println(F("\tlay\tlay\t\t\t|"));

        Serial.print(F("\t|\tstl\tstep_left\t\t"));
        Serial.println(F("\troll\troll\t\t\t|"));

        Serial.print(F("\t|\tstr\tstep_right\t\t"));
        Serial.println(F("\tpitch\tpitch\t\t\t|"));

        Serial.print(F("\t|\tlr\tlook right\t\t"));
        Serial.println(F("\trollb\troll_body\t\t|"));

        Serial.print(F("\t|\tll\tlook left\t\t"));
        Serial.println(F("\tpitchb\tpitch_body\t\t|"));

        Serial.print(F("\t|\thome\thome servos\t\t"));
        Serial.println(F("\ty\ty_axis\t\t\t|"));

        Serial.print(F("\t|\tfon\tfollow mode on\t\t"));
        Serial.println(F("\ty\ty_axis\t\t\t|"));

        Serial.print(F("\t|\tfoff\tfollow mode off\t\t"));
        Serial.println(F("\tx\tx_axis\t\t\t|"));

        Serial.println();
        
        
        Serial.print(F("\t|\tst\tsweep tibias\t\t"));
        Serial.print(F("\tservo\tdebug servo "));Serial.print(debug_servo);Serial.println(F("\t\t|"));

        Serial.print(F("\t|\trst\tramp sweep tibias\t"));
        Serial.println(F("\ts+\tnext debug_servo\t|"));

        Serial.print(F("\t|\tsf\tsweep femurs\t\t"));
        Serial.println(F("\ts-\tprev debug_servo\t|"));

        Serial.print(F("\t|\trsf\tramp sweep femurs\t"));
        Serial.print(F("\tleg\tdebug leg "));Serial.print(debug_leg);Serial.println(F("\t\t|"));

        Serial.print(F("\t|\tsc\tsweep coaxes\t\t"));
        Serial.println(F("\tl+\tnext debug_leg\t\t|"));

        Serial.print(F("\t|\tmpu\tmpu on/off\t\t"));
        Serial.println(F("\ts-\tprev debug_leg\t\t|"));

        Serial.print(F("\t|\tuss\tus sensors on/off\t"));
        Serial.println(F("\tmp3\tpreview mp3s\t\t|"));

        Serial.print(F("\t|\tpir\tpir sensors on/off\t"));
        Serial.println(F("\tvup\tvolume up\t\t|"));

        Serial.print(F("\t|\twake\twake\t\t\t"));
        Serial.println(F("\tvdn\tvolume down\t\t|"));

        Serial.println(F("\t---------------------------------------------------------------------------------"));
        Serial.println(F("\tType a command input or 'h' for help:"));
        Serial.println();
      } else {
        Serial.print(cmd);
        Serial.println(F(" is not a valid command input.\nTry again, else type 'h' for help."));
      }
  }
}


void test_oled() {
          oled_request((char*)"a");
          delay(6000);

          oled_request((char*)"0n");
          delay(3000);
          oled_request((char*)"1n");
          delay(3000);
          oled_request((char*)"2n");
          delay(3000);
          oled_request((char*)"3n");
          delay(3000);
          oled_request((char*)"4n");
          delay(3000);
          oled_request((char*)"5n");
          delay(3000);
          oled_request((char*)"6n");
          delay(3000);
          oled_request((char*)"7n");
          delay(3000);
          oled_request((char*)"8n");
          delay(3000);
          oled_request((char*)"9n");
          delay(3000);


/*
          oled_request((char*)"c");
          delay(3000);
          oled_request((char*)"g");
          delay(3000);
          oled_request((char*)"d");
          delay(1500);
*/
}


void test_rgb() {
        rgb_request((char*)"MONRHyn");
        delay(3000);
        rgb_request((char*)"i");
        delay(3000);
        rgb_request((char*)"k");
        delay(3000);
        rgb_request((char*)"l");
        delay(3000);
        rgb_request((char*)"MRNSHun");
        delay(1500);
        rgb_request((char*)"MSNRHun");
        delay(1500);
        rgb_request((char*)"m");
        delay(3000);
        rgb_request((char*)"n");
        delay(3000);
        rgb_request((char*)"MUNUDzn");
        delay(3000);
        rgb_request((char*)"MSNSDzn");
        delay(3000);
}


/*
   -------------------------------------------------------
   Serial Communication Functions
   -------------------------------------------------------
*/
void rgb_request(char* commands) {
  if (serial_oled) {
    serial_oled = command_slave((char*)"X");
  }
  serial_resp = command_slave(commands);
}

void oled_request(char* commands) {
  if (!serial_oled) {
    serial_oled = command_slave((char*)"X");
  }
  serial_resp = command_slave(commands);
  serial_oled = command_slave((char*)"X");
}

int command_slave(char* commands) {
  int command_response = 0;

  if(commands && slave_active) {
    int i = 0;
    while(commands[i]) {
      if (debug6 && (commands[i] != 'Z')) {
        (serial_oled) ? Serial.print(F("OLED Command: ")) : Serial.print(F("System Command: "));
      }

      Wire1.beginTransmission((uint8_t)SLAVE_ID);
      Wire1.write(char(commands[i]));
      Wire1.endTransmission();
      if (debug6 && (commands[i] != 'Z')) {
        Serial.print(commands[i]);
      }
      
      Wire1.beginTransmission((uint8_t)SLAVE_ID);
      int available = Wire1.requestFrom((uint8_t)SLAVE_ID, (uint8_t)2);
              
      if (available == 2) {
        command_response = Wire1.read() << 8 | Wire1.read(); 
        if (debug6 && (commands[i] != 'Z')) {
          Serial.print("\tresponse: ");
          Serial.print(command_response);
        }
      }
      Wire1.endTransmission();

      if (debug6 && (commands[i] != 'Z')) {
        Serial.println();
      }

      //if resetting slave, pause 15secs for display graphics if splash_active
      if (commands[i] == 'Z' || commands[i] == 'Y') {
        if (debug) Serial.print(F("Slave Circuit intializing..."));
        int pcnt = 6;
        if (commands[i] == 'Z' && oled_active && splash_active) {
          pcnt = 14;
        }
        for(int n=0;n<pcnt;n++) {
          if (debug) Serial.print(F("."));
          delay(1000);
        }
        if (commands[i] == 'Z' && oled_active && splash_active) {
          if (debug) Serial.println(F("\tOK"));
        } else {
          if (debug) Serial.println(F("\t\tOK"));
        }
        delay(1000);
      }
      
      i++;
    }
  }

  return command_response;
}

void powering_down(void) {
  digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
  if (debug) Serial.println ("System Shutting Down... ");

  if (oled_active) {
    oled_request((char*)"k");
  }
  if (mp3_active) {
    mp3_play(22);
    delay(2500);
    mp3_play(5);
  }

  set_stop_active();
  init_home();

  if (rgb_active) {
    rgb_request((char*)"Id");
  }
  if (oled_active) {
    oled_request((char*)"k");
  }
  delay(2500);
}



//MP3 Setup
//DFPlayer Mini Pro
void mp3_play(int track) {

  mp3_status = 1;
//  #if MP3_PLAYER_TYPE > 0
    switch (track) {
      case 1: DFPlayer.playSpecFile("/MP3/0001.mp3"); break;
      case 2: DFPlayer.playSpecFile("/MP3/0002.mp3"); break;
      case 3: DFPlayer.playSpecFile("/MP3/0003.mp3"); break;
      case 4: DFPlayer.playSpecFile("/MP3/0004.mp3"); break;
      case 5: DFPlayer.playSpecFile("/MP3/0005.mp3"); break;
      case 6: DFPlayer.playSpecFile("/MP3/0006.mp3"); break;
      case 7: DFPlayer.playSpecFile("/MP3/0007.mp3"); break;
      case 8: DFPlayer.playSpecFile("/MP3/0008.mp3"); break;
      case 9: DFPlayer.playSpecFile("/MP3/0009.mp3"); break;
      case 10: DFPlayer.playSpecFile("/MP3/0010.mp3"); break;
      case 11: DFPlayer.playSpecFile("/MP3/0011.mp3"); break;
      case 12: DFPlayer.playSpecFile("/MP3/0012.mp3"); break;
      case 13: DFPlayer.playSpecFile("/MP3/0013.mp3"); break;
      case 14: DFPlayer.playSpecFile("/MP3/0014.mp3"); break;
      case 15: DFPlayer.playSpecFile("/MP3/0015.mp3"); break;
      case 16: DFPlayer.playSpecFile("/MP3/0016.mp3"); break;
      case 17: DFPlayer.playSpecFile("/MP3/0017.mp3"); break;
      case 18: DFPlayer.playSpecFile("/MP3/0018.mp3"); break;
      case 19: DFPlayer.playSpecFile("/MP3/0019.mp3"); break;
      case 20: DFPlayer.playSpecFile("/MP3/0020.mp3"); break;
      case 21: DFPlayer.playSpecFile("/MP3/0021.mp3"); break;
      case 22: DFPlayer.playSpecFile("/MP3/0022.mp3"); break;
      case 23: DFPlayer.playSpecFile("/MP3/0023.mp3"); break;
      case 24: DFPlayer.playSpecFile("/MP3/0024.mp3"); break;
      case 25: DFPlayer.playSpecFile("/MP3/0025.mp3"); break;
      case 26: DFPlayer.playSpecFile("/MP3/0026.mp3"); break;
      case 27: DFPlayer.playSpecFile("/MP3/0027.mp3"); break;
    }
//  #else
//    DFPlayer.playMp3Folder(track);    
//  #endif
}

void mp3_volume(int vol) {
  int svol = 0;
  if (vol < 0) {
    svol = analogRead(MP3_VOL_PIN);
    if (svol < vpot_max) {
      if (vpot_loop == vpot_cnt) {
        sound_vol = map((vpot_avg / vpot_loop), vpot_min, vpot_max, vol_max, vol_min);
        vpot_avg = 0;
        vpot_cnt = 0;
      } else if (vpot_loop) {
        vpot_avg += svol;
        vpot_cnt++;        
        svol = 0;
      } else {
        sound_vol = map(svol, vpot_min, vpot_max, vol_max, vol_min);
      }
    }

    lastPotUpdate = millis();
  } else {
    svol = vol;
    sound_vol = vol;
  }

  if (svol && sound_vol != sound_volp) {
    sound_volp = sound_vol;

//    #if MP3_PLAYER_TYPE > 0
      DFPlayer.setVol(sound_vol);
//    #else
//      DFPlayer.volume(sound_vol);
//    #endif
    if (debug1) {
      Serial.print("set vol: ");
      Serial.println(sound_vol);
    }
  }
}

//DEV: mp3 player queue system
void check_mp3() {
//  #if MP3_PLAYER_TYPE > 0
    if (mp3_status && (DFPlayer.getCurTime() == DFPlayer.getTotalTime()) && DFPlayer.getFileName()) {
      DFPlayer.pause();
      mp3_status = 0;
    }
//  #else
//    int value = DFPlayer.readState();
//    if (value == 2 || value == 0 || DFPlayer.available()) {
//      play_mp3_queue(DFPlayer.readType());
//    }
//  #endif

  lastMP3Update = millis();
}

void add_to_mp3_queue(int track) {
  for (int i=0;i<5;i++) {
    if (!mp3_queue[i]) {
      mp3_queue[i] = track;
      Serial.print(F("queued "));
      Serial.println(track);
      break;
    }
  }
}

void play_mp3_queue(uint8_t type) {
/*
  #if MP3_PLAYER_TYPE > 0
//  Serial.print(F("type "));
//  Serial.println(type);
  #else
    if (type == DFPlayerPlayFinished || type == 11) {
      for (int i=0;i<5;i++) {
        if (mp3_queue[i]) {
          mp3_play(mp3_queue[i]);
          Serial.print(F("play track "));Serial.println(mp3_queue[i]);
          mp3_queue[i] = 0;
          break;
        }
        if (i == 4) {
          mp3_status = 0;
        }
      }
    }
  #endif
*/
}
