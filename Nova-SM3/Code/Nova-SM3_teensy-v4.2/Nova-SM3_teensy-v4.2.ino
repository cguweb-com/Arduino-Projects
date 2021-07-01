/*
 *   NovaSM3 - a Spot-Mini Micro clone
 *   Version: 4.2
 *   Version Date: 2021-06-02
 *   
 *   Author:  Chris Locke - cguweb@gmail.com
 *   GitHub Project:  https://github.com/cguweb-com/Arduino-Projects/tree/main/Nova-SM3
 *   Thingiverse:  https://www.thingiverse.com/thing:4767006
 *   Instructables Project:  https://www.instructables.com/Nova-Spot-Micro-a-Spot-Mini-Clone/
 *   YouTube Playlist:  https://www.youtube.com/watch?v=00PkTcGWPvo&list=PLcOZNHwM_I2a3YZKf8FtUjJneKGXCfduk
 *   
 *   RELEASE NOTES:
 *      Teensy 4.0 performance: 4% storage / 22% memory
 *      Replaced Arduino Mega with Teensy 4.0
 *      
 *
 *   DEV NOTES:
 *      BUG:  hangs / crashes randomly still:
 *        a. loose connections from dupont / pins
 *        b. ground issue(?) from Nano circuit / power switch
 *        c. if slave disabled, PWM doesn't boot / powerup
 *      PS2 remote interfering again, this time with restart in follow() (could it be the delay?)
 *      Do we still need spi library? or was that for voice recognition module?
 *      BUG: ramping: on interruption of ramp, servo speed is set to the speed of the point of interrupt
 *      work on integrating MPU data into movements
 *      
 *      see more 'DEV NOTE' comments in code for more bugs/tasks
 *      
*/
//DEVWORK
int  test_loops = 0;  
int  test_steps = 0;  

//set Nova SM3 version
#define VERSION 4.2

//debug vars for displaying operation runtime data for debugging
const byte debug  = 1;             //general messages
const byte debug1 = 0;            //ps2 commands and pir sensors
const byte debug2 = 0;            //debug servo steps
const byte debug3 = 0;            //ramping and sequencing
const byte debug4 = 0;            //amperage and battery 
const byte debug5 = 0;            //mpu
const byte debug6 = 0;            //serial communication output/response and serial terminal commands
const byte debug7 = 0;            //uss sensors
const byte plotter = 0;           //plot servo steps, turn off debug1

byte debug_leg = 0;               //default debug leg (3 servos) (changed by serial command input)
int debug_servo = 2;              //default debug servo (changed by serial command input)
int debug_loops = 3;              //default loops for debug movements
int debug_loops2 = 3;             //movement decremented loop
int debug_spd = 10;               //default speed for debug movements

//activate/deactivate devices
byte slave_active = 1;            //activate slave arduino nano
byte pwm_active = 1;              //activate pwm controller / servos
byte ps2_active = 0;              //activate PS2 remote control 
byte serial_active = 1;           //activate serial monitor command input
byte mpu_active = 0;              //activate MPU6050 
byte rgb_active = 1;              //activate RGB modules
byte oled_active = 1;             //activate OLED display
byte pir_active = 1;              //activate PIR motion sensor
byte uss_active = 1;              //activate Ultra-Sonic sensors
byte amp_active = 0;              //activate amperate monitoring
byte batt_active = 0;             //activate battery level monitoring
byte buzz_active = 1;             //activate simple tone sounds
byte melody_active = 1;           //activate melodic tone sounds
byte skip_splash = 1;             //skip 15s of loading graphics

//include supporting libraries
#include <SPI.h>
#include <Wire.h>
#include <PS2X_lib.h>
#include <Adafruit_PWMServoDriver.h>
#include "MPU6050_conf.h"
#include "T4_PowerButton.h"


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
const float min_spd = 96.0;       //min is higher than max, since this is the time increment delay between servo calls 
const float max_spd = 1.0;        //maximum, fastest speed
float default_spd = 13.0;
float spd = default_spd;
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
#define BUZZ 22                   //piezo buzzer pin

//rgb leds
int pattern_int = 500;            //set pattern delay between
int pattern_cnt = 0;              //set number of loops of pattern

//pir sensor
#define PIR_FRONT 4
#define PIR_LEFT 5
#define PIR_RIGHT 6
unsigned int pirInterval = 500;
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

//mpu6050 sensor
const int MPU = 0x68;
unsigned int mpuInterval = 40;
unsigned int mpuInterval_prev = 0;
unsigned long lastMPUUpdate = 0;
float mroll, mpitch, myaw;
float mroll_prev, mpitch_prev, myaw_prev;
float mpu_mroll = 0.00;
float mpu_mpitch = 0.00;
float mpu_myaw = 0.00;
float mpu_trigger_thresh = 0.1;
int mpu_is_active = mpu_active;
int mpu_c = 0;

//DEV NOTE: these are used for an experimental oscillation prevention check, but really, a PID controller
//          should be researched and coded to replace this messy, unreliable solution
//
float mpu_oscill_thresh = 33.0;
int mpu_oscill_grace = 3;
int mpu_oscill_limit = 3;
int mpu_oscill_cnt = mpu_oscill_limit;

//PS2 controller
#define PS2_DAT 24
#define PS2_CMD 25
#define PS2_SEL 26
#define PS2_CLK 27
PS2X ps2x;
unsigned int ps2Interval = 50;
unsigned long lastPS2Update = 0;
int ps2_select = 1;               //sets default button set


//slave arduino and serial commands
#define SLAVE_ID 1
byte serial_oled = 0;            //switch for serial or oled commands
int serial_resp;
int ByteReceived;
unsigned int serialInterval = 60;
unsigned long lastSerialUpdate = 0;
String serial_input;


//amperage monitor
#define AMP_PIN A2
#define PWR_PIN 4
unsigned int ampInterval = 15000;
unsigned long lastAmpUpdate = 0;
int amp_cnt = 0;
int amp_thresh = 10;            //loop count for consecutive amperage alarms to prevent false positives
int amp_warning = 0;            //current alarm warning level
int amp_loop = 1;               //if warning level set, this changes accordingly to prevent false positives
float amp_limit = 6.5;          //aperage draw limit before triggering alarms


//battery monitor
#define BATT_MONITOR A1
unsigned long batteryInterval = 15000;
unsigned long lastBatteryUpdate = 0;
int batt_cnt = 0;
int batt_skip = 0;
float batt_voltage = 11.4;                          //fully charged battery minimum nominal voltage
float batt_voltage_prev = 11.4;                     //comparison voltage to prevent false positives
float batt_levels[4] = {11.1, 10.9, 10.7, 10.5};    //voltage drop alarm levels(4)
float batt_adjust = 0.95;                           //account for common voltage drop of components to prevent false readings


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
int move_x_steps[2] = {-30, 30};
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

//vars used to compensate for center of gravity
float step_weight_factor = 1;           
float step_height_factor = 1.25;


/*
   -------------------------------------------------------
   Function Prototypes
    :required for functions executed from servo class ( I know, I know, poor OOP design calling functions outside of a class ;p )
   -------------------------------------------------------
*/
void set_ramp(int servo, float sp, float r1_spd, float r1_dist, float r2_spd, float r2_dist);
void amperage_check(int aloop);
void powering_down(void);


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
  digitalWrite(LED_PIN, HIGH);
  delay(500);

  if (buzz_active) {
    tone(BUZZ, 1, 500);
  }

  //setup power off button
  set_arm_power_button_callback(&powering_down);
  if (debug) {
    if (arm_power_button_pressed()) {
      Serial.println("System Restarted");
    }
  }

  if (debug) {
    Serial.begin(19200);

    //allow serial to connect
    while (!Serial) {
      delay(1);
    }

    Serial.println(F("\n========================================"));
    Serial.print(F("NOVA SM3 v"));
    Serial.println(VERSION);
    Serial.println(F("========================================"));
  }

  //show active settings, including delays to prevent IDE reset firing setup functions on upload/connect
  if (debug) {
    Serial.println(F("Hardware Settings:"));
    if (slave_active) Serial.println(F("  Slave Circuit"));
    delay(500);
    if (pwm_active) Serial.println(F("  PWM Controller"));
    delay(500);
    if (ps2_active) Serial.println(F("  PS2 Remote"));
    delay(500);
    if (serial_active) Serial.println(F("  Serial Commands"));
    delay(500);
    if (mpu_active) Serial.println(F("  MPU6050 IMU"));
    delay(500);
    if (rgb_active) Serial.println(F("  RGB LEDs"));
    delay(500);
    if (oled_active) Serial.println(F("  OLED Display"));
    delay(500);
    if (pir_active) Serial.println(F("  PIR Sensors"));
    delay(500);
    if (uss_active) Serial.println(F("  Ultra-Sonic Sensors"));
    delay(500);
    if (amp_active) Serial.println(F("  ACS712 Current Sensor"));
    delay(500);
    if (batt_active) Serial.println(F("  Battery Monitor"));
    delay(500);
    if (buzz_active) Serial.println(F("  Buzzer"));
    delay(500);
    if (melody_active) Serial.println(F("  Melody"));
    delay(500);
    if (skip_splash) Serial.println(F("  Skip Splash Screen"));
    delay(500);
    Serial.println();
  } else {
    delay(7000); 
  }
  digitalWrite(LED_PIN, LOW);

  //seed arduino pin A0, otherwise random() functions will not be truely random
  randomSeed(analogRead(0));

  if (melody_active) {
    //pay homage to the first robot in my life at age 7, R2D2!! 
    for (int i=0; i<2; i++) {
      play_phrases();
      delay(random(200,800));
    }
  }
  if (buzz_active) {
    //pay homage to the first video came system in my life at age 7, Atari2600!! 
    for (int b = 0; b < 8; b++) {
      tone(BUZZ, (b * 1.5)*200);
      delay(20);
      noTone(BUZZ);
      delay(20);
    }
    noTone(BUZZ);
  }

  //setup 2nd i2c bus
  Wire1.begin();
  Wire1.setSDA(SDA2_PIN);
  Wire1.setSCL(SCL2_PIN);

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
          if (debug) Serial.print(F("MPU6050 calibrating... "));
        }
        calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
        initMPU6050(); 
        if (debug) Serial.println(F("  OK\n"));
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

  //init ps2 controller
  if (ps2_active) {
    if (debug) Serial.print(F("PS2 Controller intializing..."));
    delay(500);
    if (ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false) == 0) {
      if (debug) Serial.println(F(" OK"));
    } else {
      if (debug) Serial.println(F(" Error!"));
      ps2_active = 0;
    }
  }

  //init pwm controller
  if (pwm_active) {
    if (debug) Serial.print(F("PWM Controller intializing..."));
    if (buzz_active) {
      digitalWrite(LED_PIN, HIGH);
      for (int b = 3; b > 0; b--) {
        tone(BUZZ, 1);
        delay(150);
        tone(BUZZ, 10);
        delay(250);
        noTone(BUZZ);
        delay(250);
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

    pwm1.begin();
    pwm1.setOscillatorFrequency(OSCIL_FREQ);
    pwm1.setPWMFreq(SERVO_FREQ);
    delay(200);
    if (debug) Serial.println(F(" OK"));
    if (debug) {
      Serial.print(TOTAL_SERVOS); Serial.print(F(" Servos intializing..."));
    }

    //set default speed factor
    spd_factor = mapfloat(spd, min_spd, max_spd, min_spd_factor, max_spd_factor);

    //initialize servos and populate related data arrays with defaults
    init_home();
    delay(1000);
    if (debug) Serial.println(F(" OK"));

    //DEV NOTE:  this was done because otherwise, there is some electrical(?) interference to
    //           the PS2 controller / wiring that was causing the remote to fire all buttons at
    //           arduino boot time, obviously causing Nova to go haywire with unexpected random 
    //           movement commands!
    //
    //           After testing, it appeared to be the PWM controller interfering, so I added code
    //           to halt its output until the PS2 was fully initialized, which is found at the end 
    //           of the ps2_check() function.
    //
    //disable PWM output after initializing servos
    if (ps2_active) {
      digitalWrite(OE_PIN, HIGH);
      delay(100);
    }
  }

  //(re)boot slave nano
  if (slave_active) {
    if (!skip_splash) {
      command_slave((char*)"Z");
    } else {
      command_slave((char*)"Y");
    }
  }

  if (buzz_active) {
    for (int b = 0; b < 12; b++) {
      tone(BUZZ, (b*100));
      delay(50);
      noTone(BUZZ);
      delay(25);
    }
    noTone(BUZZ);
  }

  for (int b = 3; b > 0; b--) {
    digitalWrite(LED_PIN, HIGH);
    delay((100 * b));
    digitalWrite(LED_PIN, LOW);
    delay((100 * b));
  }

  if (!mpu_active) {
    if (debug) {
      Serial.println(F("Nova SM3 OK!"));
      Serial.println(F("========================================"));

      delay(500);
      if (!plotter && serial_active) {
        delay(1000);
        Serial.println();
        Serial.println(F("Type a command input or 'h' for help:"));
      }
    }
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
    step_trot(x_dir);
  } else if (move_forward) {
    step_forward(x_dir);
  } else if (move_backward) {
    step_backward(x_dir);
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
  if (ps2_active) {
    if (millis() - lastPS2Update > ps2Interval) ps2_check();
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
    if(millis() - lastBatteryUpdate > batteryInterval) battery_check();
  }

  if (moveDelayInterval && millis() - lastMoveDelayUpdate > moveDelayInterval) {
    delay_sequences();
  }
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
   PS2 Check
    :provide general description and explanation here - too much to comment by line
   -------------------------------------------------------
*/
void ps2_check() {
  if (pwm_oe) {
    ps2x.read_gamepad(false, false);

    if (!move_demo && !move_funplay) {
      if (ps2x.Button(PSB_START)) {
        if (debug1)
          Serial.println(F("Start Pressed"));
        if (ps2_select == 1) {
          rgb_request((char*)"MVNV");
/*
          if (mpu_active) {
            mpu_active = 0;
            rgb_request((char*)"vGn");
          } else {
            mpu_active = 1;
            rgb_request((char*)"xFn");
          }
*/
        } else if (ps2_select == 2) {
          if (debug1)
            Serial.println(F("start / stop march"));
          if (!move_march) {
            set_stop();
            spd = 3;
            set_speed();
            y_dir = 0;
            x_dir = 0;
            z_dir = 0;
            move_steps = 50;
            if (mpu_is_active) mpu_active = 0;
            move_march = 1;
            if (oled_active) {
              oled_request((char*)"d");
            }
          } else {
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
        } else if (ps2_select == 3) {
          rgb_request((char*)"MVNV");

/*
          if (uss_active) {
            uss_active = 0;
            rgb_request((char*)"vGn");
          } else {
            uss_active = 1;
            rgb_request((char*)"xFn");
          }

          if (pir_halt) {
            pir_halt = 0;
            rgb_request((char*)"vGn");
          } else {
            pir_halt = 1;
            rgb_request((char*)"xFn");
          }
*/
        } else if (ps2_select == 4) {
//          run_demo();
          rgb_request((char*)"MVNV");

          if (!move_follow) {
            spd = 3;
            set_speed();
            move_follow = 1;
            if (buzz_active) {
              for (int b = 3; b > 0; b--) {
                tone(BUZZ, 1000);          
                delay(100);  
                noTone(BUZZ);         
                delay(100);  
              }
              noTone(BUZZ);         
            }
          } else {
            move_follow = 0;
            if (buzz_active) {
              for (int b = 2; b > 0; b--) {
                tone(BUZZ, 2000);          
                delay(100);  
                noTone(BUZZ);         
                delay(100);  
              }
              noTone(BUZZ);         
            }
          }

        }
      }
  
      if (ps2x.ButtonReleased(PSB_SELECT)) {
        (ps2_select < 4) ? ps2_select++ : ps2_select = 1;
        if (rgb_active) {
          rgb_request((char*)"MQNQ");

          if (ps2_select == 1) {
            rgb_request((char*)"tEn");
          } else if (ps2_select == 2) {
            rgb_request((char*)"uEn");
          } else if (ps2_select == 3) {
            rgb_request((char*)"vEn");
          } else if (ps2_select == 4) {
            rgb_request((char*)"wEn");
          }
        }
        if (buzz_active) {
          for (int b = ps2_select; b > 0; b--) {
            tone(BUZZ, 2000);          
            delay(70);  
            noTone(BUZZ);         
            delay(70);  
          }
          noTone(BUZZ);         
        }
        if (oled_active) {
          if (ps2_select == 1) {
            oled_request((char*)"g");
          } else if (ps2_select == 2) {
            oled_request((char*)"h");
          } else if (ps2_select == 3) {
            oled_request((char*)"i");
          } else if (ps2_select == 4) {
            oled_request((char*)"j");
          }
        }
        if (debug1) {
          Serial.print(F("\tSelected ")); Serial.println(ps2_select);
        }
      }

      //gait joysticks
      if (ps2_select == 2) {
        y_dir = mapfloat(ps2x.Analog(PSS_LY), 0, 255, y_dir_steps[1], y_dir_steps[0]);
        x_dir = mapfloat(ps2x.Analog(PSS_LX), 0, 255, x_dir_steps[1], x_dir_steps[0]);
        z_dir = mapfloat(ps2x.Analog(PSS_RY), 0, 255, z_dir_steps[1], z_dir_steps[0]);

        //set move_steps to min 40 to maintain march-in-place
        //DEVNOTE: make this switchable via PS2
        move_steps = map(ps2x.Analog(PSS_RX), 0, 255, 40, move_steps_max + (move_steps_max * 0.2));
      }

      //kinematics joysticks
      if (ps2_select == 3) {
        if (ps2x.Button(PSB_R3)) {
        } else {
          move_steps_y = map(ps2x.Analog(PSS_LY), 0, 255, (move_steps_max * 1.4), (move_steps_min * 1.4));
          move_pitch_y = 1;
          if (debug1 && move_steps_y) {
            Serial.print(F("move pitch_y "));Serial.println(move_steps_y);
          }

          move_steps_x = map(ps2x.Analog(PSS_LX), 0, 255, (move_steps_max * 1.4), (move_steps_min * 1.4));
          move_roll_x = 1;
          if (debug1 && move_steps_x) {
            Serial.print(F("move roll_x "));Serial.println(move_steps_x);
          }
        }

        if (ps2x.Button(PSB_L3)) {
          //move yaw while button pressed/held
          move_steps_yaw = map(ps2x.Analog(PSS_RX), 0, 255, (move_steps_max * 1.4), (move_steps_min * 1.4));
          if (move_steps_yaw > 2 || move_steps_yaw < -2) {
            move_yaw = 1;
            if (debug1)
              Serial.println(F("move yaw"));
          } else {
            move_yaw = 0;
          }

          //move in y while button pressed/held
          move_steps_ky = map(ps2x.Analog(PSS_RY), 0, 255, (move_steps_min * 1.4), (move_steps_max * 1.4));
          if (move_steps_ky > 2 || move_steps_ky < -2) {
            move_kin_y = 1;
            if (debug1)
              Serial.println(F("move kin_y"));
          } else {
            move_kin_y = 0;
          }
        } else {
          move_steps_yaw_x = map(ps2x.Analog(PSS_RX), 0, 255, (move_steps_max * .5), (move_steps_min * .5));
          if (move_steps_yaw_x > 2 || move_steps_yaw_x < -2) {
            move_yaw_x = 1;
            if (debug1)
              Serial.println(F("move yaw_x"));
          } else {
            move_yaw_x = 0;
          }

          move_steps_yaw_y = map(ps2x.Analog(PSS_RY), 0, 255, (move_steps_max * .8), (move_steps_min * .8));
          if (move_steps_yaw_y > 2 || move_steps_yaw_y < -2) {
            move_yaw_y = 1;
            if (debug1)
              Serial.println(F("move yaw_y"));
          } else {
            move_yaw_y = 0;
          }
        }  
      }


      if (ps2x.Button(PSB_PAD_UP)) {
        if (ps2_select == 1) {
          if (debug1)
            Serial.println(F("forward"));
          if (!move_forward) {
            set_stop();
            if (rgb_active) {
              rgb_request((char*)"Ff");
            }
            
            if (mpu_is_active) mpu_active = 0;
            move_march = 1;
            spd = 12;
            set_speed();
            move_forward = 1;
          }
        } else if (ps2_select == 2) {
        } else if (ps2_select == 3) {
          if (!move_trot) {
            set_stop();
            move_trot = 1;
            if (debug1)
              Serial.println(F("move trot"));
          }
          x_dir = map(ps2x.Analog(PSS_RX), 0, 255, move_steps_min / 4, move_steps_max / 4);
          move_steps = map(ps2x.Analog(PSS_RY), 0, 255, move_steps_max / 2, move_steps_min / 2);
          if (debug2) {
            Serial.print(F("x dir: ")); Serial.print(x_dir);
            Serial.print(F("\tmove steps: ")); Serial.println(move_steps);
          }
        }
      } else if (ps2x.ButtonReleased(PSB_PAD_UP)) {
        if (ps2_select == 1 || ps2_select == 2 || ps2_select == 3) {
          if (move_forward) {
            move_forward = 0;
            if (debug1)
              Serial.println(F("stop forward"));
          }
          if (move_march) {
            if (mpu_is_active) mpu_active = 1;
            move_march = 0;
          }
          if (move_trot) {
            move_trot = 0;
          }
        }
      }
  
      if (ps2x.Button(PSB_PAD_RIGHT)) {
        if (ps2_select == 1) {
          if (!move_right) {
            move_right = 1;
            if (debug1)
              Serial.println(F("move right"));
          }
          x_dir = map(ps2x.Analog(PSS_RX), 0, 255, move_x_steps[0], move_x_steps[1]);
          if (debug2) {
            Serial.print(F("move steps: ")); Serial.println(move_steps);
          }
        }
      } else if (ps2x.ButtonReleased(PSB_PAD_RIGHT)) {
        if (ps2_select == 1) {
          if (move_right) {
            move_right = 0;
            if (debug1)
              Serial.println(F("stop move right"));
          }
        } else if (ps2_select == 2) {
        } else if (ps2_select == 3) {
        }
      }
    
      if (ps2x.Button(PSB_PAD_LEFT)) {
        if (ps2_select == 1) {
          if (!move_left) {
            move_left = 1;
            if (debug1)
              Serial.println(F("move left"));
          }
          x_dir = map(ps2x.Analog(PSS_RX), 0, 255, move_x_steps[1], move_x_steps[0]);
          if (debug2) {
            Serial.print(F("move steps: ")); Serial.println(move_steps);
          }
        }
      } else if (ps2x.ButtonReleased(PSB_PAD_LEFT)) {
        if (ps2_select == 1) {
          if (move_left) {
            move_left = 0;
            if (debug1)
              Serial.println(F("stop move left"));
          }
        } else if (ps2_select == 2) {
        } else if (ps2_select == 3) {
        }
      }  
  
      if (ps2x.ButtonPressed(PSB_PAD_DOWN)) {
        if (ps2_select == 1 || ps2_select == 2 || ps2_select == 3 || ps2_select == 4) {
          if (!ps2x.Button(PSB_L1) && !ps2x.Button(PSB_L2) && !ps2x.Button(PSB_R1) && !ps2x.Button(PSB_R2)) {
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
      } else if (ps2x.ButtonReleased(PSB_PAD_DOWN)) {
        if (ps2_select == 1 || ps2_select == 2 || ps2_select == 3 || ps2_select == 4) {
          if (!ps2x.Button(PSB_L1) && !ps2x.Button(PSB_L2) && !ps2x.Button(PSB_R1) && !ps2x.Button(PSB_R2)) {
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
          }
        }
      }
  
  
      //TRIGGER BUTTONS
      if (ps2x.Button(PSB_L1)) {
        if (ps2_select == 1) {
          if (!move_roll) {
            set_stop();
            move_roll = 1;
            if (debug1)
              Serial.println(F("move roll"));
          }
          move_steps = map(ps2x.Analog(PSS_RY), 0, 255, move_steps_max, move_steps_min);
          if (debug2) {
            Serial.print(F("move steps: ")); Serial.println(move_steps);
          }
        } else if (ps2_select == 2) {
          set_stop();
          if (debug1)
            Serial.println(F("sit"));
          if (rgb_active) {
            rgb_request((char*)"Hi");
          }
          set_sit();
        }
      } else if (ps2x.ButtonReleased(PSB_L1)) {
        if (ps2_select == 1) {
          if (move_roll) {
            move_roll = 0;
            if (debug1)
              Serial.println(F("stop roll"));
          }
        }
      }

      if (ps2x.Button(PSB_L2)) {
        if (ps2_select == 1) {
          if (!move_pitch) {
            set_stop();
            move_pitch = 1;
            if (debug1)
              Serial.println(F("move pitch"));
          }
          x_dir = map(ps2x.Analog(PSS_RX), 0, 255, move_steps_min / 2, move_steps_max / 2);
          move_steps = map(ps2x.Analog(PSS_RY), 0, 255, move_steps_max / 3, move_steps_min / 3);
          if (debug2) {
            Serial.print(F("move steps: ")); Serial.println(move_steps);
          }
        } else if (ps2_select == 2) {
          set_stop();
          if (debug1)
            Serial.println(F("kneel"));
          set_kneel();
        }
      } else if (ps2x.ButtonReleased(PSB_L2)) {
        if (ps2_select == 1) {
          if (move_pitch) {
            move_pitch = 0;
            if (debug1)
              Serial.println(F("stop pitch"));
          }
        }
      }

      if (ps2x.Button(PSB_R1)) {
        if (ps2_select == 1) {
          if (!move_roll_body) {
            set_stop();
            move_roll_body = 1;
            if (debug1)
              Serial.println(F("move roll_body"));
          }
          move_steps = map(ps2x.Analog(PSS_RY), 0, 255, move_steps_max / 2, move_steps_min / 2);
          if (debug2) {
            Serial.print(F("move steps: ")); Serial.println(move_steps);
          }
        } else if (ps2_select == 2) {
          set_stop();
          if (debug1)
            Serial.println(F("crouch"));
          if (rgb_active) {
            rgb_request((char*)"Hh");
          }
          set_crouch();
        }
      } else if (ps2x.ButtonReleased(PSB_R2)) {
        if (ps2_select == 1) {
          if (move_roll_body) {
            move_roll_body = 0;
            if (debug1)
              Serial.println(F("stop roll_body"));
          }
        }
      }

      if (ps2x.Button(PSB_R2)) {
        if (ps2_select == 1) {
          if (!move_pitch_body) {
            set_stop();
            move_pitch_body = 1;
            if (debug1)
              Serial.println(F("move pitch_body"));
          }
          move_steps = map(ps2x.Analog(PSS_RY), 0, 255, move_steps_max, move_steps_min);
          if (debug2) {
            Serial.print(F("move steps: ")); Serial.println(move_steps);
          }
        } else if (ps2_select == 2) {
          set_stop();
          if (debug1)
            Serial.println(F("lay"));
          if (rgb_active) {
            rgb_request((char*)"Kd");
          }
          set_lay();
        }
      } else if (ps2x.ButtonReleased(PSB_R2)) {
        if (ps2_select == 1) {
          if (move_pitch_body) {
            move_pitch_body = 0;
            if (debug1)
              Serial.println(F("stop pitch_body"));
          }
        }
      }
  

      //SHAPE BUTTONS
      if (ps2x.ButtonPressed(PSB_TRIANGLE)) {
        if (ps2_select == 1) {
          set_stop();
          if (!move_y_axis) {
            move_y_axis = 1;
            if (debug1)
              Serial.println(F("move y_axis"));
          }
        } else if (ps2_select == 2) {
          set_stop();
          if (!move_wman) {
            move_wman = 1;
            if (debug1)
              Serial.println(F("move wman"));
          }
        }
      } else if (ps2x.ButtonReleased(PSB_TRIANGLE)) {
        if (ps2_select == 1) {
          if (move_y_axis) {
            move_y_axis = 0;
            if (debug1)
              Serial.println(F("stop y_axis"));
          }
        } else if (ps2_select == 2) {
          if (move_wman) {
            move_wman = 0;
            if (debug1)
              Serial.println(F("stop wman"));
          }
        }
      }

      //poll steps stick
      if (ps2x.Button(PSB_TRIANGLE)) {
        if (ps2_select == 1) {
          move_steps = map(ps2x.Analog(PSS_RY), 0, 255, move_y_steps[0], move_y_steps[1]);
          if (debug2) {
            Serial.print(F("move steps: ")); Serial.print(move_steps);
          }
          if (move_steps < 0) {
            move_steps = map(move_steps, -150, 150, move_x_steps[0], move_x_steps[1]);
          }
          if (debug2) {
            Serial.print(F(" / ")); Serial.println(move_steps);
          }
        }
      }
  
      if (ps2x.ButtonPressed(PSB_CROSS)) {
        if (ps2_select == 1) {
          set_stop();
          if (!move_x_axis) {
            move_x_axis = 1;
            if (debug1)
              Serial.println(F("move x_axis"));
          }
        }
      } else if (ps2x.ButtonReleased(PSB_CROSS)) {
        if (ps2_select == 1) {
          if (move_x_axis) {
            move_x_axis = 0;
            if (debug1)
              Serial.println(F("stop x_axis"));
          }
        }
      }

      //poll steps stick
      if (ps2x.Button(PSB_CROSS)) {
        if (ps2_select == 1) {
          move_steps = map(ps2x.Analog(PSS_RY), 0, 255, move_x_steps[0], move_x_steps[1]);
          if (debug2) {
            Serial.print(F("move steps: ")); Serial.println(move_steps);
          }
        }
      }
  
      if (ps2x.ButtonPressed(PSB_CIRCLE)) {
        if (debug1) {
          Serial.println(F("Circle"));
          Serial.print(F("speed up +1 : "));
        }
        spd -= 1;
        if (spd < max_spd) spd = max_spd;
        set_speed();
        if (debug1) {
          Serial.println(spd);
        }
      }

      if (ps2x.ButtonPressed(PSB_SQUARE)) {
        if (debug1) {
          Serial.println(F("Square"));
          Serial.print(F("speed down -1 : "));
        }
        spd += 1;
        if (spd > min_spd) spd = min_spd;
        set_speed();
        if (debug1)
          Serial.println(spd);
      }
  
  
  
      //LEG / CALIBRATION CONTROLS
      if (ps2_select == 4) {
        //RF
        if (ps2x.Button(PSB_R1)) {
          if (ps2x.Button(PSB_PAD_UP)) {
            if (!activeServo[RFC]) {
              int ms = servoPos[RFC];
              if (ps2x.Button(PSB_TRIANGLE)) {
                ms += 1;
              } else if (ps2x.Button(PSB_CROSS)) {
                ms -= 1;
              }
              update_sequencer(RF, RFC, spd, ms, 0, 0);
              if (debug1) {
                Serial.print(F("RFC: ")); Serial.println(limit_target(RFC, ms, 0));
              }
            }
          } else if (ps2x.Button(PSB_PAD_RIGHT)) {
            if (!activeServo[RFF]) {
              int ms = servoPos[RFF];
              if (ps2x.Button(PSB_TRIANGLE)) {
                ms += 1;
              } else if (ps2x.Button(PSB_CROSS)) {
                ms -= 1;
              }
              update_sequencer(RF, RFF, spd, ms, 0, 0);
              if (debug1) {
                Serial.print(F("RFF: ")); Serial.println(limit_target(RFF, ms, 0));
              }
            }
          } else if (ps2x.Button(PSB_PAD_DOWN)) {
            if (!activeServo[RFT]) {
              int ms = servoPos[RFT];
              if (ps2x.Button(PSB_TRIANGLE)) {
                ms += 1;
              } else if (ps2x.Button(PSB_CROSS)) {
                ms -= 1;
              }
              update_sequencer(RF, RFT, spd, ms, 0, 0);
              if (debug1) {
                Serial.print(F("RFT: ")); Serial.println(limit_target(RFT, ms, 0));
              }
            }
          } else if (ps2x.Button(PSB_PAD_LEFT)) {
            if (!activeServo[RFT]) {
              move_steps = map(ps2x.Analog(PSS_RY), 0, 255, move_c_steps[0] / 1.5, move_c_steps[1] / 1.5);
              update_sequencer(LF, LFT, spd, (servoHome[LFT] + move_steps), 0, 0);
              update_sequencer(LR, LRT, spd, (servoHome[LRT] + move_steps), 0, 0);
              update_sequencer(RF, RFT, spd, (servoHome[RFT] + move_steps), 0, 0);
              update_sequencer(RR, RRT, spd, (servoHome[RRT] + move_steps), 0, 0);
            }
          } else if (ps2x.ButtonReleased(PSB_PAD_UP) || ps2x.ButtonReleased(PSB_PAD_RIGHT) || ps2x.ButtonReleased(PSB_PAD_LEFT) || ps2x.ButtonReleased(PSB_PAD_DOWN)) {
            set_stop_active();
          }
        }
  
        //LF
        if (ps2x.Button(PSB_L1)) {
          if (ps2x.Button(PSB_PAD_UP)) {
            if (!activeServo[LFC]) {
              int ms = servoPos[LFC];
              if (ps2x.Button(PSB_TRIANGLE)) {
                ms += 1;
              } else if (ps2x.Button(PSB_CROSS)) {
                ms -= 1;
              }
              if (debug1) {
                Serial.print(F("LFC: ")); Serial.println(limit_target(LFC, ms, 0));
              }
              update_sequencer(LF, LFC, spd, ms, 0, 0);
            }
          } else if (ps2x.Button(PSB_PAD_RIGHT)) {
            if (!activeServo[LFF]) {
              int ms = servoPos[LFF];
              if (ps2x.Button(PSB_TRIANGLE)) {
                ms += 1;
              } else if (ps2x.Button(PSB_CROSS)) {
                ms -= 1;
              }
              update_sequencer(LF, LFF, spd, ms, 0, 0);
              if (debug1) {
                Serial.print(F("LFF: ")); Serial.println(limit_target(LFF, ms, 0));
              }
            }
          } else if (ps2x.Button(PSB_PAD_DOWN)) {
            if (!activeServo[LFT]) {
              int ms = servoPos[LFT];
              if (ps2x.Button(PSB_TRIANGLE)) {
                ms += 1;
              } else if (ps2x.Button(PSB_CROSS)) {
                ms -= 1;
              }
              update_sequencer(LF, LFT, spd, ms, 0, 0);
              if (debug1) {
                Serial.print(F("LFT: ")); Serial.println(limit_target(LFT, ms, 0));
              }
            }
          } else if (ps2x.Button(PSB_PAD_LEFT)) {
            move_steps++;
            if (!activeServo[RRC]) {
              move_steps = map(ps2x.Analog(PSS_RY), 0, 255, move_c_steps[0] / 1.5, move_c_steps[1] / 1.5);
              update_sequencer(LF, LFC, spd, (servoHome[LFC] + move_steps), 0, 0);
              update_sequencer(LR, LRC, spd, (servoHome[LRC] + move_steps), 0, 0);
              update_sequencer(RF, RFC, spd, (servoHome[RFC] + move_steps), 0, 0);
              update_sequencer(RR, RRC, spd, (servoHome[RRC] + move_steps), 0, 0);
            }
          }
        }
  
        //RR
        if (ps2x.Button(PSB_R2)) {
          if (ps2x.Button(PSB_PAD_UP)) {
            if (!activeServo[RRC]) {
              int ms = servoPos[RRC];
              if (ps2x.Button(PSB_TRIANGLE)) {
                ms += 1;
              } else if (ps2x.Button(PSB_CROSS)) {
                ms -= 1;
              }
              update_sequencer(RR, RRC, spd, ms, 0, 0);
              if (debug1) {
                Serial.print(F("RRC: ")); Serial.println(limit_target(RRC, ms, 0));
              }
            }
          } else if (ps2x.Button(PSB_PAD_RIGHT)) {
            if (!activeServo[RRF]) {
              int ms = servoPos[RRF];
              if (ps2x.Button(PSB_TRIANGLE)) {
                ms += 1;
              } else if (ps2x.Button(PSB_CROSS)) {
                ms -= 1;
              }
              update_sequencer(RR, RRF, spd, ms, 0, 0);
              if (debug1) {
                Serial.print(F("RRF: ")); Serial.println(limit_target(RRF, ms, 0));
              }
            }
          } else if (ps2x.Button(PSB_PAD_DOWN)) {
            if (!activeServo[RRT]) {
              int ms = servoPos[RRT];
              if (ps2x.Button(PSB_TRIANGLE)) {
                ms += 1;
              } else if (ps2x.Button(PSB_CROSS)) {
                ms -= 1;
              }
              update_sequencer(RR, RRT, spd, ms, 0, 0);
              if (debug1) {
                Serial.print(F("RRT: ")); Serial.println(limit_target(RRT, ms, 0));
              }
            }
          } else if (ps2x.Button(PSB_PAD_LEFT)) {
            if (!activeServo[RFC] && !activeServo[RFT]) {
              move_steps = map(ps2x.Analog(PSS_RY), 0, 255, move_c_steps[0] / 1.5, move_c_steps[1] / 1.5);
              if (debug1) {
                Serial.print(F("T/C move steps: ")); Serial.println(move_steps);
              }
              update_sequencer(LF, LFT, spd, (servoHome[LFT] + move_steps), 0, 0);
              update_sequencer(LF, LFC, spd, (servoHome[LFC] + move_steps), 0, 0);
              update_sequencer(LR, LRT, spd, (servoHome[LRT] + move_steps), 0, 0);
              update_sequencer(LR, LRC, spd, (servoHome[LRC] + move_steps), 0, 0);
              update_sequencer(RF, RFT, spd, (servoHome[RFT] + move_steps), 0, 0);
              update_sequencer(RF, RFC, spd, (servoHome[RFC] + move_steps), 0, 0);
              update_sequencer(RR, RRT, spd, (servoHome[RRT] + move_steps), 0, 0);
              update_sequencer(RR, RRC, spd, (servoHome[RRC] + move_steps), 0, 0);
            }
          }
        }
  
        //LR
        if (ps2x.Button(PSB_L2)) {
          if (ps2x.Button(PSB_PAD_UP)) {
            if (!activeServo[LRC]) {
              int ms = servoPos[LRC];
              if (ps2x.Button(PSB_TRIANGLE)) {
                ms += 1;
              } else if (ps2x.Button(PSB_CROSS)) {
                ms -= 1;
              }
              update_sequencer(LR, LRC, spd, ms, 0, 0);
              if (debug1) {
                Serial.print(F("LRC: ")); Serial.println(limit_target(LRC, ms, 0));
              }
            }
          } else if (ps2x.Button(PSB_PAD_RIGHT)) {
            if (!activeServo[LRF]) {
              int ms = servoPos[LRF];
              if (ps2x.Button(PSB_TRIANGLE)) {
                ms += 1;
              } else if (ps2x.Button(PSB_CROSS)) {
                ms -= 1;
              }
              update_sequencer(LR, LRF, spd, ms, 0, 0);
              if (debug1) {
                Serial.print(F("LRF: ")); Serial.println(limit_target(LRF, ms, 0));
              }
            }
          } else if (ps2x.Button(PSB_PAD_DOWN)) {
            if (!activeServo[LRT]) {
              int ms = servoPos[LRT];
              if (ps2x.Button(PSB_TRIANGLE)) {
                ms += 1;
              } else if (ps2x.Button(PSB_CROSS)) {
                ms -= 1;
              }
              update_sequencer(LR, LRT, spd, ms, 0, 0);
              if (debug1) {
                Serial.print(F("LRT: ")); Serial.println(limit_target(LRT, ms, 0));
              }
            }
          } else if (ps2x.Button(PSB_PAD_LEFT)) {
            if (!activeServo[RRF]) {
              move_steps = map(ps2x.Analog(PSS_RY), 0, 255, move_c_steps[0] / 1.5, move_c_steps[1] / 1.5);
              update_sequencer(LF, LFF, spd, (servoHome[LFF] + move_steps), 0, 0);
              update_sequencer(LR, LRF, spd, (servoHome[LRF] + move_steps), 0, 0);
              update_sequencer(RF, RFF, spd, (servoHome[RFF] + move_steps), 0, 0);
              update_sequencer(RR, RRF, spd, (servoHome[RRF] + move_steps), 0, 0);
            }
          }
        }
      }
    }
  } else {
    for (int i = 0; i < 10; i++) {
      ps2x.read_gamepad(false, false);
      delay(100);
    }
    if (rgb_active) {
      rgb_request((char*)"vGMRNRn");
    }
    digitalWrite(OE_PIN, LOW);
    pwm_oe = 1;
  }

  lastPS2Update = millis();
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
            int sp = 1;
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
  
          if (melody_active) {
            play_phrases();
          } else if (buzz_active) {
            for (int b = 3; b > 0; b--) {
              tone(BUZZ, 500);
              delay(70);
              noTone(BUZZ);
              delay(70);
            }
            noTone(BUZZ);
          }
  
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

    if (debug1) {
      Serial.print("F: ");Serial.print(pir_frontState);
      Serial.print("L: ");Serial.print(pir_leftState);
      Serial.print("R: ");Serial.println(pir_rightState);
    }


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
//        delay(3000);
        step_weight_factor = 1.20;
        move_steps = 25;
        if (mpu_is_active) mpu_active = 0;
        move_march = 1;

        pir_frontState = LOW;
        pir_leftState = LOW;
        pir_rightState = LOW;
      }  
  
      if (pir_frontState && !pir_leftState && !pir_rightState) {
        if (debug1)
          Serial.println("go forward");
        y_dir = 15;
      } else if (pir_frontState && pir_leftState && !pir_rightState) {
        if (debug1)
          Serial.println("go forward-left");
        y_dir = 10;
        x_dir = 10;
      } else if (pir_frontState && !pir_leftState && pir_rightState) {
        if (debug1)
          Serial.println("go forward-right");
        y_dir = 10;
        x_dir = -10;
      } else if (!pir_frontState && pir_leftState && !pir_rightState) {
        if (debug1)
          Serial.println("go left");
        x_dir = -15;
      } else if (!pir_frontState && !pir_leftState && pir_rightState) {
        if (debug1)
          Serial.println("go right");
        x_dir = 15;
      } else if (!pir_frontState && pir_leftState && pir_rightState) {
        if (debug1)
          Serial.println("go back");
        y_dir = -15;
      } else if (pir_frontState && pir_leftState && pir_rightState) {
        if (debug1)
          Serial.println("greet");
        move_loops = 2;
        move_switch = 2;
        move_wake = 1;
        move_march = 0;
      } else {
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
      }
    }
  }
}




/*
   -------------------------------------------------------
   Check Ultrasonic Reading
    :provide general description and explanation here
   -------------------------------------------------------
*/
int uss_check() {
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
    ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes - accelBias[1];   
    az = (float)accelCount[2]*aRes - accelBias[2];  
   
    readGyroData(gyroCount);  // Read the x/y/z adc values
    getGres();
 
    // calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes - gyroBias[1];  
    gz = (float)gyroCount[2]*gRes - gyroBias[2];   

    tempCount = readTempData();  // Read the x/y/z adc values
    temperature = ((float) tempCount) / 340. + 36.53; // Temperature in degrees Centigrade
  }  

 /*
    // Print gyro values in degree/sec
    Serial.print("X-gyro rate: "); Serial.print(gx, 1); Serial.print(" degrees/sec "); 
    Serial.print("Y-gyro rate: "); Serial.print(gy, 1); Serial.print(" degrees/sec "); 
    Serial.print("Z-gyro rate: "); Serial.print(gz, 1); Serial.println(" degrees/sec"); 

    // Print acceleration values in milligs!
    Serial.print("X-acceleration: "); Serial.print(1000*ax); Serial.print(" mg "); 
    Serial.print("Y-acceleration: "); Serial.print(1000*ay); Serial.print(" mg "); 
    Serial.print("Z-acceleration: "); Serial.print(1000*az); Serial.println(" mg"); 
    
   // Print temperature in degrees Centigrade      
    Serial.print("Temperature is ");  Serial.print(temperature, 2);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
    Serial.println("");

*/ 
  myaw = gz;
  mroll = gx;
  mpitch = gy;

  if (debug5) {
    Serial.print("mpu x / y / z:\t\t"); Serial.print(mroll); Serial.print(" / "); Serial.print(mpitch); Serial.print(" / "); Serial.println(myaw);
    Serial.println();
  }
    
  //on init mpu, save offsets as defaults for resetting MPU position
  if (mpuInterval != mpuInterval_prev){
    mpuInterval = mpuInterval_prev;
    mpu_mroll = mroll;
    mpu_mpitch = mpitch;
    mpu_myaw = myaw;

    //delay before starting set_axis first time
    delay(1000);
    if (debug) {
      Serial.println(F("\nNova SM3 OK!"));
      Serial.println(F("========================================"));
    
      delay(500);
      if (!plotter && serial_active) {
        Serial.println();
        Serial.println(F("Type a command input or 'h' for help:"));
      }
    }
  }

  set_axis(mroll, mpitch);

  lastMPUUpdate = millis();
}


/*
   -------------------------------------------------------
   Check Amperage Level
    :provide general description and explanation here
   -------------------------------------------------------
*/
void amperage_check(int aloop) {
  unsigned int x=0;
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
void battery_check() {
  int sensorValue = analogRead(BATT_MONITOR);
  batt_voltage = sensorValue * (5.00 / 1023.00) * 2.7; // Convert the reading values from 5v to 12V
  batt_voltage += batt_adjust; //account for voltage drop, adding it back
  if (debug4) {
    Serial.print("batt check: ");Serial.print(sensorValue); Serial.print(F(" / ")); Serial.println(batt_voltage);
  }
  if (batt_voltage < (batt_voltage_prev - .8) && batt_cnt < 3) {
    if (debug4) {
      Serial.print(batt_voltage); Serial.println(F(" - IGNORING BATT DROP of > 0.8v"));
    }
    batt_cnt++;
  } else if (batt_voltage >= (batt_voltage_prev + .05) || batt_voltage <= (batt_voltage_prev - .05)) {
    batt_cnt = 0;
    int batt_danger = 0;
    for (int i = 0; i<4; i++) {
      if (batt_voltage <= batt_levels[i]) {
        batt_danger++;

        rgb_request((char*)"GMONO");
        if (debug4 || debug1) {
          Serial.print(batt_voltage); Serial.print(" - BATTERY LOW - WARNING #"); Serial.println(i);
        }
      }
    }

    if (batt_danger) {
      if (batt_skip < batt_danger || batt_skip > 2) {
        batt_skip = batt_danger;
        if (debug4 || debug1) {
          Serial.print("BUTTON DANGER LEVEL #"); Serial.println(batt_danger);
        }
  
        if (rgb_active) {
          rgb_request((char*)"vn");
        }
        if (buzz_active) {
          for (int b = batt_danger; b > 0; b--) {
            tone(BUZZ, 1000);
            delay(70);
            noTone(BUZZ);
            delay(70);
          }
          noTone(BUZZ);
        }
    
        if (batt_danger == 4) {
          if (debug4 || debug1) {
            Serial.print(batt_voltage); Serial.println(F(" - HALTED!"));
            tone(BUZZ, 4000);
          }
          rgb_request((char*)"ADn");
          while(1);
        } else if (batt_danger < 3) {
          batt_voltage_prev = batt_voltage;
        }
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
  for (int m = 0; m < TOTAL_SERVOS; m++) {
    activeServo[m] = 0;
    activeSweep[m] = 0;
  }
  set_home();
}

void set_stop_active() {
  for (int m = 0; m < TOTAL_SERVOS; m++) {
    activeServo[m] = 0;
    activeSweep[m] = 0;
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

    //check if oscillating, and on third oscillation, stop!
    if (ar > (mroll_prev + mpu_oscill_thresh) || ar < (mroll_prev - mpu_oscill_thresh)) {
      mpu_oscill_cnt--;
//      Serial.print("oscillate: ");
//      Serial.println(mpu_oscill_cnt);
    } else {
//      mpu_oscill_grace--;
//      if (!mpu_oscill_grace) {
//        mpu_oscill_grace = 3;
        mpu_oscill_cnt = mpu_oscill_limit;
//      }
    }

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
            t += ((abs(roll_step) * 0.65) * 4);
          } else if (is_femur(i)) {
            f -= ((abs(roll_step) * 0.4) * 4);
          }
        } else { //roll right
          if (is_tibia(i)) {
            t -= ((roll_step * 0.65) * 4);
          } else if (is_femur(i)) {
            f += ((roll_step * 0.4) * 4);
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
        if (!mpu_oscill_cnt) {
          mpu_oscill_cnt = mpu_oscill_limit;
//Serial.println("stop oscill delay");  
//maybe send home instead?    
//          delay(2000);
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
    if (buzz_active) {
      play_phrases();
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
  }
}



/*
   -------------------------------------------------------
   (implied) Kinematics Functions
   -------------------------------------------------------
*/
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

void roll() {
  if (!activeSweep[RRF]) {
    servoSpeed[LFT] = limit_speed((10 * spd_factor));
    servoSweep[LFT][0] = limit_target(LFT, (servoHome[LFT] - move_steps), 0);
    servoSweep[LFT][1] = limit_target(LFT, (servoHome[LFT] + move_steps), 0);
    servoSweep[LFT][2] = 0;
    servoSweep[LFT][3] = 1;
    targetPos[LFT] = servoSweep[LFT][1];
    activeSweep[LFT] = 1;

    servoSpeed[LFF] = limit_speed((16 * spd_factor));
    servoSweep[LFF][0] = limit_target(LFF, (servoHome[LFF] + (move_steps * .667)), 0);
    servoSweep[LFF][1] = limit_target(LFF, (servoHome[LFF] - (move_steps * .667)), 0);
    servoSweep[LFF][2] = 0;
    servoSweep[LFF][3] = 1;
    targetPos[LFF] = servoSweep[LFF][1];
    activeSweep[LFF] = 1;


    servoSpeed[LRT] = limit_speed((10 * spd_factor));
    servoSweep[LRT][0] = limit_target(LRT, (servoHome[LRT] - move_steps), 0);
    servoSweep[LRT][1] = limit_target(LRT, (servoHome[LRT] + move_steps), 0);
    servoSweep[LRT][2] = 0;
    servoSweep[LRT][3] = 1;
    targetPos[LRT] = servoSweep[LRT][1];
    activeSweep[LRT] = 1;

    servoSpeed[LRF] = limit_speed((16 * spd_factor));
    servoSweep[LRF][0] = limit_target(LRF, (servoHome[LRF] + (move_steps * .667)), 0);
    servoSweep[LRF][1] = limit_target(LRF, (servoHome[LRF] - (move_steps * .667)), 0);
    servoSweep[LRF][2] = 0;
    servoSweep[LRF][3] = 1;
    targetPos[LRF] = servoSweep[LRF][1];
    activeSweep[LRF] = 1;


    servoSpeed[RFT] = limit_speed((10 * spd_factor));
    servoSweep[RFT][0] = limit_target(RFT, (servoHome[RFT] - move_steps), 0);
    servoSweep[RFT][1] = limit_target(RFT, (servoHome[RFT] + move_steps), 0);
    servoSweep[RFT][2] = 0;
    servoSweep[RFT][3] = 1;
    targetPos[RFT] = servoSweep[RFT][1];
    activeSweep[RFT] = 1;

    servoSpeed[RFF] = limit_speed((16 * spd_factor));
    servoSweep[RFF][0] = limit_target(RFF, (servoHome[RFF] + (move_steps * .667)), 0);
    servoSweep[RFF][1] = limit_target(RFF, (servoHome[RFF] - (move_steps * .667)), 0);
    servoSweep[RFF][2] = 0;
    servoSweep[RFF][3] = 1;
    targetPos[RFF] = servoSweep[RFF][1];
    activeSweep[RFF] = 1;


    servoSpeed[RRT] = limit_speed((10 * spd_factor));
    servoSweep[RRT][0] = limit_target(RRT, (servoHome[RRT] - move_steps), 0);
    servoSweep[RRT][1] = limit_target(RRT, (servoHome[RRT] + move_steps), 0);
    servoSweep[RRT][2] = 0;
    servoSweep[RRT][3] = 1;
    targetPos[RRT] = servoSweep[RRT][1];
    activeSweep[RRT] = 1;

    servoSpeed[RRF] = limit_speed((16 * spd_factor));
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
    servoSpeed[LFC] = limit_speed((10 * spd_factor));
    servoSweep[LFC][0] = limit_target(LFC, (servoHome[LFC] - move_steps), 0);
    servoSweep[LFC][1] = limit_target(LFC, (servoHome[LFC] + move_steps), 0);
    servoSweep[LFC][2] = 0;
    servoSweep[LFC][3] = 1;
    targetPos[LFC] = servoSweep[LFC][1];
    activeSweep[LFC] = 1;

    servoSpeed[LRC] = limit_speed((10 * spd_factor));
    servoSweep[LRC][0] = limit_target(LRC, (servoHome[LRC] - move_steps), 0);
    servoSweep[LRC][1] = limit_target(LRC, (servoHome[LRC] + move_steps), 0);
    servoSweep[LRC][2] = 0;
    servoSweep[LRC][3] = 1;
    targetPos[LRC] = servoSweep[LRC][1];
    activeSweep[LRC] = 1;

    servoSpeed[RFC] = limit_speed((10 * spd_factor));
    servoSweep[RFC][0] = limit_target(RFC, (servoHome[RFC] - move_steps), 0);
    servoSweep[RFC][1] = limit_target(RFC, (servoHome[RFC] + move_steps), 0);
    servoSweep[RFC][2] = 0;
    servoSweep[RFC][3] = 1;
    targetPos[RFC] = servoSweep[RFC][1];
    activeSweep[RFC] = 1;

    servoSpeed[RRC] = limit_speed((10 * spd_factor));
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

void roll_x() {
  update_sequencer(LF, LFC, spd, (servoHome[LFC] + move_steps_x), 0, 0);
  update_sequencer(LR, LRC, spd, (servoHome[LRC] + move_steps_x), 0, 0);
  update_sequencer(RF, RFC, spd, (servoHome[RFC] + move_steps_x), 0, 0);
  update_sequencer(RR, RRC, spd, (servoHome[RRC] + move_steps_x), 0, 0);

  move_roll_x = 0;

  lastMoveDelayUpdate = millis();  
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
    servoSpeed[LFT] = limit_speed((7 * spd_factor));
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

    servoSpeed[RFT] = limit_speed((7 * spd_factor));
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

    servoSpeed[LRT] = limit_speed((7 * spd_factor));
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

    servoSpeed[RRT] = limit_speed((7 * spd_factor));
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

    update_sequencer(LF, LFC, limit_speed((7 * spd_factor)), servoStepMoves[LFC][0], 0, 0);
    update_sequencer(RF, RFC, limit_speed((7 * spd_factor)), servoStepMoves[RFC][0], 0, 0);
    update_sequencer(LR, LRC, limit_speed((7 * spd_factor)), servoStepMoves[LRC][0], 0, 0);
    update_sequencer(RR, RRC, limit_speed((7 * spd_factor)), servoStepMoves[RRC][0], 0, 0);

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
    servoSpeed[LFC] = limit_speed((68 * spd_factor));
    servoSweep[LFC][0] = (servoHome[LFC]);
    servoSweep[LFC][1] = limit_target(LFC, (servoHome[LFC] + (move_steps * .05)), 0);
    servoSweep[LFC][2] = 0;
    servoSweep[LFC][3] = 1;
    targetPos[LFC] = servoSweep[LFC][1];
    activeSweep[LFC] = 1;

    servoSpeed[LFT] = limit_speed((10 * spd_factor));
    servoSweep[LFT][0] = limit_target(LFT, (servoHome[LFT] - (move_steps * .35)), 0);
    servoSweep[LFT][1] = limit_target(LFT, (servoHome[LFT] + (move_steps * .35)), 0);
    servoSweep[LFT][2] = 0;
    servoSweep[LFT][3] = 1;
    targetPos[LFT] = servoSweep[LFT][1];
    activeSweep[LFT] = 1;

    servoSpeed[LFF] = limit_speed((17 * spd_factor));
    servoSweep[LFF][0] = limit_target(LFF, (servoHome[LFF] + (move_steps * .2)), 0);
    servoSweep[LFF][1] = limit_target(LFF, (servoHome[LFF] - (move_steps * .2)), 0);
    servoSweep[LFF][2] = 0;
    servoSweep[LFF][3] = 1;
    targetPos[LFF] = servoSweep[LFF][1];
    activeSweep[LFF] = 1;


    servoSpeed[RFC] = limit_speed((68 * spd_factor));
    servoSweep[RFC][0] = (servoHome[RFC]);
    servoSweep[RFC][1] = limit_target(RFC, (servoHome[RFC] - (move_steps * .05)), 0);
    servoSweep[RFC][2] = 0;
    servoSweep[RFC][3] = 1;
    targetPos[RFC] = servoSweep[RFC][1];
    activeSweep[RFC] = 1;

    servoSpeed[RFT] = limit_speed((10 * spd_factor));
    servoSweep[RFT][0] = limit_target(RFT, (servoHome[RFT] + (move_steps * .35)), 0);
    servoSweep[RFT][1] = limit_target(RFT, (servoHome[RFT] - (move_steps * .35)), 0);
    servoSweep[RFT][2] = 0;
    servoSweep[RFT][3] = 1;
    targetPos[RFT] = servoSweep[RFT][1];
    activeSweep[RFT] = 1;

    servoSpeed[RFF] = limit_speed((17 * spd_factor));
    servoSweep[RFF][0] = limit_target(RFF, (servoHome[RFF] - (move_steps * .2)), 0);
    servoSweep[RFF][1] = limit_target(RFF, (servoHome[RFF] + (move_steps * .2)), 0);
    servoSweep[RFF][2] = 0;
    servoSweep[RFF][3] = 1;
    targetPos[RFF] = servoSweep[RFF][1];
    activeSweep[RFF] = 1;


    servoSpeed[LRC] = limit_speed((68 * spd_factor));
    servoSweep[LRC][0] = limit_target(LRC, (servoHome[LRC] - (move_steps * .05)), 0);
    servoSweep[LRC][1] = (servoHome[LRC]);
    servoSweep[LRC][2] = 0;
    servoSweep[LRC][3] = 1;
    targetPos[LRC] = servoSweep[LRC][1];
    activeSweep[LRC] = 1;

    servoSpeed[LRT] = limit_speed((10 * spd_factor));
    servoSweep[LRT][0] = limit_target(LRT, (servoHome[LRT] + (move_steps * .35)), 0);
    servoSweep[LRT][1] = limit_target(LRT, (servoHome[LRT] - (move_steps * .35)), 0);
    servoSweep[LRT][2] = 0;
    servoSweep[LRT][3] = 1;
    targetPos[LRT] = servoSweep[LRT][1];
    activeSweep[LRT] = 1;

    servoSpeed[LRF] = limit_speed((17 * spd_factor));
    servoSweep[LRF][0] = limit_target(LRF, (servoHome[LRF] - (move_steps * .2)), 0);
    servoSweep[LRF][1] = limit_target(LRF, (servoHome[LRF] + (move_steps * .2)), 0);
    servoSweep[LRF][2] = 0;
    servoSweep[LRF][3] = 1;
    targetPos[LRF] = servoSweep[LRF][1];
    activeSweep[LRF] = 1;


    servoSpeed[RRC] = limit_speed((68 * spd_factor));
    servoSweep[RRC][0] = limit_target(RRC, (servoHome[RRC] + (move_steps * .05)), 0);
    servoSweep[RRC][1] = (servoHome[RRC]);
    servoSweep[RRC][2] = 0;
    servoSweep[RRC][3] = 1;
    targetPos[RRC] = servoSweep[RRC][1];
    activeSweep[RRC] = 1;

    servoSpeed[RRT] = limit_speed((10 * spd_factor));
    servoSweep[RRT][0] = limit_target(RRT, (servoHome[RRT] - (move_steps * .35)), 0);
    servoSweep[RRT][1] = limit_target(RRT, (servoHome[RRT] + (move_steps * .35)), 0);
    servoSweep[RRT][2] = 0;
    servoSweep[RRT][3] = 1;
    targetPos[RRT] = servoSweep[RRT][1];
    activeSweep[RRT] = 1;

    servoSpeed[RRF] = limit_speed((17 * spd_factor));
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

  int lfms = fms;
  int rfms = fms;
  int ltms = tms;
  int rtms = tms;
  if (move_steps_yaw < 0) {
    lfms = (move_steps_yaw * 0.5);
    ltms = (move_steps_yaw * 0.4);
  } else {
    rfms = (move_steps_yaw * 0.5);
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
    ftms = (move_steps_yaw_y * 0.6);
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


/*
   -------------------------------------------------------
   Walking Gait Functions
   -------------------------------------------------------
*/
void step_trot(int xdir) {
//DEVWORK

//  ramp_dist = 0.3;
//  ramp_spd = 1.3;
//  use_ramp = 1;  

  float sinc0 = .35;
  float sinc1 = 1.15;

  float sinc2 = .35;
  float sinc3 = 1.15;

  //set coaxes by direction
  servoStepMoves[RFC][0] = servoHome[RFC];
  servoStepMoves[LFC][0] = servoHome[LFC];
  servoStepMoves[RFF][0] = servoHome[RFF];
  servoStepMoves[LFF][0] = servoHome[LFF];
  if (xdir < -1) {  //turn left
    xdir = abs(xdir);
    servoStepMoves[RFC][0] = limit_target(RFC, (servoHome[RFC] - xdir), 15);
    servoStepMoves[LFC][0] = limit_target(LFC, (servoHome[LFC] - (xdir * 0.6)), 20);
    servoStepMoves[RFF][0] = limit_target(RFF, (servoHome[RFF] - (xdir * 0.6)), 0);
    servoStepMoves[LFF][0] = limit_target(LFF, (servoHome[LFF] - (xdir * 2)), 0);

    sinc0 = .25;
    sinc1 = 1.05;
//    sinc2 = .05;
//    sinc3 = 1.05;
  } else if (xdir > +1) {  //turn right
    servoStepMoves[RFC][0] = limit_target(RFC, (servoHome[RFC] + (xdir * 0.6)), 20);
    servoStepMoves[LFC][0] = limit_target(LFC, (servoHome[LFC] + xdir), 15);
    servoStepMoves[RFF][0] = limit_target(RFF, (servoHome[RFF] + (xdir * 2)), 0);
    servoStepMoves[LFF][0] = limit_target(LFF, (servoHome[LFF] + (xdir * 0.6)), 0);

    sinc0 = .25;
    sinc1 = 1.05;
//    sinc2 = .05;
//    sinc3 = 1.05;
  }

  //set tibia sweep movements
  if (!activeSweep[RRT]) {
    update_sequencer(LF, LFC, limit_speed((7 * spd_factor)), servoStepMoves[LFC][0], 0, 0);
    update_sequencer(RF, RFC, limit_speed((7 * spd_factor)), servoStepMoves[RFC][0], 0, 0);
    update_sequencer(LF, LFF, limit_speed((7 * spd_factor)), servoStepMoves[LFF][0], 0, 0);
    update_sequencer(RF, RFF, limit_speed((7 * spd_factor)), servoStepMoves[RFF][0], 0, 0);
    
    servoSpeed[LFT] = limit_speed((7 * spd_factor));
    servoSweep[LFT][0] = limit_target(LFT, (servoHome[LFT] - (move_steps * sinc0)), 0);
    servoSweep[LFT][1] = limit_target(LFT, (servoHome[LFT] + (move_steps * sinc1)), 0);
    servoSweep[LFT][2] = 0;
    servoSweep[LFT][3] = 1;
    targetPos[LFT] = servoSweep[LFT][1];
    activeSweep[LFT] = 1;

    servoSpeed[RFT] = limit_speed((7 * spd_factor));
    servoSweep[RFT][0] = limit_target(RFT, (servoHome[RFT] + (move_steps * sinc0)), 0);
    servoSweep[RFT][1] = limit_target(RFT, (servoHome[RFT] - (move_steps * sinc1)), 0);
    servoSweep[RFT][2] = 0;
    servoSweep[RFT][3] = 1;
    targetPos[RFT] = servoSweep[RFT][1];
    activeSweep[RFT] = 1;

    servoSpeed[LRT] = limit_speed((7 * spd_factor));
    servoSweep[LRT][0] = limit_target(LRT, (servoHome[LRT] + (move_steps * sinc2)), 0);
    servoSweep[LRT][1] = limit_target(LRT, (servoHome[LRT] - (move_steps * sinc3)), 0);
    servoSweep[LRT][2] = 0;
    servoSweep[LRT][3] = 1;
    targetPos[LRT] = servoSweep[LRT][1];
    activeSweep[LRT] = 1;

    servoSpeed[RRT] = limit_speed((7 * spd_factor));
    servoSweep[RRT][0] = limit_target(RRT, (servoHome[RRT] - (move_steps * sinc2)), 0);
    servoSweep[RRT][1] = limit_target(RRT, (servoHome[RRT] + (move_steps * sinc3)), 0);
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

void step_forward(int xdir) {
  int lturn = 0;
  int rturn = 0;
  int lfsf = 0;
  int ltsf = 0;
  int rfsf = 0;
  int rtsf = 0;
  if (xdir < 0) {  //turn left
    lturn = 10;
    rfsf = spd_factor * 0.25;
    rtsf = spd_factor * 0.5;
  } else if (xdir > 0) {  //turn right
    rturn = 10;
    lfsf = spd_factor * 0.25;
    ltsf = spd_factor * 0.5;
  }

  int s1c = 0;
  int s1f = 15;
  int s1t = 35;

  int s2c = 0;
  int s2f = 20 + move_steps;
  int s2t = 15 + move_steps;

  int s3c = 0;
  int s3f = 30 + move_steps;
  int s3t = 10 + move_steps;

  int s4c = 0;
  int s4f = 0;
  int s4t = 0;

  //RF & LR
  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && !servoSequence[RF]) {
    update_sequencer(RF, RFC, (0*spd_factor), (servoHome[RFC] + s1c), (servoSequence[RF] + 1), 0);
    update_sequencer(RF, RFF, (4*spd_factor), (servoHome[RFF] - s1f), servoSequence[RF], 0);
    update_sequencer(RF, RFT, (3*spd_factor), (servoHome[RFT] + s1t), servoSequence[RF], 0);

    update_sequencer(LR, LRC, (0*spd_factor), (servoHome[LRC] - s1c), (servoSequence[LR] + 1), 0);
    update_sequencer(LR, LRF, (4*spd_factor), (servoHome[LRF] + (s1f * step_weight_factor)), servoSequence[LR], 0);
    update_sequencer(LR, LRT, (3*spd_factor), (servoHome[LRT] - (s1t * step_weight_factor)), servoSequence[LR], 0);
  }
  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 1) {
    update_sequencer(RF, RFC, (0*spd_factor), (servoHome[RFC] - s2c), (servoSequence[RF] + 1), 0);
    update_sequencer(RF, RFF, (3*(spd_factor-rfsf)), (servoHome[RFF] + s2f - rturn), servoSequence[RF], 0);
    update_sequencer(RF, RFT, (6*spd_factor), (servoHome[RFT] + s2t), servoSequence[RF], 0);

    update_sequencer(LR, LRC, (0*spd_factor), (servoHome[LRC] + s2c), (servoSequence[LR] + 1), 0);
    update_sequencer(LR, LRF, (3*spd_factor), (servoHome[LRF] - (s2f * step_weight_factor)), servoSequence[LR], 0);
    update_sequencer(LR, LRT, (6*spd_factor), (servoHome[LRT] - (s2t * step_weight_factor)), servoSequence[LR], 0);
  }
  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 2) {
    update_sequencer(RF, RFC, (3*spd_factor), (servoHome[RFC] - s3c), (servoSequence[RF] + 1), 0);
    update_sequencer(RF, RFF, (3*spd_factor), (servoHome[RFF] + s3f), servoSequence[RF], 0);
    update_sequencer(RF, RFT, (3*spd_factor), (servoHome[RFT] - s3t), servoSequence[RF], 0);

    update_sequencer(LR, LRC, (3*spd_factor), (servoHome[LRC] + s3c), (servoSequence[LR] + 1), 0);
    update_sequencer(LR, LRF, (3*spd_factor), (servoHome[LRF] - (s3f * step_weight_factor)), servoSequence[LR], 0);
    update_sequencer(LR, LRT, (3*spd_factor), (servoHome[LRT] + (s3t * step_weight_factor)), servoSequence[LR], 0);
  }
  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 3) {
    update_sequencer(RF, RFC, (10*spd_factor), (servoHome[RFC] - s4c), 0, 0);
    update_sequencer(RF, RFF, (10*(spd_factor-rtsf)), (servoHome[RFF] + s4f), 0, 0);
    update_sequencer(RF, RFT, (15*(spd_factor-rtsf)), (servoHome[RFT] - s4t), 0, 0);

    update_sequencer(LR, LRC, (10*spd_factor), (servoHome[LRC] + s4c), 0, 0);
    update_sequencer(LR, LRF, (10*(spd_factor-lfsf)), (servoHome[LRF] - (s4f * step_weight_factor)), 0, 0);
    update_sequencer(LR, LRT, (15*(spd_factor-ltsf)), (servoHome[LRT] + (s4t * step_weight_factor)), 0, 0);
  }

  //LF & RR
  if (!activeServo[LFC] && !activeServo[LFF] && !activeServo[LFT] && !servoSequence[LF] && servoSequence[LR] == 3) {
    update_sequencer(RR, RRC, (0*spd_factor), (servoHome[RRC] + s1c), (servoSequence[RR] + 1), 0);
    update_sequencer(RR, RRF, (4*spd_factor), (servoHome[RRF] - (s1f * step_weight_factor)), servoSequence[RR], 0);
    update_sequencer(RR, RRT, (3*spd_factor), (servoHome[RRT] + (s1t * step_weight_factor)), servoSequence[RR], 0);

    update_sequencer(LF, LFC, (0*spd_factor), (servoHome[LFC] - s1c), (servoSequence[LF] + 1), 0);
    update_sequencer(LF, LFF, (4*spd_factor), (servoHome[LFF] + s1f), servoSequence[LF], 0);
    update_sequencer(LF, LFT, (3*spd_factor), (servoHome[LFT] - s1t), servoSequence[LF], 0);
  }
  if (!activeServo[LFC] && !activeServo[LFF] && !activeServo[LFT] && servoSequence[LF] == 1) {
    update_sequencer(RR, RRC, (0*spd_factor), (servoHome[RRC] - s2c), (servoSequence[RR] + 1), 0);
    update_sequencer(RR, RRF, (3*(spd_factor-rfsf)), (servoHome[RRF] + (s2f * step_weight_factor) - rturn), servoSequence[RR], 0);
    update_sequencer(RR, RRT, (6*spd_factor), (servoHome[RRT] + (s2t * step_weight_factor)), servoSequence[RR], 0);

    update_sequencer(LF, LFC, (0*spd_factor), (servoHome[LFC] + s2c), (servoSequence[LF] + 1), 0);
    update_sequencer(LF, LFF, (3*spd_factor), (servoHome[LFF] - s2f), servoSequence[LF], 0);
    update_sequencer(LF, LFT, (6*spd_factor), (servoHome[LFT] - s2t), servoSequence[LF], 0);
  }
  if (!activeServo[LFC] && !activeServo[LFF] && !activeServo[LFT] && servoSequence[LF] == 2) {
    update_sequencer(RR, RRC, (3*spd_factor), (servoHome[RRC] - s3c), (servoSequence[RR] + 1), 0);
    update_sequencer(RR, RRF, (3*spd_factor), (servoHome[RRF] + (s3f * step_weight_factor)), servoSequence[RR], 0);
    update_sequencer(RR, RRT, (3*spd_factor), (servoHome[RRT] - (s3t * step_weight_factor)), servoSequence[RR], 0);

    update_sequencer(LF, LFC, (3*spd_factor), (servoHome[LFC] + s3c), (servoSequence[LF] + 1), 0);
    update_sequencer(LF, LFF, (3*spd_factor), (servoHome[LFF] - s3f), servoSequence[LF], 0);
    update_sequencer(LF, LFT, (3*spd_factor), (servoHome[LFT] + s3t), servoSequence[LF], 0);
  }
  if (!activeServo[LFC] && !activeServo[LFF] && !activeServo[LFT] && servoSequence[LF] == 3) {
    update_sequencer(RR, RRC, (10*spd_factor), (servoHome[RRC] - s4c), 0, 0);
    update_sequencer(RR, RRF, (10*spd_factor), (servoHome[RRF] + (s4f * step_weight_factor)), 0, 0);
    update_sequencer(RR, RRT, (15*spd_factor), (servoHome[RRT] - (s4t * step_weight_factor)), 0, 0);

    update_sequencer(LF, LFC, (10*spd_factor), (servoHome[LFC] + s4c), 0, 0);
    update_sequencer(LF, LFF, (10*spd_factor), (servoHome[LFF] - s4f), 0, 0);
    update_sequencer(LF, LFT, (15*spd_factor), (servoHome[LFT] + s4t), 0, 0);

    lastMoveDelayUpdate = millis();  
  }

}

void step_backward(int xdir) {
//this is probably not needed, since backwards is just opposite of forwards
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
//  ramp_dist = 0.2;
//  ramp_spd = 0.3;
//  use_ramp = 1;  

  moving = 1;

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
  //
  float gaitHome[TOTAL_SERVOS];
  for (int i = 0; i < TOTAL_SERVOS; i++) {
    gaitHome[i] = servoHome[i];
  }

  //apply z factors by direction, not mid-sequence
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
  
  //calculate steps, speeds, direction, and step_weight_factor
  float servoMS[TOTAL_SERVOS][6][2];   //[servo][step1,step2,etc][dist,spd]

  //coaxes
  //set default marching steps/speed
  servoMS[RFC][0][0] = gaitHome[RFC];
  servoMS[RRC][0][0] = gaitHome[RRC];
  servoMS[LFC][0][0] = gaitHome[LFC];
  servoMS[LRC][0][0] = gaitHome[LRC];
  servoMS[RFC][0][1] = servoMS[RFC][0][1] = servoMS[RRC][0][1] = servoMS[RRC][0][1] = (csfact * spd_factor);
  servoMS[RFC][1][1] = servoMS[RFC][1][1] = servoMS[RRC][1][1] = servoMS[RRC][1][1] = ((csfact * spd_factor) / sfact_div);

  //calc movement and apply weight factor by direction / load shift
  if (xdir < -1) { //turn left
    servoMS[RFC][0][0] = (gaitHome[RFC] + (abs(xdir) * cmfact));
    servoMS[RRC][0][0] = (gaitHome[RRC] + ((abs(xdir) * cmfact) + ((abs(xdir) * cmfact) * (step_weight_factor * cmfact)))); //add weight factor
    servoMS[LFC][0][0] = (gaitHome[LFC] + (abs(xdir) * cmfact));
    servoMS[LRC][0][0] = (gaitHome[LRC] + ((abs(xdir) * cmfact) + ((abs(xdir) * cmfact) * (step_weight_factor * cmfact)))); //add weight factor

    servoMS[RFC][0][1] = servoMS[LFC][0][1] = (csfact * spd_factor);
    servoMS[RRC][0][1] = servoMS[LRC][0][1] = ((csfact - (step_weight_factor * csfact)) * spd_factor);
  } else if (xdir > 1) { //turn right
    servoMS[RFC][0][0] = (gaitHome[RFC] - (xdir * cmfact));
    servoMS[RRC][0][0] = (gaitHome[RRC] - ((xdir * cmfact) + ((xdir * cmfact) * (step_weight_factor * cmfact)))); //add weight factor
    servoMS[LFC][0][0] = (gaitHome[LFC] - (xdir * cmfact));
    servoMS[LRC][0][0] = (gaitHome[LRC] - ((xdir * cmfact) + ((xdir * cmfact) * (step_weight_factor * cmfact)))); //add weight factor

    servoMS[RFC][0][1] = servoMS[LFC][0][1] = (csfact * spd_factor);
    servoMS[RRC][0][1] = servoMS[LRC][0][1] = ((csfact * spd_factor) - (csfact * (step_weight_factor-1)));
  }


  //femurs
  //set default marching steps/speed and step_weight_factor
  servoMS[RFF][0][0] = (gaitHome[RFF] - (move_steps * fmfact));
  servoMS[RRF][0][0] = (gaitHome[RRF] - ((move_steps * fmfact) * step_weight_factor));
  servoMS[LFF][0][0] = (gaitHome[LFF] + (move_steps * fmfact));
  servoMS[LRF][0][0] = (gaitHome[LRF] + ((move_steps * fmfact) * step_weight_factor));
  servoMS[RFF][0][1] = servoMS[LFF][0][1] = (fsfact * spd_factor);
  servoMS[RRF][0][1] = servoMS[LRF][0][1] = ((fsfact * spd_factor) - (fsfact * (step_weight_factor - 1)));
  servoMS[RFF][1][1] = servoMS[LFF][1][1] = servoMS[RRF][1][1] = servoMS[LRF][1][1] = ((fsfact * spd_factor) / sfact_div);

  //adjust step_weight_factor on direction
  if (ydir < -1) { //reverse
    servoMS[RFF][0][0] = (gaitHome[RFF] - ((move_steps * fmfact) * step_weight_factor));
    servoMS[RRF][0][0] = (gaitHome[RRF] - (move_steps * fmfact));
    servoMS[LFF][0][0] = (gaitHome[LFF] + ((move_steps * fmfact) * step_weight_factor));
    servoMS[LRF][0][0] = (gaitHome[LRF] + (move_steps * fmfact));
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
  servoMS[RRT][0][0] = (gaitHome[RRT] + ((move_steps * tmfact) * step_weight_factor));
  servoMS[LFT][0][0] = (gaitHome[LFT] - (move_steps * tmfact));
  servoMS[LRT][0][0] = (gaitHome[LRT] - ((move_steps * tmfact) * step_weight_factor));
  servoMS[RFT][0][1] = servoMS[RRT][0][1] = servoMS[LFT][0][1] = servoMS[LRT][0][1] = (tsfact * spd_factor);
  servoMS[RFT][1][1] = servoMS[RRT][1][1] = servoMS[LFT][1][1] = servoMS[LRT][1][1] = ((tsfact * spd_factor) / sfact_div);


  //STEP SEQUENCING
  //to start sequencer, step first paired legs upwards (SEQ1)
  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && !servoSequence[RF]) {
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
  if (!activeServo[RFC] && !activeServo[RFF] && !activeServo[RFT] && servoSequence[RF] == 1) {
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



void wake() {
  int del_cnt = 0;

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
      update_sequencer(LR, LRT, 1, ((servoHome[LRT] - servoStepMoves[LRT][0]) - (servoStepMoves[LRT][0]*step_weight_factor)), servoSequence[LR], 0);
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
      update_sequencer(RR, RRT, 1, ((servoHome[RRT] + servoStepMoves[RRT][0]) + (servoStepMoves[RRT][0]*step_weight_factor)), servoSequence[RR], 0);
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
    spd = default_spd;
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
  int pulse_wide, analog_value;
  pulse_wide = map(pangle, -rng/2, rng/2, mnw, mxw);

  return pulse_wide;
}

int pwm_to_degrees(int pulse_wide, int mxw, int mnw, int rng) {
  int pangle, analog_value;
  pangle = map(pulse_wide, mnw, mxw, -rng/2, rng/2);

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
  
    if (serial_input.length() > 0) {  //end read
      serial_command(serial_input);
      serial_input="";
    } 
  }
}


void serial_command(String cmd) {
  if (cmd) {
      if (cmd == "stop" || cmd == "0") {
        if (!plotter) Serial.println(F("stop!"));
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
      } else if (cmd == "vars") {
        if (!plotter) {
          Serial.println();
          Serial.println(F("---------------------------------------"));
          Serial.println(F("SETTINGS:"));
          Serial.println(F("---------------------------------------"));
          Serial.print(F("\tspd:\t\t\t"));Serial.println(spd);
          Serial.print(F("\tspd_factor:\t\t"));Serial.println(spd_factor);
          Serial.print(F("\tmove_steps:\t\t"));Serial.println(move_steps);
          Serial.print(F("\tx_dir:\t\t\t"));Serial.println(x_dir);
          Serial.print(F("\ty_dir:\t\t\t"));Serial.println(y_dir);
          Serial.print(F("\tz_dir:\t\t\t"));Serial.println(z_dir);
          Serial.print(F("\tstep_weight_factor:\t"));Serial.println(step_weight_factor);
          Serial.print(F("\tstep_height_factor:\t"));Serial.println(step_height_factor);
          Serial.print(F("\tdebug_servo:\t\t"));Serial.println(debug_servo);
          Serial.print(F("\tdebug_leg:\t\t"));Serial.println(debug_leg);
          Serial.println(F("---------------------------------------"));
          Serial.println();
          Serial.println();

          Serial.println(F("---------------------------------------"));
          Serial.println(F("RAMP SETTINGS:"));
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
      } else if (cmd == "oled") {
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
      } else if (cmd == "trot") {
//DEVWORK
        if (!plotter) Serial.println(F("trot"));
        spd = 5;
        set_speed();
        move_steps = 35;
        x_dir = 0;
        move_trot = 1;
      } else if (cmd == "march") {
//DEVWORK
        if (!plotter) Serial.println(F("march"));        
        spd = 5;
        set_speed();
        y_dir = 0;
        x_dir = 0;
        z_dir = 0;
        step_weight_factor = 1.20;
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
      } else if (cmd == "wake" || cmd == "a") {
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
      } else if (cmd == "servo" || cmd == "g") {
        if (!plotter) { Serial.print("debug servo ");Serial.println(debug_servo); }
        debug_loops2 = debug_loops;
        move_servo = 1;
      } else if (cmd == "leg" || cmd == "e") {
        if (!plotter) { Serial.print("debug leg ");Serial.println(debug_leg); }
        debug_loops2 = debug_loops;
        move_leg = 1;
      } else if (cmd == "foff") {
        if (!plotter) { Serial.println("debug pir follow off"); }
        move_follow = 0;
        if (buzz_active) {
          for (int b = 3; b > 0; b--) {
            tone(BUZZ, 2000);
            delay(100);
            noTone(BUZZ);
            delay(100);
          }
          noTone(BUZZ);         
        }
        if (mpu_is_active) mpu_active = 1;
      } else if (cmd == "fon") {
        if (!plotter) { Serial.println("debug pir follow on"); }
        move_follow = 1;
        if (buzz_active) {
          for (int b = 3; b > 0; b--) {
            tone(BUZZ, 1000);
            delay(100);
            noTone(BUZZ);
            delay(100);
          }
          noTone(BUZZ);         
        }
        if (mpu_is_active) mpu_active = 0;
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
      } else if (cmd == "forward" || cmd == "mf") {
        if (!plotter) Serial.println(F("march_forward"));
        spd = 5;
        set_speed();
        y_dir = 10;
        x_dir = 0;
        z_dir = 0;
        step_weight_factor = 1.20;
        move_steps = 25;
        move_march = 1;
      } else if (cmd == "backward" || cmd == "mb") {
        if (!plotter) Serial.println(F("march_backward"));
        spd = 5;
        set_speed();
        y_dir = -10;
        x_dir = 0;
        z_dir = 0;
        step_weight_factor = 1.20;
        move_steps = 30;
        move_march = 1;
      } else if (cmd == "sit") {
        if (!plotter) Serial.println(F("sit"));
        set_sit();
      } else if (cmd == "kneel") {
        if (!plotter) Serial.println(F("kneel"));
        set_kneel();
      } else if (cmd == "crouch") {
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
      } else if (cmd == "framp") {
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

        Serial.print(F("\t|\tfor\tmarch_forward\t\t"));
        Serial.println(F("\tkneel\tkneel\t\t\t|"));

        Serial.print(F("\t|\tbac\tmarch_backward\t\t"));
        Serial.println(F("\tcrouch\tcrouch\t\t\t|"));

        Serial.print(F("\t|\twman\twman\t\t\t"));
        Serial.println(F("\tlay\tlay\t\t\t|"));

        Serial.print(F("\t|\tstl\tstep_left\t\t"));
        Serial.println(F("\troll\troll\t\t\t|"));

        Serial.print(F("\t|\tstr\tstep_right\t\t"));
        Serial.println(F("\tpitch\tpitch\t\t\t|"));

        Serial.print(F("\t|\tdemo\tdemo\t\t\t"));
        Serial.println(F("\trollb\troll_body\t\t|"));

        Serial.print(F("\t|\twake\twake\t\t\t"));
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

        Serial.print(F("\t|\t\t\t\t\t"));
        Serial.println(F("\ts-\tprev debug_leg\t\t|"));

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
          delay(3000);
          oled_request((char*)"b");
          delay(3000);
          oled_request((char*)"c");
          delay(3000);
          oled_request((char*)"g");
          delay(3000);
          oled_request((char*)"d");
          delay(1500);
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

      //if resetting slave, pause 15secs for display graphics, unless skip_splash
      if (commands[i] == 'Z' || commands[i] == 'Y') {
        if (debug) Serial.print(F("Slave Circuit intializing..."));
        int pcnt = 3;
        if (commands[i] == 'Z' && oled_active && !skip_splash) {
          pcnt = 14;
        }
        for(int n=0;n<pcnt;n++) {
          if (debug) Serial.print(F("."));
          delay(1000);
        }
        if (debug) Serial.println(F(" OK"));
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
/*
#if 0
  Serial.println("Simulated crash");
  while (1); //test crash
#endif
*/
  delay(3000);
  if (debug) Serial.println ("OK!");
}



/*
   -------------------------------------------------------
   TONES 
   -------------------------------------------------------
*/
void play_phrases() {
    int K = 2000;
    switch (random(1,7)) {
        case 1:phrase1(); break;
        case 2:phrase2(); break;
        case 3:phrase1(); phrase2(); break;
        case 4:phrase1(); phrase2(); phrase1();break;
        case 5:phrase1(); phrase2(); phrase1(); phrase2(); phrase1();break;
        case 6:phrase2(); phrase1(); phrase2(); break;
    }
    for (int i = 0; i <= random(3, 9); i++){
        tone(BUZZ, K + random(-1700, 2000));          
        delay(random(70, 170));  
        noTone(BUZZ);         
        delay(random(0, 30));             
    } 
    noTone(BUZZ);  
}

void phrase1() {    
    int k = random(1000,2000);
    for (int i = 0; i <=  random(100,2000); i++){
        tone(BUZZ, k+(-i*2));          
        delay(random(.9,2));             
    } 
    for (int i = 0; i <= random(100,1000); i++){
        tone(BUZZ, k + (i * 10));          
        delay(random(.9,2));             
    } 
}

void phrase2() {
    int k = random(1000,2000);
    for (int i = 0; i <= random(100,2000); i++){
        tone(BUZZ, k+(i*2));          
        delay(random(.9,2));             
    } 
    for (int i = 0; i <= random(100,1000); i++){
        tone(BUZZ, k + (-i * 10));          
        delay(random(.9,2));             
    } 
}
