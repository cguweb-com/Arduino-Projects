

#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>

#define DEBUG 1

//setup RGB LEDs
#define LED_EYES_PIN 3
#define LED_EYES_NUM 2
Adafruit_NeoPixel led_eyes = Adafruit_NeoPixel(LED_EYES_NUM, LED_EYES_PIN, NEO_GRB + NEO_KHZ800);

//setup servo controllers
#define SERVO_FREQ 60
#define OSCIL_FREQ 27000000
#define TOTAL_SERVOS 20
#define TOTAL_LEGS 6
#define TOTAL_HEAD 2
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

//led
#define LED1 4

//sensor
#define PIR_SENSOR 5

//PS2 controller
#define PS2_DAT 6    
#define PS2_CMD 7
#define PS2_SEL 8
#define PS2_CLK 9
PS2X ps2x;

//assign servo names for array servoID ref
#define LFC 0
#define LFF 1
#define LFT 2
#define RFC 3
#define RFF 4
#define RFT 5

#define LCC 6
#define LCF 7
#define LCT 8
#define RCC 9
#define RCF 10
#define RCT 11

#define LRC 12
#define LRF 13
#define LRT 14
#define RRC 15
#define RRF 16
#define RRT 17

#define HF 18
#define HN 19

//setup servo data arrays 
int servoBones[3][TOTAL_LEGS] = {         //servoIDs for bone groups
  {0, 3, 6, 9, 12, 15},                   //coxas
  {1, 4, 7, 10, 13, 16},                  //femurs
  {2, 5, 8, 11, 14, 17},                  //tibias
};
byte activeServo[TOTAL_SERVOS];
int servoSpeed[TOTAL_SERVOS];
int servoPos[TOTAL_SERVOS];
int targetPos[TOTAL_SERVOS];
int servoSetup[TOTAL_SERVOS][2] = {       //driver, pin
  {2,14}, {2,13}, {2,12},                 //LFx
  {2,0}, {2,1}, {2,2},                    //RFx
  {1,4}, {1,5}, {1,6},                    //LCx
  {1,8}, {1,9}, {1,10},                   //RCx
  {1,0}, {1,1}, {1,2},                    //LRx
  {1,12}, {1,13}, {1,14},                 //RRx
  {2,8}, {2,9},                           //Hx
};

int servoLimit[TOTAL_SERVOS][2] = {       //min, max
  {360, 450}, {530, 320}, {150, 420},     //LFx
  {370, 280}, {230, 440}, {480, 230},     //RFx
  {300, 400}, {525, 320}, {250, 440},     //LCx
  {420, 320}, {200, 400}, {450, 250},     //RCx
  {250, 330}, {530, 320}, {140, 330},     //LRx
  {470, 390}, {240, 430}, {460, 240},     //RRx
  {180, 380}, {120, 320},                 //Hx
};
int servoHome[TOTAL_SERVOS] = {           //home pos
  400, 380, 340,                          //LFx
  330, 370, 280,                          //RFx
  350, 380, 375,                          //LCx
  370, 350, 310,                          //RCx
  290, 390, 260,                          //LRx
  430, 390, 310,                          //RRx
  250, 220,                               //Hx
};

//state intervals
unsigned int patternInterval = 12;
unsigned long lastPatternUpdate = 0;

unsigned int ps2Interval = 10;
unsigned long lastPS2Update = 0;

unsigned int pirInterval = 30;
unsigned long lastPIRUpdate = 0;

//led vars
int pattern = 0; 
int fadeStep = 0;

//sensor vars
byte pir_active = 0;
byte pir_state = LOW;
byte pir_val = 0;

//ps2 vars
int ps2_select = 1; //dpad: 1 = legs, 2 = head
int ps2_triangle = 3; //gaite: 1 = single leg, 2 = paired, 3 = tripod 
int ps2_square = 2; //height: 1 = high, 2 = medium, 3 = low 
int ps2_circle = 0; //pattern: 1-9 led patterns
int ps2_error = 0;
int ps2_active = 0;

//servo vars
const int min_spd = 32;
const int max_spd = 1;
int spd = 6;
byte start_stop = 0;
byte gaite = 3;
byte s_dir = 1;
int cmove = 5;
int fmove = 100;
int tmove = 120;

/*
 * -------------------------------------------------------
 * AsyncServo Class : control PWM servos by position
 * -------------------------------------------------------
 */
class AsyncServo
{
  Adafruit_PWMServoDriver *driver;
  int channel; 
  int servoID;
  int increment; 
  unsigned long lastUpdate;
  
  public: AsyncServo(Adafruit_PWMServoDriver *Driver, int ServoId)
  {
    driver = Driver;
    servoID = ServoId;
    channel = servoSetup[servoID][1];
    increment = 1;
  }

  void Update() {
    if (!activeServo[servoID]) return;
    if (servoPos[servoID] == targetPos[servoID]) {
      activeServo[servoID] = 0;
    }

    if((millis() - lastUpdate) > servoSpeed[servoID]) {
      lastUpdate = millis();
      if (servoPos[servoID] < targetPos[servoID]) {
        servoPos[servoID] += increment;
        driver->setPWM(channel, 0, servoPos[servoID]);
      } else if (servoPos[servoID] > targetPos[servoID]) {
        servoPos[servoID] -= increment;
        driver->setPWM(channel, 0, servoPos[servoID]);
      }
    }
  }
};


//instantiate servo objects
AsyncServo s_LFC(&pwm2, LFC);
AsyncServo s_LFF(&pwm2, LFF);
AsyncServo s_LFT(&pwm2, LFT);
AsyncServo s_RFC(&pwm2, RFC);
AsyncServo s_RFF(&pwm2, RFF);
AsyncServo s_RFT(&pwm2, RFT);

AsyncServo s_LCC(&pwm1, LCC);
AsyncServo s_LCF(&pwm1, LCF);
AsyncServo s_LCT(&pwm1, LCT);
AsyncServo s_RCC(&pwm1, RCC);
AsyncServo s_RCF(&pwm1, RCF);
AsyncServo s_RCT(&pwm1, RCT);

AsyncServo s_LRC(&pwm1, LRC);
AsyncServo s_LRF(&pwm1, LRF);
AsyncServo s_LRT(&pwm1, LRT);
AsyncServo s_RRC(&pwm1, RRC);
AsyncServo s_RRF(&pwm1, RRF);
AsyncServo s_RRT(&pwm1, RRT);

AsyncServo s_HN(&pwm2, HN);
AsyncServo s_HF(&pwm2, HF);


/*
 * -------------------------------------------------------
 * AsyncLeg Class : control PWM servos in leg groups by delay, direction, stride, and positions
 * -------------------------------------------------------
 */
class AsyncLeg
{
  Adafruit_PWMServoDriver *driver;
  int c_channel; 
  int f_channel; 
  int t_channel; 
  int coxaID;
  int femurID;
  int tibiaID;

  int tibia_cur;
  int tibia_move;

  int femur_cur;
  int femur_move;

  int coxa_cur;
  int coxa_step;
  int coxa_move;

  int dir_step;
  int del_cnt;
  int stride_factor;
  int stepped;
  int increment;
  unsigned long lastUpdate;
  
  public: AsyncLeg(Adafruit_PWMServoDriver *Driver, int CoxaId, int FemurId, int TibiaId)
  {
    driver = Driver;

    coxaID = CoxaId;
    coxa_move = 20;
    c_channel = servoSetup[coxaID][1];

    femurID = FemurId;
    femur_move = 0;
    f_channel = servoSetup[femurID][1];

    tibiaID = TibiaId;
    tibia_move = 0;
    t_channel = servoSetup[tibiaID][1];

    dir_step = 1; //0 = in place, 1 = forward, 2 = backward, 3 = left, 4 = right 
    del_cnt = 0;
    stride_factor = 0;
    stepped = 1;
    increment = 1;
  }

  void StepMotionSetup(int del, int dir, int stride, int cmove=20, int fmove=100, int tmove=120) {
    dir_step = dir;
    stride_factor = stride;

    if (stepped) {
      del_cnt = del;
    
      tibia_cur = servoPos[tibiaID];
      tibia_move = tmove;
      if (servoLimit[tibiaID][1] > servoLimit[tibiaID][0]) {
        tibia_move = (servoLimit[tibiaID][1]-tibia_move);
        if (tibia_move < servoLimit[tibiaID][0]) tibia_move = servoLimit[tibiaID][0];
      } else {
        tibia_move = (servoLimit[tibiaID][1]+tibia_move);
        if (tibia_move > servoLimit[tibiaID][0]) tibia_move = servoLimit[tibiaID][0];
      }
    
      femur_cur = servoPos[femurID];
      femur_move = fmove;
      if (servoLimit[femurID][1] > servoLimit[femurID][0]) {
        femur_move = (servoLimit[femurID][1]-femur_move);
        if (femur_move < servoLimit[femurID][0]) femur_move = servoLimit[femurID][0];
      } else {
        femur_move = (servoLimit[femurID][1]+femur_move);
        if (femur_move > servoLimit[femurID][0]) femur_move = servoLimit[femurID][0];
      }
    
      coxa_cur = servoPos[coxaID];
      coxa_step = 0;
      coxa_move = cmove;

      if (servoLimit[coxaID][1] > servoLimit[coxaID][0]) {
        if (dir_step == 2) {
          coxa_step = (servoLimit[coxaID][0]+(coxa_move*5));
          coxa_move = (servoLimit[coxaID][1]-coxa_move);
        } else if ((dir_step == 3 || dir_step == 4) && stride_factor > 0) {
          coxa_step = (servoPos[coxaID]+1);
          coxa_move = (servoPos[coxaID]-(coxa_move/stride_factor));
        } else {
          coxa_step = (servoLimit[coxaID][1]-(coxa_move*5));
          coxa_move = (servoLimit[coxaID][0]+coxa_move);
        }
        if (coxa_move < servoLimit[coxaID][0]) coxa_move = servoLimit[coxaID][0];
        if (coxa_step < servoHome[coxaID]) coxa_step = servoHome[coxaID];
      } else {
        if (dir_step == 2) {
          coxa_step = (servoLimit[coxaID][0]-(coxa_move*5));
          coxa_move = (servoLimit[coxaID][1]+coxa_move);
        } else if ((dir_step == 3 || dir_step == 4) && stride_factor > 0) {
          coxa_step = (servoPos[coxaID]-1);
          coxa_move = (servoPos[coxaID]+(coxa_move/stride_factor));
        } else {
          coxa_step = (servoLimit[coxaID][1]+(coxa_move*5));
          coxa_move = (servoLimit[coxaID][0]-coxa_move);
        }
        if (coxa_move > servoLimit[coxaID][0]) coxa_move = servoLimit[coxaID][0];
        if (coxa_step > servoHome[coxaID]) coxa_step = servoHome[coxaID];
      }
//coxa_step = 0;

      if (DEBUG == 2) {
        Serial.print(coxaID); 
        Serial.print(F("\tCoxa\tmove: ")); Serial.print(coxa_move); 
        Serial.print(F("\tcur: ")); Serial.print(servoPos[coxaID]);
        Serial.print(F("\thome: ")); Serial.print(servoHome[coxaID]);
        Serial.print(F("\tstep: ")); Serial.println(coxa_step);

        Serial.print(femurID); 
        Serial.print(F("\tFemur\tmove: ")); Serial.print(femur_move); 
        Serial.print(F("\tcur: ")); Serial.print(femur_cur);
        Serial.print(F("\thome: ")); Serial.println(servoHome[femurID]);

        Serial.print(tibiaID); 
        Serial.print(F("\tTibia\tmove: ")); Serial.print(tibia_move); 
        Serial.print(F("\tcur: ")); Serial.print(tibia_cur);
        Serial.print(F("\thome: ")); Serial.println(servoHome[tibiaID]);
      }

      stepped = 0;
    }
  }

  int StepMotion() {
    if (stepped) return 2;

    int ret = 0;
    if((millis() - lastUpdate) > servoSpeed[coxaID]) {
      lastUpdate = millis();

      if (!del_cnt) {
        if (coxa_step) {
          //coxa step
          if (servoPos[coxaID] < coxa_step) {
            servoPos[coxaID] += increment;
          } else {
            servoPos[coxaID] -= increment;
          }
          driver->setPWM(c_channel, 0, servoPos[coxaID]);
        }
        if (servoPos[coxaID] == coxa_step) {
          coxa_step = 0;
        }
  
        if (tibia_move && !coxa_step) {
          //tibia up
          if (servoPos[tibiaID] < tibia_move) {
            servoPos[tibiaID] += increment;
          } else {
            servoPos[tibiaID] -= increment;
          }
          driver->setPWM(t_channel, 0, servoPos[tibiaID]);
        } else if (!tibia_move && !coxa_move && servoPos[tibiaID] != tibia_cur) {
          //tibia down
          if (servoPos[tibiaID] < tibia_cur) {
            servoPos[tibiaID] += increment;
          } else {
            servoPos[tibiaID] -= increment;
          }
          driver->setPWM(t_channel, 0, servoPos[tibiaID]);
        }
        if (servoPos[tibiaID] == tibia_move) {
          tibia_move = 0;
        }
  
        if (femur_move && !coxa_step) {
          //femur up
          if (servoPos[femurID] < femur_move) {
            servoPos[femurID] += increment;
          } else {
            servoPos[femurID] -= increment;
          }
          driver->setPWM(f_channel, 0, servoPos[femurID]);
        } else if (!femur_move && !coxa_move && servoPos[femurID] != femur_cur) {
          //femur down
          if (servoPos[femurID] < femur_cur) {
            servoPos[femurID] += increment;
          } else {
            servoPos[femurID] -= increment;
          }
          driver->setPWM(f_channel, 0, servoPos[femurID]);
        }
        if (servoPos[femurID] == femur_move) {
          femur_move = 0;
        }
  
        //coxa forward
        if (coxa_move && !coxa_step && !femur_move && !tibia_move) {
          if (servoPos[coxaID] < coxa_move) {
            servoPos[coxaID] += increment;
          } else {
            servoPos[coxaID] -= increment;
          }
          driver->setPWM(c_channel, 0, servoPos[coxaID]);
        } else if (!coxa_move && !coxa_step && servoPos[coxaID] != coxa_cur && !femur_move && servoPos[femurID] == femur_cur) {
          if (servoPos[coxaID] < coxa_cur) {
            servoPos[coxaID] += increment;
          } else {
            servoPos[coxaID] -= increment;
          }
          driver->setPWM(c_channel, 0, servoPos[coxaID]);
        }
        if (servoPos[coxaID] == coxa_move) {
          coxa_move = 0;
        }
      } else {
        del_cnt--;
      }

      if (!coxa_step && !coxa_move && servoPos[coxaID] == coxa_cur) {
        stepped = 1;
        ret = 2;
      }
      if (!femur_move) {
        ret = 1;
      }

      return ret;
    }
  }

  void StepMotionCancel() {
    if (!stepped) {
      del_cnt = 0;
      tibia_cur = tibia_move = servoPos[tibiaID];
      femur_cur = femur_move = servoPos[femurID];
      coxa_cur = coxa_step = coxa_move = servoPos[coxaID];
      stepped = 1;
    }
  }  
};

//instantiate leg objects
AsyncLeg s_LF(&pwm2, LFC, LFF, LFT);
AsyncLeg s_RR(&pwm1, RRC, RRF, RRT);

AsyncLeg s_LC(&pwm1, LCC, LCF, LCT);
AsyncLeg s_RF(&pwm2, RFC, RFF, RFT);

AsyncLeg s_LR(&pwm1, LRC, LRF, LRT);
AsyncLeg s_RC(&pwm1, RCC, RCF, RCT);


/*
 * -------------------------------------------------------
 * Application Setup
 * -------------------------------------------------------
 */
void setup() 
{ 
  Serial.begin(9600);
  while (!Serial) {
    delay(1);
  }
  delay(500); //allow ps2 and PWM hardware to powerup
  Serial.println(F("Setting up PWM Controllers..."));

  //setup pwm controllers
  pwm1.begin();
  pwm1.setOscillatorFrequency(OSCIL_FREQ);
  pwm1.setPWMFreq(SERVO_FREQ);
  delay(50);

  pwm2.begin();
  pwm2.setOscillatorFrequency(OSCIL_FREQ);
  pwm2.setPWMFreq(SERVO_FREQ);
  delay(50);

  //initialize data arrays with defaults and connect servos
  init_home();
  Serial.print(TOTAL_SERVOS); Serial.println(F(" Servos Initialized"));
  
  //setup rgb leds
  led_eyes.begin();
  wipe_eyes();

  //setup LEDs
  pinMode(LED1, OUTPUT); 

  //setup sensors
  pinMode(PIR_SENSOR, INPUT);

  //setup ps2 controller
  ps2_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);
  delay(500);
  if(ps2_error == 0) {
    if (DEBUG) Serial.println("Found Controller, configured successfully");
  } else if(ps2_error == 1) {
    if (DEBUG) Serial.println("No controller found, try restarting");
  } else if(ps2_error == 2) {
    if (DEBUG) Serial.println("Controller found but not accepting commands");
  } else {
    if (DEBUG) Serial.println("Some other error ocurred");
  }

  blink_led(6, 25);
  Serial.println(F("\nReady!\n=====================\n"));
} 
 
void(* reset_func) (void) = 0;

/*
 * -------------------------------------------------------
 * Application Loop
 * -------------------------------------------------------
 */
void loop() { 

  //tibias
  s_LFT.Update();
  s_LCT.Update();
  s_LRT.Update();
  s_RFT.Update();
  s_RCT.Update();
  s_RRT.Update();

  //femurs
  s_LFF.Update();
  s_LCF.Update();
  s_LRF.Update();
  s_RFF.Update();
  s_RCF.Update();
  s_RRF.Update();

  //coxas
  s_LFC.Update();
  s_LCC.Update();
  s_LRC.Update();
  s_RFC.Update();
  s_RCC.Update();
  s_RRC.Update();

  //head
  s_HN.Update();
  s_HF.Update();


  if (start_stop) {
    if (gaite == 3) {
      if (s_dir == 3) {
        //move in triplets
        s_RF.StepMotion();
        s_LC.StepMotion();
        if (s_RR.StepMotion() == 2) {
          s_LF.StepMotion();
          s_RC.StepMotion();
          s_LR.StepMotion();
          if (s_LF.StepMotion() == 2 && s_RC.StepMotion() == 2 && s_LR.StepMotion() == 2) {
            if (start_stop) {
              step_left();
            } else {
              set_home();
            }
          }
        }
      } else if (s_dir == 4) {
        //move in triplets
        s_LF.StepMotion();
        s_RC.StepMotion();
        if (s_LR.StepMotion() == 2) {
          s_RF.StepMotion();
          s_LC.StepMotion();
          s_RR.StepMotion();
          if (s_RF.StepMotion() == 2 && s_LC.StepMotion() == 2 && s_RR.StepMotion() == 2) {
            if (start_stop) {
              step_right();
            } else {
              set_home();
            }
          }
        }
      } else {
        //move in triplets
        s_RF.StepMotion();
        s_LC.StepMotion();
        if (s_RR.StepMotion() == 2) {
          s_LF.StepMotion();
          s_RC.StepMotion();
          s_LR.StepMotion();
          if (s_LF.StepMotion() == 2 && s_RC.StepMotion() == 2 && s_LR.StepMotion() == 2) {
            if (start_stop) {
              if (s_dir == 1) {
                step_forward();
              } else if (s_dir == 2) {
                step_backward();
              }
            } else {
              set_home();
            }
          }
        }
      }
    } else if (gaite == 2) {
      //move in pairs
      s_LC.StepMotion();
      if (s_RF.StepMotion() == 2) {
        s_LR.StepMotion();
        if (s_RC.StepMotion() == 2) {
          s_LF.StepMotion();
          if (s_RR.StepMotion() == 2) {
            if (start_stop) {
              if (s_dir == 1) {
                step_forward();
              } else if (s_dir == 2) {
                step_backward();
              } else if (s_dir == 3) {
                step_left();
              } else if (s_dir == 4) {
                step_right();
              }
            } else {
              set_home();
            }
          }
        }
      }
    } else {
      //move in singles
      s_LF.StepMotion();
      s_RF.StepMotion();
      s_LC.StepMotion();
      s_RC.StepMotion();
      s_LR.StepMotion();
      s_RR.StepMotion();
      if (s_RR.StepMotion() == 2) {
        if (start_stop) {
          if (s_dir == 1) {
            step_forward();
          } else if (s_dir == 2) {
            step_backward();
          } else if (s_dir == 3) {
            step_left();
          } else if (s_dir == 4) {
            step_right();
          }
        } else {
          set_home();
        }
      }
    }
  }

  //check state machines
  if(millis() - lastPatternUpdate > patternInterval) updatePattern(pattern);
  if(millis() - lastPS2Update > ps2Interval) ps2_check();
  if (pir_active) {
//    if(millis() - lastPIRUpdate > pirInterval) pir_check();
  }

}


/*
 * -------------------------------------------------------
 * pir_check() : executes check on PIR Sensor
 * -------------------------------------------------------
 */
void pir_check() {
  pir_val = digitalRead(PIR_SENSOR);
  if (pir_val == HIGH) {
    digitalWrite(LED1, HIGH);
    if (pir_state == LOW) {
      if (DEBUG) Serial.println("Motion detected!");
      pir_state = HIGH;
      set_home();
      if (!start_stop) {
        gaite = 3;
        pattern = 3;
        start_stop = 1;
        step_forward();
      }
    }
  } else {
    digitalWrite(LED1, LOW);
    if (pir_state == HIGH){
      if (DEBUG) Serial.println("Motion ended!");
      pir_state = LOW;
      set_home();
      if (start_stop) {
        pattern = 0;
        start_stop = 0;
      }
    }
  }

  lastPIRUpdate = millis();
}


/*
 * -------------------------------------------------------
 * ps2_check() : monitor controller functions
 * -------------------------------------------------------
 */
void ps2_check() {
  ps2x.read_gamepad();

  //start : 
  if (ps2x.ButtonPressed(PSB_START)) {
    ps2_active = 1;
    blink_led(1, 25);
  }

  if (ps2_active) {
    //select : change from legs to head for dpad controls
    if (ps2x.ButtonPressed(PSB_SELECT)) {
      Serial.print("Select ");

      if (ps2_select == 2) {
        ps2_select = 1;
        Serial.println("Legs");
      } else {
        ps2_select = 2;
        Serial.println("Head");
      }
/*      
enable for calibrating
      if (ps2_select == 0) {
        ps2_select = 1;
      } else if (ps2_select == 1) {
        ps2_select = 2;
      } else if (ps2_select == 2) {
        ps2_select = 6;
      } else if (ps2_select == 6) {
        ps2_select = 7;
      } else if (ps2_select == 7) {
        ps2_select = 8;
      } else if (ps2_select == 8) {
        ps2_select = 12;
      } else if (ps2_select == 12) {
        ps2_select = 13;
      } else if (ps2_select == 13) {
        ps2_select = 14;
      } else if (ps2_select == 14) {
        ps2_select = 0;
      }
*/
      blink_led(ps2_select, 10);
    }
  
    //r3 : turns pir sensor on/off
    if(ps2x.ButtonPressed(PSB_R3)) {
      if (pir_active) {
        Serial.println("Stop PIR");
        pir_active = 0;
      } else {
        Serial.println("Start PIR");
        pir_active = 1;
      }
    }
  
    //l3 : 
    if(ps2x.ButtonPressed(PSB_L3)) {
      Serial.println("Home");
      s_dir = 0;
      start_stop = 0;
      set_home();
    }
  
  //r2 / l2 : slower / faster speed
  if(ps2x.ButtonPressed(PSB_R2)) {
    Serial.print("Spd Down ");
    if (spd < min_spd) {
      spd++;
    }
    set_speed();
    Serial.println(spd);
  }
  if(ps2x.ButtonPressed(PSB_L2)) {
    Serial.print("Spd Up ");
    if (spd > max_spd) {
      spd--;
    }
    set_speed();
    Serial.println(spd);
  }
  
  //r1 / l1 : smaller / larger stride
  if(ps2x.ButtonPressed(PSB_R1)) {
    Serial.print("Stride longer ");
    if (cmove > 1) {
      cmove--;
    }
    Serial.println(cmove);
  }
  if(ps2x.ButtonPressed(PSB_L1)) {
    Serial.print("Stride shorter ");
    if (cmove < 50) {
      cmove++;
    }
    Serial.println(cmove);
  }

    //dpad controls
    if(ps2x.Button(PSB_PAD_UP)) {
      if (ps2_select == 1 && (!start_stop || s_dir != 1)) {
        if (DEBUG) Serial.println("Forward legs");
        s_dir = 1;
        start_stop = 1;
        step_forward();
      } else if (ps2_select == 2) {
        if (DEBUG) Serial.println("Up head");
        head_up();
      } 
/*
enable for calibrating
    int sc = ps2_select;
    int sf = (sc+3);

    if (!activeServo[sc] && targetPos[sc] == servoLimit[sc][0]) {
      activeServo[sc] = 1;
      targetPos[sc] = servoLimit[sc][1];
      if (DEBUG) { Serial.print(sc); Serial.print(" / "); Serial.print(servoSetup[sc][1]); Serial.print(" max: "); Serial.println(servoLimit[sc][1]); }
    } else if (!activeServo[sc] && targetPos[sc] == servoLimit[sc][1]) {
      activeServo[sc] = 1;
      targetPos[sc] = servoHome[sc];
      if (DEBUG) { Serial.print(sc); Serial.print(" / "); Serial.print(servoSetup[sc][1]); Serial.print(" home: "); Serial.println(servoHome[sc]); }
    } else if (!activeServo[sc]) {
      activeServo[sc] = 1;
      targetPos[sc] = servoLimit[sc][0];
      if (DEBUG) { Serial.print(sc); Serial.print(" / "); Serial.print(servoSetup[sc][1]); Serial.print(" min: "); Serial.println(servoLimit[sc][0]); }
    }

    if (!activeServo[sf] && targetPos[sf] == servoLimit[sf][0]) {
      activeServo[sf] = 1;
      targetPos[sf] = servoLimit[sf][1];
      if (DEBUG) { Serial.print(sf); Serial.print(" / "); Serial.print(servoSetup[sf][1]); Serial.print(" max: "); Serial.println(servoLimit[sf][1]); }
    } else if (!activeServo[sf] && targetPos[sf] == servoLimit[sf][1]) {
      activeServo[sf] = 1;
      targetPos[sf] = servoHome[sf];
      if (DEBUG) { Serial.print(sf); Serial.print(" / "); Serial.print(servoSetup[sf][1]); Serial.print(" home: "); Serial.println(servoHome[sf]); }
    } else if (!activeServo[sf]) {
      activeServo[sf] = 1;
      targetPos[sf] = servoLimit[sf][0];
      if (DEBUG) { Serial.print(sf); Serial.print(" / "); Serial.print(servoSetup[sf][1]); Serial.print(" min: "); Serial.println(servoLimit[sf][0]); }
    }


*/
    }

    if(ps2x.Button(PSB_PAD_DOWN)) {
      if (ps2_select == 1 && (!start_stop || s_dir != 2)) {
        if (DEBUG) Serial.println("Backward legs");
        gaite = 3;
        s_dir = 2;
        start_stop = 1;
        step_backward();
      } else if (ps2_select == 2) {
        if (DEBUG) Serial.println("Down head");
        head_down();
      }
    }
    if(ps2x.Button(PSB_PAD_LEFT)) {
      if (ps2_select == 1 && (!start_stop || s_dir != 3)) {
        if (DEBUG) Serial.println("Left legs");
        s_dir = 3;
        start_stop = 1;
        step_left();
      } else if (ps2_select == 2) {
        if (DEBUG) Serial.println("Left head");
        head_left();
      }
    }
    if(ps2x.Button(PSB_PAD_RIGHT)) {
      if (ps2_select == 1 && (!start_stop || s_dir != 4)) {
        if (DEBUG) Serial.println("Right legs");
        s_dir = 4;
        start_stop = 1;
        step_right();
      } else if (ps2_select == 2) {
        if (DEBUG) Serial.println("Right head");
        head_right();
      }
    }

    //triangle : gaite selector
    if(ps2x.ButtonPressed(PSB_TRIANGLE)) {
      Serial.print("Triangle / Gaite ");
      if (ps2_triangle == 3) {
        ps2_triangle = 1;
      } else {
        ps2_triangle++;
      }
      Serial.println(ps2_triangle);
      gaite = ps2_triangle;
      blink_led(ps2_triangle, 15);
    }
  
    //square : height selector
    if(ps2x.ButtonPressed(PSB_SQUARE)) {
      Serial.print("Square / Height ");
      if (ps2_square == 3) {
        ps2_square = 1;
        fmove = 80;
        tmove = 100;
      } else if (ps2_square == 1) {
        ps2_square = 2;
        fmove = 100;
        tmove = 120;
      } else {
        ps2_square = 3;
        fmove = 140;
        tmove = 160;
      }
      Serial.println(ps2_square);
      blink_led(ps2_square, 15);
    }
  
    //led_eyes patterns and stop
    if(ps2x.ButtonPressed(PSB_CROSS)) {
      Serial.println("Stop Eyes");
      pattern = 0;
    }
    if(ps2x.ButtonPressed(PSB_CIRCLE)) {
      if (ps2_circle < 7) {
        ps2_circle++;
      } else {
        ps2_circle = 0;
      }
      Serial.print("Set Pattern to "); Serial.println(ps2_circle);
      pattern = ps2_circle;
    }
  }

  lastPS2Update = millis();
}


/*

      if (DEBUG) { Serial.println(F("Rest")); }
      start_stop = 0;
      pattern = 0;
      set_rest();

      if (DEBUG) { Serial.println(F("Head Home")); }
      pattern = 0;
      head_home();

      if (DEBUG) { Serial.println(F("Reset")); }
      reset_func();

      if (DEBUG) { Serial.println(F("Stride")); }
      stride = 0;
      stride = 0.5
      stride = 1.0
      stride = 1.5
*/


/*
 * -------------------------------------------------------
 * Basic Leg Movement Functions
 * -------------------------------------------------------
 */
void step_forward() {
  int del = 0;
  s_dir = 1;

  if (gaite == 3) {
    //move in triplets
    s_RF.StepMotionSetup(0, s_dir, 0, cmove, fmove, tmove);
    s_LC.StepMotionSetup(0, s_dir, 0, cmove, fmove, tmove);
    s_RR.StepMotionSetup(0, s_dir, 0, cmove, fmove, tmove);
  
    s_LF.StepMotionSetup(0, s_dir, 0, cmove, fmove, tmove);
    s_RC.StepMotionSetup(0, s_dir, 0, cmove, fmove, tmove);
    s_LR.StepMotionSetup(0, s_dir, 0, cmove, fmove, tmove);  
  } else if (gaite == 2) {
    //move in pairs
    del = 60;

    s_LC.StepMotionSetup(0, s_dir, 0, cmove, fmove, tmove);
    s_RF.StepMotionSetup(del, s_dir, 0, cmove, fmove, tmove);
  
    s_LR.StepMotionSetup(0, s_dir, 0, cmove, fmove, tmove);
    s_RC.StepMotionSetup(del, s_dir, 0, cmove, fmove, tmove);
  
    s_LF.StepMotionSetup(0, s_dir, 0, cmove, fmove, tmove);
    s_RR.StepMotionSetup(del, s_dir, 0, cmove, fmove, tmove);
  } else {
    //move in singles
    del = 60;

    s_LF.StepMotionSetup(0, s_dir, 0, cmove, fmove, tmove);
    s_RF.StepMotionSetup(del, s_dir, 0, cmove, fmove, tmove);
    s_LC.StepMotionSetup(del*2, s_dir, 0, cmove, fmove, tmove);
    s_RC.StepMotionSetup(del*3, s_dir, 0, cmove, fmove, tmove);
    s_LR.StepMotionSetup(del*4, s_dir, 0, cmove, fmove, tmove);
    s_RR.StepMotionSetup(del*5, s_dir, 0, cmove, fmove, tmove);    
  }
}

void step_backward() {
  int del = 0;
  s_dir = 2;

  if (gaite == 3) {
    //move in triplets
    s_RF.StepMotionSetup(0, 2, 0);
    s_LC.StepMotionSetup(0, 2, 0);
    s_LF.StepMotionSetup(0, 2, 0);
    s_RR.StepMotionSetup(0, 2, 0);
  
    s_RC.StepMotionSetup(0, 2, 0);
    s_LR.StepMotionSetup(0, 2, 0);  
  } else if (gaite == 2) {
    //move in pairs
    del = 60;

    s_LC.StepMotionSetup(0, 2, 0);
    s_RF.StepMotionSetup(del, 2, 0);
  
    s_LR.StepMotionSetup(0, 2, 0);
    s_RC.StepMotionSetup(del, 2, 0);
  
    s_LF.StepMotionSetup(0, 2, 0);
    s_RR.StepMotionSetup(del, 2, 0);
  } else {
    //move in singles
    del = 60;

    s_LF.StepMotionSetup(0, 2, 0);
    s_RF.StepMotionSetup(del, 2, 0);
    s_LC.StepMotionSetup(del*2, 2, 0);
    s_RC.StepMotionSetup(del*3, 2, 0);
    s_LR.StepMotionSetup(del*4, 2, 0);
    s_RR.StepMotionSetup(del*5, 2, 0);    
  }
}


void step_left() {
  int del = 60;
  s_dir = 3;

  if (gaite == 3) {
    //move in triplets
    s_RF.StepMotionSetup(0, 3, 0);
    s_LC.StepMotionSetup(del, 3, 4);
    s_RR.StepMotionSetup(0, 3, 0);
  
    s_LF.StepMotionSetup(del, 3, 4);
    s_RC.StepMotionSetup(0, 3, 0);
    s_LR.StepMotionSetup(del, 3, 4);  
  } else if (gaite == 2) {
    //move in pairs
    s_LC.StepMotionSetup(0, 3, 4);
    s_RF.StepMotionSetup(del, 3, 0);
  
    s_LR.StepMotionSetup(0, 3, 4);
    s_RC.StepMotionSetup(del, 3, 0);
  
    s_LF.StepMotionSetup(0, 3, 4);
    s_RR.StepMotionSetup(del, 3, 0);
  } else {
    //move in singles
    s_LF.StepMotionSetup(0, 3, 4);
    s_RF.StepMotionSetup(del, 3, 0);
    s_LC.StepMotionSetup(del*2, 3, 4);
    s_RC.StepMotionSetup(del*3, 3, 0);
    s_LR.StepMotionSetup(del*4, 3, 4);
    s_RR.StepMotionSetup(del*5, 3, 0);    
  }
}

void step_right() {
  int del = 60;
  s_dir = 4;

  if (gaite == 3) {
    //move in triplets
    s_RF.StepMotionSetup(del, 4, 4);
    s_LC.StepMotionSetup(0, 4, 0);
    s_RR.StepMotionSetup(del, 4, 4);
  
    s_LF.StepMotionSetup(0, 4, 0);
    s_RC.StepMotionSetup(del, 4, 4);
    s_LR.StepMotionSetup(0, 4, 0);  
  } else if (gaite == 2) {
    //move in pairs
    s_LC.StepMotionSetup(0, 4, 0);
    s_RF.StepMotionSetup(del, 4, 4);
  
    s_LR.StepMotionSetup(0, 4, 0);
    s_RC.StepMotionSetup(del, 4, 4);
  
    s_LF.StepMotionSetup(0, 4, 0);
    s_RR.StepMotionSetup(del, 4, 4);
  } else {
    //move in singles
    s_LF.StepMotionSetup(0, 4, 0);
    s_RF.StepMotionSetup(del, 4, 4);
    s_LC.StepMotionSetup(del*2, 4, 0);
    s_RC.StepMotionSetup(del*3, 4, 4);
    s_LR.StepMotionSetup(del*4, 4, 0);
    s_RR.StepMotionSetup(del*5, 4, 4);    
  }
}


/*
 * -------------------------------------------------------
 * Scripted Movement Functions
 * -------------------------------------------------------
 */
void set_home() {
  pattern = 0;
  for(int m=0; m<(TOTAL_LEGS*3)+TOTAL_HEAD; m++) {
    activeServo[m] = 1;
    targetPos[m] = servoHome[m];
  }
}

void init_home() {
  for (int i=0; i < TOTAL_SERVOS; i++) { 
    activeServo[i] = 0;
    servoSpeed[i] = spd;
    servoPos[i] = servoHome[i];
    targetPos[i] = servoHome[i];
    if (servoSetup[i][0] == 1) {
      pwm1.setPWM(servoSetup[i][1], 0, servoHome[i]);
    } else {
      pwm2.setPWM(servoSetup[i][1], 0, servoHome[i]);
    }
  }
}

void set_speed() {
  for (int i=0; i < TOTAL_SERVOS; i++) { 
    servoSpeed[i] = spd;
  }
}

void set_rest() {
  //set rest
  //coxas
  int c = 3;
  for(int m=0; m<TOTAL_SERVOS; m++) {
    c--;
    if (c == 2) {
      activeServo[m] = 1;
      targetPos[m] = servoLimit[m][1];
    }
    if (!c) {
      c = 3;
    }
  }

  //femurs
  c = 3;
  for(int m=0; m<TOTAL_SERVOS; m++) {
    c--;
    if (c == 1) {
      activeServo[m] = 1;
      targetPos[m] = servoLimit[m][1];
    }
    if (!c) {
      c = 3;
    }
  }

  //tibis
  c = 3;
  for(int m=0; m<TOTAL_SERVOS; m++) {
    c--;
    if (!c) {
      activeServo[m] = 1;
      targetPos[m] = servoLimit[m][0];
      c = 3;
    }
  }

}


void head_home() {
  activeServo[HF] = 1;
  targetPos[HF] = servoHome[HF];
  activeServo[HN] = 1;
  targetPos[HN] = servoHome[HN];
}

void head_up() {
  int minc = 10;
  if (!activeServo[HF]) {
    activeServo[HF] = 1;
    if (servoLimit[HF][1] > servoLimit[HF][0]) {
      if ((servoPos[HF]+minc) <= servoLimit[HF][1]) {
        targetPos[HF] = servoPos[HF]+minc;
      }
    } else {
      if ((servoPos[HF]-minc) >= servoLimit[HF][1]) {
        targetPos[HF] = servoPos[HF]-minc;
      }
    }
  }
}

void head_down() {
  int minc = 10;
  if (!activeServo[HF]) {
    activeServo[HF] = 1;
    if (servoLimit[HF][1] > servoLimit[HF][0]) {
      if ((servoPos[HF]-minc) >= servoLimit[HF][0]) {
        targetPos[HF] = servoPos[HF]-minc;
      }
    } else {
      if ((servoPos[HF]+minc) >= servoLimit[HF][1]) {
        targetPos[HF] = servoPos[HF]+minc;
      }
    }
  }
}

void head_left() {
  int minc = 10;
  if (!activeServo[HN]) {
    activeServo[HN] = 1;
    if (servoLimit[HN][1] > servoLimit[HN][0]) {
      if ((servoPos[HN]+minc) <= servoLimit[HN][1]) {
        targetPos[HN] = servoPos[HN]+minc;
      }
    } else {
      if ((servoPos[HN]-minc) >= servoLimit[HN][1]) {
        targetPos[HN] = servoPos[HN]-minc;
      }
    }
  }
}

void head_right() {
  int minc = 10;
  if (!activeServo[HN]) {
    activeServo[HN] = 1;
    if (servoLimit[HN][1] > servoLimit[HN][0]) {
      if ((servoPos[HN]-minc) >= servoLimit[HN][0]) {
        targetPos[HN] = servoPos[HN]-minc;
      }
    } else {
      if ((servoPos[HN]+minc) >= servoLimit[HN][1]) {
        targetPos[HN] = servoPos[HN]+minc;
      }
    }
  }
}



/*
 * -------------------------------------------------------
 * LED Functions
 * -------------------------------------------------------
 */
void blink_led(int times, int del) {
  digitalWrite(LED1, LOW);
  for (int i=0; i<times; i++) {
    digitalWrite(LED1, HIGH);
    delay((del*10));
    digitalWrite(LED1, LOW);
    delay((del*10));
  }
}

void  updatePattern(int pat){ 
  switch(pat) {
    case 0:
        colorWipe(led_eyes.Color(0, 0, 0)); // off
        break;
    case 1:
        fade(0,0, 0,0, 125,0, 400, 0); // fade from blue to black
        break;
    case 2:
        fade(0,255, 0,64, 0,0, 400, 1); // fade from black to orange and back
        break;                  
    case 3:
        fade(0,0, 0,125, 0,0, 400, 1); // fade from green to black
        break;
    case 4:
        fade(0,125, 0,125, 0,0, 400, 1); // fade from yellow to black
        break;
    case 5:
        fade(0,125, 0,0, 0,0, 400, 1); // fade from red to black
        break;
    case 6:
        fade(0,125, 0,0, 0,125, 400, 1); // fade from purple to black
        break;
    case 7:
        colorWipe(led_eyes.Color(255, 255, 255)); //white
        break;
  }
}

void fade(int redStartValue, int redEndValue, int greenStartValue, int greenEndValue, int blueStartValue, int blueEndValue, int totalSteps, int fadeBack) {
  static float redIncrement, greenIncrement, blueIncrement;
  static float red, green, blue;
  static boolean fadeUp = false;
  
  if (fadeStep == 0){ // first step is to initialise the initial colour and increments
    red = redStartValue;
    green = greenStartValue;
    blue = blueStartValue;
    fadeUp = false;
  
    redIncrement = (float)(redEndValue - redStartValue) / (float)totalSteps;
    greenIncrement = (float)(greenEndValue - greenStartValue) / (float)totalSteps;
    blueIncrement = (float)(blueEndValue - blueStartValue) / (float)totalSteps;
    fadeStep = 1; // next time the function is called start the fade
  } else { // all other steps make a new colour and display it
    // make new colour
    red += redIncrement;
    green +=  greenIncrement;
    blue += blueIncrement;
   
    // set up the pixel buffer
    for (int i = 0; i < led_eyes.numPixels(); i++) {
    led_eyes.setPixelColor(i, led_eyes.Color((int)red,(int)green,(int)blue));
    }
  
    // now display it
    led_eyes.show();
    fadeStep += 1; // go on to next step
    if(fadeStep >= totalSteps) { // finished fade
      if(fadeUp){ // finished fade up and back
         fadeStep = 0;
         return; // so next call recalabrates the increments
      }
      // now fade back
      if(fadeBack) { 
        fadeUp = true;
        redIncrement = -redIncrement;
        greenIncrement = -greenIncrement;
        blueIncrement = -blueIncrement;
        fadeStep = 1; // don't calculate the increments again but start at first change
      }
    }
  }
}

void colorWipe(uint32_t c) { // modified from Adafruit example to make it a state machine
  static int i =0;
  led_eyes.setPixelColor(i, c);
  led_eyes.show();
  i++;
  if(i >= led_eyes.numPixels()){
    i = 0;
    wipe_eyes(); // blank out led_eyes
  }
  lastPatternUpdate = millis(); // time for next change to the display
}

void wipe_eyes(){ // clear all LEDs
  for(int i=0;i<led_eyes.numPixels();i++){
    led_eyes.setPixelColor(i, led_eyes.Color(0,0,0));
  }
}
