//use copy of software calibration file for home & limits
#include "NovaServos.h"

//set which servos to home / activate for calibration
byte calibServo[TOTAL_SERVOS] = {
  1, 1, 1,                          //RFx
  1, 1, 1,                          //LFx
  1, 1, 1,                          //RRx
  1, 1, 1,                          //LRx
};

//run calibration test(s)
//NOTE: be aware / cautious of potential leg-to-leg collisions 
//      when running multiple tests with multiple servos/legs
//
int spd = 5;                        //speed of servo movements (higher=slower)
int calib_loops = 2;                //how many times to run test(s)
byte test_sweep = 1;                //sweep active servo(s) from home-to-max-to-min-to-home
byte test_min = 0;                  //move active servo(s) to min position
byte test_max = 0;                  //move active servo(s) to max position
byte test_home = 1;                 //move active servo(s) to home position


#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver();

byte use_limit = 0;  //0=min, 1=max

void setup() {
  Serial.begin(19200);
  Serial.println("Servo Calibration");

  pwm1.begin();
  pwm1.setOscillatorFrequency(25000000);
  pwm1.setPWMFreq(60);
  Serial.println("PWM Passed Setup... initializing servos...");

  for (int i = 0; i < TOTAL_SERVOS; i++) {
    servoSpeed[i] = spd;
    servoPos[i] = servoHome[i];
    if (calibServo[i]) {
      Serial.print("servo #");Serial.print(i);Serial.print("... ");
      pwm1.setPWM(servoSetup[i][1], 0, servoPos[i]);
      delay(500);
      Serial.println("\tOK");
    }
  }
  Serial.println("----------------------------\nbegin calibration routine...");
  delay(3000);
}

void loop() {
  int l = 1;
  while (calib_loops) {
    Serial.print("calibration loop ");Serial.println(l);
    if (test_sweep) {
      sweep_active();
      l++;
      delay(3000);
    }
  
    if (test_min) {
      use_limit = 0;
      set_limit();
      delay(3000);
    }
  
    if (test_max) {
      use_limit = 1;
      set_limit();
      delay(3000);
    }
  
    if (test_home) {
      set_home();
      delay(3000);
    }

    l++;
    calib_loops--;
    Serial.println("");
  }
}

void sweep_active() {
    for (int i = 0; i < TOTAL_SERVOS; i++) {
      if (calibServo[i]) {
        int smin = servoLimit[i][0];
        int smax = servoLimit[i][1];
        if (servoLimit[i][0] > servoLimit[i][1]) {
          smin = servoLimit[i][1];
          smax = servoLimit[i][0];
        }
        Serial.print("servo # ");Serial.print(i);
        Serial.print(" to max "); Serial.print(smax);
        for (int x=servoHome[i];x<smax; x++) {
          servoPos[i] = x;
          pwm1.setPWM(servoSetup[i][1], 0, servoPos[i]);
          delay(spd);
        }
        Serial.print(" to min "); Serial.print(smin);
        for (int x=smax;x>smin; x--) {
          servoPos[i] = x;
          pwm1.setPWM(servoSetup[i][1], 0, servoPos[i]);
          delay(spd);
        }
        Serial.print(" to home "); Serial.println(servoHome[i]);
        for (int x=smin;x<servoHome[i]; x++) {
          servoPos[i] = x;
          pwm1.setPWM(servoSetup[i][1], 0, servoPos[i]);
          delay(spd);
        }
        delay((spd*10));
      }
    }
}

void set_limit() {
    for (int i = 0; i < TOTAL_SERVOS; i++) {
      if (calibServo[i]) {
        Serial.print("servo # ");Serial.print(i);
        int spos = servoLimit[i][0];
        if (use_limit) {
          spos = servoLimit[i][1];
          Serial.print(" to max ");
        } else {
          Serial.print(" to min ");
        }
        Serial.println(spos);
        if (servoPos[i] > spos) {
          for (int x=servoPos[i];x>spos; x--) {
            servoPos[i] = x;
            pwm1.setPWM(servoSetup[i][1], 0, servoPos[i]);
            delay(spd);
          }
        } else {
          for (int x=servoPos[i];x<spos; x++) {
            servoPos[i] = x;
            pwm1.setPWM(servoSetup[i][1], 0, servoPos[i]);
            delay(spd);
          }
        }
        delay((spd*10));
      }
    }
}

void set_home() {
    for (int i = 0; i < TOTAL_SERVOS; i++) {
      if (calibServo[i]) {
        Serial.print("servo # ");Serial.print(i);
        int spos = servoHome[i];
        Serial.print(" to home ");
        Serial.println(spos);
        if (servoPos[i] > spos) {
          for (int x=servoPos[i];x>spos; x--) {
            servoPos[i] = x;
            pwm1.setPWM(servoSetup[i][1], 0, servoPos[i]);
            delay(spd);
          }
        } else {
          for (int x=servoPos[i];x<spos; x++) {
            servoPos[i] = x;
            pwm1.setPWM(servoSetup[i][1], 0, servoPos[i]);
            delay(spd);
          }
        }
        delay((spd*10));
      }
    }
}
