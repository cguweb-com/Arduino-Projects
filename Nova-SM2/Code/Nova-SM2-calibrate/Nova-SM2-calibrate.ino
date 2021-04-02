
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver();

int spd = 10;
int cal_coax = 1; //1 = home  2 = run 
int cal_femur = 2; //1 = home  2 = run 
int cal_tibia = 1; //1 = home  2 = run 

//RF
uint8_t servonum1 = 0; //coax
#define SERVOMIN1  330
#define SERVOMAX1  450
#define SERVOHOME1  368

uint8_t servonum2 = 1; //femur
#define SERVOMIN2  226
#define SERVOMAX2  466
#define SERVOHOME2  301

uint8_t servonum3 = 2; //tibia
#define SERVOMIN3  322
#define SERVOMAX3  548
#define SERVOHOME3  435

/*
//LF
uint8_t servonum1 = 4; //coax
#define SERVOMIN1  393
#define SERVOMAX1  273
#define SERVOHOME1  355

uint8_t servonum2 = 5; //femur
#define SERVOMIN2  497
#define SERVOMAX2  257
#define SERVOHOME2  422

uint8_t servonum3 = 6; //tibia
#define SERVOMIN3  468
#define SERVOMAX3  242
#define SERVOHOME3  355
*/


/*
//RR
uint8_t servonum1 = 8; //coax
#define SERVOMIN1  310
#define SERVOMAX1  430
#define SERVOHOME1  348

uint8_t servonum2 = 9; //femur
#define SERVOMIN2  269
#define SERVOMAX2  509
#define SERVOHOME2  344

uint8_t servonum3 = 10; //tibia
#define SERVOMIN3  283
#define SERVOMAX3  509
#define SERVOHOME3  396
*/

/*
//LR
uint8_t servonum1 = 12; //coax
#define SERVOMIN1  402
#define SERVOMAX1  282
#define SERVOHOME1  364

uint8_t servonum2 = 13; //femur
#define SERVOMIN2  416
#define SERVOMAX2  176
#define SERVOHOME2  341

uint8_t servonum3 = 14; //tibia
#define SERVOMIN3  419
#define SERVOMAX3  193
#define SERVOHOME3  306
*/



void setup() {
  Serial.begin(19200);
  Serial.println("Servo Calibration");

  pwm1.begin();
  pwm1.setOscillatorFrequency(25000000);
  pwm1.setPWMFreq(60);
  Serial.println("Passed Setup... initializing servos...");

  if (cal_coax) {
    pwm1.setPWM(servonum1, 0, SERVOHOME1);
    Serial.print("coax...");
  }
  if (cal_femur) {
    pwm1.setPWM(servonum2, 0, SERVOHOME2);
    Serial.print("femur...");
  }
  if (cal_tibia) {
    pwm1.setPWM(servonum3, 0, SERVOHOME3);
    Serial.print("tibia...");
  }
  Serial.println(" ready!");
  delay(3000);
}

void loop() {

  if (cal_coax == 2) {
    if (SERVOMIN1 < SERVOMAX1) {
      Serial.print("coax min "); Serial.print(SERVOMIN1);
      Serial.print(" to max "); Serial.println(SERVOMAX1);
      for (int i=SERVOMIN1;i<SERVOMAX1+1; i++) {
        pwm1.setPWM(servonum1, 0, i);
        delay(spd);
      }
      Serial.print("coax max "); Serial.print(SERVOMAX1);
      Serial.print(" to min "); Serial.println(SERVOMIN1);
      for (int i=SERVOMAX1;i>SERVOMIN1-1; i--) {
        pwm1.setPWM(servonum1, 0, i);
        delay(spd);
      }
    } else {
      Serial.print("coax min "); Serial.print(SERVOMIN1);
      Serial.print(" to max "); Serial.println(SERVOMAX1);
      for (int i=SERVOMIN1;i>=SERVOMAX1; i--) {
        pwm1.setPWM(servonum1, 0, i);
        delay(spd);
      }
      Serial.print("coax max "); Serial.print(SERVOMAX1);
      Serial.print(" to min "); Serial.println(SERVOMIN1);
      for (int i=SERVOMAX1;i<=SERVOMIN1; i++) {
        pwm1.setPWM(servonum1, 0, i);
        delay(spd);
      }
    }
  }

  if (cal_femur == 2) {
    if (SERVOMIN2 < SERVOMAX2) {
      Serial.print("femur min "); Serial.print(SERVOMIN2);
      Serial.print(" to max "); Serial.println(SERVOMAX2);
      for (int i=SERVOMIN2;i<=SERVOMAX2; i++) {
        pwm1.setPWM(servonum2, 0, i);
        delay(spd);
      }
      Serial.print("femur max "); Serial.print(SERVOMAX2);
      Serial.print(" to min "); Serial.println(SERVOMIN2);
      for (int i=SERVOMAX2;i>=SERVOMIN2; i--) {
        pwm1.setPWM(servonum2, 0, i);
        delay(spd);
      }
    } else {
      Serial.print("femur min "); Serial.print(SERVOMIN2);
      Serial.print(" to max "); Serial.println(SERVOMAX2);
      for (int i=SERVOMIN2;i>=SERVOMAX2; i--) {
        pwm1.setPWM(servonum2, 0, i);
        delay(spd);
      }
      Serial.print("femur max "); Serial.print(SERVOMAX2);
      Serial.print(" to min "); Serial.println(SERVOMIN2);
      for (int i=SERVOMAX2;i<=SERVOMIN2; i++) {
        pwm1.setPWM(servonum2, 0, i);
        delay(spd);
      }      
    }
  }

  if (cal_tibia == 2) {
    if (SERVOMIN2 < SERVOMAX2) {
      Serial.print("tibia min "); Serial.print(SERVOMIN3);
      Serial.print(" to max "); Serial.println(SERVOMAX3);
      for (int i=SERVOMIN3;i<=SERVOMAX3; i++) {
        pwm1.setPWM(servonum3, 0, i);
        delay(spd);
      }
      Serial.print("tibia max "); Serial.print(SERVOMAX3);
      Serial.print(" to min "); Serial.println(SERVOMIN3);
      for (int i=SERVOMAX3;i>=SERVOMIN3; i--) {
        pwm1.setPWM(servonum3, 0, i);
        delay(spd);
      }
    } else {
      Serial.print("tibia min "); Serial.print(SERVOMIN3);
      Serial.print(" to max "); Serial.println(SERVOMAX3);
      for (int i=SERVOMIN3;i>=SERVOMAX3; i--) {
        pwm1.setPWM(servonum3, 0, i);
        delay(spd);
      }
      Serial.print("tibia max "); Serial.print(SERVOMAX3);
      Serial.print(" to min "); Serial.println(SERVOMIN3);
      for (int i=SERVOMAX3;i<=SERVOMIN3; i++) {
        pwm1.setPWM(servonum3, 0, i);
        delay(spd);
      }      
    }
  }

}
