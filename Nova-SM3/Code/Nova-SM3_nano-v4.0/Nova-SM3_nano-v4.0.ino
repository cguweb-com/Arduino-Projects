/*
 *   NovaSM3 - a Spot-Mini Micro clone
 *   Version: 4.0
 *   Version Date: 2021-04-21
 *   
 *   Author:  Chris Locke - cguweb@gmail.com
 *   GitHub Project:  https://github.com/cguweb-com/Arduino-Projects/tree/main/Nova-SM3
 *   Thingiverse:  https://www.thingiverse.com/thing:4767006
 *   Instructables Project:  https://www.instructables.com/Nova-Spot-Micro-a-Spot-Mini-Clone/
 *   YouTube Playlist:  https://www.youtube.com/watch?v=00PkTcGWPvo&list=PLcOZNHwM_I2a3YZKf8FtUjJneKGXCfduk
 *   
 *   RELEASE NOTES:
 *      Arduino nano performance: 77% storage / 37% memory
 *
 *   DEV NOTES:
 * 
 */

//set Nova SM3 version
#define VERSION 4.0

byte debug1 = 0;

byte rgb_active = 1;              //activate RGB modules
byte oled_active = 1;             //activate OLED display
byte uss_active = 1;              //activate Ultra-Sonic sensors


#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//#include <NewPing.h>

//serial connection
#define RESET_PIN 4
#define SLAVE_ID 1
byte serial_oled = 0;
int resp;
char req;

//oled display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
unsigned long lastOLEDUpdate = 0;
unsigned int oledInterval = 250;
unsigned long lastOLEDClear = 0;
unsigned long oledClearInterval = 5000;
int oled_command = 0;

//button & trim pot
#define BTN_ACTIVE_PIN 3
#define LED_BRIGHT_PIN A3
int rgb_bright = 80;

//rgb leds
#define LED_EYES_PIN 2
#define LED_EYES_NUM 4
Adafruit_NeoPixel led_eyes = Adafruit_NeoPixel(LED_EYES_NUM, LED_EYES_PIN, NEO_GRB + NEO_KHZ800);
unsigned int rgbInterval = 30;
unsigned long lastRGBUpdate = 0;
int rgb_del = 256;
int current_led = 0;
int fadeStep = 0;
int fade_steps = 400;
int pattern = 0;                  //set which light pattern is used
int pattern_int = 400;            //set pattern delay between
int pattern_cnt = 3;              //set number of loops of pattern
int pattern_step = 0;
int cur_rgb_val1[3] = {55, 0, 200};  //set color of left lights
int cur_rgb_val2[3] = {55, 0, 200};  //set color of right lights
int pattern_side = 0;             // set left or right lights


//ultrasonic sensors
#define L_TRIGPIN 9
#define L_ECHOPIN 8
#define R_TRIGPIN 7
#define R_ECHOPIN 6
#define MAX_DISTANCE 200
//NewPing sonar_l(L_TRIGPIN, L_ECHOPIN, MAX_DISTANCE);
//NewPing sonar_r(R_TRIGPIN, R_ECHOPIN, MAX_DISTANCE);

float duration;
float durationpulse;
int dist_l;
int dist_r;

//spotmicro bmp for oled
const unsigned char smbmp [] PROGMEM = {
  // 'spot, 128x64px
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xc0, 0x00, 0x0e, 0x00, 0x01, 0xff, 0xfc, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xc0, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xc1, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xff, 0xc0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0f, 0xc7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb, 0xff, 0xc0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0f, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb, 0xff, 0xc0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0c, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf7, 0xff, 0x80, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xef, 0xff, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xdf, 0xff, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xdf, 0xff, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xbf, 0xfe, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x80, 0x00, 0x07, 0x7f, 0xff, 0x7f, 0xfc, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0x80, 0x00, 0x00, 0x1f, 0xf0, 0x7f, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x0f, 0xfd, 0xbf, 0x00, 0x00, 0x00, 0xff, 0x80, 0xfc, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x3f, 0xf1, 0x7e, 0x00, 0x00, 0x01, 0xfe, 0x01, 0xfc, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0xff, 0xc3, 0xfc, 0x00, 0x00, 0x00, 0xf8, 0x01, 0xf8, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x01, 0xfe, 0x03, 0xf8, 0x00, 0x00, 0x00, 0xf8, 0x03, 0xf0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x37, 0xf8, 0x07, 0xf0, 0x00, 0x00, 0x00, 0x70, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x3f, 0xe0, 0x0f, 0xe0, 0x00, 0x00, 0x00, 0x70, 0x0f, 0xc0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x3f, 0x00, 0x0f, 0xe0, 0x00, 0x00, 0x00, 0x30, 0x0f, 0x80, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x1f, 0x00, 0x0f, 0xc0, 0x00, 0x00, 0x00, 0x38, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x3e, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x00, 0x18, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x07, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x06, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x07, 0x80, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x07, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x03, 0xc0, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x01, 0xe0, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0xf0, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x78, 0x03, 0xe0, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x38, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x1c, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x1c, 0x00, 0x03, 0x80, 0x00, 0x00, 0x00, 0x7f, 0xc0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x6f, 0xf0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x21, 0xfc, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x20, 0x3e, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x30, 0x07, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x30, 0x03, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x03, 0x00, 0x00, 0x00, 0x30, 0x00, 0xe0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x03, 0x00, 0x00, 0x00, 0x40, 0x00, 0x70, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x02, 0x40, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x20, 0x00, 0x00, 0x78, 0x00, 0x0e, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x78, 0x00, 0x0d, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x01, 0x20, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x78, 0x00, 0x00, 0x80, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x01, 0x20, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x30, 0x00, 0x03, 0xc0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xe0, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};



void setup() {
  digitalWrite(RESET_PIN, HIGH);
  delay(200);
  pinMode(RESET_PIN, OUTPUT);     

  Wire.begin(SLAVE_ID); 
  Wire.onRequest(requestCallback);
  Wire.onReceive(receiveEvent);

  delay(200);
  Serial.begin(19200);

  pinMode(LED_BRIGHT_PIN, INPUT);
//  pinMode(BTN_ACTIVE_PIN, INPUT);
  
  led_eyes.begin();
  led_eyes.setBrightness(rgb_bright);
  wipe_eyes();

  if (uss_active) {
    pinMode(L_TRIGPIN, OUTPUT);
    pinMode(L_ECHOPIN, INPUT);
    pinMode(R_TRIGPIN, OUTPUT);
    pinMode(R_ECHOPIN, INPUT);
  }

  if (oled_active) {
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
  
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 20);
    display.println("Initializing...");
    display.display();
    delay(1500);
    pattern_cnt = 5;
    pattern_int = 250;
    pattern = 11;
    display.clearDisplay();
    display.display();
  
    //animate bitmap
    display.clearDisplay();
    delay(500);
    display.drawBitmap(0, 0,  smbmp, 128, 64, WHITE);
    display.display();
    for (int i=0;i<4;i++) {
      display.startscrollright(0x00, 0x07);
      delay(500);
      display.startscrollleft(0x00, 0x07);
      delay(500);
    }
    display.stopscroll();
    display.clearDisplay();
    display.display();
    delay(500);

    //Display banner
    for (int i=25;i>0;i--) {
      int d = (i*2);
      if (i > 22) d = (i*10);
      display.setCursor(20, 10);
      display.setTextSize(2);
      display.println("NOVA-SM3");
      display.setCursor(50, 26);
      display.setTextSize(1);
      display.print("v");    
      display.println(VERSION);
      display.display();
      delay(d);
      display.clearDisplay();
      display.display();
      delay(d);
    }
    display.clearDisplay();
    display.display();  
    sleep_display(&display);
  }

  if (debug1) Serial.println(F("Ready!"));
}



void loop() {

/*
   -------------------------------------------------------
   Check State Machines
    :check active state machine(s) for execution time by its respective interval
   -------------------------------------------------------
*/
  if (rgb_active) {
    if (millis() - lastRGBUpdate > rgbInterval) rgb_check(pattern);
  }
  if (oled_active) {
    if (millis() - lastOLEDUpdate > oledInterval) oled_check(oled_command);
    if (millis() - lastOLEDClear > oledClearInterval) oled_clear();
  }

/*
  if (digitalRead(BTN_ACTIVE_PIN) == 0) {
    if (debug1)
      Serial.println(F("btn active 1"));
    delay(1000);
    if (digitalRead(BTN_ACTIVE_PIN) == 0) {
      if (debug1)
        Serial.println(F("btn active 2"));
      pattern_cnt = 3;
      pattern_int = 100;
      rgb_check(3);
      if (oled_active) {
        oled_active = 0;
        oled_clear();
      } else {
        oled_active = 1;
      }
    }
  }
*/
}

void receiveEvent(int aCount) {
  byte x;
  while (0 < Wire.available()) {
    x = Wire.read();
    req = x;
  }
}

void requestCallback() {  
  byte send_resp = 0;
  resp = 0;
  if (debug1) {
    Serial.print(F("oled: "));Serial.print(oled_active);Serial.print("\t");
    Serial.print(F("rgb: "));Serial.print(rgb_bright);Serial.print("\t");
    if (serial_oled) {
      Serial.print(F("OLED Command "));
    } else {
      Serial.print(F("System Command "));
    }
  }
  if (debug1) Serial.print(F("req: "));
  if (debug1) Serial.println(req);

  if (serial_oled) {
    switch (req) {
      case 'X':
        //toggle between rgb/uss (0) & oled command (1) inputs
        serial_oled = 0;
        send_resp = 1;
        resp = serial_oled;
        break;  
      case 'Z':
        reset_slave();
        break;  
      default:
        send_resp = req;
        oled_command = req;
        break;
    }
  } else {
    switch (req) {  
      //uss distances
      case 'a':
        send_resp = 1;
        resp = get_distance(0);
        break;
      case 'b':
        send_resp = 1;
        resp = get_distance(1);
        break;
      case 'c':
        oled_command = 9997;
        break;

      //pattern setting
      case 'd':
        pattern = 1;
        break;
      case 'e':
        pattern = 2;
        break;
      case 'f':
        pattern = 3;
        break;
      case 'g':
        pattern = 4;
        break;
      case 'h':
        pattern = 5;
        break;
      case 'i':
        pattern = 6;
        break;
      case 'j':
        pattern = 7;
        break;
      case 'k':
        pattern = 8;
        break;
      case 'l':
        pattern = 9;
        break;
      case 'm':
        pattern = 10;
        break;
      case 'n':
        pattern = 11;
        break;
      case 'o':
        pattern = 12;
        break;
      case 'p':
        pattern = 0;
        break;
      case 'q': //reserved
        break;
      case 'r': //reserved
        break;
  
      //pattern_cnt setting
      case 's':
        pattern_cnt = 0;
        break;
      case 't':
        pattern_cnt = 1;
        break;
      case 'u':
        pattern_cnt = 2;
        break;
      case 'v':
        pattern_cnt = 3;
        break;
      case 'w':
        pattern_cnt = 4;
        break;
      case 'x':
        pattern_cnt = 5;
        break;
      case 'y':
        pattern_cnt = 10;
        break;
      case 'z':
        pattern_cnt = 25;
        break;
      case 'A':
        pattern_cnt = 100;
        break;
      case 'B': //reserved
        break;
      case 'C': //reserved
        break;

      //pattern_int setting
      case 'D':
        pattern_int = fade_steps = 30;
        break;
      case 'E':
        pattern_int = fade_steps = 50;
        break;
      case 'F':
        pattern_int = fade_steps = 100;
        break;
      case 'G':
        pattern_int = fade_steps = 250;
        break;
      case 'H':
        pattern_int = fade_steps = 500;
        break;
      case 'I':
        pattern_int = fade_steps = 1000;
        break;
      case 'J':
        pattern_int = fade_steps = 2500;
        break;
      case 'K':
        pattern_int = fade_steps = 5000;
        break;
      case 'L': //reserved
        break;

      //left and right led colors
      case 'M':
        //set left
        pattern_side = 0;
        break;
      case 'N':
        //set right
        pattern_side = 1;
        break;
      case 'O': //red
        if (pattern_side) {
          cur_rgb_val2[0] = 255; cur_rgb_val2[1] = 0; cur_rgb_val2[2] = 0;
        } else {
          cur_rgb_val1[0] = 255; cur_rgb_val1[1] = 0; cur_rgb_val1[2] = 0;
        }
        break;
      case 'P': //green
        if (pattern_side) {
          cur_rgb_val2[0] = 0; cur_rgb_val2[1] = 255; cur_rgb_val2[2] = 0;
        } else {
          cur_rgb_val1[0] = 0; cur_rgb_val1[1] = 255; cur_rgb_val1[2] = 0;
        }
        break;
      case 'Q': //blue
        if (pattern_side) {
          cur_rgb_val2[0] = 0; cur_rgb_val2[1] = 0; cur_rgb_val2[2] = 255;
        } else {
          cur_rgb_val1[0] = 0; cur_rgb_val1[1] = 0; cur_rgb_val1[2] = 255;
        }
        break;
      case 'R': //yellow
        if (pattern_side) {
          cur_rgb_val2[0] = 255; cur_rgb_val2[1] = 255; cur_rgb_val2[2] = 55;
        } else {
          cur_rgb_val1[0] = 255; cur_rgb_val1[1] = 255; cur_rgb_val1[2] = 55;
        }
        break;
      case 'S': //purple
        if (pattern_side) {
          cur_rgb_val2[0] = 55; cur_rgb_val2[1] = 0; cur_rgb_val2[2] = 200;
        } else {
          cur_rgb_val1[0] = 55; cur_rgb_val1[1] = 0; cur_rgb_val1[2] = 200;
        }
        break;
      case 'T': //orange
        if (pattern_side) {
          cur_rgb_val2[0] = 255; cur_rgb_val2[1] = 155; cur_rgb_val2[2] = 0;
        } else {
          cur_rgb_val1[0] = 255; cur_rgb_val1[1] = 155; cur_rgb_val1[2] = 0;
        }
        break;
      case 'U': //white
        if (pattern_side) {
          cur_rgb_val2[0] = 255; cur_rgb_val2[1] = 255; cur_rgb_val2[2] = 255;
        } else {
          cur_rgb_val1[0] = 255; cur_rgb_val1[1] = 255; cur_rgb_val1[2] = 255;
        }
        break;
      case 'V': //grey
        if (pattern_side) {
          cur_rgb_val2[0] = 155; cur_rgb_val2[1] = 155; cur_rgb_val2[2] = 155;
        } else {
          cur_rgb_val1[0] = 155; cur_rgb_val1[1] = 155; cur_rgb_val1[2] = 155;
        }
        break;
      case 'W': //reserved
        break;

      case 'X':
        //toggle between rgb/uss (0) & oled command (1) inputs
        serial_oled = 1;
        send_resp = 1;
        resp = serial_oled;
        break;  

      case 'Z':
        reset_slave();
        break;  
    }
  }

  if (send_resp) {
    if (debug1) Serial.print(F("Sending response..."));
    uint8_t buffer[2];
    buffer[0] = resp >> 8;
    buffer[1] = resp & 0xff;  
    Wire.write(buffer, 2);
    if (debug1) Serial.println(F("sent!"));
  }

}

int get_distance(int side) {
  int tpin = L_TRIGPIN;
  int epin = L_ECHOPIN;
  if (side) {
    tpin = R_TRIGPIN;
    epin = R_ECHOPIN;
  }
  digitalWrite(tpin, LOW);
  delayMicroseconds(2);
  digitalWrite(tpin, HIGH);
  delayMicroseconds(8);
  digitalWrite(tpin, LOW);
  durationpulse = pulseIn(epin, HIGH);
  int dist = durationpulse*0.034/2;
  if (dist < MAX_DISTANCE) {
    if (side) {
      dist_r = dist;
      if (debug1) Serial.print(F("Right ultrasonic sensor:\t"));
    } else {
      dist_l = dist;
      if (debug1) Serial.print(F("Left ultrasonic sensor:\t\t"));
    }
  }
/*  
  unsigned int uS = 0;
  int dist = 0;
  if (side) {
    uS = sonar_r.ping(); // Send ping, get ping time in microseconds (uS).
    dist = sonar_r.convert_cm(uS); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
    if (dist < MAX_DISTANCE) {
      dist_r = dist;
      if (debug1) Serial.print(F("Right ultrasonic sensor:\t"));
    }
  } else {
    uS = sonar_l.ping(); // Send ping, get ping time in microseconds (uS).
    dist = sonar_l.convert_cm(uS); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
    if (dist < MAX_DISTANCE) {
      dist_l = dist;
      if (debug1) Serial.print(F("Left ultrasonic sensor:\t\t"));
    }
  }
*/

  if (dist < MAX_DISTANCE) {
    if (debug1) Serial.println(dist);
  }

  return dist;
}



/*
   -------------------------------------------------------
   OLED Functions
   -------------------------------------------------------
*/
void oled_check(char cmd) {
  if (cmd > 0) {
    if (debug1) Serial.print(F("Print to OLED: "));
    if (debug1) Serial.println(cmd);
    wake_display(&display);
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    if (cmd == 97) { //a
      display.println("Grrrrrrr!"); //wake
    } else if (cmd == 98) { //b
      display.println("Stay!"); //stay
    } else if (cmd == 99) { //c
      display.setCursor(0, 0);
      display.print("Lt: ");
      display.println(dist_l);
      display.setCursor(0, 20);
      display.print("Rt: ");
      display.println(dist_r);
    } else if (cmd == 100) { //d
      display.println("March!"); //march
    } else if (cmd == 101) {  //e
      display.println("INTRUDER"); //intruder alert
      display.setTextSize(3);
      display.setCursor(0, 15);
      display.println("ALERT!");
    } else if (cmd == 102) {  //f
      display.println("ALERT");
      display.setCursor(0, 19);
      display.println("COMPLETE!");
    } else if (cmd == 103) {  //g
      display.println("Mode 1"); //select 1
    } else if (cmd == 104) {  //h
      display.println("Mode 2"); //select 2
    } else if (cmd == 105) {  //i
      display.println("Mode 3"); //select 3
    } else if (cmd == 106) {  //j
      display.println("Mode 4"); //select 4
    } else if (cmd == 107) {  //k
      display.println("SYSTEM"); //halt
      display.setTextSize(3);
      display.setCursor(0, 15);
      display.println("HALT!");
    } else if (cmd == 108) {  //l
      display.println("Ready!"); //ready
    }
    display.display();
    oled_command = 0;
    oledClearInterval = 5000;
    lastOLEDClear = millis();
  }

  lastOLEDUpdate = millis();
}

void oled_clear() {
  if (oled_command == 0) {
    if (debug1) Serial.println(F("Clear OLED"));
    display.clearDisplay();
    sleep_display(&display);
  }
  oledClearInterval = 5000000;
  lastOLEDClear = millis();
}

void sleep_display(Adafruit_SSD1306* display) {
  display->ssd1306_command(SSD1306_DISPLAYOFF);
}

void wake_display(Adafruit_SSD1306* display) {
  display->ssd1306_command(SSD1306_DISPLAYON);
}




/*
   -------------------------------------------------------
   LED Functions
   -------------------------------------------------------
*/

void rgb_check(int pat) {
  int bright = analogRead(LED_BRIGHT_PIN);
  rgb_bright = map(bright, 0, 1023, 100, 0);

  led_eyes.setBrightness(rgb_bright);

  switch (pat) {
    case 0:
      colorWipe(led_eyes.Color(0, 0, 0)); // off
      break;
    case 1:
      fade(0, 0, 0, 0, 0, 125, fade_steps, 1); // fade from blue to black
      break;
    case 2:
      fade(0, 255, 0, 64, 0, 0, fade_steps, 1); // fade from black to orange and back
      break;
    case 3:
      fade(0, 0, 0, 125, 0, 0, fade_steps, 1); // fade from green to black
      break;
    case 4:
      fade(0, 125, 0, 125, 0, 0, fade_steps, 1); // fade from yellow to black
      break;
    case 5:
      fade(0, 125, 0, 0, 0, 0, fade_steps, 1); // fade from red to black
      break;
    case 6:
      fade(0, 125, 0, 0, 0, 125, fade_steps, 1); // fade from purple to black
      break;
    case 7:
      colorWipe(led_eyes.Color(255, 255, 255)); //white
      break;
    case 8:
      if (pattern_cnt) {
        rgbInterval = pattern_int;
        colorWipePaired(led_eyes.Color(0, 32, 0)); // green wipe paired
      } else if (rgbInterval == pattern_int) {
        rgbInterval = 30;
        wipe_eyes();
      }
      break;
    case 9:
      rgb_flow(1000);
      break;
    case 10:
      rainbow(1000);
      break;
    case 11:
      if (pattern_cnt) {
        rgbInterval = pattern_int;
        blink_rgb(cur_rgb_val1, cur_rgb_val2);
      } else if (rgbInterval == pattern_int) {
        rgbInterval = 30;
        wipe_eyes();
      }
      break;
    case 12:
      if (pattern_cnt) {
        rgbInterval = pattern_int;
        colorWipeWave(led_eyes.Color(0, 32, 0)); // green wipe wave
      } else if (rgbInterval == pattern_int) {
        rgbInterval = 30;
        wipe_eyes();
      }
      break;
  }
}



void fade(int redStartValue, int redEndValue, int greenStartValue, int greenEndValue, int blueStartValue, int blueEndValue, int totalSteps, int fadeBack) {
  static float redIncrement, greenIncrement, blueIncrement;
  static float red, green, blue;
  static boolean fadeUp = false;

  if (fadeStep == 0) { // first step is to initialise the initial colour and increments
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
      led_eyes.setPixelColor(i, led_eyes.Color((int)red, (int)green, (int)blue));
    }

    // now display it
    led_eyes.show();
    fadeStep += 1; // go on to next step
    if (fadeStep >= totalSteps) { // finished fade
      if (fadeUp) { // finished fade up and back
        fadeStep = 0;
        return; // so next call recalabrates the increments
      }
      // now fade back
      if (fadeBack) {
        fadeUp = true;
        redIncrement = -redIncrement;
        greenIncrement = -greenIncrement;
        blueIncrement = -blueIncrement;
        fadeStep = 1; // don't calculate the increments again but start at first change
      }
    }
  }
}

void colorWipe(uint32_t c) {
  static int i = 0;
  static int i2 = 1;
  wipe_eyes();
  led_eyes.setPixelColor(i, c);
  led_eyes.setPixelColor(i2, c);
  led_eyes.show();
  i++;
  i2++;
  if (i >= led_eyes.numPixels()) {
    i = 0;
    wipe_eyes(); // blank out led_eyes
  }
  if (i2 >= led_eyes.numPixels()) {
    i2 = 0;
    wipe_eyes(); // blank out led_eyes
  }
  lastRGBUpdate = millis();
}

void colorWipePaired(uint32_t c) {
  wipe_eyes();
  led_eyes.setPixelColor(current_led, c);
  if (current_led == 0) {
    led_eyes.setPixelColor(3, c);
    current_led++;
  } else {
    led_eyes.setPixelColor(2, c);
    current_led--;
  }
  led_eyes.show();
  lastRGBUpdate = millis();
}


void colorWipeWave(uint32_t c) {
  if (!current_led) {
    wipe_eyes();
  } else {  
    led_eyes.setPixelColor((current_led-1), c);
    led_eyes.show();
  }

  if (current_led == led_eyes.numPixels()) {
    current_led = 0;
  } else {
    current_led++;
  }

  lastRGBUpdate = millis();
}

void rainbow(int wait) {
  for (long firstPixelHue = 0; firstPixelHue < 5 * 65536; firstPixelHue += 256) {
    for (int i = 0; i < led_eyes.numPixels(); i++) {
      int pixelHue = firstPixelHue + (i * 65536L / led_eyes.numPixels());
      led_eyes.setPixelColor(i, led_eyes.gamma32(led_eyes.ColorHSV(pixelHue)));
    }
    led_eyes.show(); // Update strip with new contents
    delayMicroseconds(wait);
  }
  lastRGBUpdate = millis();
}

void rgb_flow(int wait) {
  unsigned int rgbColour[3];

  rgbColour[0] = 255;
  rgbColour[1] = 255;
  rgbColour[2] = 255;

  for (int decColour = 0; decColour < 3; decColour += 1) {
    int incColour = decColour == 2 ? 0 : decColour + 1;
    for (int i = 0; i < 255; i += 5) {
      rgb_del -= 5;
      if (rgb_del < 1) rgb_del = 1;
      rgbColour[decColour] -= 5;
      rgbColour[incColour] += 5;

      for (int ledcnt = 0; ledcnt < 4; ledcnt += 1) {
        led_eyes.setPixelColor(ledcnt, rgbColour[0], rgbColour[1], rgbColour[2]);
        led_eyes.show();
        delayMicroseconds(wait);
      }
    }
  }

  lastRGBUpdate = millis();
}

void wipe_eyes() { // clear all LEDs
  for (int i = 0; i < led_eyes.numPixels(); i++) {
    led_eyes.setPixelColor(i, led_eyes.Color(0, 0, 0));
  }
  led_eyes.show();
}

void blink_rgb(int rgb_val[3], int rgb_val2[3]) {
  if (pattern_cnt) {
    if (!pattern_step) {
      wipe_eyes();
      if (pattern_cnt) {
        pattern_step = 1;
      }
    } else {
      if (rgb_val2[0] || rgb_val2[1] || rgb_val2[2]) {
        for (int i = 0; i < led_eyes.numPixels(); i++) {
          if (i == 0 || i == 1) { 
            led_eyes.setPixelColor(i, led_eyes.Color(rgb_val[0], rgb_val[1], rgb_val[2]));
          } else {
            led_eyes.setPixelColor(i, led_eyes.Color(rgb_val2[0], rgb_val2[1], rgb_val2[2]));
          }
        }
      } else {
        for (int i = 0; i < led_eyes.numPixels(); i++) {
          led_eyes.setPixelColor(i, led_eyes.Color(rgb_val[0], rgb_val[1], rgb_val[2]));
        }
      }
      led_eyes.show();
      pattern_step = 0;
      pattern_cnt--;
    }
  } else {
    wipe_eyes();
  }

  lastRGBUpdate = millis();
}

void reset_slave() {
  digitalWrite(RESET_PIN, LOW);
}
