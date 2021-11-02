/*
 *   NovaSM3 - a Spot-Mini Micro clone
 *   Version: 5.2
 *   Version Date: 2021-10-18
 *   
 *   Author:  Chris Locke - cguweb@gmail.com
 *   GitHub Project:  https://github.com/cguweb-com/Arduino-Projects/tree/main/Nova-SM2
 *   Thingiverse:  https://www.thingiverse.com/thing:4767006
 *   Instructables Project:  https://www.instructables.com/Nova-Spot-Micro-a-Spot-Mini-Clone/
 *   YouTube Playlist:  https://www.youtube.com/watch?v=00PkTcGWPvo&list=PLcOZNHwM_I2a3YZKf8FtUjJneKGXCfduk
 *   
 *   RELEASE NOTES:
 *      Arduino nano performance: 77% storage / 37% memory
 *      Added saving / retreiving eeprom data to enable changing settings from Mega
 *      Replaced i2c oled with SPI oled
 *      Added NewPing for ultrasonic sensors
 *      Added command switches for active settings
 *      Added battery gauge display code
 *      Added radar target display code
 *
 *   DEV NOTES:
 * 
 */

//set Nova SM3 version
#define VERSION 5.2

//load default active values, to be refreshed from eprom on boot
byte debug1 = 0;
byte rgb_active = 1;              //activate RGB modules
byte oled_active = 1;             //activate OLED display
byte uss_active = 1;              //activate Ultra-Sonic sensors
byte splash_active = 1;           //activate intro graphics at startup

#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1331.h>
#include <SPI.h>
#include <EEPROM.h>
#include <NewPing.h>

//serial connection
#define RESET_PIN 3
#define SLAVE_ID 1
byte serial_oled = 0;
int resp;
char req;

//oled display
int sclk = 13;    //scl or sck
int mosi = 11;    //sda or mosi
int cs = 10;
int dc = 9;
int rst = 8;
#define SCREEN_WIDTH 96
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1331 display = Adafruit_SSD1331(cs, dc, rst);

// Color definitions
#define LIGHTGREY       0xC618      /* 192, 192, 192 */
#define WHITE           0xFFFF      /* 255, 255, 255 */
#define YELLOW          0xFFE0      /* 255, 255,   0 */
#define GREENYELLOW     0xAFE5      /* 173, 255,  47 */
#define GREEN           0x07E0      /*   0, 255,   0 */
#define DARKGREEN       0x03E0      /*   0, 128,   0 */
#define DARKCYAN        0x03EF      /*   0, 128, 128 */
#define CYAN            0x07FF      /*   0, 255, 255 */
#define BLUE            0x001F      /*   0,   0, 255 */
#define NAVY            0x000F      /*   0,   0, 128 */
#define MAROON          0x7800      /* 128,   0,   0 */
#define PURPLE          0x780F      /* 128,   0, 128 */
#define MAGENTA         0xF81F      /* 255,   0, 255 */
#define PINK                        0xF81F
#define RED             0xF800      /* 255,   0,   0 */
#define ORANGE          0xFD20      /* 255, 165,   0 */
#define OLIVE           0x7BE0      /* 128, 128,   0 */
#define DARKGREY        0x7BEF      /* 128, 128, 128 */
#define BLACK           0x0000      /*   0,   0,   0 */



unsigned long lastOLEDUpdate = 0;
unsigned int oledInterval = 250;
unsigned long lastOLEDClear = 0;
unsigned long oledClearInterval = 5000;
int oled_command = 0;
int oled_default = 0;

//button & trim pot
#define OLED_ACTIVE_PIN 12
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
#define SONAR_NUM 2
#define L_TRIGPIN 7
#define L_ECHOPIN 6
#define R_TRIGPIN 5
#define R_ECHOPIN 4
#define MAX_DISTANCE 300
NewPing sonar[SONAR_NUM] = {
  NewPing(L_TRIGPIN, L_ECHOPIN, MAX_DISTANCE),
  NewPing(R_TRIGPIN, R_ECHOPIN, MAX_DISTANCE), 
};
int dist_l;
int dist_r;

int blevel_id = 0;
float batt_levels[10] = {                           //voltage drop alarm levels(10)
   11.2, 11.1, 11.0, 10.9, 10.8, 
   10.7, 10.6, 10.5, 10.4, 10.3,
};


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


// Linda "Nova" Harrison, 64x64px
const unsigned char lindabmp [] PROGMEM = {
  0xff, 0xe0, 0x00, 0x00, 0x00, 0x03, 0x80, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 
  0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 
  0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 
  0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 
  0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x1f, 
  0xfe, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xf0, 0x1f, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xf8, 0x0f, 
  0xfe, 0x00, 0x00, 0x00, 0x00, 0xff, 0xf8, 0x0f, 0xfc, 0x00, 0x00, 0x00, 0x01, 0xff, 0xfc, 0x0f, 
  0xfc, 0x00, 0x00, 0x00, 0x03, 0xff, 0xfc, 0x0f, 0xfc, 0x00, 0x00, 0x00, 0x07, 0xff, 0xfc, 0x0f, 
  0xfc, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xfc, 0x3f, 0xf8, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xfc, 0x3f, 
  0xf8, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xfc, 0x7f, 0xf8, 0x00, 0x00, 0x00, 0xff, 0xff, 0xfc, 0x7f, 
  0xf8, 0x00, 0x00, 0x00, 0x03, 0xff, 0xfc, 0x7f, 0xf0, 0x00, 0x00, 0x01, 0xf0, 0x7f, 0xf8, 0x7f, 
  0xf0, 0x00, 0x00, 0x07, 0xf8, 0x3f, 0xe0, 0x7f, 0xf0, 0x00, 0x00, 0x0e, 0x00, 0x1f, 0x80, 0x7f, 
  0xf0, 0x00, 0x00, 0x1c, 0x00, 0x3f, 0x00, 0x3f, 0xf0, 0x00, 0x00, 0x3e, 0x04, 0x7e, 0x00, 0x7f, 
  0xe0, 0x00, 0x00, 0x3f, 0xcf, 0xfc, 0x00, 0x7f, 0xe0, 0x00, 0x00, 0x3f, 0xff, 0xfe, 0x20, 0x3f, 
  0xe0, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xe0, 0x3f, 0xe0, 0x00, 0x00, 0x3f, 0xff, 0xff, 0x80, 0x3f, 
  0xe0, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xf0, 0x3f, 0xe0, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xf0, 0x3f, 
  0xe0, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xe0, 0x3f, 0xc0, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xe0, 0x3f, 
  0xc0, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xe0, 0x3f, 0xc0, 0x00, 0x00, 0x07, 0xff, 0xff, 0xe0, 0x1f, 
  0xc0, 0x00, 0x00, 0x07, 0xff, 0xff, 0xc0, 0x1f, 0xc0, 0x00, 0x00, 0x07, 0xff, 0x9f, 0xc0, 0x1f, 
  0x80, 0x00, 0x00, 0x07, 0xff, 0xc3, 0x80, 0x1f, 0x80, 0x00, 0x00, 0x07, 0xff, 0xe3, 0x00, 0x1f, 
  0x80, 0x00, 0x00, 0x07, 0xff, 0xf6, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x07, 0xff, 0xfc, 0x00, 0x1f, 
  0x00, 0x00, 0x00, 0x07, 0xe0, 0x60, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x1f, 
  0x00, 0x00, 0x00, 0x00, 0x0f, 0xf0, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf0, 0x00, 0x1f, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x0f, 
  0x00, 0x00, 0x00, 0x00, 0x0f, 0xc0, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xc0, 0x00, 0x0f, 
  0x00, 0x00, 0x00, 0x00, 0x0f, 0x80, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x07, 
  0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 
  0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01
};


void save_ep_data();
void load_ep_data();
void reset_slave();


void setup() {
  Serial.begin(19200);
  delay(200);

  //=======================================
  //IMPORTANT: FLASH EEPROM BEFORE USING!!
  //=======================================
  // set debug1 off and set all switches to active, then uncomment the two lines below and upload the code to flash EEPROM.
  // after the Nano finishes booting, comment-out these two lines, and re-upload the code again.
  // now the main Nova code running on the Teensy will set these switches according to their setting in Teensy Code ;)
  //=======================================
  //save_ep_data();
  //delay(1000);


  load_ep_data();
  delay(500);

  if (debug1) {
    Serial.println("eeprom loaded:");
    Serial.print("debug1: ");Serial.println(debug1);
    Serial.print("rgb_active: ");Serial.println(rgb_active);
    Serial.print("oled_active: ");Serial.println(oled_active);
    Serial.print("uss_active: ");Serial.println(uss_active);
    Serial.print("splash_active: ");Serial.println(splash_active);
    Serial.println();
  }

  digitalWrite(RESET_PIN, HIGH);
  delay(20);
  pinMode(RESET_PIN, OUTPUT);   
  
  Wire.begin(SLAVE_ID); 
  Wire.onRequest(requestCallback);
  Wire.onReceive(receiveEvent);
  delay(200);

  pinMode(LED_BRIGHT_PIN, INPUT);
  pinMode(OLED_ACTIVE_PIN, INPUT);
  digitalWrite(OLED_ACTIVE_PIN, LOW);

  if (rgb_active) {
    led_eyes.begin();
    led_eyes.setBrightness(rgb_bright);
    wipe_eyes();
  }

  if (oled_active) {
    display.begin();
    display.setFont();

    display.fillScreen(BLACK);        
    display.setTextSize(1);
    display.setTextColor(YELLOW);
    display.setCursor(0, 20);
    display.println("Initializing ");
    display.setCursor(0, 38);
    display.setTextColor(CYAN);
    display.println("  Hardware...");
    delay(1500);
    display.fillScreen(BLACK);

    if (splash_active) {
      //animate bitmap
      for (int i=0;i<12;i++) {
        display.drawBitmap(-20, 10,  smbmp, 128, 64, YELLOW);
        delay(30);
        display.drawBitmap(-20, 10,  smbmp, 128, 64, WHITE);
        delay(60);
        display.drawBitmap(-20, 10,  smbmp, 128, 64, PURPLE);
        delay(30);
      }
    }
    display.fillScreen(BLACK);
    display.setCursor(10, 10);
    display.setTextSize(3);
    display.setTextColor(MAGENTA);
    display.println("NOVA");
    display.setCursor(10, 35);
    display.setTextSize(2);
    display.setTextColor(YELLOW);
    display.print("SM3");
    display.setCursor(50, 45);
    display.setTextSize(1);
    display.setTextColor(GREEN);
    display.print("v");
    display.println(VERSION);
    delay(500);

    if (splash_active) {
      //Display banner
      int ac = 1;
      for (int i=15;i>0;i--) {
        int d = (i*2);
        if (i > 12) d = (i*10);
        if (ac) {
          display.setTextColor(YELLOW);
          ac = 0;
        } else {
          display.setTextColor(MAGENTA);
          ac = 1;
        }
        display.setCursor(10, 10);
        display.setTextSize(3);
        display.println("NOVA");
        display.setCursor(10, 35);
        display.setTextSize(2);
        display.print("SM3");
        display.setCursor(50, 45);
        display.setTextSize(1);
        display.print("v");
        display.println(VERSION);
        delay(d);
        display.fillScreen(BLACK);
        delay(d);
      }
    }

    display.fillScreen(BLACK);
    sleep_display(&display);
  }

  if (debug1) Serial.println(F("Ready!"));
  pattern = 12;
  pattern_cnt = 32; //4 per sequence for pattern 12
  pattern_int = 50;
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

  if (digitalRead(OLED_ACTIVE_PIN) == 1) {
    if (debug1)
      Serial.println(F("btn active 1"));
    delay(1000);
    if (digitalRead(OLED_ACTIVE_PIN) == 1) {
      if (debug1)
        Serial.println(F("btn active 2"));
      pattern_cnt = 3;
      pattern_int = 100;
      rgb_check(10);

      if (oled_active) {
        if (debug1)
          Serial.println(F("oled inactive"));
        display.begin();
        display.fillScreen(RED);
        display.setTextColor(WHITE);
        display.setTextSize(2);
        display.setCursor(25,15);
        display.print("OLED");
        display.setCursor(25,35);
        display.print("OFF!");
        delay(3000);
        oled_active = 0;
        oled_clear();
      } else {
        if (debug1)
          Serial.println(F("oled active"));
        oled_active = 1;
//DEV_NOTE: need to check here if oled was active / initialized at boot
//          if not, activate to eprom and reboot?
        wake_display(&display);
        display.begin();
        display.fillScreen(GREEN);
        display.setTextColor(BLACK);
        display.setTextSize(2);
        display.setCursor(25,15);
        display.print("OLED");
        display.setCursor(30,35);
        display.print("ON!");
        delay(3000);
        oled_clear();
      }
    }
  }
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
      case '0': blevel_id = 0; break;
      case '1': blevel_id = 1; break;
      case '2': blevel_id = 2; break;
      case '3': blevel_id = 3; break;
      case '4': blevel_id = 4; break;
      case '5': blevel_id = 5; break;
      case '6': blevel_id = 6; break;
      case '7': blevel_id = 7; break;
      case '8': blevel_id = 8; break;
      case '9': blevel_id = 9; break;
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

      case 'Y':
        splash_active = 0;
        EEPROM.write(4,splash_active);
        delay(300);
        reset_slave();
        break;  

      case 'Z':
        splash_active = 1;
        EEPROM.write(4,splash_active);
        delay(300);
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
  int dist = sonar[side].ping_cm();
  if (dist <= MAX_DISTANCE) {
    if (side) {
      dist_r = dist;
      if (debug1) Serial.print(F("Right ultrasonic sensor:\t"));
    } else {
      dist_l = dist;
      if (debug1) Serial.print(F("Left ultrasonic sensor:\t\t"));
    }
  }
  if (debug1) Serial.print(side);Serial.print(" : ");Serial.println(dist);

  return dist;
}



/*
   -------------------------------------------------------
   OLED Functions
   -------------------------------------------------------
*/
void oled_check(char cmd) {
  byte clear_int = 0;

  if (cmd > 0) {
    oled_default = 0;
    if (debug1) Serial.print(F("Print to OLED: "));
    if (debug1) Serial.println(cmd);
    wake_display(&display);
    display.fillScreen(BLACK);
    display.setTextSize(2);
    display.setCursor(12, 12);
    display.setTextColor(GREEN);

    if (cmd == 97) { //a
      display.println("Grrrrr!"); //wake
    } else if (cmd == 98) { //b
      display.println("Stay!"); //stay
    } else if (cmd == 99) { //c
      display.setTextSize(3); //uss display
      display.setCursor(0, 10);
      display.setTextColor(MAGENTA);
      display.print("L:");
      display.setTextColor(YELLOW);
      display.println(dist_l);
      display.setCursor(0, 40);
      display.setTextColor(MAGENTA);
      display.print("R:");
      display.setTextColor(YELLOW);
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
      display.setCursor(10, 10);
      display.setTextColor(MAGENTA);
      display.println("SYSTEM"); //halt
      display.setTextSize(2);
      display.setCursor(8, 30);
      display.setTextColor(RED);
      display.println("HALTED!");
    } else if (cmd == 108) {  //l
      display.setTextColor(BLUE);
      display.println("READY!"); //ready
    } else if (cmd == 109) {  //m
      alarm_display();
    } else if (cmd == 110) {  //n
      battery_gauge();
    } else if (cmd == 111) {  //o
      display.fillScreen(BLACK);
      display.setTextSize(2);
      display.setTextColor(YELLOW);
      display.setCursor(10, 4);
      display.println("PLEASE"); //stand back
      display.setTextColor(ORANGE);
      display.setCursor(15, 24);
      display.println("STAND");
      display.setTextColor(RED);
      display.setCursor(25, 44);
      display.println("BACK");
    } else if (cmd == 112) {  //p
      radar_display(48, 36); //forward
    } else if (cmd == 113) {  //q
      radar_display(60, 24); //forward-left
    } else if (cmd == 114) {  //r
      radar_display(36, 24); //forward-right
    } else if (cmd == 115) {  //s
      radar_display(72, 18); //left
    } else if (cmd == 116) {  //t
      radar_display(24, 18); //right
    } else if (cmd == 117) {  //u
      radar_display(48, 12); //backward
    } else if (cmd == 118) {  //v
      radar_display(48, 12); //greet
    } else if (cmd == 119) {  //w
      radar_display(48, 6); //stop
    } else if (cmd == 120) {  //x
      display.println("Mode 5"); //select 5
    } else if (cmd == 121) { //y
      display.fillScreen(BLACK);
      display.drawBitmap(16, 0,  lindabmp, 64, 64, LIGHTGREY);
      oledClearInterval = 7000;
      clear_int = 1;
    }
    oled_command = 0;
    if (!clear_int) {
      oledClearInterval = 5000;
    }
    lastOLEDClear = millis();
  } else if (!oled_default && (oledClearInterval != 5000 && oledClearInterval != 7000)) {
    //default display
    oled_default = 1;
    wake_display(&display);
    default_display();
  }

  lastOLEDUpdate = millis();
}

void alarm_display() {
  display.fillScreen(BLACK);
//DEV: future use
}


void radar_display(int cir_x, int cir_cnt) {
  display.fillScreen(BLACK);
  
  display.drawLine(0, 32, 96, 32, YELLOW);
  display.drawLine(48, 0, 48, 64, YELLOW);
  delay(100);
  display.drawLine(12, 0, 12, 64, BLACK);
  display.drawLine(13, 0, 13, 64, BLACK);
  display.drawLine(14, 0, 14, 64, BLACK);
  display.drawLine(24, 0, 24, 64, BLACK);
  display.drawLine(25, 0, 25, 64, BLACK);
  display.drawLine(26, 0, 26, 64, BLACK);
  display.drawLine(36, 0, 36, 64, BLACK);
  display.drawLine(37, 0, 37, 64, BLACK);
  display.drawLine(38, 0, 38, 64, BLACK);
  display.drawLine(60, 0, 60, 64, BLACK);
  display.drawLine(61, 0, 61, 64, BLACK);
  display.drawLine(62, 0, 62, 64, BLACK);
  display.drawLine(72, 0, 72, 64, BLACK);
  display.drawLine(73, 0, 73, 64, BLACK);
  display.drawLine(74, 0, 74, 64, BLACK);
  display.drawLine(84, 0, 84, 64, BLACK);
  display.drawLine(85, 0, 85, 64, BLACK);
  display.drawLine(86, 0, 86, 64, BLACK);

  display.drawLine(0, 12, 96, 12, BLACK);
  display.drawLine(0, 13, 96, 13, BLACK);
  display.drawLine(0, 14, 96, 14, BLACK);
  display.drawLine(0, 22, 96, 22, BLACK);
  display.drawLine(0, 23, 96, 23, BLACK);
  display.drawLine(0, 24, 96, 24, BLACK);
  display.drawLine(0, 40, 96, 40, BLACK);
  display.drawLine(0, 41, 96, 41, BLACK);
  display.drawLine(0, 42, 96, 42, BLACK);
  display.drawLine(0, 50, 96, 50, BLACK);
  display.drawLine(0, 51, 96, 51, BLACK);
  display.drawLine(0, 52, 96, 52, BLACK);
  
  if (cir_x) {
    for (int i=5;i<cir_cnt;i+=5) {
      delay(50);
      display.drawCircle(cir_x, 32, i, WHITE);
    }
    for (int i=cir_cnt;i>5;i-=5) {
      delay(50);
      display.drawCircle(cir_x, 32, i, GREEN);
    }
  }
}


void battery_gauge() {
  float blevel = batt_levels[blevel_id];

  display.fillScreen(BLACK);
  display.setCursor(0,0);
  if (blevel <= batt_levels[8]) {
    display.setTextSize(2);
    display.setTextColor(YELLOW);
    display.print("D");
    delay(50);
    display.print("A");
    delay(50);
    display.print("N");
    delay(50);
    display.print("G");
    delay(50);
    display.print("E");
    delay(50);
    display.print("R");
    delay(50);
    display.println("!!");
    delay(200);
    
    display.setTextColor(CYAN);
    display.setCursor(0,22);
    display.println("CHARGE");
    display.setCursor(0,42);
    display.println(" BATTERY");
  } else {
    display.setCursor(0,0);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.print("Bat ");
    display.setTextSize(2);
    if (blevel >= batt_levels[0]) {
      display.setTextColor(GREEN);
      display.print("+11 ");
    } else if (blevel >= batt_levels[3]) {
      display.setTextColor(GREEN);
      display.print(blevel);
    } else {
      display.setTextColor(YELLOW);
      display.print(blevel);
    }
    display.setTextColor(MAGENTA);
    display.println("v");
  }
  delay(10);

  if (blevel >= batt_levels[0]) {
    display.fillRect(  2, 45, 16, 15, GREEN);
    display.fillRect( 20, 40, 16, 20, GREEN);
    display.fillRect( 38, 35, 16, 25, GREEN);
    display.fillRect( 56, 30, 16, 30, GREEN);
    display.fillRect( 74, 25, 16, 35, GREEN);
  } else if (blevel >= batt_levels[1]) {
    display.fillRect(  2, 45, 16, 15, GREEN);
    display.fillRect( 20, 40, 16, 20, GREEN);
    display.fillRect( 38, 35, 16, 25, GREEN);
    display.fillRect( 56, 30, 16, 30, GREEN);
    display.fillRect( 74, 30, 16, 30, YELLOW);
  } else if (blevel >= batt_levels[2]) {
    display.fillRect(  2, 45, 16, 15, GREEN);
    display.fillRect( 20, 40, 16, 20, GREEN);
    display.fillRect( 38, 35, 16, 25, GREEN);
    display.fillRect( 56, 35, 16, 25, YELLOW);
    display.fillRect( 74, 30, 16, 30, YELLOW);
  } else if (blevel >= batt_levels[3]) {
    display.fillRect(  2, 45, 16, 15, GREEN);
    display.fillRect( 20, 40, 16, 20, GREEN);
    display.fillRect( 38, 40, 16, 20, YELLOW);
    display.fillRect( 56, 35, 16, 25, YELLOW);
    display.fillRect( 74, 30, 16, 30, YELLOW);
  } else if (blevel >= batt_levels[4]) {
    display.fillRect(  2, 50, 16, 10, GREEN);
    display.fillRect( 20, 45, 16, 15, YELLOW);
    display.fillRect( 38, 40, 16, 20, YELLOW);
    display.fillRect( 56, 35, 16, 25, YELLOW);
    display.fillRect( 74, 30, 16, 30, YELLOW);
  } else if (blevel >= batt_levels[5]) {
    display.fillRect(  2, 55, 16,  5, YELLOW);
    display.fillRect( 20, 50, 16, 10, YELLOW);
    display.fillRect( 38, 45, 16, 15, YELLOW);
    display.fillRect( 56, 40, 16, 20, YELLOW);
    display.fillRect( 74, 35, 16, 25, RED);
  } else if (blevel >= batt_levels[6]) {
    display.fillRect(  2, 55, 16,  5, YELLOW);
    display.fillRect( 20, 50, 16, 10, YELLOW);
    display.fillRect( 38, 45, 16, 15, YELLOW);
    display.fillRect( 56, 40, 16, 20, RED);
    display.fillRect( 74, 40, 16, 20, RED);
  } else if (blevel >= batt_levels[7]) {
    display.fillRect(  2, 55, 16,  5, YELLOW);
    display.fillRect( 20, 55, 16,  5, YELLOW);
    display.fillRect( 38, 50, 16, 10, RED);
    display.fillRect( 56, 45, 16, 15, RED);
    display.fillRect( 74, 40, 16, 20, RED);
  } else if (blevel >= batt_levels[8]) {
    display.fillRect(  2, 55, 16,  5, YELLOW);
    display.fillRect( 20, 55, 16,  5, RED);
    display.fillRect( 38, 55, 16,  5, RED);
    display.fillRect( 56, 50, 16, 10, RED);
    display.fillRect( 74, 45, 16, 15, RED);
  } else if (blevel >= batt_levels[9]) {
    display.fillRect(  2, 55, 16,  5, RED);
    display.fillRect( 20, 55, 16,  5, RED);
    display.fillRect( 38, 55, 16,  5, RED);
    display.fillRect( 56, 55, 16,  5, RED);
    display.fillRect( 74, 55, 16,  5, RED);
  }
}


void oled_clear() {
  if (oled_command == 0 && !oled_default) {
    if (debug1) Serial.println(F("Clear OLED"));
    display.fillScreen(BLACK);
    sleep_display(&display);
  }
  oledClearInterval = 5000000;
  lastOLEDClear = millis();
}


void default_display(void) {
  uint8_t w,h;

  display.setTextSize(1);
  display.setTextColor(MAGENTA);
  display.setCursor(0, 46);
  display.print("My Name Is");
  display.setTextColor(YELLOW);
  display.setCursor(66, 46);
  display.print("Nova");

  display.setAddrWindow(0, 58, 96, 64);
  for (h = 58; h < 64; h++) {
    for (w = 0; w < 96; w++) {
      if (w > 90) {
        display.writePixel(w, h, LIGHTGREY);
      } else if (w > 85) {
        display.writePixel(w, h, WHITE);
      } else if (w > 80) {
        display.writePixel(w, h, YELLOW);
      } else if (w > 75) {
        display.writePixel(w, h, GREENYELLOW);
      } else if (w > 70) {
        display.writePixel(w, h, GREEN);
      } else if (w > 65) {
        display.writePixel(w, h, DARKGREEN);
      } else if (w > 60) {
        display.writePixel(w, h, DARKCYAN);
      } else if (w > 55) {
        display.writePixel(w, h, CYAN);
      } else if (w > 50) {
        display.writePixel(w, h, BLUE);
      } else if (w > 45) {
        display.writePixel(w, h, NAVY);
      } else if (w > 40) {
        display.writePixel(w, h, MAROON);
      } else if (w > 35) {
        display.writePixel(w, h, PURPLE);
      } else if (w > 30) {
        display.writePixel(w, h, MAGENTA);
      } else if (w > 25) {
        display.writePixel(w, h, PINK);
      } else if (w > 20) {
        display.writePixel(w, h, RED);
      } else if (w > 15) {
        display.writePixel(w, h, ORANGE);
      } else if (w > 10) {
        display.writePixel(w, h, OLIVE);
      } else {
        display.writePixel(w, h, DARKGREY);
      }
    }
  }
  display.endWrite();
}

void sleep_display(Adafruit_SSD1331* display) {
    display->enableDisplay(0);
}

void wake_display(Adafruit_SSD1331* display) {
    display->enableDisplay(1);
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
    if (pattern_cnt) pattern_cnt--;
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


void load_ep_data() {
  //retrieve stored vars from eprom
  debug1 = EEPROM.read(0);
  rgb_active = EEPROM.read(1);
  oled_active = EEPROM.read(2);
  uss_active = EEPROM.read(3);
  splash_active = EEPROM.read(4);
}


void save_ep_data() {
  //save vars to eprom
  EEPROM.write(0,debug1);
  EEPROM.write(1,rgb_active);
  EEPROM.write(2,oled_active);
  EEPROM.write(3,uss_active);
  EEPROM.write(4,splash_active);
}


void reset_slave() {
  digitalWrite(RESET_PIN, LOW);
}
