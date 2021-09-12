
/*
 * Nova Remote
 * 
 * - finish NRF communication protocol for sending and receiving data
 * 
 */


int debug = 1;
int debug2 = 0;
int plotter = 0;

int nrf_active = 0;

#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//  NRF24 Pin Wiring for Arduino Mega/Nano:
//  pin 7/7 -> CE pin
//  pin 8/8 -> CSN pin
//  pin 52/13 -> SCK pin
//  pin 51/11 -> MOSI pin
//  pin 50/12 -> MISO pin
//  pin xx -> IRQ pin (unused)

//NRF24L01 transceiver
RF24 radio(7, 8);
unsigned int nrfInterval = 50;
unsigned long lastNRFUpdate = 0;
uint8_t address[][6] = {"1Node", "2Node"};
bool radioNumber = 0; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
String tr_data = "";
String rc_data = "";
float payload = 0.0;

#define POT1_PIN A8
#define POT2_PIN A9
#define POT3_PIN A10
#define POT4_PIN A11
int p1 = 0;
int p1p = 0;
int p2 = 0;
int p2p = 0;
int p3 = 0;
int p3p = 0;
int p4 = 0;
int p4p = 0;
int pot_slack = 20;

int jstick_center[2] = {490, 540};
#define RJX_PIN A5
#define RJY_PIN A4
//#define RJBTN_PIN A12
int rjb = 1;
int rjbp = 1;
byte rjxcen = 1;
byte rjycen = 1;

#define LJX_PIN A7
#define LJY_PIN A6   
//#define LJBTN_PIN A15 //PIN 58
int ljb = 1;
int ljbp = 1;
byte ljxcen = 1;
byte ljycen = 1;

//rgb leds
#define LED_STRIP_PIN 2
#define LED_STRIP_NUM 5
Adafruit_NeoPixel led_strip = Adafruit_NeoPixel(LED_STRIP_NUM, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);
unsigned int rgbInterval = 30;
unsigned long lastRGBUpdate = 0;
int rgb_del = 256;
int current_led = 0;
int fadeStep = 0;
int fade_steps = 400;
int fade_back = 1;
int pattern = 0;                  //set which light pattern is used
int pattern_int = 400;            //set pattern delay between
int pattern_cnt = 3;              //set number of loops of pattern
int pattern_step = 0;
int cur_rgb_val1[3] = {55, 0, 200};
int cur_rgb_val2[3] = {125, 125, 0};
int rgb_num = 99;
int rgb_bright = 25;


#define BTN1_PIN 3 //left front
#define BTN2_PIN 5 //right front
#define BTN3_PIN 4 //left back
#define BTN4_PIN 6 //right back
int b1 = 0;
int b1p = 0;
int b2 = 0;
int b2p = 0;
int b3 = 0;
int b3p = 0;
int b4 = 0;
int b4p = 0;

#define SEL1_PIN 9  //left green
#define SEL2_PIN 10 //right red
int s1 = 0;
int s1p = 0;
int s2 = 0;
int s2p = 0;
int s3 = 0;
int s3p = 0;

//D20 = SDA
//D21 = SCL
#define OLED_ADDR 0x3D
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
unsigned long lastOLEDUpdate = 0;
unsigned int oledInterval = 60;
  
#define OLED_ADDR2 0x3C
#define SCREEN_WIDTH2 128
#define SCREEN_HEIGHT2 32
Adafruit_SSD1306 display2(SCREEN_WIDTH2, SCREEN_HEIGHT2, &Wire, OLED_RESET);
unsigned long lastOLED2Update = 0;
unsigned int oled2Interval = 60;

void fade(int redStartValue, int redEndValue, int greenStartValue, int greenEndValue, int blueStartValue, int blueEndValue, int totalSteps, int fadeBack, int rgbNum = 99);
void blink_rgb_num(int rgb_val[3], int rgb_val2[3], int rgbNum=99);

void setup() {
  Serial.begin(19200);
  while (!Serial) {
  }

  if (nrf_active) {
    if (!radio.begin()) {
      if (debug2) Serial.println(F("radio hardware is not responding!!"));
      while (1) {}
    } else {
      if (debug2) Serial.println(F("radio hardware is ready!"));
      radio.setPALevel(RF24_PA_LOW);
      radio.setPayloadSize(sizeof(payload));
      radio.setChannel(124);
      radio.openWritingPipe(address[radioNumber]);
      radio.openReadingPipe(1, address[!radioNumber]);
      radio.stopListening();
      if (debug2) Serial.println(F("radio ready to transmit"));
    }
  }

  pinMode(POT1_PIN, INPUT);
  pinMode(POT2_PIN, INPUT);
  pinMode(POT3_PIN, INPUT);
  pinMode(POT4_PIN, INPUT);

  pinMode(LJX_PIN, INPUT);
  pinMode(LJY_PIN, INPUT);
//  pinMode(LJBTN_PIN, INPUT_PULLUP);

  pinMode(RJX_PIN, INPUT);
  pinMode(RJY_PIN, INPUT);
//  pinMode(RJBTN_PIN, INPUT_PULLUP);

  led_strip.begin();
  led_strip.setBrightness(rgb_bright);
  wipe_strip();

  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
  pinMode(BTN3_PIN, INPUT_PULLUP);
  pinMode(BTN4_PIN, INPUT_PULLUP);

  pinMode(SEL1_PIN, INPUT_PULLUP);
  pinMode(SEL2_PIN, INPUT_PULLUP);
  
  // initialize with the I2C addr
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);  
  display2.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR2);  
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Initializing...");
  display.display();
    
  display2.clearDisplay();
  display2.setTextSize(1);
  display2.setTextColor(WHITE);
  display2.setCursor(0,0);
  display2.println("Initializing...");
  display2.display();
  
  delay(2000);
  display.clearDisplay();
  display2.clearDisplay();
  
  // Scroll full screen
  display.setCursor(0,0);
  display.setTextSize(2);
  display.println("NOVA-SM3");
  display.setCursor(0,30);
  display.setTextSize(2);
  display.println("REMOTE");
  display.display();
  
  display2.setCursor(0,0);
  display2.setTextSize(1);
  display2.print("NOVA-SM3 REMOTE");
  display2.display();
  
  display.startscrollright(0x00, 0x07);
  display2.startscrollright(0x00, 0x07);
  delay(1000);
  display.stopscroll();
  display2.stopscroll();
  delay(200);
  display.startscrollleft(0x00, 0x07);
  display2.startscrollleft(0x00, 0x07);
  delay(1000);
  display.stopscroll();
  display2.stopscroll();
  delay(200);    
  display.startscrollright(0x00, 0x07);
  display2.startscrollright(0x00, 0x07);
  delay(1000);
  display.stopscroll();
  display2.stopscroll();
  delay(200);
  display.startscrollleft(0x00, 0x07);
  display2.startscrollleft(0x00, 0x07);
  delay(1000);
  display.stopscroll();
  display2.stopscroll();
  
  display.clearDisplay();
  display.display();
  display2.clearDisplay();
  display2.display();

  if (debug) Serial.println(F("Ready!"));
  pattern = 9;
  pattern_cnt = 16;
  pattern_int = 150;
}

void loop() {
  check_pots();
  check_buttons();
  check_joysticks();

  if (nrf_active && (millis() - lastNRFUpdate > nrfInterval)) rc_data = nrf_check();
  if (millis() - lastRGBUpdate > rgbInterval) rgb_check(pattern);
  if (millis() - lastOLEDUpdate > oledInterval) oled_refresh();
  if (millis() - lastOLED2Update > oled2Interval) oled2_refresh();


  if (debug) {
    if (!plotter) {
      Serial.print("pot1:");
      Serial.print(p1);
      Serial.print("\tpot2:");
      Serial.print(p2);
      Serial.print("\tpot3:");
      Serial.print(p3);
      Serial.print("\tpot4:");
      Serial.print(p4);
    }
  
    if (!plotter) {
      Serial.print("\tbtn1:");
      Serial.print(b1);
      Serial.print("\tbtn2:");
      Serial.print(b2);
      Serial.print("\tbtn3:");
      Serial.print(b3);
      Serial.print("\tbtn4:");
      Serial.print(b4);
  
      Serial.print("\tsel1:");
      Serial.print(s1);
      Serial.print("\tsel2:");
      Serial.print(s2);
      Serial.print("\tsel3:");
      Serial.print(s3);
    }
  
    Serial.print("\trjb:");
    Serial.print(rjb);
    Serial.print("\trjx:");
    Serial.print(analogRead(RJX_PIN));
    Serial.print("\trjy:");
    Serial.print(analogRead(RJY_PIN));
  
    Serial.print("\tljb:");
    Serial.print(ljb);
    Serial.print("\tljx:");
    Serial.print(analogRead(LJX_PIN));
    Serial.print("\tljy:");
    Serial.println(analogRead(LJY_PIN));
    delay(50);
  }

}


bool nrf_check() {
  bool resp = false;

//  if (tr_data) {
    resp = radio.write(&payload, sizeof(payload));
    if (resp) {
      if (debug2) {
        Serial.print(F("Transmission successful! "));
        Serial.println(payload);
        delay(1000);
      }
    } else {
      //handle failed transmission
    }
//  }

  return resp;
}


void nrf_request(String req_data) {
  //make a request expecting a response
  bool resp = radio.write(&req_data, sizeof(req_data));
  if (resp) {
    uint8_t pipe;
    radio.startListening();
    if (radio.available(&pipe)) {             // is there a payload? get the pipe number that recieved it
        uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
        radio.read(&rc_data, bytes);            // fetch payload from FIFO
        Serial.print(F("Received "));
        Serial.print(bytes);                    // print the size of the payload
        Serial.print(F(" bytes on pipe "));
        Serial.print(pipe);                     // print the pipe number
        Serial.print(F(": "));
        Serial.println(rc_data);                // print the payload's value
    }
    radio.stopListening();
  }
}


/*
  
SEL1_PIN = PSB_START
SEL2_PIN = PSB_SELECT

RJX_PIN = PSS_RX
RJY_PIN = PSS_RY
LJX_PIN = PSS_LX
LJY_PIN = PSS_LY

BTN1_PIN = PSB_L1
BTN3_PIN = PSB_L2
BTN2_PIN = PSB_R1
BTN4_PIN = PSB_R2

POT1_PIN = (spd)
POT3_PIN = (ramp_spd/ramp_dist)
POT2_PIN = (move_steps)
POT4_PIN = (step_height_factor)

- unassigned -
PSB_PAD_UP
PSB_PAD_RIGHT
PSB_PAD_LEFT
PSB_PAD_DOWN

PSB_TRIANGLE
PSB_CROSS
PSB_CIRCLE
PSB_SQUARE

PSB_L3
PSB_R3

*/

void check_pots() {
  int p1c = analogRead(POT1_PIN);
  if (p1c != p1p) {
    p1 = p1c;
    p1p = p1c;
    p1 = map(p1, 0, 1023, 128, 0);
    if (p1 > pot_slack) {
      bdrawline(2,1,1,128,1);
      wdrawline(2,1,1,p1,1);

      cur_rgb_val1[0] = 125; cur_rgb_val1[1] = 0; cur_rgb_val1[2] = 125;
      cur_rgb_val2[0] = 0; cur_rgb_val2[1] = 125; cur_rgb_val2[2] = 125;
      rgb_num = 0;
      if (p1 > 100) {
        rgb_num = 4;
      } else if (p1 > 75) {
        rgb_num = 3;
      } else if (p1 > 50) {
        rgb_num = 2;
      } else if (p1 > 25) {
        rgb_num = 1;
      }
      pattern = 9;
      pattern_cnt = 1;
      pattern_int = 10;
    }
  }

  int p2c = analogRead(POT2_PIN);
  if (p2c != p2p) {
    p2 = p2c;
    p2p = p2c;
    p2 = map(p2, 0, 1023, 128, 0);
    if (p2 > pot_slack) {
      bdrawline(2,1,5,128,5);
      wdrawline(2,1,5,p2,5);

      cur_rgb_val1[0] = 255; cur_rgb_val1[1] = 0; cur_rgb_val1[2] = 55;
      cur_rgb_val2[0] = 55; cur_rgb_val2[1] = 0; cur_rgb_val2[2] = 255;
      rgb_num = 0;
      if (p2 > 100) {
        rgb_num = 4;
      } else if (p2 > 75) {
        rgb_num = 3;
      } else if (p2 > 50) {
        rgb_num = 2;
      } else if (p2 > 25) {
        rgb_num = 1;
      }
      pattern = 9;
      pattern_cnt = 1;
      pattern_int = 10;    
    }
  }

  int p3c = analogRead(POT3_PIN);
  if (p3c != p3p) {
    p3 = p3c;
    p3p = p3c;
    p3 = map(p3, 0, 1023, 128, 0);
    if (p3 > pot_slack) {
      bdrawline(2,1,26,128,26);
      wdrawline(2,1,26,p3,26);

      cur_rgb_val1[0] = 55; cur_rgb_val1[1] = 255; cur_rgb_val1[2] = 55;
      cur_rgb_val2[0] = 55; cur_rgb_val2[1] = 55; cur_rgb_val2[2] = 255;
      rgb_num = 0;
      if (p3 > 100) {
        rgb_num = 4;
      } else if (p3 > 75) {
        rgb_num = 3;
      } else if (p3 > 50) {
        rgb_num = 2;
      } else if (p3 > 25) {
        rgb_num = 1;
      }
      pattern = 9;
      pattern_cnt = 1;
      pattern_int = 10;
    }
  }

  int p4c = analogRead(POT4_PIN);
  if (p4c != p4p) {
    p4 = p4c;
    p4p = p4c;
    p4 = map(p4, 0, 1023, 128, 0);
    if (p4 > pot_slack) {
      bdrawline(2,1,31,128,31);
      wdrawline(2,1,31,p4,31);

      cur_rgb_val1[0] = 255; cur_rgb_val1[1] = 55; cur_rgb_val1[2] = 55;
      cur_rgb_val2[0] = 255; cur_rgb_val2[1] = 55; cur_rgb_val2[2] = 255;
      rgb_num = 0;
      if (p4 > 100) {
        rgb_num = 4;
      } else if (p4 > 75) {
        rgb_num = 3;
      } else if (p4 > 50) {
        rgb_num = 2;
      } else if (p4 > 25) {
        rgb_num = 1;
      }
      pattern = 9;
      pattern_cnt = 1;
      pattern_int = 10;
    }
  }
}

void check_buttons() {
  int b1c = digitalRead(BTN1_PIN);
  if (b1c != b1p) {
    b1 = b1c;
    b1p = b1c;
    if (b1 == 1) {
      tr_data = "BTN1";
      wfillcircle(2,40,20,5);
      led_strip.setPixelColor(0, led_strip.Color(255, 0, 0));
    } else {
      bfillcircle(2,40,20,5);
      led_strip.setPixelColor(0, led_strip.Color(0, 0, 0));
    }
    led_strip.show();
  }

  int b2c = digitalRead(BTN2_PIN);
  if (b2c != b2p) {
    b2 = b2c;
    b2p = b2c;
    if (b2) {
      wfillcircle(2,88,20,5);
      led_strip.setPixelColor(4, led_strip.Color(255, 0, 0));
    } else {
      bfillcircle(2,88,20,5);
      led_strip.setPixelColor(4, led_strip.Color(0, 0, 0));
    }
    led_strip.show();
  }

  int b3c = digitalRead(BTN3_PIN);
  if (b3c != b3p) {
    b3 = b3c;
    b3p = b3c;
    if (b3) {
      wfillcircle(2,30,10,5);
      led_strip.setPixelColor(1, led_strip.Color(125, 125, 0));
    } else {
      bfillcircle(2,30,10,5);
      led_strip.setPixelColor(1, led_strip.Color(0, 0, 0));
    }
    led_strip.show();
  }

  int b4c = digitalRead(BTN4_PIN);
  if (b4c != b4p) {
    b4 = b4c;
    b4p = b4c;
    if (b4) {
      wfillcircle(2,98,10,5);
      led_strip.setPixelColor(3, led_strip.Color(125, 125, 0));
    } else {
      bfillcircle(2,98,10,5);
      led_strip.setPixelColor(3, led_strip.Color(0, 0, 0));
    }
    led_strip.show();
  }


  int s1c = digitalRead(SEL1_PIN);
  if (s1c != s1p) {
    s1 = s1c;
    s1p = s1c;
    if (s1) {
      wfillcircle(2,32,15,10);
      led_strip.setPixelColor(2, led_strip.Color(0, 255, 0));
    } else {
      bfillcircle(2,32,15,10);
      led_strip.setPixelColor(2, led_strip.Color(0, 0, 0));
    }
    led_strip.show();
  }

  int s2c = digitalRead(SEL2_PIN);
  if (s2c != s2p) {
    s2 = s2c;
    s2p = s2c;
    if (s2) {
      wfillcircle(2,96,15,10);
      led_strip.setPixelColor(2, led_strip.Color(255, 0, 0));
    } else {
      bfillcircle(2,96,15,10);
      led_strip.setPixelColor(2, led_strip.Color(0, 0, 0));
    }
    led_strip.show();
  }
}


void check_joysticks() {

  int ljx = analogRead(LJX_PIN);
  int ljy = analogRead(LJY_PIN);

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(10,18);
  display.print(ljx);
  display.print(",");
  display.println(ljy);

  if (ljx > jstick_center[1]) {
    ljxcen = 0;
    led_strip.setPixelColor(0, led_strip.Color(0, 255, 0));
  } else if (ljx < jstick_center[0]) {
    ljxcen = 0;
    led_strip.setPixelColor(4, led_strip.Color(0, 255, 0));
  } else if (!ljxcen) {
    ljxcen = 1;
    led_strip.setPixelColor(0, led_strip.Color(0, 0, 0));
    led_strip.setPixelColor(4, led_strip.Color(0, 0, 0));
  }
  if (ljy > jstick_center[1]) {
    ljycen = 0;
    led_strip.setPixelColor(1, led_strip.Color(0, 0, 255));
  } else if (ljy < jstick_center[0]) {
    ljycen = 0;
    led_strip.setPixelColor(3, led_strip.Color(0, 0, 255));
  } else if (!ljycen) {
    ljycen = 1;
    led_strip.setPixelColor(1, led_strip.Color(0, 0, 0));
    led_strip.setPixelColor(3, led_strip.Color(0, 0, 0));
  }
  led_strip.show();


  
  int rjx = analogRead(RJX_PIN);
  int rjy = analogRead(RJY_PIN);

  display.setCursor(10,47);
  display.print(rjx);
  display.print(",");
  display.println(rjy);

  if (rjx > jstick_center[1]) {
    rjxcen = 0;
    led_strip.setPixelColor(0, led_strip.Color(0, 255, 0));
  } else if (ljx < jstick_center[0]) {
    rjxcen = 0;
    led_strip.setPixelColor(4, led_strip.Color(0, 255, 0));
  } else if (!rjxcen) {
    rjxcen = 1;
    led_strip.setPixelColor(0, led_strip.Color(0, 0, 0));
    led_strip.setPixelColor(4, led_strip.Color(0, 0, 0));
  }
  if (rjy > jstick_center[1]) {
    rjycen = 0;
    led_strip.setPixelColor(1, led_strip.Color(0, 0, 255));
  } else if (rjy < jstick_center[0]) {
    rjycen = 0;
    led_strip.setPixelColor(3, led_strip.Color(0, 0, 255));
  } else if (!rjycen) {
    rjycen = 1;
    led_strip.setPixelColor(1, led_strip.Color(0, 0, 0));
    led_strip.setPixelColor(3, led_strip.Color(0, 0, 0));
  }
  led_strip.show();


/*
  int ljb = digitalRead(LJBTN_PIN);
  if (!ljb) {
    wfillcircle(1,110,24,6);
  } else {
    bfillcircle(1,110,24,6);
  }

  int rjbc = digitalRead(RJBTN_PIN);
  if (!rjb) {
    wfillcircle(1,110,54,6);
  } else {
    bfillcircle(1,110,54,6);
  }
*/
}


void bfillcircle(int d, int cx, int cy, int r) {
  if (d == 2) {
    display2.fillCircle(cx, cy, r, SSD1306_BLACK);
    display2.display();
  } else {
    display.fillCircle(cx, cy, r, SSD1306_BLACK);
    oled_refresh();
  }
}
void wfillcircle(int d, int cx, int cy, int r) {
  if (d == 2) {
    display2.fillCircle(cx, cy, r, SSD1306_WHITE);
    display2.display();
  } else {    
    display.fillCircle(cx, cy, r, SSD1306_WHITE);
    oled_refresh();
  }
}

void bdrawline(int d, int sx, int sy, int ex, int ey) {
  if (d == 2) {
    display2.drawLine(sx, sy, ex, ey, SSD1306_BLACK);
    display2.display();
  } else {
    display.drawLine(sx, sy, ex, ey, SSD1306_BLACK);
    oled_refresh();
  }
}

void wdrawline(int d, int sx, int sy, int ex, int ey) {
  if (d == 2) {
    display2.drawLine(sx, sy, ex, ey, SSD1306_WHITE);
    display2.display();
  } else {
    display.drawLine(sx, sy, ex, ey, SSD1306_WHITE);
    oled_refresh();
  }
}

void oled_refresh() {
  display.display();
}
void oled2_refresh() {
  display2.display();
}



/*
   -------------------------------------------------------
   LED Functions
   -------------------------------------------------------
*/

void rgb_check(int pat) {
  switch (pat) {
    case 0:
      colorWipe(led_strip.Color(0, 0, 0)); // off
      break;
    case 1:
      fade(0, 0, 0, 0, 0, 125, fade_steps, fade_back, rgb_num); // fade from blue to black
      break;
    case 2:
      fade(0, 255, 0, 64, 0, 0, fade_steps, 1, rgb_num); // fade from orange to black
      break;
    case 3:
      fade(0, 0, 0, 125, 0, 0, fade_steps, 1, rgb_num); // fade from green to black
      break;
    case 4:
      fade(0, 125, 0, 125, 0, 0, fade_steps, 1, rgb_num); // fade from yellow to black
      break;
    case 5:
      fade(0, 125, 0, 0, 0, 0, fade_steps, 1, rgb_num); // fade from red to black
      break;
    case 6:
      fade(0, 125, 0, 0, 0, 125, fade_steps, 1, rgb_num); // fade from purple to black
      break;
    case 7:
      colorWipe(led_strip.Color(255, 255, 255)); //white
      break;
    case 8:
      rainbow(pattern_int);
      break;
    case 9:
      if (pattern_cnt) {
        rgbInterval = pattern_int;
        blink_rgb_num(cur_rgb_val1, cur_rgb_val2, rgb_num);
      } else if (rgbInterval == pattern_int) {
        rgbInterval = 30;
        wipe_strip();
      }
      break;
    case 10:
      if (pattern_cnt) {
        rgbInterval = pattern_int;
        colorWipeWave(led_strip.Color(0, 32, 0)); // green wipe wave
      } else if (rgbInterval == pattern_int) {
        rgbInterval = 30;
        wipe_strip();
      }
      break;
    case 11:
      fade(cur_rgb_val1[0], cur_rgb_val2[0], cur_rgb_val1[1], cur_rgb_val2[1], cur_rgb_val1[2], cur_rgb_val2[2], fade_steps, fade_back, rgb_num); // fade from color1 to color2
      break;
  }
}



void fade(int redStartValue, int redEndValue, int greenStartValue, int greenEndValue, int blueStartValue, int blueEndValue, int totalSteps, int fadeBack, int rgbNum = 99) {
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
    if (rgbNum != 99) {
      led_strip.setPixelColor(rgbNum, led_strip.Color((int)red, (int)green, (int)blue));
    } else {
      for (int i = 0; i < led_strip.numPixels(); i++) {
        led_strip.setPixelColor(i, led_strip.Color((int)red, (int)green, (int)blue));
      }
    }

    // now display it
    led_strip.show();
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
  wipe_strip();
  led_strip.setPixelColor(i, c);
  led_strip.setPixelColor(i2, c);
  led_strip.show();
  i++;
  i2++;
  if (i >= led_strip.numPixels()) {
    i = 0;
    wipe_strip(); // blank out led_strip
  }
  if (i2 >= led_strip.numPixels()) {
    i2 = 0;
    wipe_strip(); // blank out led_strip
  }
  lastRGBUpdate = millis();
}

void colorWipeWave(uint32_t c) {
  if (!current_led) {
    wipe_strip();
  } else {  
    if (pattern_cnt) pattern_cnt--;
    led_strip.setPixelColor((current_led-1), c);
    led_strip.show();
  }

  if (current_led == led_strip.numPixels()) {
    current_led = 0;
  } else {
    current_led++;
  }

  lastRGBUpdate = millis();
}

void rainbow(int wait) {
  for (long firstPixelHue = 0; firstPixelHue < 5 * 65536; firstPixelHue += 256) {
    for (int i = 0; i < led_strip.numPixels(); i++) {
      int pixelHue = firstPixelHue + (i * 65536L / led_strip.numPixels());
      led_strip.setPixelColor(i, led_strip.gamma32(led_strip.ColorHSV(pixelHue)));
    }
    led_strip.show(); // Update strip with new contents
    delayMicroseconds(wait);
  }
  lastRGBUpdate = millis();
}


void wipe_strip() { // clear all LEDs
  for (int i = 0; i < led_strip.numPixels(); i++) {
    led_strip.setPixelColor(i, led_strip.Color(0, 0, 0));
  }
  led_strip.show();
}

void blink_rgb_num(int rgb_val[3], int rgb_val2[3], int rgbNum=99) {
  if (pattern_cnt) {
    if (pattern_step) {
      if (rgbNum != 99) {
        led_strip.setPixelColor(rgbNum, led_strip.Color(rgb_val[0], rgb_val[1], rgb_val[2]));
      } else {
        for (int i = 0; i < led_strip.numPixels(); i++) {
          led_strip.setPixelColor(i, led_strip.Color(rgb_val[0], rgb_val[1], rgb_val[2]));
        }
      }
      pattern_step = 0;
    } else {
      if (rgbNum != 99) {
        led_strip.setPixelColor(rgbNum, led_strip.Color(rgb_val2[0], rgb_val2[1], rgb_val2[2]));
      } else {
        for (int i = 0; i < led_strip.numPixels(); i++) {
          led_strip.setPixelColor(i, led_strip.Color(rgb_val2[0], rgb_val2[1], rgb_val2[2]));
        }
      }
      pattern_step = 1;
    }
    led_strip.show();
    pattern_cnt--;
  } else {
    wipe_strip();
  }

  lastRGBUpdate = millis();
}
