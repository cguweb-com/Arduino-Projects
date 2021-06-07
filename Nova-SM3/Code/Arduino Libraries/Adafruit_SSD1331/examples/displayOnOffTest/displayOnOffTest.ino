/*
 * This is an example sketch that shows how to toggle the SSD1331 OLED display
 * on and off at runtime to avoid screen burn-in.
 * 
 * The sketch also demonstrates how to erase a previous value by re-drawing the 
 * older value in the screen background color prior to writing a new value in
 * the same location. This avoids the need to call fillScreen() to erase the
 * entire screen followed by a complete redraw of screen contents.
 * 
 * Written by Phill Kelley. BSD license.
 */
 
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1331.h>
#include <SPI.h>

#define SerialDebugging true

// The SSD1331 is connected like this (plus VCC plus GND)
const uint8_t   OLED_pin_scl_sck        = 13;
const uint8_t   OLED_pin_sda_mosi       = 11;
const uint8_t   OLED_pin_cs_ss          = 10;
const uint8_t   OLED_pin_res_rst        = 9;
const uint8_t   OLED_pin_dc_rs          = 8;

// connect a push button to ...
const uint8_t   Button_pin              = 2;

// SSD1331 color definitions
const uint16_t  OLED_Color_Black        = 0x0000;
const uint16_t  OLED_Color_Blue         = 0x001F;
const uint16_t  OLED_Color_Red          = 0xF800;
const uint16_t  OLED_Color_Green        = 0x07E0;
const uint16_t  OLED_Color_Cyan         = 0x07FF;
const uint16_t  OLED_Color_Magenta      = 0xF81F;
const uint16_t  OLED_Color_Yellow       = 0xFFE0;
const uint16_t  OLED_Color_White        = 0xFFFF;

// The colors we actually want to use
uint16_t        OLED_Text_Color         = OLED_Color_Black;
uint16_t        OLED_Backround_Color    = OLED_Color_Blue;

// declare the display
Adafruit_SSD1331 oled =
    Adafruit_SSD1331(
        OLED_pin_cs_ss,
        OLED_pin_dc_rs,
        OLED_pin_sda_mosi,
        OLED_pin_scl_sck,
        OLED_pin_res_rst
     );

// assume the display is off until configured in setup()
bool            isDisplayVisible        = false;

// declare size of working string buffers. Basic strlen("d hh:mm:ss") = 10
const size_t    MaxString               = 16;

// the string being displayed on the SSD1331 (initially empty)
char oldTimeString[MaxString]           = { 0 };

// the interrupt service routine affects this
volatile bool   isButtonPressed         = false;


// interrupt service routine
void senseButtonPressed() {
    if (!isButtonPressed) {
        isButtonPressed = true;
    }
}


void displayUpTime() {

    // calculate seconds, truncated to the nearest whole second
    unsigned long upSeconds = millis() / 1000;

    // calculate days, truncated to nearest whole day
    unsigned long days = upSeconds / 86400;

    // the remaining hhmmss are
    upSeconds = upSeconds % 86400;

    // calculate hours, truncated to the nearest whole hour
    unsigned long hours = upSeconds / 3600;

    // the remaining mmss are
    upSeconds = upSeconds % 3600;

    // calculate minutes, truncated to the nearest whole minute
    unsigned long minutes = upSeconds / 60;

    // the remaining ss are
    upSeconds = upSeconds % 60;

    // allocate a buffer
    char newTimeString[MaxString] = { 0 };

    // construct the string representation
    sprintf(
        newTimeString,
        "%lu %02lu:%02lu:%02lu",
        days, hours, minutes, upSeconds
    );

    // has the time string changed since the last oled update?
    if (strcmp(newTimeString,oldTimeString) != 0) {

        // yes! home the cursor
        oled.setCursor(0,0);

        // change the text color to the background color
        oled.setTextColor(OLED_Backround_Color);

        // redraw the old value to erase
        oled.print(oldTimeString);

        // home the cursor
        oled.setCursor(0,0);
        
        // change the text color to foreground color
        oled.setTextColor(OLED_Text_Color);
    
        // draw the new time value
        oled.print(newTimeString);
    
        // and remember the new value
        strcpy(oldTimeString,newTimeString);
        
    }

}


void setup() {

    // button press pulls pin LOW so configure HIGH
    pinMode(Button_pin,INPUT_PULLUP);

    // use an interrupt to sense when the button is pressed
    attachInterrupt(digitalPinToInterrupt(Button_pin), senseButtonPressed, FALLING);

    #if (SerialDebugging)
    Serial.begin(115200); while (!Serial); Serial.println();
    #endif

    // settling time
    delay(250);

    // ignore any power-on-reboot garbage
    isButtonPressed = false;

    // initialise the SSD1331
    oled.begin();
    oled.setFont();
    oled.fillScreen(OLED_Backround_Color);
    oled.setTextColor(OLED_Text_Color);
    oled.setTextSize(1);

    // the display is now on
    isDisplayVisible = true;

}


void loop() {

    // unconditional display, regardless of whether display is visible
    displayUpTime();

    // has the button been pressed?
    if (isButtonPressed) {
        
        // yes! toggle display visibility
        isDisplayVisible = !isDisplayVisible;

        // apply
        oled.enableDisplay(isDisplayVisible);

        #if (SerialDebugging)
        Serial.print("button pressed @ ");
        Serial.print(millis());
        Serial.print(", display is now ");
        Serial.println((isDisplayVisible ? "ON" : "OFF"));
        #endif

        // confirm button handled
        isButtonPressed = false;
        
    }

    // no need to be in too much of a hurry
    delay(100);
   
}
