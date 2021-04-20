/*
BUG NOTES:
 - display slows movements!?
 - when looping playback, seems to loop again at end, but just the delay parts
 - rencoder knob 
    : if controls are on, only moves within pot_allowance, then jumps back to pot setting
    : if controls are off, rencoder knob works as expected, but turning controls back on resets position(s) to pot(s)

DEV NOTES:
 - display update only line(s) needed
 - give warning on mem clear
 - when step recording limit reached, HALt (overwrites and starts over otherwise)
 - allow setting / saving of home position to memory
 - add control of delay between playback steps (ie: step_playback_delay)

HARDWARE NOTES:
 - add on/off switch for lasers
 - wire motor/pot pins correctly for successive numbering/pairing of pot and motor
 - add 'worklight' on controls (add dimmable white led)

*/

#include <Servo.h>
#include <EEPROM.h>
#include <FastLED.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#define LED_PIN     11
#define NUM_LEDS    1
CRGB leds[NUM_LEDS];

byte debug = 0;
byte debug_display = 0; //set on disables realtime display updates

//establish servo objects
Servo servo_grip;
Servo servo_hand;
Servo servo_wrist;
Servo servo_elbow;
Servo servo_shoulder;
Servo servo_chest;

//declare servo motor arrays
const byte motor_cnt = 6;
const char* motor_name[motor_cnt] = {"servo_grip", "servo_hand", "servo_wrist", "servo_elbow", "servo_shoulder", "servo_chest"};
const char* motor_label[motor_cnt] = {"Grip", "Hand", "Wrist", "Elbow", "Shoulder", "Chest"};
const byte motor_pin[motor_cnt] = {2, 3, 4, 5, 6, 7};
const byte motor_pot[motor_cnt] = {3, 2, 1, 6, 5, 4};

byte motor_val[motor_cnt] = {90, 90, 90, 90, 90, 90};
byte motor_val_prev[motor_cnt] = {90, 90, 90, 90, 90, 90};
byte motor_min[motor_cnt] = {80, 10, 10, 10, 45, 15};
byte motor_max[motor_cnt] = {130, 170, 170, 170, 170, 160};
byte motor_home[motor_cnt] = {120, 90, 118, 55, 133, 90};
byte motor_attach_pos[motor_cnt] = {90, 90, 90, 90, 90, 90};

//prototype motor functions
void move_motors(byte motor=99);
void step_motor(byte m, byte s);
void show_motor_vars(byte motor=99);
void switch_laser(long laser_delay=0);

//buttons
const byte home_btn = 8; 
const byte control_btn = 12; 
const byte playrecord_btn = 13; 
const byte record_btn = 9; 
const byte reset_memory_btn = 22; 

//rotary encoder controller & switch
byte rencoderA = 34;
byte rencoderB = 32;
byte rencoderS = 30;

//worklite leds & pot
const int worklite_dimmer = A0;
const int worklite = 10;
int worklite_write_prev = 0;

//laser guides
byte laser_one = 37;
byte laser_switch = 0;
long laser_timer_prev = 0;
long laser_interval = 0;

//USING TEST PINS fOR PHOTORES
//photoresistor
/* disabled
const int photores = A7;
int photores_value = 0;
int photores_led = 26;
*/

//operational variables
byte sync_move = 1;
byte sync_move_spd_factor = 6;
byte speed_control = 0;
byte servo_power = 0;
byte servo_step_speed_control = 0;  //toggle speed control on/off
byte servo_step_speed = 25;  //millisecond delay between servo steps
byte servo_step_speed_prev = 25;  //millisecond delay between servo steps
byte servo_step_speed_min = 5;  //min millisecond delay between servo steps
int servo_step_speed_max = 200;  //max millisecond delay between servo steps
byte loop_control = 0;
byte playback_loop_control = 0;
int step_playback_loops;  //number of times to loop playback steps
int step_playback_loops_prev;  //prev number of times to loop playback steps
int step_playback_delay = 0;  //delay between steps on playback
int step_playback_long_delay = 1000;  //delay to use when duplicate successive step(s)
byte homing = 0;  //active homing switch
byte homebtn = 0;  //track home btn switches
byte controlling = 0;  //active controlling switch
byte recording = 0;  //active recording switch
byte playing = 0;  //active playing switch
byte playing_steps = 0;  //active playing steps switch
byte moving = 0;
byte btn_pressed = 0;  //track when a btn is depressed

//recording variables
byte step_limit_fixed = 40;  //max number of steps to record
byte step_limit = step_limit_fixed;  //max number of steps to record iterator
byte step_positions = motor_cnt;  //ie: number of motors
byte step_cnt = 0;  //iterate recorded steps
int position_key = 1;  //iterate recorded step positions, start at 1 since 0 is recorded step_cnt

//rencoder variables
byte rencoding = 0;
byte motor_control = 1;

//oled display vars
String dis_line1 = "";
String dis_line2 = "";
String dis_line3 = "";
byte dis_var_int1 = 0;
byte dis_var_int2 = 0;
byte dis_line2_int = 0;
byte dis_spd = 1;
long display_timer_prev = 0;
long display_interval = 3000;


void setup() {
  //turn on serial monitor
  Serial.begin(9600);

  //set rgb led
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);

  //set buttons
  pinMode(home_btn, INPUT);
  digitalWrite(home_btn, HIGH);
  pinMode(control_btn, INPUT);
  digitalWrite(control_btn, HIGH);
  pinMode(playrecord_btn, INPUT);
  digitalWrite(playrecord_btn, HIGH);
  pinMode(record_btn, INPUT);
  digitalWrite(record_btn, HIGH);
  pinMode(reset_memory_btn, INPUT);
  digitalWrite(reset_memory_btn, HIGH);

  //set rotary encoder & switch
  pinMode(rencoderA, INPUT);
  pinMode(rencoderB, INPUT);
  pinMode(rencoderS, INPUT);
  digitalWrite(rencoderS, HIGH);

  //set worklite leds & pot
  pinMode(worklite_dimmer, INPUT);
  pinMode(worklite, OUTPUT);

  //set laser guides
  pinMode(laser_one, OUTPUT);

/* disabled
  //USING TEST PINS fOR PHOTORES
  pinMode(photores, INPUT);
  pinMode (photores_led, OUTPUT);
*/  

  //pause before attaching servos
  Serial.println(F("\n***********************************************"));
  Serial.println(F("Welcome to cguBOT v8  ::  May 18, 2019"));
  Serial.println(F("***********************************************\n"));
  Serial.println(F("Attaching servos..."));
  flash_led(0, 1, 0.5, 0);
  flash_led(1, 1, 0.5, 0);
  flash_led(2, 1, 0.5, 0);
  flash_led(3, 1, 0.5, 0);
  flash_led(4, 1, 0.5, 0);

  //init ic2 oled display
  //scl = 21; sda = 20
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();

  //attach servos to home after delay
  delay(1500);
  set_home();
  Serial.println(F("servos ready!"));

  //get steps recorded in memory
  step_cnt = EEPROM.read(0);
  //reset limit and position if steps
  if (step_cnt > 0) {
    step_limit = (step_limit_fixed - step_cnt);
    position_key += (step_cnt*step_positions);
  }
  if (debug) {
    Serial.print(F("MEMORY: steps: "));
    Serial.print(step_cnt);
    Serial.print(F(" position: "));
    Serial.println(position_key);
  }

  //display splash screen
  dis_line1 = "cguBOT v8";
  dis_line2 = "Initializing... ";
  display_text(0);

  //display motor info
  show_motor_vars();

  //give instructions
  Serial.println(F("\n***********************************************"));
  Serial.println(F("*** INSTRUCTIONS ***"));
  Serial.println(F("***********************************************\n\n"));
  Serial.println(F("  : Home:\n    Press the home button to reset positions"));
  Serial.println(F("  : Control Mode:\n    Press the control button to activate controls and move servos to control positions."));
  Serial.println(F("  : Step Control:\n    Press the step button to toggle step control."));
  Serial.println(F("    : Step Adjustment:\n    Press the step button to toggle between motors, and turn knob to adjust steps."));
  Serial.println(F("  : Speed Control:\n    Long-press the step button for 2 seconds to toggle speed control."));
  Serial.println(F("    : Speed Adjustment:\n    Turn the step button knob to increase or decrease playback speed."));
  Serial.println(F("  : Playback Loop Control:\n    Long-press the step button for 4 seconds to toggle loop control."));
  Serial.println(F("    : Loop Adjustment:\n    Turn the step button knob to increase or decrease playback loops."));
  Serial.println(F("  : Play Mode:\n    Press the play/record button to start playing."));
  Serial.println(F("    : Cancel Playing:\n    Press the record step button while in play mode to cancel playback."));
  Serial.println(F("  : Record Mode:\n    Long-press the play/record button to start recording."));
  Serial.println(F("    : Record Step:\n    In record mode, press the record step button to save servo positions to memory."));
  Serial.println(F("  : Clear Steps:\n    Long-press the memory button until led flashes red to clear all recorded steps."));
  Serial.println(F("  : Reset Arduino:\n    Long-press the control button through reset warning message to reset microcontroller."));
  Serial.println(F("\n***********************************************\n\n"));

  Serial.print(F("Initializing..."));
  flash_led(4, 3, 5, 0);
  delay(1000);
  flash_led(1, 3, .5, 0);
  Serial.println(F(" READY!\n\n"));
  dis_line2 = "Ready!";
  display_text(1);

}

//declare reset arduino function
void(* resetFunc) (void) = 0;

//main program loop
void loop() {  
  //monitor buttons
  if (!homing && !playing) {
    check_home_btn();
    check_record_btn();
    check_control_btn();
    check_rencoder_btn();
    check_rencoder();
    check_reset_memory_btn();
  }
  if (!homing) {
    check_playrecord_btn();
  }

  //activate control pots
  if (controlling && !homing && !playing && !moving) {
    read_pots();
  }

  adjust_worklite();

  //run backround process checks
  unsigned long current_timer = millis();

  //delay laser switch off
  if (laser_interval) {
    if (current_timer - laser_timer_prev > laser_interval) {
      laser_timer_prev = current_timer;   
      laser_interval = 0;
      digitalWrite (laser_one, LOW);
    }
  }

  //notify of detached servos
  if (!servo_power) {
    flash_led(3, 3, 10, 0);
    flash_led(0, 1, 5, 0);
  }

  //update display to dashboard
  if (!homing) {
    unsigned long current_timer = millis();
    if ((current_timer - display_timer_prev) > (display_interval)) {
      display_timer_prev = current_timer;   
      if (servo_power) {
        dis_line2 = "Ready!";
      } else {
        dis_line2 = "Powered down!";
      }
      display_text(1);
    }
  }
}

/*
void monitor_photores() {
  //USING TEST PINS fOR PHOTORES
  photores_value = analogRead(photores);
  if (debug) {
    Serial.print("Photoresistor : ");
    Serial.println(photores_value);
    delay(500);
  }
  if (photores_value > 900) {
    digitalWrite (photores_led, HIGH);
  } else {
    digitalWrite (photores_led, LOW);
  }
}
*/

/*
 * read_pots
 * read each servo pot and move servo as needed
 */
void read_pots() {
  int pot_allowance = 1;  //adjust +/-5 to prevent servo jitters from flucuating pots
  int pot_val = 0;

  //grip section
  pot_val = map(analogRead(motor_pot[0]), 0, 1023, motor_max[0], motor_min[0]);
  if ((motor_val[0] > (pot_val+pot_allowance)) || (motor_val[0] < (pot_val-pot_allowance))) {
    motor_val[0] = pot_val;
    motor_control = 0;
    move_motors();
  }

  //hand section
  pot_val = map(analogRead(motor_pot[1]), 0, 1023, motor_max[1], motor_min[1]);
  if ((motor_val[1] > (pot_val+pot_allowance)) || (motor_val[1] < (pot_val-pot_allowance))) {
    motor_val[1] = pot_val;
    motor_control = 1;
    move_motors();
  }

  //wrist section
  pot_val = map(analogRead(motor_pot[2]), 0, 1023, motor_max[2], motor_min[2]);
  if ((motor_val[2] > (pot_val+pot_allowance)) || (motor_val[2] < (pot_val-pot_allowance))) {
    motor_val[2] = pot_val;
    motor_control = 2;
    move_motors();
  }

  //elbow section
  pot_val = map(analogRead(motor_pot[3]), 0, 1023, motor_max[3], motor_min[3]);
  if ((motor_val[3] > (pot_val+pot_allowance)) || (motor_val[3] < (pot_val-pot_allowance))) {
    motor_val[3] = pot_val;
    motor_control = 3;
    move_motors();
  }

  //shoulder section
  pot_val = map(analogRead(motor_pot[4]), 0, 1023, motor_max[4], motor_min[4]);
  if ((motor_val[4] > (pot_val+pot_allowance)) || (motor_val[4] < (pot_val-pot_allowance))) {
    motor_val[4] = pot_val;
    motor_control = 4;
    move_motors();
  }

  //chest section
  pot_val = map(analogRead(motor_pot[5]), 0, 1023, motor_max[5], motor_min[5]);
  if ((motor_val[5] > (pot_val+pot_allowance)) || (motor_val[5] < (pot_val-pot_allowance))) {
    motor_val[5] = pot_val;
    motor_control = 5;
    move_motors();
  }
}

void check_rencoder_btn() {
  if (!btn_pressed && (digitalRead(rencoderS) == 0)) {
    display_text(1);
    String temp_dis_line2 = dis_line2;
    delay(1000);
    //if pressed for 1 second, toggle speed control
    if (digitalRead(rencoderS) == 0) {
      Serial.println(F("Speed Control?"));
      dis_line2 = "Speed Control?";
      display_text(1);
      delay(2000);
      //if pressed for 1 more second, toggle playback loop control
      if (digitalRead(rencoderS) == 0) {
        btn_pressed = 1;
        loop_control = 1;
        servo_step_speed_control = 0;
        flash_led(3, 3, 0.5, 0);
        if (!playback_loop_control) {
          flash_led(1, 1, 0.5, 0);
          playback_loop_control = 1;
          Serial.println(F("Playback Loop Control On!"));
          dis_line2 = "Loop Control On!";
        } else {
          flash_led(0, 1, 0.5, 0);
          playback_loop_control = 0;
          Serial.println(F("Playback Loop Control Off!"));
          dis_line2 = "Loop Control Off!";
        }
        display_text(1);
        delay(1000);
        if (playing) flash_led(1, 1, 0.5, 1);
        loop_control = 0;
        dis_line2 = temp_dis_line2;
        display_text(1);
      } else {
        btn_pressed = 1;
        speed_control = 1;
        playback_loop_control = 0;
        flash_led(2, 3, 0.5, 0);
        if (!servo_step_speed_control) {
          flash_led(1, 1, 0.5, 0);
          servo_step_speed_control = 1;
          Serial.println(F("Speed Control On!"));
          dis_line2 = "Speed Control On!";
        } else {
          flash_led(0, 1, 0.5, 0);
          servo_step_speed_control = 0;
          Serial.println(F("Speed Control Off!"));
          dis_line2 = "Speed Control Off!";
        }
        display_text(1);
        delay(1000);
        speed_control = 0;
        if (recording) flash_led(0, 1, 0.5, 1);
        dis_line2 = temp_dis_line2;
        display_text(1);
      }
    } else {
      rencoding = 1;
      playback_loop_control = 0;
      servo_step_speed_control = 0;
      if (motor_control < (step_positions-1)) {
        motor_control++;
      } else {
        motor_control = 0;
      }
      Serial.print(F("Motor #"));
      Serial.print((motor_control+1));
      Serial.println(F(" selected"));
      display_text(1);
      delay(500);
      rencoding = 0;
    }
  }
}

void check_rencoder() {
  int add_val = 1;
  int add_spd = 10;
  int add_loop = 5;
  int rencoder_a = digitalRead(rencoderA);
  int rencoder_b = digitalRead(rencoderB);

  if (rencoder_a != rencoder_b) {     
    String temp_dis_line2 = dis_line2;

    //set speed if controlling, else loops if controlling, else control steps if not playing
    if (servo_step_speed_control) {
      speed_control = 1;
      servo_step_speed_prev = servo_step_speed;
      if (!rencoder_a) { 
        if (servo_step_speed < servo_step_speed_max) {
          servo_step_speed += add_spd;
        }
      } else {
        if (servo_step_speed > servo_step_speed_min) {
          servo_step_speed -= add_spd;
        }
      }
      if (debug) {
        Serial.print(F("Speed prev/cur : "));
        Serial.print(servo_step_speed_prev);
        Serial.print(F(" / "));
        Serial.println(servo_step_speed);
      }
      display_text(1);
      delay(500);
      speed_control = 0;
      dis_line2 = temp_dis_line2;
      display_text(1);
    } else if (playback_loop_control) {
      loop_control = 1;
      step_playback_loops_prev = step_playback_loops;
      if (!rencoder_a) { 
        step_playback_loops += add_loop;
      } else if (step_playback_loops >= add_loop) {
        step_playback_loops -= add_loop;
      }
      if (debug) {
        Serial.print(F("Playback Loops prev/cur : "));
        Serial.print(step_playback_loops_prev);
        Serial.print(F(" / "));
        Serial.println(step_playback_loops);
      }
      display_text(1);
      delay(500);
      loop_control = 0;
      dis_line2 = temp_dis_line2;
      display_text(1);
    } else if (!playing) {
      rencoding = 1;
      if (!rencoder_a) { 
        motor_val[motor_control] += add_val;
      } else {
        motor_val[motor_control] -= add_val;
      }
      move_motors();
      rencoding = 0;
    }
  }
}


/*
 * check_home_btn
 * monitor / set home position button
 */
void check_home_btn() {
  if (!homing && !homebtn && digitalRead(home_btn) == 0) {
    homebtn = 1;
    String temp_dis_line2 = dis_line2;
    Serial.println(F("Homing..."));
    dis_line2 = "Homing...";
    display_text(1);
    delay(1500);
    set_home();
    motor_control = 0;
    if (!servo_power) {
      Serial.println(F("Powered down!\n"));
      dis_line2 = "Powering down...";
    } else {
      Serial.println(F("Powered up & ready!\n"));
      dis_line2 = "Powering up...";
    }
    display_text(1);
    delay(1500);
    dis_line2 = temp_dis_line2;
    display_text(1);
    homebtn = 0;
  }
}



/*
 * check_reset_memory_btn
 * monitor / set reset memory button action
 */
void check_reset_memory_btn() {
  if (!btn_pressed && (digitalRead(reset_memory_btn) == 0)) {
    delay(1000);
    //if pressed for 1 second, reset memory / steps to 0
    if (digitalRead(reset_memory_btn) == 0) {
      //re-initialize memory
      btn_pressed = 1;
      flash_led(4, 10, 0.5, 1);
      Serial.print(F("*** Clearing "));
      Serial.print(EEPROM.read(0));
      Serial.print(F(" steps from memory... "));
      dis_line1 = "CLEARING MEMORY";
      dis_var_int1 = EEPROM.read(0);
      dis_line2 = "RECORDED STEPS";
      dis_line3 = "Processing...";
      display_text(4);

      //physically clear steps from memory
      for (int i = 0 ; i < EEPROM.length() ; i++) {
        EEPROM.write(i, 0);
      }

      //reset recording vars
      step_cnt = 0;
      step_limit = step_limit_fixed;
      position_key = 1;

      delay(500);
      Serial.println(F("Memory Cleared! ***"));
      dis_line2 = "Memory Cleared!";
      display_text(1);

      flash_led(1, 3, 0.5, 0);
    } else if (btn_pressed) {
      btn_pressed = 0;
    }
  } else if (btn_pressed) {
    btn_pressed = 0;
  }
}


/*
 * check_control_btn
 * monitor / set control button alternating on & off
 */
void check_control_btn() {
  int set_controls = 0;
  if (!btn_pressed && (digitalRead(control_btn) == 0)) {
    String temp_dis_line2 = dis_line2;
    set_controls = 1;
    //if pressed for 1 second, warn about to reset and cancel default action!
    delay(1000);
    if (!btn_pressed && (digitalRead(control_btn) == 0)) {
      set_controls = 0;
      flash_led(0, 1, 0.5, 1);
      Serial.println(F("*** RESET IN 2 SECONDS. Release to Cancel! ***"));
      dis_line1 = "RESET IN";
      dis_var_int1 = 3;
      dis_line2 = "SECONDS";
      dis_line3 = "Release to Cancel!";
      display_text(4);
  
      //if pressed for 2 additional seconds, reset arduino!
      delay(2000);
      if (digitalRead(control_btn) == 0) {
        set_controls = 0;
        btn_pressed = 1;
        flash_led(0, 1, 1.0, 0);
        flash_led(3, 1, 1.0, 0);
        flash_led(0, 1, 1.0, 0);
        flash_led(3, 1, 1.0, 0);
        Serial.print(F("*** RETTING SYSTEM ***"));
        dis_line1 = "RESETTING";
        dis_var_int1 = NULL;
        dis_line2 = "SYSTEM";
        dis_line3 = "Please wait...";
        display_text(4);
        delay(3000);
        flash_led(1, 5, 0.5, 0);
        display.clearDisplay();
        delay(1000);
        resetFunc();
      } else {
        flash_led(3, 3, 0.5, 0);
        Serial.println(F("*** RESET CANCELLED! ***"));
        dis_line1 = "RESET";
        dis_var_int1 = NULL;
        dis_line2 = "CANCELLED!";
        dis_line3 = "Please wait...";
        display_text(4);
        delay(2000);        
        if (recording) flash_led(0, 1, 0.5, 1);
        dis_line2 = temp_dis_line2;
        display_text(1);
      }
    }
  } else if (btn_pressed) {
    btn_pressed = 0;
  }

  if (!btn_pressed && set_controls) {
    String temp_dis_line2 = dis_line2;
    btn_pressed = 1;
    flash_led(3, 3, 0.5, 0);
    Serial.print(F("Controlling "));
    if (controlling == 1) {
      controlling = 0;
      Serial.println(F("set off!"));
      dis_line2 = "Controls Off!";
      display_text(1);
      delay(500);
      flash_led(0, 1, 1.0, 0);
    } else {
      controlling = 1;
      Serial.println(F("set on!"));
      dis_line2 = "Controls On!";
      display_text(1);
      delay(500);
      flash_led(1, 1, 1.0, 0);
    }
    if (recording) flash_led(0, 1, 0.5, 1);
    delay(1000);
    dis_line2 = temp_dis_line2;
    display_text(1);
  }
}



/*
 * check_record_btn
 * monitor / set recording button action
 */
void check_record_btn() {
  if (!btn_pressed && (digitalRead(record_btn) == 0)) {
    String temp_dis_line2 = dis_line2;
    btn_pressed = 1;
    if (recording) {
      Serial.println(F("Saving Step..."));
      dis_line2 = "Saving Step...";
      display_text(1);
      delay(1000);
      flash_led(4, 5, 0.5, 0);
      record_step();
      delay(1000);
      flash_led(0, 1, 0.5, 1);
      Serial.println(F("done!"));
      dis_line2 = "Recorded Successfully!";
      display_text(1);
  
      if (recording) flash_led(0, 1, 0.5, 1);
      delay(1500);
      dis_line2 = temp_dis_line2;
      display_text(1);
    } else if (playing) {
      playing = 0;
      step_playback_loops = 0;
      flash_led(1, 3, 0.5, 0);
      flash_led(0, 3, 0.5, 0);
      Serial.println(F("Playing cancelled!"));
      dis_line2 = "Playing Cancelled!";
      dis_spd = 0;
      display_text(1);
      delay(1500);
      dis_line2 = temp_dis_line2;
      dis_line2_int = 0;
      display_text(1);
    }
    btn_pressed = 0;
  } else if (btn_pressed) {
    btn_pressed = 0;
  }

}


/*
 * check_playrecord_btn
 * monitor / set playing & recording mode button
 */
void check_playrecord_btn() {
  if (!btn_pressed && (digitalRead(playrecord_btn) == 0)) {
    String temp_dis_line2 = dis_line2;
    delay(1000);
    //if pressed for 1 second, set record mode, else set play mode
    if (!btn_pressed && (digitalRead(playrecord_btn) == 0)) {
      int steps_rec = EEPROM.read(0);
      btn_pressed = 1;
      playing = 0;
      if (recording) {
        flash_led(0, 3, 0.5, 0);
        recording = 0;
        Serial.println(F("Recording off! "));
        Serial.print(steps_rec);
        Serial.print(F(" Steps recorded."));
        dis_line2 = "Rec'd Steps: ";
        dis_line2_int = steps_rec;
        display_text(1);
        delay(3000);
        dis_line2 = temp_dis_line2;
        display_text(1);
      } else {
        flash_led(0, 3, 0.5, 1);
        recording = 1;
        Serial.print(F("Recording on! "));
        Serial.print(steps_rec);
        Serial.print(F(" Steps recorded."));
        dis_line2 = "Recording...";
        display_text(1);
        delay(1500);
      }
    } else if (!recording && !btn_pressed) {
      if (playing) {
        flash_led(3, 3, 0.5, 0);
        playing = 0;
        Serial.println(F("Playing off!"));
        dis_line2 = "Playing Off!";
        display_text(1);
      } else {
        playing = 1;
        flash_led(1, 3, 0.5, 1);
        Serial.println(F("Playing On!"));
        dis_line2 = "Playing...";
        display_text(1);
        play_steps();
        delay(1500);
        playing = 0;
        flash_led(0, 3, 0.5, 0);
        Serial.println(F("Playing Off!"));
        dis_line2 = "Playing Off!";
        display_text(1);
      }
      delay(2000);
      dis_line2 = temp_dis_line2;
      display_text(1);

    }
  } else if (btn_pressed) {
    btn_pressed = 0;
  }
}



void record_step() {
  if (step_limit > 0) {

    //iterate through motors
    for (int i=0; i<motor_cnt; i++) {
      if (motor_val[i] < motor_min[i]) {
        motor_val[i] = motor_min[i];
      } else if (motor_val[i] > motor_max[i]) {
        motor_val[i] = motor_max[i];
      }
      EEPROM.write(position_key, motor_val[i]);
      position_key++;
    }

    //update step count and limit
    step_cnt = ((position_key-1)/step_positions);
    EEPROM.write(0, step_cnt);
    step_limit = (step_limit_fixed - step_cnt);

    if (debug) {
      Serial.print(F("recorded steps : "));
      Serial.println(step_cnt);
    }
  } else {
    Serial.println(F("Recording limit reached!"));
    dis_line1 = "RECORDING";
    dis_var_int1 = NULL;
    dis_line2 = "LIMIT REACHED";
    dis_line3 = "";
    display_text(4);
  }
}


void play_steps() {
  playing_steps = 1;
  int play_step_cnt = EEPROM.read(0);
  String temp_dis_line2 = dis_line2;

  if (play_step_cnt > 0) {
    if (debug) {
      Serial.print(F("Playing "));
      Serial.print(play_step_cnt);
      Serial.print(F(" steps / "));
      Serial.print((play_step_cnt*step_positions));
      Serial.print(F(" positions "));
      Serial.println(F(" / "));
      Serial.print(step_playback_loops);
      Serial.println(F(" loops"));
    }
    if (!debug_display) {
      dis_line2 = "Playing...";
      display_text(1);
      delay(1000);
    }

    int cnt = 0;
    int step = 1;
    int duplicate_step = 1;
    for (int i=0; i<=(play_step_cnt*step_positions); i++) {
      check_record_btn();
      if (!playing) {
        break;
      }
      //skip position 0 (ie: steps_cnt)
      if (i != 0) {
        if (!cnt) {
          if (debug) {
            Serial.print(F("playing step #"));
            Serial.print(step);
            Serial.println(F(" : "));
          }
          if (!debug_display) {
            dis_line3 = "Playing Step ";
            dis_var_int1 = step;
            dis_var_int2 = play_step_cnt;
            display_text(1);
          }
//          monitor_photores();
          delay(step_playback_delay);
        }
        int pos = EEPROM.read(i);
        if ((pos > 0) && (pos < 180)) {
          if (motor_val[cnt] != pos) {
            duplicate_step = 0;
            motor_val[cnt] = pos;
            if (!sync_move) {
              move_motors();
            }
          }

          //reset cnt / loop for next step
          if (cnt == (motor_cnt-1)) {
            step++;
            cnt = 0;
          } else {
            cnt++;
          }
          if (sync_move) {
            move_motors();
          }
        }
//        monitor_photores();

        if (step > play_step_cnt) {
          break;
        }
      }

      //check if dupe step and add pause/delay purposely
      if (cnt == 0) {
        if (duplicate_step) {
          if (debug) {
            Serial.print(F("Duplicate step...  delaying "));
            Serial.print(step_playback_long_delay);
            Serial.println(F("ms."));
          }
          delay(step_playback_long_delay);
        } else {
          //reset for next step
          duplicate_step = 1;
        }
      }
    }

    step_playback_loops--;
    if (step_playback_loops > 1) {
      play_steps();
    } else {
      step_playback_loops = 0;
      if (playing) {
        Serial.println(F("Playing finished."));
        dis_line2 = "Playing finished.";
        dis_spd = 0;
        display_text(1);
        delay(1000);
      }
    }
  } else {
    Serial.println(F("No steps to play!"));
    dis_line2 = "No steps!";
    display_text(1);
    flash_led(1, 3, 0.5, 0);
    playing = 0;
    Serial.println(F("Playing off!"));
    delay(1000);
  }

  dis_line2 = temp_dis_line2;
  dis_line2_int = 0;
  display_text(1);
  playing_steps = 0;
}


/*
 * set_home
 * return servos to home positions
 */
void set_home() {
  if (!homing) {
    flash_led(2, 3, 0.5, 1);
    homing = 1;

    //(re)attach motors
    if (!servo_power) {
      flash_led(1, 3, 0.5, 0);
      servo_shoulder.attach(motor_pin[4]);
      servo_elbow.attach(motor_pin[3]);
      delay(1000);
      servo_wrist.attach(motor_pin[2]);
      servo_hand.attach(motor_pin[1]);
      delay(1000);
      servo_grip.attach(motor_pin[0]);
      servo_chest.attach(motor_pin[5]);
      flash_led(2, 1, 0.5, 1);
    }

    //iterate through motors
    for (int i=0; i<motor_cnt; i++) {
      motor_val[i] = motor_home[i];
      if (!sync_move) {
        move_motors();
      }
    }
    if (sync_move) {
      move_motors();
    }

    //detach motors
    if (servo_power) {
      flash_led(0, 3, 0.5, 0);
      servo_chest.detach();
      servo_grip.detach();
      servo_hand.detach();
      servo_wrist.detach();
      servo_elbow.detach();
      servo_shoulder.detach();
      servo_power = 0;
    } else {
      servo_power = 1;
    }

    flash_led(2, 3, 0.5, 0);
    homing = 0;
    if (recording) flash_led(0, 1, 0.5, 1);
  }
}


//dis_style: 0=init 1=dashboard 2=recording 3=playing 4=warning
void display_text(int dis_style) {
  display.clearDisplay();

  if (dis_style == 0) {
    //splash view
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setCursor(0,0);
    display.print(dis_line1); 

    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0,24);
    display.print(dis_line2); 
  } else if (dis_style == 1) {
    //dashboard view
    if (rencoding) {
      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(0,0);
      display.print(motor_label[motor_control]); 
  
      display.setTextColor(WHITE);
      display.setTextSize(2);
      display.setCursor(0,8);
      display.print(motor_val_prev[motor_control]); 

      display.setTextColor(WHITE);
      display.setTextSize(2);
      display.setCursor(35,8);
      display.print("/"); 

      display.setTextColor(WHITE);
      display.setTextSize(2);
      display.setCursor(60,8);
      display.print(motor_val[motor_control]); 

      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(0,24);
      display.print(dis_line2); 
    } else if (speed_control) {
      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(0,0);
      display.print("Set Speed!"); 
  
      display.setTextColor(WHITE);
      display.setTextSize(2);
      display.setCursor(0,8);
      display.print(servo_step_speed_prev); 

      display.setTextColor(WHITE);
      display.setTextSize(2);
      display.setCursor(35,8);
      display.print("/"); 

      display.setTextColor(WHITE);
      display.setTextSize(2);
      display.setCursor(60,8);
      display.print(servo_step_speed); 

      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(0,24);
      display.print(dis_line2); 
    } else if (loop_control) {
      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(0,0);
      display.print("Set Loops!"); 
  
      display.setTextColor(WHITE);
      display.setTextSize(2);
      display.setCursor(0,8);
      display.print(step_playback_loops_prev); 

      display.setTextColor(WHITE);
      display.setTextSize(2);
      display.setCursor(35,8);
      display.print("/"); 

      display.setTextColor(WHITE);
      display.setTextSize(2);
      display.setCursor(60,8);
      display.print(step_playback_loops); 

      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(0,24);
      display.print(dis_line2); 
    } else {
      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(0,0);
      display.print(motor_label[motor_control]); 
  
      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(75,0);
      display.print(motor_val_prev[motor_control]); 

      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(98,0);
      display.print("/"); 

      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(110,0);
      display.print(motor_val[motor_control]); 

      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(0,12);
      display.print(dis_line2); 
  
      if (dis_line2_int > 0) {
        display.setTextColor(WHITE);
        display.setTextSize(1);
        display.setCursor(110,12);
        display.print(dis_line2_int); 
      } else if (dis_spd == 1) {
        display.setTextColor(WHITE);
        display.setTextSize(1);
        display.setCursor(90,12);
        display.print(servo_step_speed);     

        display.setTextColor(WHITE);
        display.setTextSize(1);
        display.setCursor(110,12);
        display.print("ms");
      } else {
        display.setTextColor(WHITE);
        display.setTextSize(1);
        display.setCursor(90,12);
        display.print("        ");
      }
      
      if (dis_line3 != "" && dis_var_int1) {
        display.setTextColor(WHITE);
        display.setTextSize(1);
        display.setCursor(0,24);
        display.print(dis_line3); 
      
        display.setTextColor(WHITE);
        display.setTextSize(1);
        display.setCursor(80,24);
        display.print(dis_var_int1); 
    
        display.setTextColor(WHITE);
        display.setTextSize(1);
        display.setCursor(98,24);
        display.print("/"); 
    
        display.setTextColor(WHITE);
        display.setTextSize(1);
        display.setCursor(110,24);
        display.print(dis_var_int2);
      } else {
        display.setTextColor(WHITE);
        display.setTextSize(1);
        display.setCursor(0,24);
        display.print("Memory: "); 
      
        display.setTextColor(WHITE);
        display.setTextSize(1);
        display.setCursor(80,24);
        display.print(step_cnt); 
    
        display.setTextColor(WHITE);
        display.setTextSize(1);
        display.setCursor(98,24);
        display.print("/"); 
    
        display.setTextColor(WHITE);
        display.setTextSize(1);
        display.setCursor(110,24);
        display.print(step_limit); 
      }
    }
  } else if (dis_style == 2) {
//cguDEV: complete this for recording
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0,0);
    display.print(dis_line1); 

  } else if (dis_style == 3) {
//cguDEV: complete this for playing
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0,0);
    display.print(dis_line1);
  } else if (dis_style == 4) {
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0,0);
    display.print(dis_line1);

    if (dis_var_int1 != NULL) {
      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(0,12);
      display.print(dis_var_int1);

      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(20,12);
      display.print(dis_line2);
    } else {
      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(0,12);
      display.print(dis_line2);
    }

    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0,24);
    display.print(dis_line3);
  }

  //refresh screen
  display.display();

  dis_line3 = "";
  dis_var_int1 = 0;
  dis_var_int2 = 0;
  dis_line2_int = 0;
  dis_spd = 1;
}


void move_motors(byte motor=99) {
  moving = 1;

  laser_switch = 1;
  switch_laser();

  if(!rencoding) {
    unsigned long current_timer = millis();
    if(!rencoding && (current_timer - display_timer_prev > display_interval)) {
      display_timer_prev = current_timer;   
      if (!debug_display) {
        display_text(1);
      }
    }
  } else {
    if (!debug_display) {
      display_text(1);
    }
  }
  check_rencoder();

  byte i_val = 0;
  byte i_cnt = motor_cnt;
  //run specific motor if not placeholder
  if (motor < 99) {
    i_val = motor;
    i_cnt = motor+1;
  }

  //define move_dir array : 0=neg 1=pos
  byte move_dir[motor_cnt] = {0, 0, 0, 0, 0, 0};

  //define moved array : 0=no 1=yes
  byte moved[motor_cnt] = {0, 0, 0, 0, 0, 0};

  //define all_moved
  byte all_moved = 0;

  if (debug) {
    Serial.print("ival / icnt: ");
    Serial.print(i_val);
    Serial.print(" / ");
    Serial.println(i_cnt);
  }

  byte deg_min = 0;
  byte deg_max = 180;
  for (int i=i_val; i<i_cnt; i++) {
    //safeguard: fix any min/max violations!
    if (motor_val[i] < motor_min[i]) {
      motor_val[i] = motor_min[i];
    } else if (motor_val[i] >= motor_max[i]) {
      motor_val[i] = motor_max[i];
    }
    if (motor_val_prev[i] < motor_min[i]) {
      motor_val_prev[i] = motor_min[i];
    } else if (motor_val_prev[i] >= motor_max[i]) {
      motor_val_prev[i] = motor_max[i];
    }

    //set direction to positive as needed, and min/max degrees
    if (motor_val_prev[i] < motor_val[i]) {
      move_dir[i] = 1;
      if (i == i_val || i_cnt == 1) {
        deg_min = motor_val_prev[i];
        deg_max = motor_val[i];
      } else {
        deg_min = min(deg_min, motor_val_prev[i]);
        deg_max = max(deg_max, motor_val[i]);
      }
    } else {
      if (i == i_val || i_cnt == 1) {
        deg_min = motor_val[i];
        deg_max = motor_val_prev[i];
      } else {
        deg_min = min(deg_min, motor_val[i]);
        deg_max = max(deg_max, motor_val_prev[i]);
      }
    }
  }
  if (debug) {
    Serial.print("record: ");
    Serial.print(motor_val[0]);
    Serial.print("\t");
    Serial.print(motor_val[1]);
    Serial.print("\t");
    Serial.print(motor_val[2]);
    Serial.print("\t");
    Serial.print(motor_val[3]);
    Serial.print("\t");
    Serial.print(motor_val[4]);
    Serial.print("\t");
    Serial.println(motor_val[5]);

    Serial.print("min / max : ");
    Serial.print(deg_min);
    Serial.print(" / ");
    Serial.println(deg_max);
  }

  //move all servos by iterating all possible degrees
  for (byte deg=deg_min; deg<=deg_max; deg++) {

    //iterate each motor
    for (int i=i_val; i<i_cnt; i++) {

      //move if different from prev, iterating prev til equal
      if (!moved[i] && (motor_val[i] != motor_val_prev[i])) {

        if (!move_dir[i]) {
          //move negative
          if (debug) {
            Serial.print(motor_label[i]);
            Serial.print(F(" neg prev/cur : "));
            Serial.print(motor_val_prev[i]);
            Serial.print(F(" / "));
            Serial.println(motor_val[i]);
          }
          motor_val_prev[i]--;
        } else {
          //move positive
          if (debug) {
            Serial.print(motor_label[i]);
            Serial.print(F(" pos prev/cur : "));
            Serial.print(motor_val_prev[i]);
            Serial.print(F(" / "));
            Serial.println(motor_val[i]);
          }
          motor_val_prev[i]++;
        }

        //step motor!
        step_motor(i, motor_val_prev[i]);
        if (!sync_move) {
          delay(servo_step_speed);
        }
      } else if (!moved[i] && (motor_val[i] == motor_val_prev[i])) {
        moved[i] = 1;
        all_moved++;
      }
    }
    if (all_moved == motor_cnt) {
      if (debug) {
        Serial.print("Break at ");
        Serial.println(deg);
      }
      break;
    }

    if (sync_move) {
      //delay once for all
      delay(servo_step_speed);
    }
  }
  laser_switch = 0;
  switch_laser(5000);

  moving = 0;
}

void step_motor(byte m, byte s) {
  switch (m) {
    case 0:
      servo_grip.write(s);
      break;
    case 1:
      servo_hand.write(s);
      break;
    case 2:
      servo_wrist.write(s);
      break;
    case 3:
      servo_elbow.write(s);
      break;
    case 4:
      servo_shoulder.write(s);
      break;
    case 5:
      servo_chest.write(s);
      break;
  }
}


void show_motor_vars(byte motor=99) {
  byte i_val = 0;
  byte i_cnt = motor_cnt;
  if (motor < 99) {
    i_val = motor;
    i_cnt = motor+1;
  }

  for (int i=i_val; i<i_cnt; i++) {
    Serial.print(F("Motor:\t"));
    Serial.print((i+1));
    Serial.print(F("  "));
    Serial.print(motor_name[i]);
    if (motor_name[i] != "servo_shoulder") {
      Serial.print(F("\t\t"));
    } else {
      Serial.print(F("\t"));
    }
    Serial.print(motor_label[i]);
    if (motor_label[i] != "Shoulder") {
      Serial.print(F("\t\t"));
    } else {
      Serial.print(F("\t"));
    }
    Serial.print(motor_pin[i]);
    Serial.print(F("\t"));
    Serial.print(motor_pot[i]);
    Serial.print(F("\t"));
    Serial.print(motor_val[i]);
    Serial.print(F("\t"));
    Serial.print(motor_val_prev[i]);
    Serial.print(F("\t"));
    Serial.print(motor_min[i]);
    Serial.print(F("\t"));
    Serial.print(motor_max[i]);
    Serial.print(F("\t"));
    Serial.println(motor_home[i]);
  }
}

void adjust_worklite() {
  int worklite_write = map(analogRead(worklite_dimmer), 0, 1023, 0, 255);
  if (worklite_write < 20) {
    worklite_write = 0;
  }
  if (worklite_write > (worklite_write_prev+5) || worklite_write < (worklite_write_prev-5)) {
    analogWrite(worklite, worklite_write);
    worklite_write_prev = worklite_write;
  }
}

void switch_laser(long laser_delay = 0) {
  if (laser_switch) {
    digitalWrite (laser_one, HIGH);
  } else {
    if (laser_delay) {
      laser_interval = laser_delay;
    } else {
      digitalWrite (laser_one, LOW);
    }
  }
}


void flash_led(int color, int vcnt, float spd, int onoff) {
  int r = 0;
  int g = 0;
  int b = 0;
  if (color == 0) {
    r = 25;
  } else if (color == 1) {
    g = 25;
  } else if (color == 2) {
    b = 25;
  } else if (color == 3) {
    //yellow
    r = 25;
    g = 25;
    b = 0;
  } else if (color == 4) {
    //purple
    r = 6; //60
    g = 0;
    b = 19; //190
  }

  spd = spd*100;
  //iterate led(s)
  for (int ledcnt = 0; ledcnt < NUM_LEDS; ledcnt += 1) {
    //flash vcnt times
    for (int cnt = vcnt;cnt > 0;cnt--) {
      leds[ledcnt] = CRGB(0, 0, 0);
      FastLED.show();
      check_home_btn();
      delay(spd);
      leds[ledcnt] = CRGB(r, g, b);
      FastLED.show();
      check_home_btn();
      delay(spd);
    }
    if (onoff == 0) {
      leds[ledcnt] = CRGB(0, 0, 0);
      FastLED.show();
    } 
  }
}
