#include <T4_PowerButton.h>

IntervalTimer powerButtonWatchDogTimer;
IntervalTimer somethingTimer;

elapsedMicros elapsedFirst;

void poweroff_cb()
{
    Serial.println("Powerbutton callback");
    delay(200);
}

bool button_pressed = false;

callback_ex_action poweroff_cb_ex()
{
    if(!button_pressed)
    {
        elapsedFirst = 0;
        button_pressed = true;
        Serial.println("powerdown pressed");
    }
    powerButtonWatchDogTimer.end();
    powerButtonWatchDogTimer.begin(powerButtonWatchDog, 100000);
    if(elapsedFirst > 10000000)
      return callback_ex_action_poweroff;
    return callback_ex_action_poweroff_keeparmed;
}

/*
 * Without the watchdog calling rearm_power_button_callback(), normal 
 * processing of loop() will never resume. IntervalTimers however will
 */
void powerButtonWatchDog()
{
    powerButtonWatchDogTimer.end(); // One Shot
    rearm_power_button_callback();
    button_pressed = false;
    noInterrupts();
    Serial.println("powerdown released");
    interrupts();
}

void something()
{
    Serial.printf("something: %d\n", arm_power_button_pressed());
}

void setup() {

    while(!Serial);
    Serial.begin(9600);
    delay(1000);
    Serial.printf("Hello\n");
    progInfo();
    delay(1000);
    somethingTimer.begin(something, 1000000);  // Check buttons and leds every 0.001 sec
    set_arm_power_button_press_time_emergency(arm_power_button_press_time_emergency_off);
    // Both callbacks may be installed
    //set_arm_power_button_callback(poweroff_cb);
    set_arm_power_button_callback_ex(poweroff_cb_ex);
}

void loop() {
    noInterrupts(); // Otherwise Serial gets messed up
    Serial.printf("Main\n");
    interrupts();
    delay(2000);
}
