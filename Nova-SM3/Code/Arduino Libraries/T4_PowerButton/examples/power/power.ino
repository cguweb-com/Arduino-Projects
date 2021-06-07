#include "T4_PowerButton.h"


void myCallback(void) {
  digitalWriteFast( 13, 1);
  Serial.println ("Callback called - Switching off now.");
#if 0
  Serial.println("Simulated crash");
  while (1); //test crash
#endif
  delay(100);
}


void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000 );

  if (arm_power_button_pressed()) {
    Serial.println("Restart after power down");
  } else {
    Serial.println("First power on");
  }

  pinMode(13, OUTPUT);

  set_arm_power_button_callback(&myCallback); //Start callback
}

void loop() {
  digitalWriteFast( 13, !digitalReadFast(13));
  delay(100);
}
