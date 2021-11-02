/*
 * 
 *   NovaSM3 - a Spot-Mini Micro clone
 *   Version: 5.2
 *   Version Date: 2021-10-26
 *   
 *   Author:  Chris Locke - cguweb@gmail.com
 *   Nova's website:  https://novaspotmicro.com
 *   GitHub Project:  https://github.com/cguweb-com/Arduino-Projects/tree/main/Nova-SM3
 *   YouTube Playlist:  https://www.youtube.com/watch?v=00PkTcGWPvo&list=PLcOZNHwM_I2a3YZKf8FtUjJneKGXCfduk
 *     
 *     
 *   NOTE:
 *   while this is a "class", it does not follow the rules of writing classes,
 *   as it uses externally defined position data arrays and servo ramping parameters, 
 *   as well as a call to the external function amperage_check().
 *   
 *   This code is currently still in development, and eventually will have these issues
 *   addressed, making it more globally and open-source friendly for use in other projects
 *   without all of the external dependencies.
 *     
 *   =============================================================
 *     
 *   Copyright (c) 2020-2021 Christopher M. Locke and others
 *   Distributed under the terms of the MIT License. 
 *   SPDX-License-Identifier: MIT
 *   
 *   Permission is hereby granted, free of charge, to any person obtaining
 *   a copy of this software and associated documentation files (the
 *   "Software"), to deal in the Software without restriction, including
 *   without limitation the rights to use, copy, modify, merge, publish,
 *   distribute, sublicense, and/or sell copies of the Software, and to
 *   permit persons to whom the Software is furnished to do so, subject to
 *   the following conditions:
 *   
 *   The above copyright notice and this permission notice shall be
 *   included in all copies or substantial portions of the Software.
 *   
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *   EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 *   MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *   NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 *   LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 *   OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 *   WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *   
*/


/*
   -------------------------------------------------------
   AsyncServo Class
    :non-blocking control of PWM servos for all steps, movements, and sequences
   -------------------------------------------------------
*/
class AsyncServo {
    //properties of servo object
    Adafruit_PWMServoDriver *driver;    //reference to driver instantiation
    int servoID;                        //config ID of servo
    int pwmBoard;                       //name of driver for servo, to support multiple PWM controllers
    int incUnit;                        //pulse increment for each pulse movement of servo
    unsigned long lastUpdate;           //dynamic state timer / delay between servo pulses, derived from servoSpeed[servoID]

    
    //method to instantiate servo object
    public: AsyncServo(Adafruit_PWMServoDriver *Driver, int ServoId) {
      driver = Driver;                    //referenced copy of driver instantiation
      servoID = ServoId;                  //config ID of servo
      pwmBoard = servoSetup[servoID][1];  //pulse increment for each pulse movement of servo
      incUnit = 1;                        //pulse increment for each pulse movement of servo
    }

    void Update() {
      //move servo if active
      if (activeServo[servoID]) {
        //if servo is at destination, set to inactive
        if (servoPos[servoID] == targetPos[servoID]) {
          activeServo[servoID] = 0;
        }

        //servo stepping state machine
        if ((millis() - lastUpdate) > servoSpeed[servoID]) {
          lastUpdate = millis();

          //interpolate servoRamp 
          if (servoRamp[servoID][0] && servoRamp[servoID][1]) {
            if (servoRamp[servoID][2] && servoRamp[servoID][3]) {  //ramp up start
              servoSpeed[servoID] = servoRamp[servoID][2];
              servoRamp[servoID][2] = 0;
              servoRamp[servoID][3]--;
              servoRamp[servoID][1]--;
            } else if (!servoRamp[servoID][2] && servoRamp[servoID][3] && servoRamp[servoID][4]) {  //ramp up step
              servoSpeed[servoID] -= servoRamp[servoID][4];
              servoRamp[servoID][3]--;
              servoRamp[servoID][1]--;
              if (servoRamp[servoID][3] < 1) {  //ramp up end
                servoRamp[servoID][3] = 0;
                servoRamp[servoID][4] = 0;
              }
            } else if (servoRamp[servoID][5] && servoRamp[servoID][6] && servoRamp[servoID][1] <= servoRamp[servoID][6]) {  //ramp down start
              servoSpeed[servoID] += servoRamp[servoID][7];
              servoRamp[servoID][5] = 0;
              servoRamp[servoID][6]--;
              servoRamp[servoID][1]--;
            } else if (!servoRamp[servoID][5] && servoRamp[servoID][6] && servoRamp[servoID][7]) {  //ramp down step
              servoSpeed[servoID] += servoRamp[servoID][7];
              servoRamp[servoID][6]--;
              servoRamp[servoID][1]--;
              if (servoRamp[servoID][6] < 1) {  //ramp down end
                servoRamp[servoID][6] = 0;
                servoRamp[servoID][7] = 0;
              }
            } else if (!servoRamp[servoID][2] && !servoRamp[servoID][3] && !servoRamp[servoID][4] && !servoRamp[servoID][5] && !servoRamp[servoID][6] && !servoRamp[servoID][7]) {  //ramp clear
              for (int j = 1; j < 8; j++) {
                servoRamp[servoID][j] = 0;
              }
            } else if (servoRamp[servoID][6] && servoRamp[servoID][1]) {  //moving inbetween ramping
              servoRamp[servoID][1]--;
            }
          }
  
          if (debug2 && servoID == debug_servo) {
            if (plotter) {
              Serial.print(F("sPos:"));
              Serial.print(servoPos[servoID]);
              Serial.print(F("\t"));
              Serial.print(F("sSpd:"));
              Serial.println(servoSpeed[servoID]);
            } else {
//              Serial.print(F("sPos / sSpd \t"));
//              Serial.print(servoPos[servoID]);
//              Serial.print(F(" / "));
//              Serial.println(servoSpeed[servoID]);
            }
          }


          //move servo if not delaying
          if (!servoDelay[servoID][0]) {  

            //limit target by min/max limit comparisons
            if (servoLimit[servoID][0] > servoLimit[servoID][1]) {
              //left leg, "min" is higher than "max"
              if (targetPos[servoID] > servoLimit[servoID][0]) {
                targetPos[servoID] = servoLimit[servoID][0];
              } else if (targetPos[servoID] < servoLimit[servoID][1]) {
                targetPos[servoID] = servoLimit[servoID][1];
              }
            } else {
              //right leg
              if (targetPos[servoID] < servoLimit[servoID][0]) {
                targetPos[servoID] = servoLimit[servoID][0];
              } else if (targetPos[servoID] > servoLimit[servoID][1]) {
                targetPos[servoID] = servoLimit[servoID][1];
              }
            }

            //move servo by increment in target direction from current position comparison
            if (servoPos[servoID] < targetPos[servoID]) {
              servoPos[servoID] += incUnit;
              driver->setPWM(pwmBoard, 0, servoPos[servoID]);
            } else if (servoPos[servoID] > targetPos[servoID]) {
              servoPos[servoID] -= incUnit;
              driver->setPWM(pwmBoard, 0, servoPos[servoID]);
            }

            //count step
            servoStep[servoID]++;

            //check for end of step(s), where current position equals target position
            if (servoPos[servoID] == targetPos[servoID]) {
              //reset servo steps
              servoStep[servoID] = 0;

              //reset servo speed if done ramping
              if (use_ramp && servoRamp[servoID][0]) {
                servoSpeed[servoID] = servoRamp[servoID][0];
                servoRamp[servoID][0] = 0;
              }

              //Important! 
              //this is the amp check that will catch jammed / over-extended motors!
              if (amp_active) amperage_check(0);
            }
          } else {
            //decrement servo delay, if set, essentially skipping this step
            servoDelay[servoID][0]--;
          }
        }
      }


      //sweep servo if active
      if (activeSweep[servoID] && !activeServo[servoID]) {
        //if servo is done sweeping, set to inactive
        if (servoPos[servoID] == targetPos[servoID] && !servoSweep[servoID][3]) {
          activeSweep[servoID] = 0;
          //switch direction
          (servoSwitch[servoID]) ? servoSwitch[servoID] = 0 : servoSwitch[servoID] = 1;
        }

        //servo sweeping state machine
        if ((millis() - lastUpdate) > servoSpeed[servoID]) {
          lastUpdate = millis();
          
          if (debug2 && servoID == debug_servo) {
            Serial.print(servoID); Serial.print(F("\ttarget: ")); Serial.print(targetPos[servoID]);
            Serial.print(F("\tstart: ")); Serial.print(servoSweep[servoID][0]);
            Serial.print(F("\ttarget: ")); Serial.print(servoSweep[servoID][1]);
            Serial.print(F("\tdir: ")); Serial.print(servoSweep[servoID][2]);
            Serial.print(F("\tloops: ")); Serial.print(servoSweep[servoID][3]);
          }

          //if no delay (or delay expired)
          if (!servoSweep[servoID][4]) {

            //interpolate servoRamp 
            if (servoRamp[servoID][0] && servoRamp[servoID][1]) {
              if (servoRamp[servoID][2] && servoRamp[servoID][3]) {  //ramp up start
                servoSpeed[servoID] = servoRamp[servoID][2];
                servoRamp[servoID][2] = 0;
                servoRamp[servoID][3]--;
                servoRamp[servoID][1]--;
              } else if (!servoRamp[servoID][2] && servoRamp[servoID][3] && servoRamp[servoID][4]) {  //ramp up step
                servoSpeed[servoID] -= servoRamp[servoID][4];
                servoRamp[servoID][3]--;
                servoRamp[servoID][1]--;
                if (servoRamp[servoID][3] < 1) {  //ramp up end
                  servoRamp[servoID][3] = 0;
                  servoRamp[servoID][4] = 0;
                }
              } else if (servoRamp[servoID][5] && servoRamp[servoID][6] && servoRamp[servoID][1] <= servoRamp[servoID][6]) {  //ramp down start
                servoSpeed[servoID] += servoRamp[servoID][7];
                servoRamp[servoID][5] = 0;
                servoRamp[servoID][6]--;
                servoRamp[servoID][1]--;
              } else if (!servoRamp[servoID][5] && servoRamp[servoID][6] && servoRamp[servoID][7]) {  //ramp down step
                servoSpeed[servoID] += servoRamp[servoID][7];
                servoRamp[servoID][6]--;
                servoRamp[servoID][1]--;
                if (servoRamp[servoID][6] < 1) {  //ramp down end
                  servoRamp[servoID][6] = 0;
                  servoRamp[servoID][7] = 0;
                }
              } else if (!servoRamp[servoID][2] && !servoRamp[servoID][3] && !servoRamp[servoID][4] && !servoRamp[servoID][5] && !servoRamp[servoID][6] && !servoRamp[servoID][7]) {  //ramp clear
                for (int j = 1; j < 8; j++) {
                  servoRamp[servoID][j] = 0;
                }
              } else if (servoRamp[servoID][6] && servoRamp[servoID][1]) {  //moving inbetween ramping
                servoRamp[servoID][1]--;
              }
            }
  
            if (debug2 && servoID == debug_servo) {
              if (plotter) {
                Serial.print(F("sPos:"));
                Serial.print(servoPos[servoID]);
                Serial.print(F("\tsSpd:"));
                Serial.println(servoSpeed[servoID]);
              } else {
//                Serial.print(F("sPos / sSpd \t"));
//                Serial.print(servoPos[servoID]);
//                Serial.print(F(" / "));
//                Serial.println(servoSpeed[servoID]);
              }
            }

            //sweep servo by increment in target direction from current position comparison
            if (servoPos[servoID] < targetPos[servoID]) {
              servoPos[servoID] += incUnit;
              if (debug2 && servoID == debug_servo) {
                Serial.print(F("\tadd inc to pos: ")); Serial.print(servoPos[servoID]);
              }
              driver->setPWM(pwmBoard, 0, servoPos[servoID]);
            } else if (servoPos[servoID] > targetPos[servoID]) {
              servoPos[servoID] -= incUnit;
              if (debug2 && servoID == debug_servo) {
                Serial.print(F("\tsub inc to pos: ")); Serial.print(servoPos[servoID]);
              }
              driver->setPWM(pwmBoard, 0, servoPos[servoID]);
            }

            //change direction of sweep type if target reached
            if (servoPos[servoID] == targetPos[servoID] && !servoSweep[servoID][2]) {
              servoSweep[servoID][2] = 1;
              targetPos[servoID] = servoSweep[servoID][0];

              //reset ramp for next sweep
              if (use_ramp && servoRamp[servoID][0]) {
                servoSpeed[servoID] = servoRamp[servoID][0];
                set_ramp(servoID, servoSpeed[servoID], 0, 0, 0, 0);
              }
              if (debug2 && servoID == debug_servo) {
                Serial.print(F("\treversed"));
              }
            } else if (servoPos[servoID] == servoSweep[servoID][0] && servoSweep[servoID][2]) {
              servoSweep[servoID][2] = 0;
              servoSweep[servoID][3]--;
              if (servoSweep[servoID][3]) {
                targetPos[servoID] = servoSweep[servoID][0];
                if (debug2 && servoID == debug_servo) {
                  Serial.print(F("\tforward inc: ")); Serial.print(incUnit);
                }
              }
            }

            //count step
            servoStep[servoID]++;

            //check for end of sweep(s), where current position equals target position
            if (servoPos[servoID] == targetPos[servoID]) {
              //reset servo steps
              servoStep[servoID] = 0;

              //reset speed if done ramping
              if (use_ramp && servoRamp[servoID][0]) {
                servoSpeed[servoID] = servoRamp[servoID][0];
                servoRamp[servoID][0] = 0;
              }

              //Important! 
              //this is the amp check that will catch locked / over-extended motors
              if (amp_active) amperage_check(0);
            }
          } else {
            //decrement sweep delay, if set, essentially skipping this step
            servoSweep[servoID][4]--;
          }

          if (debug2 && servoID == debug_servo) {
            Serial.println(F(""));
          }
        }
      }
    }

};
