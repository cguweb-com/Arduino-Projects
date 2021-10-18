#include <PS2X_lib.h>


//PS2 controller
#define PS2_DAT 7
#define PS2_CMD 8
#define PS2_SEL 9
#define PS2_CLK 10
PS2X ps2x;

begin() {
  //init ps2 controller
  if (ps2_active) {
    if (debug) Serial.print(F("PS2 Controller intializing..."));
    delay(500);
    if (ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false) == 0) {
      if (debug) Serial.println(F("\t\t\tOK"));
    } else {
      if (debug) Serial.println(F(" Error!"));
      ps2_active = 0;
    }
  }
}


/*
   -------------------------------------------------------
   PS2 Check
    :provide general description and explanation here - too much to comment by line
   -------------------------------------------------------
*/
void ps2_check() {
  if (pwm_oe) {
    ps2x.read_gamepad(false, false);

    if (!move_demo && !move_funplay) {
      if (ps2x.Button(PSB_START)) {
        if (debug1)
          Serial.println(F("Start Pressed"));
        if (ps2_select == 1) {
          rgb_request((char*)"MVNV");
/*
          if (mpu_active) {
            mpu_active = 0;
            rgb_request((char*)"vGn");
          } else {
            mpu_active = 1;
            rgb_request((char*)"xFn");
          }
*/
        } else if (ps2_select == 2) {
          if (debug1)
            Serial.println(F("start / stop march"));
          if (!move_march) {
            set_stop();
            spd = 3;
            set_speed();
            y_dir = 0;
            x_dir = 0;
            z_dir = 0;
            move_steps = 50;
            if (mpu_is_active) mpu_active = 0;
            move_march = 1;
            if (oled_active) {
              oled_request((char*)"d");
            }
          } else {
            move_march = 0;
            if (mpu_is_active) mpu_active = 1;
            set_stop();
            y_dir = 0;
            x_dir = 0;
            z_dir = 0;
            if (oled_active) {
              oled_request((char*)"d");
            }
          }
        } else if (ps2_select == 3) {
          rgb_request((char*)"MVNV");

/*
          if (uss_active) {
            uss_active = 0;
            rgb_request((char*)"vGn");
          } else {
            uss_active = 1;
            rgb_request((char*)"xFn");
          }

          if (pir_halt) {
            pir_halt = 0;
            rgb_request((char*)"vGn");
          } else {
            pir_halt = 1;
            rgb_request((char*)"xFn");
          }
*/
        } else if (ps2_select == 4) {
//          run_demo();
          rgb_request((char*)"MVNV");

          if (!move_follow) {
            spd = 3;
            set_speed();
            move_follow = 1;
            if (buzz_active) {
              for (int b = 3; b > 0; b--) {
                tone(BUZZ, 1000);          
                delay(100);  
                noTone(BUZZ);         
                delay(100);  
              }
              noTone(BUZZ);         
            }
          } else {
            move_follow = 0;
            if (buzz_active) {
              for (int b = 2; b > 0; b--) {
                tone(BUZZ, 2000);          
                delay(100);  
                noTone(BUZZ);         
                delay(100);  
              }
              noTone(BUZZ);         
            }
          }

        }
      }
  
      if (ps2x.ButtonReleased(PSB_SELECT)) {
        (ps2_select < 4) ? ps2_select++ : ps2_select = 1;
        if (rgb_active) {
          rgb_request((char*)"MQNQ");

          if (ps2_select == 1) {
            rgb_request((char*)"tEn");
          } else if (ps2_select == 2) {
            rgb_request((char*)"uEn");
          } else if (ps2_select == 3) {
            rgb_request((char*)"vEn");
          } else if (ps2_select == 4) {
            rgb_request((char*)"wEn");
          }
        }
        if (buzz_active) {
          for (int b = ps2_select; b > 0; b--) {
            tone(BUZZ, 2000);          
            delay(70);  
            noTone(BUZZ);         
            delay(70);  
          }
          noTone(BUZZ);         
        }
        if (oled_active) {
          if (ps2_select == 1) {
            oled_request((char*)"g");
          } else if (ps2_select == 2) {
            oled_request((char*)"h");
          } else if (ps2_select == 3) {
            oled_request((char*)"i");
          } else if (ps2_select == 4) {
            oled_request((char*)"j");
          }
        }
        if (debug1) {
          Serial.print(F("\tSelected ")); Serial.println(ps2_select);
        }
      }

      //gait joysticks
      if (ps2_select == 2) {
        y_dir = mapfloat(ps2x.Analog(PSS_LY), 0, 255, y_dir_steps[1], y_dir_steps[0]);
        x_dir = mapfloat(ps2x.Analog(PSS_LX), 0, 255, x_dir_steps[1], x_dir_steps[0]);
        z_dir = mapfloat(ps2x.Analog(PSS_RY), 0, 255, z_dir_steps[1], z_dir_steps[0]);

        //set move_steps to min 40 to maintain march-in-place
        //DEVNOTE: make this switchable via PS2
        move_steps = map(ps2x.Analog(PSS_RX), 0, 255, 40, move_steps_max + (move_steps_max * 0.2));
      }

      //kinematics joysticks
      if (ps2_select == 3) {
        if (ps2x.Button(PSB_R3)) {
        } else {
          move_steps_y = map(ps2x.Analog(PSS_LY), 0, 255, (move_steps_max * 1.4), (move_steps_min * 1.4));
          move_pitch_y = 1;
          if (debug1 && move_steps_y) {
            Serial.print(F("move pitch_y "));Serial.println(move_steps_y);
          }

          move_steps_x = map(ps2x.Analog(PSS_LX), 0, 255, (move_steps_max * 1.4), (move_steps_min * 1.4));
          move_roll_x = 1;
          if (debug1 && move_steps_x) {
            Serial.print(F("move roll_x "));Serial.println(move_steps_x);
          }
        }

        if (ps2x.Button(PSB_L3)) {
          //move yaw while button pressed/held
          move_steps_yaw = map(ps2x.Analog(PSS_RX), 0, 255, (move_steps_max * 1.4), (move_steps_min * 1.4));
          if (move_steps_yaw > 2 || move_steps_yaw < -2) {
            move_yaw = 1;
            if (debug1)
              Serial.println(F("move yaw"));
          } else {
            move_yaw = 0;
          }

          //move in y while button pressed/held
          move_steps_ky = map(ps2x.Analog(PSS_RY), 0, 255, (move_steps_min * 1.4), (move_steps_max * 1.4));
          if (move_steps_ky > 2 || move_steps_ky < -2) {
            move_kin_y = 1;
            if (debug1)
              Serial.println(F("move kin_y"));
          } else {
            move_kin_y = 0;
          }
        } else {
          move_steps_yaw_x = map(ps2x.Analog(PSS_RX), 0, 255, (move_steps_max * .5), (move_steps_min * .5));
          if (move_steps_yaw_x > 2 || move_steps_yaw_x < -2) {
            move_yaw_x = 1;
            if (debug1)
              Serial.println(F("move yaw_x"));
          } else {
            move_yaw_x = 0;
          }

          move_steps_yaw_y = map(ps2x.Analog(PSS_RY), 0, 255, (move_steps_max * .8), (move_steps_min * .8));
          if (move_steps_yaw_y > 2 || move_steps_yaw_y < -2) {
            move_yaw_y = 1;
            if (debug1)
              Serial.println(F("move yaw_y"));
          } else {
            move_yaw_y = 0;
          }
        }  
      }


      if (ps2x.Button(PSB_PAD_UP)) {
        if (ps2_select == 1) {
          if (debug1)
            Serial.println(F("forward"));
          if (!move_forward) {
            set_stop();
            if (rgb_active) {
              rgb_request((char*)"Ff");
            }
            
            if (mpu_is_active) mpu_active = 0;
            move_march = 1;
            spd = 12;
            set_speed();
            move_forward = 1;
          }
        } else if (ps2_select == 2) {
        } else if (ps2_select == 3) {
          if (!move_trot) {
            set_stop();
            move_trot = 1;
            if (debug1)
              Serial.println(F("move trot"));
          }
          x_dir = map(ps2x.Analog(PSS_RX), 0, 255, move_steps_min / 4, move_steps_max / 4);
          move_steps = map(ps2x.Analog(PSS_RY), 0, 255, move_steps_max / 2, move_steps_min / 2);
          if (debug2) {
            Serial.print(F("x dir: ")); Serial.print(x_dir);
            Serial.print(F("\tmove steps: ")); Serial.println(move_steps);
          }
        }
      } else if (ps2x.ButtonReleased(PSB_PAD_UP)) {
        if (ps2_select == 1 || ps2_select == 2 || ps2_select == 3) {
          if (move_forward) {
            move_forward = 0;
            if (debug1)
              Serial.println(F("stop forward"));
          }
          if (move_march) {
            if (mpu_is_active) mpu_active = 1;
            move_march = 0;
          }
          if (move_trot) {
            move_trot = 0;
          }
        }
      }
  
      if (ps2x.Button(PSB_PAD_RIGHT)) {
        if (ps2_select == 1) {
          if (!move_right) {
            move_right = 1;
            if (debug1)
              Serial.println(F("move right"));
          }
          x_dir = map(ps2x.Analog(PSS_RX), 0, 255, move_x_steps[0], move_x_steps[1]);
          if (debug2) {
            Serial.print(F("move steps: ")); Serial.println(move_steps);
          }
        }
      } else if (ps2x.ButtonReleased(PSB_PAD_RIGHT)) {
        if (ps2_select == 1) {
          if (move_right) {
            move_right = 0;
            if (debug1)
              Serial.println(F("stop move right"));
          }
        } else if (ps2_select == 2) {
        } else if (ps2_select == 3) {
        }
      }
    
      if (ps2x.Button(PSB_PAD_LEFT)) {
        if (ps2_select == 1) {
          if (!move_left) {
            move_left = 1;
            if (debug1)
              Serial.println(F("move left"));
          }
          x_dir = map(ps2x.Analog(PSS_RX), 0, 255, move_x_steps[1], move_x_steps[0]);
          if (debug2) {
            Serial.print(F("move steps: ")); Serial.println(move_steps);
          }
        }
      } else if (ps2x.ButtonReleased(PSB_PAD_LEFT)) {
        if (ps2_select == 1) {
          if (move_left) {
            move_left = 0;
            if (debug1)
              Serial.println(F("stop move left"));
          }
        } else if (ps2_select == 2) {
        } else if (ps2_select == 3) {
        }
      }  
  
      if (ps2x.ButtonPressed(PSB_PAD_DOWN)) {
        if (ps2_select == 1 || ps2_select == 2 || ps2_select == 3 || ps2_select == 4) {
          if (!ps2x.Button(PSB_L1) && !ps2x.Button(PSB_L2) && !ps2x.Button(PSB_R1) && !ps2x.Button(PSB_R2)) {
            set_stop();
            if (debug1)
              Serial.println(F("stay"));
            if (rgb_active) {
              rgb_request((char*)"Hg");
            }
            set_stay();
            if (oled_active) {
              oled_request((char*)"b");
            }
          }
        }
      } else if (ps2x.ButtonReleased(PSB_PAD_DOWN)) {
        if (ps2_select == 1 || ps2_select == 2 || ps2_select == 3 || ps2_select == 4) {
          if (!ps2x.Button(PSB_L1) && !ps2x.Button(PSB_L2) && !ps2x.Button(PSB_R1) && !ps2x.Button(PSB_R2)) {
            if (!activeServo[RFF] && !activeServo[LFF] && !activeServo[RRF] && !activeServo[LRF]) {
              spd = 1;
              set_speed();
              move_loops = 2;
              move_switch = 2;
              for (int i = 0; i < TOTAL_SERVOS; i++) {
                servoPos[i] = servoHome[i];
              }
              move_wake = 1;
            }
          }
        }
      }
  
  
      //TRIGGER BUTTONS
      if (ps2x.Button(PSB_L1)) {
        if (ps2_select == 1) {
          if (!move_roll) {
            set_stop();
            move_roll = 1;
            if (debug1)
              Serial.println(F("move roll"));
          }
          move_steps = map(ps2x.Analog(PSS_RY), 0, 255, move_steps_max, move_steps_min);
          if (debug2) {
            Serial.print(F("move steps: ")); Serial.println(move_steps);
          }
        } else if (ps2_select == 2) {
          set_stop();
          if (debug1)
            Serial.println(F("sit"));
          if (rgb_active) {
            rgb_request((char*)"Hi");
          }
          set_sit();
        }
      } else if (ps2x.ButtonReleased(PSB_L1)) {
        if (ps2_select == 1) {
          if (move_roll) {
            move_roll = 0;
            if (debug1)
              Serial.println(F("stop roll"));
          }
        }
      }

      if (ps2x.Button(PSB_L2)) {
        if (ps2_select == 1) {
          if (!move_pitch) {
            set_stop();
            move_pitch = 1;
            if (debug1)
              Serial.println(F("move pitch"));
          }
          x_dir = map(ps2x.Analog(PSS_RX), 0, 255, move_steps_min / 2, move_steps_max / 2);
          move_steps = map(ps2x.Analog(PSS_RY), 0, 255, move_steps_max / 3, move_steps_min / 3);
          if (debug2) {
            Serial.print(F("move steps: ")); Serial.println(move_steps);
          }
        } else if (ps2_select == 2) {
          set_stop();
          if (debug1)
            Serial.println(F("kneel"));
          set_kneel();
        }
      } else if (ps2x.ButtonReleased(PSB_L2)) {
        if (ps2_select == 1) {
          if (move_pitch) {
            move_pitch = 0;
            if (debug1)
              Serial.println(F("stop pitch"));
          }
        }
      }

      if (ps2x.Button(PSB_R1)) {
        if (ps2_select == 1) {
          if (!move_roll_body) {
            set_stop();
            move_roll_body = 1;
            if (debug1)
              Serial.println(F("move roll_body"));
          }
          move_steps = map(ps2x.Analog(PSS_RY), 0, 255, move_steps_max / 2, move_steps_min / 2);
          if (debug2) {
            Serial.print(F("move steps: ")); Serial.println(move_steps);
          }
        } else if (ps2_select == 2) {
          set_stop();
          if (debug1)
            Serial.println(F("crouch"));
          if (rgb_active) {
            rgb_request((char*)"Hh");
          }
          set_crouch();
        }
      } else if (ps2x.ButtonReleased(PSB_R2)) {
        if (ps2_select == 1) {
          if (move_roll_body) {
            move_roll_body = 0;
            if (debug1)
              Serial.println(F("stop roll_body"));
          }
        }
      }

      if (ps2x.Button(PSB_R2)) {
        if (ps2_select == 1) {
          if (!move_pitch_body) {
            set_stop();
            move_pitch_body = 1;
            if (debug1)
              Serial.println(F("move pitch_body"));
          }
          move_steps = map(ps2x.Analog(PSS_RY), 0, 255, move_steps_max, move_steps_min);
          if (debug2) {
            Serial.print(F("move steps: ")); Serial.println(move_steps);
          }
        } else if (ps2_select == 2) {
          set_stop();
          if (debug1)
            Serial.println(F("lay"));
          if (rgb_active) {
            rgb_request((char*)"Kd");
          }
          set_lay();
        }
      } else if (ps2x.ButtonReleased(PSB_R2)) {
        if (ps2_select == 1) {
          if (move_pitch_body) {
            move_pitch_body = 0;
            if (debug1)
              Serial.println(F("stop pitch_body"));
          }
        }
      }
  

      //SHAPE BUTTONS
      if (ps2x.ButtonPressed(PSB_TRIANGLE)) {
        if (ps2_select == 1) {
          set_stop();
          if (!move_y_axis) {
            move_y_axis = 1;
            if (debug1)
              Serial.println(F("move y_axis"));
          }
        } else if (ps2_select == 2) {
          set_stop();
          if (!move_wman) {
            move_wman = 1;
            if (debug1)
              Serial.println(F("move wman"));
          }
        }
      } else if (ps2x.ButtonReleased(PSB_TRIANGLE)) {
        if (ps2_select == 1) {
          if (move_y_axis) {
            move_y_axis = 0;
            if (debug1)
              Serial.println(F("stop y_axis"));
          }
        } else if (ps2_select == 2) {
          if (move_wman) {
            move_wman = 0;
            if (debug1)
              Serial.println(F("stop wman"));
          }
        }
      }

      //poll steps stick
      if (ps2x.Button(PSB_TRIANGLE)) {
        if (ps2_select == 1) {
          move_steps = map(ps2x.Analog(PSS_RY), 0, 255, move_y_steps[0], move_y_steps[1]);
          if (debug2) {
            Serial.print(F("move steps: ")); Serial.print(move_steps);
          }
          if (move_steps < 0) {
            move_steps = map(move_steps, -150, 150, move_x_steps[0], move_x_steps[1]);
          }
          if (debug2) {
            Serial.print(F(" / ")); Serial.println(move_steps);
          }
        }
      }
  
      if (ps2x.ButtonPressed(PSB_CROSS)) {
        if (ps2_select == 1) {
          set_stop();
          if (!move_x_axis) {
            move_x_axis = 1;
            if (debug1)
              Serial.println(F("move x_axis"));
          }
        }
      } else if (ps2x.ButtonReleased(PSB_CROSS)) {
        if (ps2_select == 1) {
          if (move_x_axis) {
            move_x_axis = 0;
            if (debug1)
              Serial.println(F("stop x_axis"));
          }
        }
      }

      //poll steps stick
      if (ps2x.Button(PSB_CROSS)) {
        if (ps2_select == 1) {
          move_steps = map(ps2x.Analog(PSS_RY), 0, 255, move_x_steps[0], move_x_steps[1]);
          if (debug2) {
            Serial.print(F("move steps: ")); Serial.println(move_steps);
          }
        }
      }
  
      if (ps2x.ButtonPressed(PSB_CIRCLE)) {
        if (debug1) {
          Serial.println(F("Circle"));
          Serial.print(F("speed up +1 : "));
        }
        spd -= 1;
        if (spd < max_spd) spd = max_spd;
        set_speed();
        if (debug1) {
          Serial.println(spd);
        }
      }

      if (ps2x.ButtonPressed(PSB_SQUARE)) {
        if (debug1) {
          Serial.println(F("Square"));
          Serial.print(F("speed down -1 : "));
        }
        spd += 1;
        if (spd > min_spd) spd = min_spd;
        set_speed();
        if (debug1)
          Serial.println(spd);
      }
  
  
  
      //LEG / CALIBRATION CONTROLS
      if (ps2_select == 4) {
        //RF
        if (ps2x.Button(PSB_R1)) {
          if (ps2x.Button(PSB_PAD_UP)) {
            if (!activeServo[RFC]) {
              int ms = servoPos[RFC];
              if (ps2x.Button(PSB_TRIANGLE)) {
                ms += 1;
              } else if (ps2x.Button(PSB_CROSS)) {
                ms -= 1;
              }
              update_sequencer(RF, RFC, spd, ms, 0, 0);
              if (debug1) {
                Serial.print(F("RFC: ")); Serial.println(limit_target(RFC, ms, 0));
              }
            }
          } else if (ps2x.Button(PSB_PAD_RIGHT)) {
            if (!activeServo[RFF]) {
              int ms = servoPos[RFF];
              if (ps2x.Button(PSB_TRIANGLE)) {
                ms += 1;
              } else if (ps2x.Button(PSB_CROSS)) {
                ms -= 1;
              }
              update_sequencer(RF, RFF, spd, ms, 0, 0);
              if (debug1) {
                Serial.print(F("RFF: ")); Serial.println(limit_target(RFF, ms, 0));
              }
            }
          } else if (ps2x.Button(PSB_PAD_DOWN)) {
            if (!activeServo[RFT]) {
              int ms = servoPos[RFT];
              if (ps2x.Button(PSB_TRIANGLE)) {
                ms += 1;
              } else if (ps2x.Button(PSB_CROSS)) {
                ms -= 1;
              }
              update_sequencer(RF, RFT, spd, ms, 0, 0);
              if (debug1) {
                Serial.print(F("RFT: ")); Serial.println(limit_target(RFT, ms, 0));
              }
            }
          } else if (ps2x.Button(PSB_PAD_LEFT)) {
            if (!activeServo[RFT]) {
              move_steps = map(ps2x.Analog(PSS_RY), 0, 255, move_c_steps[0] / 1.5, move_c_steps[1] / 1.5);
              update_sequencer(LF, LFT, spd, (servoHome[LFT] + move_steps), 0, 0);
              update_sequencer(LR, LRT, spd, (servoHome[LRT] + move_steps), 0, 0);
              update_sequencer(RF, RFT, spd, (servoHome[RFT] + move_steps), 0, 0);
              update_sequencer(RR, RRT, spd, (servoHome[RRT] + move_steps), 0, 0);
            }
          } else if (ps2x.ButtonReleased(PSB_PAD_UP) || ps2x.ButtonReleased(PSB_PAD_RIGHT) || ps2x.ButtonReleased(PSB_PAD_LEFT) || ps2x.ButtonReleased(PSB_PAD_DOWN)) {
            set_stop_active();
          }
        }
  
        //LF
        if (ps2x.Button(PSB_L1)) {
          if (ps2x.Button(PSB_PAD_UP)) {
            if (!activeServo[LFC]) {
              int ms = servoPos[LFC];
              if (ps2x.Button(PSB_TRIANGLE)) {
                ms += 1;
              } else if (ps2x.Button(PSB_CROSS)) {
                ms -= 1;
              }
              if (debug1) {
                Serial.print(F("LFC: ")); Serial.println(limit_target(LFC, ms, 0));
              }
              update_sequencer(LF, LFC, spd, ms, 0, 0);
            }
          } else if (ps2x.Button(PSB_PAD_RIGHT)) {
            if (!activeServo[LFF]) {
              int ms = servoPos[LFF];
              if (ps2x.Button(PSB_TRIANGLE)) {
                ms += 1;
              } else if (ps2x.Button(PSB_CROSS)) {
                ms -= 1;
              }
              update_sequencer(LF, LFF, spd, ms, 0, 0);
              if (debug1) {
                Serial.print(F("LFF: ")); Serial.println(limit_target(LFF, ms, 0));
              }
            }
          } else if (ps2x.Button(PSB_PAD_DOWN)) {
            if (!activeServo[LFT]) {
              int ms = servoPos[LFT];
              if (ps2x.Button(PSB_TRIANGLE)) {
                ms += 1;
              } else if (ps2x.Button(PSB_CROSS)) {
                ms -= 1;
              }
              update_sequencer(LF, LFT, spd, ms, 0, 0);
              if (debug1) {
                Serial.print(F("LFT: ")); Serial.println(limit_target(LFT, ms, 0));
              }
            }
          } else if (ps2x.Button(PSB_PAD_LEFT)) {
            move_steps++;
            if (!activeServo[RRC]) {
              move_steps = map(ps2x.Analog(PSS_RY), 0, 255, move_c_steps[0] / 1.5, move_c_steps[1] / 1.5);
              update_sequencer(LF, LFC, spd, (servoHome[LFC] + move_steps), 0, 0);
              update_sequencer(LR, LRC, spd, (servoHome[LRC] + move_steps), 0, 0);
              update_sequencer(RF, RFC, spd, (servoHome[RFC] + move_steps), 0, 0);
              update_sequencer(RR, RRC, spd, (servoHome[RRC] + move_steps), 0, 0);
            }
          }
        }
  
        //RR
        if (ps2x.Button(PSB_R2)) {
          if (ps2x.Button(PSB_PAD_UP)) {
            if (!activeServo[RRC]) {
              int ms = servoPos[RRC];
              if (ps2x.Button(PSB_TRIANGLE)) {
                ms += 1;
              } else if (ps2x.Button(PSB_CROSS)) {
                ms -= 1;
              }
              update_sequencer(RR, RRC, spd, ms, 0, 0);
              if (debug1) {
                Serial.print(F("RRC: ")); Serial.println(limit_target(RRC, ms, 0));
              }
            }
          } else if (ps2x.Button(PSB_PAD_RIGHT)) {
            if (!activeServo[RRF]) {
              int ms = servoPos[RRF];
              if (ps2x.Button(PSB_TRIANGLE)) {
                ms += 1;
              } else if (ps2x.Button(PSB_CROSS)) {
                ms -= 1;
              }
              update_sequencer(RR, RRF, spd, ms, 0, 0);
              if (debug1) {
                Serial.print(F("RRF: ")); Serial.println(limit_target(RRF, ms, 0));
              }
            }
          } else if (ps2x.Button(PSB_PAD_DOWN)) {
            if (!activeServo[RRT]) {
              int ms = servoPos[RRT];
              if (ps2x.Button(PSB_TRIANGLE)) {
                ms += 1;
              } else if (ps2x.Button(PSB_CROSS)) {
                ms -= 1;
              }
              update_sequencer(RR, RRT, spd, ms, 0, 0);
              if (debug1) {
                Serial.print(F("RRT: ")); Serial.println(limit_target(RRT, ms, 0));
              }
            }
          } else if (ps2x.Button(PSB_PAD_LEFT)) {
            if (!activeServo[RFC] && !activeServo[RFT]) {
              move_steps = map(ps2x.Analog(PSS_RY), 0, 255, move_c_steps[0] / 1.5, move_c_steps[1] / 1.5);
              if (debug1) {
                Serial.print(F("T/C move steps: ")); Serial.println(move_steps);
              }
              update_sequencer(LF, LFT, spd, (servoHome[LFT] + move_steps), 0, 0);
              update_sequencer(LF, LFC, spd, (servoHome[LFC] + move_steps), 0, 0);
              update_sequencer(LR, LRT, spd, (servoHome[LRT] + move_steps), 0, 0);
              update_sequencer(LR, LRC, spd, (servoHome[LRC] + move_steps), 0, 0);
              update_sequencer(RF, RFT, spd, (servoHome[RFT] + move_steps), 0, 0);
              update_sequencer(RF, RFC, spd, (servoHome[RFC] + move_steps), 0, 0);
              update_sequencer(RR, RRT, spd, (servoHome[RRT] + move_steps), 0, 0);
              update_sequencer(RR, RRC, spd, (servoHome[RRC] + move_steps), 0, 0);
            }
          }
        }
  
        //LR
        if (ps2x.Button(PSB_L2)) {
          if (ps2x.Button(PSB_PAD_UP)) {
            if (!activeServo[LRC]) {
              int ms = servoPos[LRC];
              if (ps2x.Button(PSB_TRIANGLE)) {
                ms += 1;
              } else if (ps2x.Button(PSB_CROSS)) {
                ms -= 1;
              }
              update_sequencer(LR, LRC, spd, ms, 0, 0);
              if (debug1) {
                Serial.print(F("LRC: ")); Serial.println(limit_target(LRC, ms, 0));
              }
            }
          } else if (ps2x.Button(PSB_PAD_RIGHT)) {
            if (!activeServo[LRF]) {
              int ms = servoPos[LRF];
              if (ps2x.Button(PSB_TRIANGLE)) {
                ms += 1;
              } else if (ps2x.Button(PSB_CROSS)) {
                ms -= 1;
              }
              update_sequencer(LR, LRF, spd, ms, 0, 0);
              if (debug1) {
                Serial.print(F("LRF: ")); Serial.println(limit_target(LRF, ms, 0));
              }
            }
          } else if (ps2x.Button(PSB_PAD_DOWN)) {
            if (!activeServo[LRT]) {
              int ms = servoPos[LRT];
              if (ps2x.Button(PSB_TRIANGLE)) {
                ms += 1;
              } else if (ps2x.Button(PSB_CROSS)) {
                ms -= 1;
              }
              update_sequencer(LR, LRT, spd, ms, 0, 0);
              if (debug1) {
                Serial.print(F("LRT: ")); Serial.println(limit_target(LRT, ms, 0));
              }
            }
          } else if (ps2x.Button(PSB_PAD_LEFT)) {
            if (!activeServo[RRF]) {
              move_steps = map(ps2x.Analog(PSS_RY), 0, 255, move_c_steps[0] / 1.5, move_c_steps[1] / 1.5);
              update_sequencer(LF, LFF, spd, (servoHome[LFF] + move_steps), 0, 0);
              update_sequencer(LR, LRF, spd, (servoHome[LRF] + move_steps), 0, 0);
              update_sequencer(RF, RFF, spd, (servoHome[RFF] + move_steps), 0, 0);
              update_sequencer(RR, RRF, spd, (servoHome[RRF] + move_steps), 0, 0);
            }
          }
        }
      }
    }
  } else {
    for (int i = 0; i < 10; i++) {
      ps2x.read_gamepad(false, false);
      delay(100);
    }
    if (rgb_active) {
      rgb_request((char*)"vGMRNRn");
    }
    digitalWrite(OE_PIN, LOW);
    pwm_oe = 1;
  }

  lastPS2Update = millis();
}
