Nova-SM2_mega-v3.3.ino: In function 'void setup()':
Nova-SM2_mega-v3.3.ino:396:20: warning: ISO C++ forbids converting a string constant to 'char*' [-Wwrite-strings]
   command_slave("Z");

Nova-SM2_mega-v3.3.ino: In function 'init_home()':
Nova-SM2_mega-v3.3.ino:1853:22: warning: iteration 4 invokes undefined behavior [-Waggressive-loop-optimizations]
     servoSequence[i] = 0;
     ~~~~~~~~~~~~~~~~~^~~
Nova-SM2_mega-v3.3.ino:1845:21: note: within this loop
   for (int i = 0; i < TOTAL_SERVOS; i++) {


Nova-SM2_mega-v3.3.ino: In function 'setup':
Nova-SM2_mega-v3.3.ino:1853:22: warning: iteration 4 invokes undefined behavior [-Waggressive-loop-optimizations]
     servoSequence[i] = 0;
                      ^
Nova-SM2_mega-v3.3.ino:1845:21: note: within this loop
   for (int i = 0; i < TOTAL_SERVOS; i++) {
                     ^
