
#include <EEPROM.h>

int addr = 0;
int recorded = 0;
int printed = 0;
int step_cnt = 0;
int steps[19];

int test1 = 98;
int test2 = 54;
int test3 = 29;
int test4 = 71;
int test5 = 23;
int test6 = 49;

int test21 = 198;
int test22 = 154;
int test23 = 129;
int test24 = 171;
int test25 = 123;
int test26 = 149;

int test31 = 61;
int test32 = 38;
int test33 = 172;
int test34 = 10;
int test35 = 8;
int test36 = 155;


void setup() {
  Serial.begin(9600);
}

void loop() {

  if (!printed) {
    int step_cnt = EEPROM.read(0);

    if (step_cnt > 0) {
      Serial.println("printing test data from array and eprom");
      Serial.print("saved steps : ");
      Serial.println(step_cnt);
      Serial.print("size of data: ");
      Serial.println((step_cnt*6));
      Serial.println(" ");
      delay(3000);
  
      int cnt = 0;
      int step = 1;
      for (int i=0; i<=(step_cnt*6); i++) {
        if (i != 0) {
          if (!cnt) {
            Serial.print("step #");
            Serial.println(step);
            Serial.print("data: ");
          }
          Serial.print(EEPROM.read(i));
          if (cnt == 5) {
            step++;
            cnt = 0;
            Serial.println(" ");
          } else {
            cnt++;
            Serial.print(", ");
          }
          delay(100);
          if (step > step_cnt) {
            break;
          }
        }
      }
  
      delay(1000);
      Serial.println(" ");
      Serial.println("DONE");
      printed = 1;
    }
  }


  if (!recorded) {
    Serial.println("saving test data to array and eprom");

    //first element is for count of steps later
    EEPROM.write(addr, 0);
    addr++;

    EEPROM.write(addr, test1);
    addr++;
    EEPROM.write(addr, test2);
    addr++;
    EEPROM.write(addr, test3);
    addr++;
    EEPROM.write(addr, test4);
    addr++;
    EEPROM.write(addr, test5);
    addr++;
    EEPROM.write(addr, test6);
    addr++;

    EEPROM.write(addr, test21);
    addr++;
    EEPROM.write(addr, test22);
    addr++;
    EEPROM.write(addr, test23);
    addr++;
    EEPROM.write(addr, test24);
    addr++;
    EEPROM.write(addr, test25);
    addr++;
    EEPROM.write(addr, test26);
    addr++;
    
    EEPROM.write(addr, test31);
    addr++;
    EEPROM.write(addr, test32);
    addr++;
    EEPROM.write(addr, test33);
    addr++;
    EEPROM.write(addr, test34);
    addr++;
    EEPROM.write(addr, test35);
    addr++;
    EEPROM.write(addr, test36);

    step_cnt = (addr/6);
    EEPROM.write(0, step_cnt);

    recorded = 1;
    Serial.print("recorded steps : ");
    Serial.println(step_cnt);
    Serial.println("");
    delay(3000);
    Serial.println(" ");
    Serial.println("DONE");

  }

}
