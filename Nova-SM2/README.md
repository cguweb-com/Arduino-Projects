
# Nova SM2 - a Spot-Mini Micro clone
![Nova-SM2](https://raw.githubusercontent.com/cguweb-com/Arduino-Projects/main/Nova-SM2/Nova-SM2.jpg)  
This github project contains the 3D STL files, parts list, electrical diagrams, and the software for my spot-mini clone project, Nova Spot Micro  

These files were not created with public consumption in mind, and are constantly under development, so use at your own risk!  

A list of links (yes, Amazon affiliate links!) to all of the hardware components used can be found below for this project.  

I'd be happy to answer any questions, and/or receive any type of feedback... happy hacking!  

##    
## PROJECT NOTES  

   Version: **3.2**

   Version Date: **2021-03-31**


   Author:  **Chris Locke** - cguweb@gmail.com

   GitHub Project:  https://github.com/cguweb-com/Arduino-Projects/tree/main/Nova-SM2

   Thingiverse:  https://www.thingiverse.com/thing:4767006

   Instructables Project:  https://www.instructables.com/Nova-Spot-Micro-a-Spot-Mini-Clone/

   YouTube Playlist:  https://www.youtube.com/watch?v=00PkTcGWPvo&list=PLcOZNHwM_I2a3YZKf8FtUjJneKGXCfduk


##
### PARTS LIST

[(1) Mega Pro CH340GATmega 2560](https://amzn.to/3epOrjm)  
[(1) PS2 Receiver & Remote](https://www.robotshop.com/en/lynxmotion-ps2-controller-v4.html)  
[(1)	12A DC-DC Step Down Buck Converter 4.5V-30V](https://amzn.to/3rzTNMG)  
[(1) DC-DC Mini Buck Converter 3v-12v](https://amzn.to/2O8npCF)  
[(1) Mini Digital DC Voltmeter](https://amzn.to/3sYWd7Y)  
[(1) ACS712 Current Sensor Module](https://amzn.to/3rzUfum)  
[(1) MOSFET IRF540N](https://amzn.to/2PKW6P3)  
[(1) DIODE 1n4004](https://amzn.to/3l3lc7f)  
[(1) 2k Resistor](https://amzn.to/2ODEgwS)  
[(2) 1k Resistor](https://amzn.to/2ODEgwS)  
[(1) Lighted Rocker Switch 12v](https://amzn.to/3sX9zlg)  
[(1) XT 60 Plug & High Current Switch](https://amzn.to/3ejLoJI)  
[(1) 11.1v 3S Lipo Pack](https://amzn.to/38tbtCq)  
[(1) 16-Channel 12-bit PWM/Servo Driver](https://amzn.to/3eovgGJ)  
[(12) DS3218 20KG Full Metal Gear 270 Degree Digital Servo](https://amzn.to/2OlkcPY)  
[(12) 3-Pin Servo Extension Cable](https://amzn.to/3v7eXEe)  
[(8) 8x16x5mm Ball Bearings](https://amzn.to/3elVytj)  
[(12) 25T Aluminum Servo Arm Horn](https://amzn.to/3qvkOji)  
[(1) MH-SR602 PIR Motion Sensor Module](https://amzn.to/3cjFYf0)  
[(4) WS2812B 5050 RGB LED](https://amzn.to/3t66Y8u)  
[(2) HC-SR04 Ultrasonic Module Distance Sensor](https://amzn.to/3cm6c0F)  
[(1) GY-521 MPU-6050 3 Axis Accelerometer Gyroscope Module](https://amzn.to/3rzxcA7)  
[(1) SSD1306 128X64 OLED Display Module](https://amzn.to/3rxRoT1)  
[(1) Piezo Buzzer](https://amzn.to/3vdlPAa)  
[(1) Push Button Switch](https://amzn.to/3ld95oB)  
[() 5.08mm Screw Connection PCB Terminal](https://amzn.to/30thAlD)  
[(2) Small Double-Sided PCB Board](https://amzn.to/3bxj0ln)  
[(1) Hatchbox Yellow PLA](https://amzn.to/3t1rbwn)  
[(1) Hatchbox Black PLA](https://amzn.to/3elUHJ7)   
[(1) YOYI Purple TPU](https://amzn.to/38qA8r2)  
[18AWG Silicone Wire](https://amzn.to/38qEj6i)  
[22AWG Hookup Wire](https://amzn.to/3rzxFCn)  
[Dupont Jumper Wires](https://amzn.to/3l1k304)  
[Loctite Threadlocker](https://amzn.to/38ojfxc)  
And lots of various size 3mm, 4mm, and 5mm screws and nuts  


##    
### RELEASE NOTES  

      - Arduino mega performance: 42% storage / 57% memory
      - commented code for github release
      - updated NovaServos after recalibrating servos
      - added serial commands for testing servos / legs


##    
### DEV NOTES  
      - [ ] BUG: during testing routines, on abrupt change of routine, ramping hangs, stepping every +/-1000ms
      - [ ] re-calibrate servo home positions to balance Nova's COG!! seems to be back-heavy
      - [ ] x_axis: tweak pattern, adjusting use of move_steps to not near fall over backwards on startup
      - [ ] finish tweaking left and right stepping
      - [ ] finish forward step (w/ left, right, backwards!)
      - [ ] write a stable fixed speed / step walking routine
      - [ ] fix 'stay' routine (ie: tends to fall backward when coming off of kneel or sit positions into stay)

      see **DEV NOTES** in code comments for more bugs/tasks

