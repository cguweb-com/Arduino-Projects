
# Nova SM3 - a Spot-Mini Micro clone
![Nova-SM3](https://raw.githubusercontent.com/cguweb-com/Arduino-Projects/main/Nova-SM3/novasm3.png)  
This github project contains the 3D STL files, parts list, electrical diagrams, and the software for my spot-mini clone project, Nova Spot Micro  

These files were not created with public consumption in mind, and are constantly under development, so use at your own risk!  

A list of links (yes, Amazon affiliate links!) to all of the hardware components used can be found below for this project.  

I'd be happy to answer any questions, and/or receive any type of feedback... happy hacking!  

##    
## PROJECT NOTES  

   Version: **4.1**

   Version Date: **2021-04-30**


   Author:  **Chris Locke** - cguweb@gmail.com

   GitHub Project:  https://github.com/cguweb-com/Arduino-Projects/tree/main/Nova-SM3

   Thingiverse:  https://www.thingiverse.com/thing:4767006

   Discord Server:  https://discord.gg/Fj8NsHED

   YouTube Playlist:  https://www.youtube.com/watch?v=00PkTcGWPvo&list=PLcOZNHwM_I2a3YZKf8FtUjJneKGXCfduk

   (under construction)
   Instructables Project:  https://www.instructables.com/Nova-Spot-Micro-a-Spot-Mini-Clone/


##
### PARTS LIST

[(1) Mega Pro CH340GATmega 2560](https://amzn.to/3ecuUm9)  
[(1) PS2 Receiver & Remote](https://www.robotshop.com/en/lynxmotion-ps2-controller-v4.html)  
[(1) 12A DC-DC Step Down Buck Converter 4.5V-30V](https://amzn.to/3aViM79)  
[(1) DC-DC Buck Boost Converter XL6009 5-32 V to 1.25-35 V](https://amzn.to/2QNi5Ww)  
[(1) Mini Digital DC Voltmeter](https://amzn.to/3aUs4Qw)  
[(1) ACS712 Current Sensor Module](https://amzn.to/3nRBQZj)  
[(1) MOSFET IRF540N](https://amzn.to/2RiOPH0)  
[(1) DIODE 1n4004](https://amzn.to/3xHldDV)  
[(1) 2k Resistor](https://amzn.to/3vG2bfx)  
[(2) 1k Resistor](https://amzn.to/3vG2bfx)  
[(1) Lighted Rocker Switch 12v](https://amzn.to/3eLnDc5)  
[(1) XT 60 Plug & High Current Switch](https://amzn.to/2QM6TJC)  
[(1) 11.1v 3S Lipo Pack](https://amzn.to/3vG2ebd)  
[(1) 16-Channel 12-bit PWM/Servo Driver](https://amzn.to/3vxvP6z)  
[(12) DS3218 20KG Full Metal Gear 270 Degree Digital Servo](https://amzn.to/3vzSFdJ)  
[(12) 3-Pin Servo Extension Cable](https://amzn.to/3gWBYFs)  
[(8) 8x16x5mm Ball Bearings](https://amzn.to/3t7yQbX)  
[(12) 25T Aluminum Servo Arm Horn](https://amzn.to/3h0J5wn)  
[(1) MH-SR602 PIR Motion Sensor Module](https://amzn.to/3eM0Hta)  
[(4) WS2812B 5050 RGB LED](https://amzn.to/3nEsCPX)  
[(2) HC-SR04 Ultrasonic Module Distance Sensor](https://amzn.to/3ta1Zn6)  
[(1) GY-521 MPU-6050 3 Axis Accelerometer Gyroscope Module](https://amzn.to/3nNjIQ3)  
[(1) SSD1306 128X64 OLED Display Module](https://amzn.to/3ueKJyb)  
[(1) Piezo Buzzer](https://amzn.to/3t7yUbH)  
[(1) Push Button Switch](https://amzn.to/3aTJwVC)  
[() 5.08mm Screw Connection PCB Terminal](https://amzn.to/2RgGjbJ)  
[(2) Small Double-Sided PCB Board](https://amzn.to/2SpqCQ5)  
[(1) Hatchbox Yellow PLA](https://amzn.to/3tcJNJl)  
[(1) Hatchbox Black PLA](https://amzn.to/3ebC9Ld)   
[(1) YOYI Purple TPU](https://amzn.to/3xEkzHf)  
[18AWG Silicone Wire](https://amzn.to/3aVjuRR)  
[22AWG Hookup Wire](https://amzn.to/3aWSpxw)  
[Dupont Jumper Wires](https://amzn.to/3eMeoZg)  
[M2 M3 M4 Alloy](https://amzn.to/337GfgT)  
[M3 M4 M5 M6 304 Stainless Steel Hex Cap](https://amzn.to/335KvNR)


##    
### RELEASE NOTES  

      - Arduino mega performance: 36% storage / 31% memory
      - splt AsyncServo class into an include file
      - created serial connection to arduino nano / slave
      - created functions for serial communication to nano / slave
      - moved uss, rgb, and oled functions to nano / slave companion file: Nova-SM3_nano-vX.x
      - Bug Fix: state machine intervals were not being set / reset for USS and MPU sensors
      - added stop on uss alarm and halt on pir alarm

##    
### DEV NOTES  

      - [ ] BUG: ramping: on interruption of ramp, servo speed is set to the speed of the point of interrupt
      - [ ] re-calibrate servo home positions to balance Nova's COG!! seems to be back-heavy
      - [ ] x_axis: tweak pattern, adjusting use of move_steps to not near fall over backwards on startup
      - [ ] finish tweaking left and right stepping
      - [ ] finish forward step (w/ left, right, backwards!)
      - [ ] write a z-axis control that is sticky, so it maintains the set height on subsequent moves
      - [ ] write a stable fixed speed / step walking routine
      - [ ] fix 'stay' routine (ie: tends to fall backward when coming off of kneel or sit positions into stay)
      - [ ] work on integrating MPU data into movements

      see **DEV NOTES** in code comments for more bugs/tasks

