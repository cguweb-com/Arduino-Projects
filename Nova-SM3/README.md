This project is licensed under the terms of the MIT license.

# Nova SM3 - a Spot-Mini Micro clone
![Nova-SM3](https://raw.githubusercontent.com/cguweb-com/Arduino-Projects/main/Nova-SM3/novasm3.png)  
This github project contains the 3D STL files, parts list, electrical diagrams, and the software for my spot-mini clone project, Nova Spot Micro  

These files were not created with public consumption in mind, and are constantly under development, so use at your own risk!  

A list of links (yes, Amazon affiliate links!) to all of the hardware components used can be found on Nova's website at https://novaspotmicro.com  

I'd be happy to answer any questions, and/or receive any type of feedback... happy hacking!  

##    
## PROJECT NOTES  

   Version: **5.0**

   Version Date: **2021-06-07**


   Author:  **Chris Locke** - cguweb@gmail.com

   Nova's Website:  https://novaspotmicro.com

   GitHub Project:  https://github.com/cguweb-com/Arduino-Projects/tree/main/Nova-SM3

   Discord Server:  https://discord.gg/Fj8NsHED

   YouTube Playlist:  https://www.youtube.com/watch?v=00PkTcGWPvo&list=PLcOZNHwM_I2a3YZKf8FtUjJneKGXCfduk



##
### PARTS LIST
   The full linked parts list is now maintained on Nova's website only. My apologies, but its become too much work to maintain multiple versions of it across different platforms.
   
   https://novaspotmicro.com/parts-list/


##
### STL Files
   STL files have markings on them to aid in orientation when assembling Nova. Please use the following legend if you have trouble with any parts.

   **Frame** 
   
      - T = Top of the part (this side will be the closest to the sky) 
      
      - FO = Front Outer piece (Most outer part of the front (head)) 
      
      - FM = Front Middle piece (Attaches between FI and FO) 
      
      - FI = Front Inner piece (Attaches to the front of the chassis) 
      
      - F = Front (arrow points to head of NOVA) 
      
      - RI = Rear Inner (Closest to chassis) 
      
      - RM = Rear Middle 
      
      - RO = Rear Outer 
      

   **Legs**
   
      - Has an arrow pointing in towards the belly of NOVA  
   
      - L = Left side (NOVA’s Left – from standing behind her)
      
      - R = Right side 
   
   **Covers** 
   
      - BR = Bottom Rear piece 
      
      - BF = Bottom Front piece 
      
      - TR = Top Rear Piece 
      
      - TF = Top Front Piece  

   **Head and Rear** 
   
      - T = Top (point towards sky) 



##    
### RELEASE NOTES  

      - Teensy 4.0 performance: 5% storage / 22% memory
      - Replaced Arduino Mega with Teensy 4.0
      - Replaced PS2 Library with Teensy version
      - : https://github.com/KurtE/Arduino-PS2X
      - Code for Second i2c Bus      
      - Code for On/Off Button
      - Replaced IMU/MPU6050 Code / Library
      - : https://github.com/kriswiner/MPU6050
      - Restructured setup() routine and startup debugging
      - Removed unused SPI library

##    
### DEV NOTES  

      - [ ] BUG: ramping: on interruption of ramp, servo speed is set to the speed of the point of interrupt
      - [ ] work on integrating MPU data into movements

      see **DEV NOTES** in code comments for more bugs/tasks

