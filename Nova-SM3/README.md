
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
   The full linked parts list is now maintained on the Instructables page only. My apologies, but its become too much work to maintain multiple html editor versions of it (why everyone just doesn't use HTML is beyond me!)
   https://www.instructables.com/Nova-Spot-Micro-a-Spot-Mini-Clone/


##    
### RELEASE NOTES  

      - Arduino mega performance: 36% storage / 33% memory
      - re-calibrated servo home positions to balance Nova's COG!!
      - added setup() delay to prevent hardware reset execution 
      - added skip_splash variable to disable boot graphics 
      - step_march: new gait using joysticks to control "fake" kinematics :)

##    
### DEV NOTES  

      - [ ] BUG: ramping: on interruption of ramp, servo speed is set to the speed of the point of interrupt
      - [x] re-calibrate servo home positions to balance Nova's COG!! seems to be back-heavy
      - [ ] x_axis: tweak pattern, adjusting use of move_steps to not near fall over backwards on startup
      - [x] finish tweaking left and right stepping
      - [x] finish forward step (w/ left, right, backwards!)
      - [x] write a z-axis control that is sticky, so it maintains the set height on subsequent moves
      - [x] write a stable fixed speed / step walking routine
      - [ ] fix 'stay' routine (ie: tends to fall backward when coming off of kneel or sit positions into stay)
      - [ ] work on integrating MPU data into movements

      see **DEV NOTES** in code comments for more bugs/tasks

