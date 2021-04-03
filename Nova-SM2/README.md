# Nova-SM2 - a Spot-Mini Micro clone
![Nova-SM2](https://raw.githubusercontent.com/cguweb-com/Arduino-Projects/main/Nova-SM2/Nova-SM2.jpg)  
This is the software for my spot-mini clone project, Nova Spot Micro  

It was not written with public consumption in mind, and is still under development, so use at your own risk!  

A list of links (yes, Amazon affiliate links!) to all of the hardware components used can be found on the Instructables page for this project, linked below.  

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
   ### RELEASE NOTES:
   
      - Arduino mega performance: 42% storage / 57% memory
      - commented code for github release
      - updated NovaServos after recalibrating servos
      - added serial commands for testing servos / legs

  
##    
   ### DEV NOTES:
      - [ ] BUG: during testing routines, on abrupt change of routine, ramping hangs, stepping every +/-1000ms
      - [ ] re-calibrate servo home positions to balance Nova's COG!! seems to be back-heavy
      - [ ] x_axis: tweak pattern, adjusting use of move_steps to not near fall over backwards on startup
      - [ ] finish tweaking left and right stepping
      - [ ] finish forward step (w/ left, right, backwards!)
      - [ ] write a stable fixed speed / step walking routine
      - [ ] fix 'stay' routine (ie: tends to fall backward when coming off of kneel or sit positions into stay)
      
      see **DEV NOTES** in code comments for more bugs/tasks
      
