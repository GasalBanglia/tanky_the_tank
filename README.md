# tanky_the_tank
Program file for a custom Arduino-powered robotic tank. Tanky is programmed to move in straight(ish) lines and check his environment every so often for obstacles. If one is sensed, Tanky backs up and changes direction.  This repeats until he runs out of charge or someone picks him up and disconnects his battery.

COMPONENTS
The tank includes a motor driver board for 4 motors, an arduino nano, an ultrasonic rangefinder mounted on a stepper motor for obstacle detection within a ~100 degree sweep, a darlington transistor driver for the stepper, and a front bumper attached to two soft push switches for obstacles that are difficult to detect otherwise.
