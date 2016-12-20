# Air-Hockey-Robot-Control

Code by: Derek Cheng derekwch@usc.edu
USC AME-441a Senior Projects Laboratory - Fall '16
Optimizing a Trajectory Tracking System - Air Hockey Robot
Derek Cheng -- Nicole Pay -- Doug Jones -- Peter Argo

This repo contains all the code written to control a trajectory tracking air hockey
robot as part of my team's senior project at the University of Southern California
Mechanical Engineering program. The vision system is written in C++ with Microsoft 
Visual Studio. OpenCV is applied for contour detection, and puck trajectory and 
predictions are calculated from contour detection results. The motor control takes
serial packets from the vision system and converts them into signals for the Arduino
to move the motors.

Final Report Abstract: 
As trajectory tracking develops, it can be used in applications such as finding and 
shooting down drones flying over restricted airspace.  To test the limitations of a 
trajectory tracking system, the concept was applied to build an air hockey robot that 
was controlled by a dual-axis stepper motor H-Bot configuration and vision system. 
The goal was to detect a puck travelling at less than 10±2 ft/s, predict and intercept 
its trajectory, and hit it back at towards a human player 100% of the time. Experimental 
results showed that the robot system was able to intercept pucks moving up to 12±2 ft/s 
but without perfect accuracy. Straight moving pucks were returned with 90.5% accuracy 
and pucks that bounced off the sidewall with a 85.1% accuracy, thus the robot failed to 
meet expectations. Future iterations of the robot would implement closed-loop motor 
control, optimized software code, and improved motor and computer hardware to increase 
the overall system performance.

Video: https://www.youtube.com/watch?v=AjJ8416kZDU
