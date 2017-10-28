# 2D Monte Carlo Localization
## Background

A simple implementation of the MCL particle filter algorithm, as outlined by Thrun 
and Norvig in their online [Intro to AI](http://www.ai-class.com) course.

This program is designed to be used under [ROS](http://www.ros.org) (Robot Operating
System). Specifically, use the 
[UML_MCL](http://www.cs.uml.edu/ecg/pub/uploads/MRspr12/uml_mcl.tar.gz)
package and the 
[MCL tools](http://www.cs.uml.edu/ecg/pub/uploads/MRspr12/sample_hw7_r2.tar.gz) 
helper kit. It is a simulation of a robot in 60m x 4m hallway, where the goal is to 
help the robot localize and find its way to a goal. 

## The Robot
### Sensors
The robot has 5 range sensors at -90, -45, 0, 45, and 90 degrees, each with a 
max range of 5 meters. The sensors are imperfect, so there is error in the 
values they return. The code for generating error can be found in sensor.py.

### Motors
The robot is able to spin and move forward. The motors are imperfect, so the
robot does not always do exactly what it is told to do. The code for generating
error can be found in motor.py.

## MCL.py
The localization code is located in MCL.py. There are three parts to the code:
particle filter, wall avoidance, and clustering. The robot will wander until
it has localized with a high confidence, then drive to the goal (in between
the two Xs, around (-8, -4)). 
