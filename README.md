Demo
========
Video: https://youtu.be/ljwVBU90XW4

Synopsis
========
This project is the first attempt in the world to apply Central Pattern Generators (combined with a novel learning algorithm) to compliant humanoid robots, such as Atlas. 

Motivation
==========
The project was developed as Major Qualifying Project (senior year undergraduate project at WPI). 

Creation
========
The project was completed on April 25, 2016 by myself. My advisor was Dr. Michael Gennert.

Simulink was chosen to implement the control module (please see included report or presentation) to keep connections organized. All learning functionality was implemented in Python. 

The system was tested in Gazebo and on the real robot.

System Overview
===============
<img src="https://github.com/bnurbekov/Humanoid_Robot_Learning_To_Walk/blob/master/MQPSystem.png" width="350"/>

Central Pattern Generator
===============
The purpose of this module:
+ reducing a state (parameter) space from 100 states to an average of 10 states;
+ generating rhythmic patterns.

Consists of the following layers:

1. Rhythmic generator layer generates synchronized oscillatory movements.
2. Pattern formation and motor neuron layers incorporate afferent sensory feedback for movement modulation [1]. 

Learning
========
Parameters for CPG are learnt by
+ CMA-ES (learning);
+ Kohonen self-organizing maps (memory) → Success, Failure Map.

Covariance Matrix Adaptation – Evolutionary Strategies
------------------------------------------------------
The purpose of this module:
+ learn set of parameters for CPG that lead to stable walking gate initially

Qualitative Adaptive Reward Learning
------------------------------------
The purpose of this module:
+ memorizing a particular region in parameter (state) space that leads to success/failure;
+ finding global optimal set of parameters in success region after failure region is learnt.

Installation
============
As this was a school project, it was not meant to be distributed and used. Nonetheless, it's certainly possible to recreate the environment used. The Simulink diagram and the Python modules will have to modified in that case to use proper ROS topics. Also, the .mex file for the custom ROS subscriber will have to be recompiled (the compilation command is included into the repo).
