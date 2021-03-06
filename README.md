# PythonRobotics
*Bachelor's degree thesis project at University of Pavia*

## General Overview

PythonRobotics is an Open Source project born around 2016 that aimed to collect a set of algorithms for robotic navigation implemented in Python language and widely used in both academic and industrial contexts. The philosophy of this project is to collect self-contained simulations, i.e., each algorithm is contained in a single file to be compiled and executed independently.  
Robotic navigation is an autonomous system that allows a robot to move through an environment and reach a destination without direct control of an operator. To do this, we need to break down the problem into 3 main sub-problems:
- localization: the determination of the position the robot occupies in a given instant within an environment
- mapping: the construction of a map of the space in which the robot is located with obstacles and traversable points, which is fundamental for the next point
- planning: the determination of a path that takes the robot from a starting point to an end point while avoiding obstacles.

This project focuses on the last of these main sub-problems, so given a scenario map containing the obstacles, the goal is to plan a valid path to a destination by implementing 2 algorithms from the Bug family. Once the path is generated, a graphical simulation will follow for understanding how it works.  
For this project the programming language used was Python which has a number of advantages especially for those programs that must simulate the operation of algorithms, as it provides excellent libraries for mathematical calculations and matrix operations. It introduces besides also optimal bookcases as it regards the graphical visualization that turns out then simple and much intuitive; direct consequence of all that is that it benefits the readability and the quality of the code in how much do not have to be re-implemented functions base.

## Creation of the simulation environment

The first step for the creation of the simulation regards the reading and the discretization of the map with the obstacles.  
In this context a map defines a data structure that represents the environment where the robot moves; there are different types of maps, but the one adopted in this project is a grid-based map: this means that the space is divided into equal cells, whose size is determined by the resolution of the map.  
A cell is considered occupied, and therefore not accessible, if it contains a piece of an obstacle, free and therefore traversable otherwise.  
As shown in the example below, it is clear how the resolution affects the quality but also the success or failure of the path generation, since a very low resolution of the map would not allow the robot to reach the destination.

![Map_Discretization](https://user-images.githubusercontent.com/48442855/139585171-35b3a548-bf2a-4763-9c06-5980053f239b.png)

Once the map has been discretized, it is passed to the actual phase of pianification of the path; numerous algorithms exist for the scope, everyone uses different techniques and methods.  
2 algorithms of the Bug family, that is the family of defined complete algorithms, have been chosen. They principally use 2 different movement strategies: 
1. the first one called motion-to-goal, that is the displacement of the robot towards the goal position
2. the second one called boundary-following, that is the circumnavigation of an obstacle. 

By appropriately combining these two strategies, the robot moves from a starting point to a destination point while avoiding obstacles. The 2 algorithms in the Bug family that we are going to look at are called Bug1 and Bug2.

## Bug1 Algorithm

In Bug1 algorithm, the robot starts from the starting point in motion-to-goal strategy, then moves towards the goal following the line that connects the starting point with the end point, until it encounters an obstacle at the point called hit point. At this point we move to the boundary-following phase.  
This phase foresees the complete circumnavigation of the obstacle, during which the point closest to the objective, called leave point, is sought.  
Once the circumnavigation is over and it has returned to the hit point, the robot reaches the leave point following the shortest path and returns in motion-to-goal strategy to resume the path towards the target, following this time a second straight line that connects the point of detachment from the obstacle with the destination point.

![image](https://user-images.githubusercontent.com/48442855/139586200-9c418ef6-e685-4b0c-824a-2b0bd40ae077.png)

**Output of the program:**

![Bug1](https://user-images.githubusercontent.com/48442855/139586867-e7fc710f-38c0-4808-8f31-be47a0c6c36d.png)

## Bug2 Algorithm

The Bug2 algorithm, instead, behaves in the same way in the initial phase, but it modifies its behavior when an obstacle is encountered: in this case it always passes in the boundary-following phase, but this time it does not perform a complete circumnavigation but it stops when it intersects the initial straight line that joins point of departure and point of arrival in the point that is always called leave point closer to the objective; when this happens it returns in motion-to-goal strategy always following the same straight line. So unlike before the line is always the same and is the one drawn initially.

![image](https://user-images.githubusercontent.com/48442855/139586522-b2d361bc-deb1-4042-8618-73fdabb4e0e8.png)

**Output of the program:**

![Bug2](https://user-images.githubusercontent.com/48442855/139586874-3aa1fad9-37da-47e6-b9fd-50d0f2530aac.png)

## The Program

The program has been developed according to the paradigms of object-oriented programming, therefore the structure of the code is equal for both the algorithms.  
The principal classes represent: 
- the abstraction of the two algorithms, that deal with the generation of the path
- the abstraction of the map, that deals with the generation of the obstacles and the graphical visualization.

The code therefore turns out equal for the 2 algorithms, except for 2 functions that are the specific ones that conceptually differentiate the algorithms: the control of end boundary-following, that serves in order to pass from circumnavigation to displacement towards the objective, and the control of not attainable destination, situation that could happen with a particular disposition of the obstacles.

## The Simulation

The simulation can therefore be logically subdivided in 4 macro sectors:
- the reading of the map, that can be formed from a whole of polygons or it can be passed like parameter to the program as a pgm file (a grey-scale image), this in order to simplify the creation of new scenarios
- the generation of the obstacles, that discretizes the map and generates a matrix formed from boolean values
- the simulation, that takes care of 3 fundamental aspects: 
  - the discretization of the straight line that guides the robot towards the destination,
  - the choice of the correct strategy and 
  - the generation of the final path
- the visualization that is done in 2 ways, a textual and a graphical representation using the python library matplotlib.

**Bug1 Simulation:**

https://user-images.githubusercontent.com/48442855/139586693-beca577f-28bf-4e50-8ad8-d5428b5e6fce.mov

**Bug2 Simulation:**

https://user-images.githubusercontent.com/48442855/139586723-6d0b7764-27d3-4849-b8a5-023cf3ba6927.mov
