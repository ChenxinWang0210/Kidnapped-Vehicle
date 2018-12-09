# Kidnapped Vehicle Project 
Udacity Self-Driving Car Engineer Nanodegree Program


## Project Description

The goal of this project is to implement a 2 dimensional particle filter in C++, which helps a kidnapped vehicle identify its location. The filter is given a map and a (noisy) GPS estimate of the vehicle's initial location. At each time step, the filter gets observation and control data from [a simulator](https://github.com/udacity/self-driving-car-sim/releases) provided by Udacity. The filter will estimate the vehicle's position and send the estimates to the the simulator for display. 

Udacity provides [descriptions and starter code](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project) for this project. 


## Compile and Run the project

The main program can be built and ran by doing the following from the project top directory (instructions from Udacity)

```
* ./clean.sh 
* ./build.sh
* ./run.sh
* run Udacity simulator

```
## Results
The following gif displays the particle filter estimations. The RMSE values of x, y, and yaw show the accuracy of these estimations.

![](kidnapped_vehicle.gif)







