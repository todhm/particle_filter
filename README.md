# Particle Filter Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

[image1]: ./pictures/particle_pic.png
[image2]: ./pictures/newplot.png

This is part of udacity self-driving car nanodegree project part3. In this project particle filter model was built based on c++  to estimate the location of robot in terms of map coordinate system.  This code consists of following processes.
* Initialize the predefined number of particles(100) in my code.
* Predict the Coordinate of each particles.
* Update the particle's coordinate and direction based on the sensor measuring the landmark and update weight based on predicted measurement and it's noise. 
* Resample Filter based on their weight. 

The source code of this project consists with followings.
* main.cpp: Reads the data and send the processed data to simulator.
* particle_filter.h/particle_filter.cpp: code to proceed the particle filter. 
 

The visualization of this project was made with the Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

---

## Basic Build Instructions

I used xcode as main IDE. You can execute project with ide_profiles/xcode/PARTICLE_FILTER.xcodeproj file. 

Also you can execute project with following steps.
0. Install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems and [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) for windows.
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
4. Run it: `./particle_filter`  
5. Run the simulator and watch the results.
---
## Location Of Landmarks. 
Here is the coordinate of landmark which were used to update the measurements and weight of each particle. 
[![alt text][image2]](./pictures/basic-scatter.html)

## Results
* The result of model was measured by Cumulative mean weighted error. 

[![alt text][image1]]( https://youtu.be/HEFLnO1-Ci0)

---
## Discussion  
#### The crucial tuning point that I have struggled to finish this project.
* You have to generate default_random_engine only one time in the prediction step and initialization step to avoid generate multiple Gaussian distribution at once.
