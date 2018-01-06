# Kidnapped Vehicle Project

Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data.

This repository contains all the code needed to complete the final project for the Localization course using by C++ in Udacity's Self-Driving Car Nanodegree.

You can find starter code created by the Udacity on [here](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project).

---

[//]: # (Image References)
[image1]: ./assets/timestep262.png
[image2]: ./assets/timestep418.png
[image3]: ./assets/timestep546.png
[image4]: ./assets/timestep748.png
[image5]: ./assets/timestep842.png

## Dependencies
* [Simulator](https://github.com/udacity/self-driving-car-sim/releases)
* [uWebSocketIO](https://github.com/uWebSockets/uWebSockets)
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## How to use
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
    * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./particle_filter`
5. Run Simulator with Project 3: Kidnapped Vehicle
6. Click Start button

## Examples of Result
![Timestep 262][image1]
![Timestep 418][image2]
![Timestep 546][image3]
![Timestep 748][image4]
![Timestep 842][image5]
