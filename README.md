# Extended Kalman Filter Project


In this project I utilized a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. This is an extended Kalman Filter implementation in C++ for fusing lidar and radar sensor measurements.



[//]: # (Image References)

[image1]: ./images/dataset1.jpeg "Test on dataset-1"
[image2]: ./images/dataset2.jpeg "Test on dataset-2"



**INPUT**: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

**In this case, we have two 'noisy' sensors:**
- A lidar sensor that measures our position in cartesian-coordinates `(x, y)`
- A radar sensor that measures our position and relative velocity (the velocity within line of sight) in polar coordinates `(rho, phi, drho)`


**OUTPUT**: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x

["estimate_y"] <= kalman filter estimated position y

["rmse_x"]

["rmse_y"]

["rmse_vx"]

["rmse_vy"]

---
**We want to predict our position, and how fast we are going in what direction at any point in time:**
- In essence: the position and velocity of the system in cartesian coordinates: `(x, y, vx, vy)`
- NOTE: We are assuming a **constant velocity model (CV)** for this particular system

## Generating Additional Data

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.


## Filter result

I use RMSE metric for evaluating the quality of my implementation. You can see my result below. There are include two tests on different datasets.

![alt text][image1]
![alt text][image2]








## Other Important Dependencies

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF
