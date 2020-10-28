# **Extended Kalman Filter** 

##  Kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements

### Obtaining RMSE values that are lower than the tolerance
---

**Extended Kalman Filter Project**

The goals / steps of this project are the following:
* Utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.
* Obtaining RMSE values that are lower than the tolerance.

[//]: # (Image References)

[image1]: ./Images_forReadMe/dataset1.gif "dataset1"
[image2]: ./Images_forReadMe/dataset1.mp4 "dataset1"
[image3]: ./Images_forReadMe/dataset2.gif "dataset2"
[image4]: ./Images_forReadMe/dataset2.mp4 "dataset2"

---
### README

- A extended kalman filter was implemented to estimate the state of a moving object which can be found in the [src folder](./src)

- Below is the result of the EKF in two datasets of figure 8. Each dataset is the inverse of the other.

| Dataset 1 | Dataset 2 |
| ------------- | ------------- |
| ![alt text][image1]| ![alt text][image3] |
|Full video:|
| [Dataset 1](./Images_forReadMe/dataset1.mp4) | [Dataset 2](./Images_forReadMe/dataset2.mp4) |

### Repo contained

#### 1. uWebSockets

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO.

#### 2. Simulator

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

#### 3. Functional Code

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake .. 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. make
5. ./ExtendedKF

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

### Other Important Dependencies

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


### Conclusion

* It was interesting to learn more about sensor fusion for both lidar and radar where each has its own strenghts and weaknesses. 
* A quick sensor comparison chart is as follows:

|         | Camera | LIDAR | RADAR |
| ------------- | ------------- | ------------- | ------------- |
|  Resolution       | Good | Okay | Bad |
|  Noise       | Good | Bad | Bad |
|  Velocity       | Bad | Bad | Good |
|  All-Weather       | Bad | Bad | Good |
|  Size       | Good | Bad | Good |

* An extended kalman filter was used due to non-linear attributes from Radar measurements.

### More resources/ information

* Kalman filter is an efficient estimation/tracking tool that allows us to integrate controls and measurements in continuous space and find continuous state estimates, and it's widely used in various components of self driving car, including localization, tracking, prediction and etc. See open source autonomous driving framework [Autoware](https://gitlab.com/autowarefoundation/autoware.ai/autoware/-/wikis/home) for an example. 

* [Blog explanation](http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/) about math behind kalman filter, particularly concerning the derivation of Kalman gain K.

* ROS provides an excellent generalized extended kalman fitler library that allows us to perform robot location tracking. It uses Constant Turn Rate and Acceleration motion model to perform prediction, and measurements from various sensor like IMU, GPS, Wheel Odometry to perform update step. Here is the [paper on the ekf framework](http://docs.ros.org/en/melodic/api/robot_localization/html/_downloads/robot_localization_ias13_revised.pdf), and here is the [code implementation](https://github.com/cra-ros-pkg/robot_localization/blob/melodic-devel/src/ekf.cpp)

#### Tracking Multiple Objects and Sensor Fusion 
The below papers and resources concern tracking multiple objects, using Kalman Filters as well as other techniques!
- [No Blind Spots: Full-Surround Multi-Object Tracking for Autonomous Vehicles using Cameras & LiDARs by A. Rangesh and M. Trivedi](https://arxiv.org/pdf/1802.08755.pdf)
- [Multiple Sensor Fusion and Classification for Moving Object Detection and Tracking by R.O. Chavez-Garcia and O. Aycard](https://hal.archives-ouvertes.fr/hal-01241846/document)

#### Stereo cameras
The below papers cover various methods of using stereo camera set-ups for object detection and tracking.
- [Robust 3-D Motion Tracking from Stereo Images: A Model-less Method by Y.K. Yu, et. al.](http://www.cse.cuhk.edu.hk/~khwong/J2008_IEEE_TIM_Stereo%20Kalman%20.pdf)
- [Vehicle Tracking and Motion Estimation Based on Stereo Vision Sequences by A. Barth ]

#### Deep Learning-based approaches
The below papers include various deep learning-based approaches to 3D object detection and tracking.
- [Fast and Furious: Real Time End-to-End 3D Detection, Tracking and Motion Forecasting with a Single Convolutional Net by W. Luo, et. al.](https://openaccess.thecvf.com/content_cvpr_2018/papers/Luo_Fast_and_Furious_CVPR_2018_paper.pdf)
- [VoxelNet: End-to-End Learning for Point Cloud Based 3D Object Detection by Y. Zhou and O. Tuzel](https://arxiv.org/abs/1711.06396)

#### More papers 

- [Multiple Object Tracking using Kalman Filter and Optical Flow by S. Shantaiya, et. al.](http://www.ejaet.com/PDF/2-2/EJAET-2-2-34-39.pdf)
- [Kalman Filter Based Multiple Objects Detection-Tracking Algorithm Robust to Occlusion by J-M Jeong, et. al.](https://pdfs.semanticscholar.org/f5a2/bf3df3126d2923a617b977ec2b4e1c829a08.pdf)
- [Tracking Multiple Moving Objects Using Unscented Kalman Filtering Techniques by X. Chen, et. al.](https://arxiv.org/pdf/1802.01235.pdf)
- [LIDAR-based 3D Object Perception by M. Himmelsbach, et. al]
- [Fast multiple objects detection and tracking fusing color camera and 3D LIDAR for intelligent vehicles by S. Hwang, et. al.](https://www.researchgate.net/publication/309503024_Fast_multiple_objects_detection_and_tracking_fusing_color_camera_and_3D_LIDAR_for_intelligent_vehicles)
- [3D-LIDAR Multi Object Tracking for Autonomous Driving by A.S.](https://repository.tudelft.nl/islandora/object/uuid%3Af536b829-42ae-41d5-968d-13bbaa4ec736)

































