# SFND 3D Object Tracking

Welcome to the final project of the camera course. By completing all the lessons, you now have a solid understanding of keypoint detectors, descriptors, and methods to match them between successive images. Also, you know how to detect objects in an image using the YOLO deep-learning framework. And finally, you know how to associate regions in a camera image with Lidar points in 3D space. Let's take a look at our program schematic to see what we already have accomplished and what's still missing.

<img src="images/course_code_structure.png" width="779" height="414" />

In this final project, you will implement the missing parts in the schematic. To do this, you will complete four major tasks: 
1. First, you will develop a way to match 3D objects over time by using keypoint correspondences. 
2. Second, you will compute the TTC based on Lidar measurements. 
3. You will then proceed to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, you will conduct various tests with the framework. Your goal is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor. In the last course of this Nanodegree, you will learn about the Kalman filter, which is a great way to combine the two independent TTC measurements into an improved version which is much more reliable than a single sensor alone can be. But before we think about such things, let us focus on your final project in the camera course. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.

### FP.5 Performance Evaluation 1
The results I get with the LiDAR are more stable than with the camera, which makes sense considering that the LiDAR is an active sensor.
Even though in some frames the distance increases when it should decrease the results are pretty stable I would say
<p align="center">
  <img  src="https://github.com/hect1995/SFND_3D_Object_Tracking/blob/master/results/lidar_1st.png">
</p>

### FP.6 Performance Evaluation 2
As I showed in Mid-term project, the combination of **FAST/BRIEF** as the combination as it's fast and it detects relatively more keypoints. The TTC from camera does not fit perfectly to the LiDAR but still makes sense.
As for combination of for excample HARRIS/ORB, the TTC output from camera becomes really unstable. Sometimes TTC is `NaN` or `inf` because there are too few keypoints detected or matches found. Moreover, as both vehicles drive to a similar speed when I compute the median of all the ratios, sometimes the results gives 1.

Results of each frame for bad Detector/Descriptor combination are in the left part of the table below, good combination is listed in the right part.

| Frame | Combination | TTC Lidar | TTC Camera | Combination | TTC Lidar | TTC Camera |
| --- | --- | --- |--- | --- | --- |--- |
|0/1 | HARRIS/ORB |14.09 | -inf|FAST/BRIEF |12.9722 | 12.1312|
|1/2 | HARRIS/ORB |16.68 | 10.01|FAST/BRIEF | 12.264 | 10.7908|
|2/3 | HARRIS/ORB |15.90 | 9.67|FAST/BRIEF |13.9161 | 12.5844|
|3/4 | HARRIS/ORB |12.67 | 4.56|FAST/BRIEF |7.11572 | 14.1171|
|4/5 | HARRIS/ORB |11.98 | 5.13|FAST/BRIEF |16.2511 | 14.7374|
|5/6 | HARRIS/ORB |13.12 | 5.58|FAST/BRIEF |12.4213 | 13.0071|
|6/7 | HARRIS/ORB |13.04 | 5.75|FAST/BRIEF | 34.3404 | 13.5165|
|7/8 | HARRIS/ORB |12.80 | 68.5|FAST/BRIEF |9.34376 | 32.985|
|8/9 | HARRIS/ORB |8.95 | 14.53|FAST/BRIEF |18.1318 | 12.7521|
|9/10 | HARRIS/ORB |9.90 | 23.5|FAST/BRIEF |18.0318 | 11.5258|
|10/11 | HARRIS/ORB |9.59 | 7.54|FAST/BRIEF |3.83244 | 12.1|
|11/12 | HARRIS/ORB |8.57 | 5.70|FAST/BRIEF |-10.8537 | 11.7805|
|12/13 | HARRIS/ORB |9.22307 | 13.4095|FAST/BRIEF |9.22307 | 11.6205|
|13/14 | HARRIS/ORB |10.9678 | inf|FAST/BRIEF |10.9678 | 11.1323|
|14/15 | HARRIS/ORB | 8.09422 | nan|FAST/BRIEF |8.09422 | 12.8498|
|15/16 | HARRIS/ORB |3.17535 | 12.5305|FAST/BRIEF |3.17535 | 11.328|
|16/17 | HARRIS/ORB | 11.99424 | nan|FAST/BRIEF |-9.99424 | 10.8182|
|17/18 | HARRIS/ORB |8.30978 | nan|FAST/BRIEF |8.30978 | 11.8703|

