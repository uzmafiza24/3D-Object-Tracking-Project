# SFND 3D Object Tracking

<p align="center">
  <img src="images/Writeup/image5.png" width="1000" height="414" />
</p>

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

# SFND 3D Object Tracking Rubric

## FP.1 Match 3D Objects

_Implement the method matchBoundingBoxes, which takes as input both the previous and the current data frames and provides as output the ids of the matched regions of interest (i.e. the boxID property). Matches must be the ones with the highest number of keypoint correspondences._

Implemented in camFusion_Student.cpp

## FP.2 Compute Lidar Based TTC

_Compute the time-to-collision in second for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame._

Implemented in camFusion_Student.cpp.

## FP.3 Associate Keypoint Correspondences with Bounding Boxes

_Prepare the TTC computation based on camera measurements by associating keypoint correspondences to the bounding boxes which enclose them. All matches which satisfy this condition must be added to a vector in the respective bounding box._

Implemented in camFusion_Student.cpp.

## FP.4 Compute Camera-based TTC

_Compute the time-to-collision in second for all matched 3D objects using only keypoint correspondences from the matched bounding boxes between current and previous frame._ 

Implemented in camFusion_Student.cpp.

## FP.5 Performance Evaluation 1

_Find examples where the TTC estimate of the Lidar sensor does not seem plausible. Describe your observations and provide a sound argumentation why you think this happened. Several examples (2-3) have been identified and described in detail. The assertion that the TTC is off has been based on manually estimating the distance to the rear of the preceding vehicle from a top view perspective of the Lidar points._

*   As can be seen from the graph in Figure 1 calculating the TTC using lidar gives good results. 
*   There is a clear trend in TTC (reduces over time) as the ego car gets closer to the preceding vehicle. 
*   The lidar TTC estimation uses the median of the x lidar points between frames which reduces the influence of outliers. This makes the TTC estimation more robust and avoids severe errors. 
*   The estimations is not perfect however, as there are some ‘jumps’ in TTC between frames. 
*   This maybe due to noise in the lidar data which affects the spread and consistency of the point cloud lidar data. 
*   In particular the edges in the horizontal axis have large spreads in some frames which can affect the TTC and throw the estimation way off. This maybe due to reflections from the cars rear lights. 
*   See Figures 2-4  for inconsistencies in the captured lidar data between frames.
*   Figures show a top down view of the back of the preceding vehicle as measured by the lidar sensor.
  
<p align="center">
  <img src="images/Writeup/image1.png" width="779" height="414" />

</p>

<center>Figure 1 Graph of TTC lidar</center>

<p align="center">
  <img src="images/Writeup/image3.png" width="779"  />
</p>

<center>Figure 2 Large spread at edges in y axis</center>

<p align="center">
  <img src="images/Writeup/image7.png" width="779"  />
</p>
<center>Figure 3 Tight spread overall</center>


<p align="center">
  <img src="images/Writeup/image2.png" width="779" />
</p>

<center>Figure 4 Wide spread of points overall and a few outliers</center>


## FP.6 Performance Evaluation 2

_Run several detector / descriptor combinations and look at the differences in TTC estimation. Find out which methods perform best and also include several examples where camera-based TTC estimation is way off. As with Lidar, describe your observations again and also look into potential reasons. All detector / descriptor combinations implemented in previous chapters have been compared with regard to the TTC estimate on a frame-by-frame basis. To facilitate comparison, a spreadsheet and graph should be used to represent the different TTCs._

*   Figure 5 shows the comparison between the TTC estimate using lidar and camera. 
*   As with lidar, the camera based method of TTC estimation does a good job as there is a clear trend as the ego vehicle gets closer to the preceding vehicle.
*   The detector type is the biggest factor in the performance of the TTC calculation.
*   Figure 6 shows the TTC camera for all detector / descriptor combinations. 
*   The following combinations are not displayed to make the graph clearer: HARRIS/BRISK, HARRIS/BRIEF, HARRIS/FREAK, HARRIS/SIFT, ORB/BRISK, ORB/BRIEF, ORB/ORB, ORB/FREAK, ORB/SIFT. 
*   They have been removed as in some instances they fail completely (nan) to calculate the TTC and also produce severe outliers with TTC estimates of ~500s. 
*   It's clear that the detector types HARRIS and ORB fail or produce poor performance due to the small number of keypoints found. This leads to few reliable matches between frames. 
*   SHITOMASI, BRISK, AKAZE and SIFT base detectors produce good results but the TTC can be unstable in some frames. Also they have the disadvantage of being computationally expensive. 
*   A FAST based detector seems to perform the best. This is due to number of keypoints found. This was evident in the mid term project.
*   The TTC estimates using a FAST based detector are shown in Figure 7. 
*   In general there is a relationship between the number of keypoints found and performance. So there could be scenarios where the scene in the image/ frame does not present many key points (e.g low light, night time). Therefore estimating TTC using camera even with a FAST based descriptor may fall down.

<p align="center">
  <img src="images/Writeup/image6.png" width="779" height="414"/>
</p>
<center>Figure 5 TTC comparison of Lidar and camera</center>


<p align="center">
  <img src="images/Writeup/image4.png" width="779" height="414"/>
</p>

<center>Figure 6 TTC using camera for all detector / descriptor combinations </center>


<p align="center">
  <img src="images/Writeup/image8.png" width="779" height="414"/>
</p>
<center>Figure 7 FAST based detector to estimate TTC</center>





