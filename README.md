# SC-A-LOAM

## What is SC-A-LOAM? 
- A real-time LiDAR SLAM package that integrates A-LOAM and ScanContext. 
    - **A-LOAM** for odometry (i.e., consecutive motion estimation)
    - **ScanContext** for coarse global localization (i.e., place recognition as kidnapped robot problem without initial pose)
    - and iSAM2 of GTSAM is used for pose-graph optimization. 
- This package aims to show ScanContext's handy applicability. 
    - The only things a user should do is just to include `Scancontext.h`, call `makeAndSaveScancontextAndKeys` and `detectLoopClosureID`. 

## Features 
1. A modular implementation 
    - The only difference from A-LOAM is the addition of the `laserPosegraphOptimization.cpp` file. In the new file, we subscribe the point cloud topic and odometry topic (as a result of A-LOAM, published from `laserMapping.cpp`). That is, our implementation is generic to any front-end odometry methods. Thus, our pose-graph optimization module (i.e., `laserPosegraphOptimization.cpp`) can easily be integrated with any odometry algorithms such as non-LOAM family or even other sensors (e.g., visual odometry).  
2.  A strong place recognition and loop closing 
    - We integrated ScanContext as a loop detector into A-LOAM, and ISAM2-based pose-graph optimization is followed. (see https://youtu.be/okML_zNadhY?t=313 to enjoy the drift-closing moment)
3. Altitude stabilization using consumer-level GPS  
    - To make a result more trustworthy, we supports GPS (consumer-level price, such as U-Blox EVK-7P)-based altitude stabilization. The LOAM family of methods are known to be susceptible to z-errors in outdoors. We used the robust loss for only the altitude term. For the details, see the variable `robustGPSNoise` in the `laserPosegraphOptimization.cpp` file. 

## Prerequisites (dependencies)
- We mainly depend on ROS, Ceres (for A-LOAM), and GTSAM (for pose-graph optimization). 
    - For the details to install the prerequisites, please follow the A-LOAM and LIO-SAM repositiory. 
- The below examples are done under ROS melodic (ubuntu 18) and GTSAM version 4.x. 

## How to use? 
- First, install the abovementioned dependencies, and follow this lines. 
```
    mkdir -p ~/catkin_scaloam_ws/src
    cd ~/catkin_scaloam_ws/src
    git clone https://github.com/gisbi-kim/SC-A-LOAM.git
    cd ../
    catkin_make
    source ~/catkin_scaloam_ws/devel/setup.bash
    roslaunch aloam_velodyne aloam_mulran.launch # for MulRan dataset setting 
```

## Example Results 

### Riverside 01, MulRan dataset 
- The MulRan dataset provides lidar scans (Ouster OS1-64, horizontally mounted, 10Hz) and consumer level gps (U-Blox EVK-7P, 4Hz) data.
    - About how to use (publishing data) data: see here https://github.com/irapkaist/file_player_mulran
- example videos on Riverside 01 sequence. 
    1. with consumer level GPS-based altitude stabilization: https://youtu.be/FwAVX5TVm04
    2. without the z stabilization: https://youtu.be/okML_zNadhY 
- example result:

<p align="center"><img src="picture/riverside01.png" width=800></p>

### KITTI 05 
- The first, reconfigure the N_SCANS-related lines in `scanRegistration.cpp` file (the default parameter is set for MulRan dataset).  
- example video: TODO

## Acknowledgements
- Thanks to LOAM, A-LOAM, and LIO-SAM code authors. The major codes in this repository are borrowed from their efforts.

## Maintainer 
- please contact me through `paulgkim@kaist.ac.kr` 

## TODO
- More examples on other datasets (KITTI, complex urban dataset, etc.)
- Delayed RS loop closings 
- data saver (e.g., pose-graph, optimized map)
- SLAM with multi-session localization 
- Efficient whole map visualization 
