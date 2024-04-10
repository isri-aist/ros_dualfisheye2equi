#ros_dual2equi
Dual fisheye image to equirectangular image mapping. Exploits precomputed pixel coordinates mapping stored in data/*.txt (the provided file is only valid for 1280x960 pixels and same geometry as the calibrated Ricoh Theta S camera from MIS lab).

#How to install

- git clone in the src directory of your ROS catkin workspace 
- catkin_make
- source your setup.bash

#How to run

- set the inputImagesTopic and outputImagesTopic (see launch file) to respectively the dual-fisheye images topic and the output equirectangular images topic
- launch with `roslaunch ros_dual2equi dual2equi_bgr8.launch`

#Notes

Consider using [libPeR](https://github.com/PerceptionRobotique/libPeR_base) to compute other mappings (see [dualfisheye2equi](https://github.com/PerceptionRobotique/dualfisheye2equi)).
Authors: G. Caron
Date: 2020

