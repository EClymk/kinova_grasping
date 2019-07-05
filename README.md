# Otter Grasping with Kinova Mico

This repository contains a ROS package for running a simple test on a Kinova Jaco arm.  
## Paper

Modified from GGCNN Grasping, thanks to Doug Morrison's work.

```text
@article{morrison2018closing, 
	title={Closing the Loop for Robotic Grasping: A Real-time, Generative Grasp Synthesis Approach}, 
	author={Morrison, Douglas and Corke, Peter and Leitner, Jürgen}, 
	booktitle={Robotics: Science and Systems (RSS)}, 
	year={2018} 
}
```

## Installation

This code was developed with Python 2.7 on Ubuntu 16.04 with ROS Kinetic.  Python requirements can be found in `requirements.txt`.

You will also require the [Kinova ROS Packages](https://github.com/Kinovarobotics/kinova-ros) and [Realsense Camera Packages](http://wiki.ros.org/realsense_camera).

A 3D printed mount for the Intel Realsense SR300 on the Kinova Jaco arm can be found in the `cad` folder.


**Contact**

Any questions or comments contact [ECly](mailto:yixl16@mails.tsinghua.edu.cn).
