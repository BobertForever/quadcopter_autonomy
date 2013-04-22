Quadcopter Autonomy
===================

ARDrone Fly
-----------

ROS based code used to fly the ARDrone 2 Quadcopter in various ways. The general dependency for this package is the [ARDrone_Autonomy](https://github.com/AutonomyLab/ardrone_autonomy) package.
###Wiimote Controller
The wiimote-based nodes require ROS [wiimote drivers](https://github.com/ros-drivers/joystick_drivers).
To run this node:
* Start the ARDrone drivers  
```
    rosrun ardrone_autonomy ardrone_driver
```
* Start the willmote drivers  
```
    rosrun wiimote wiimote_node.py
```
* Start the controller code

optionally, you can run the launch file which starts all of these  
```
    roslaunch ardrone_fly ardrone_wiimote_teleop.launch
```  

###Ball Following
The Ball following node requires a blob tracking driver. For our purposes, [CMVision](https://github.com/dutchcheesehead/ROSMAV/tree/master/cmvision) was used. The colors.txt file in the root of the package contains the data for which blob it will follow.
To run this node:
* Start the ARDrone drivers  
```
    rosrun ardrone_autonomy ardrone_driver
```
* Start CMVision  
```
    roslaunch ardrone_fly cmvision_blob_detector.launch
```
* Start the controller code
