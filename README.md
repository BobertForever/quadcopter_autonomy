#ARDrone Fly
ROS based code used to fly the ARDrone 2 Quadcopter in various ways. The general dependency for this package is the [ARDrone_Autonomy](https://github.com/AutonomyLab/ardrone_autonomy) package.

##Wiimote Controller
The wiimote-based nodes require ROS [wiimote drivers](https://github.com/ros-drivers/joystick_drivers).
To run this node:
* Start the ARDrone drivers  
```
    rosrun ardrone_autonomy ardrone_driver
```

* Start the wiimote drivers  
```
    rosrun wiimote wiimote_node.py
```  

* Start the wiimote controller code 
```
	rosrun ardrone_fly wiimoteFly
```

Optionally, you can run the launch file which starts all the nodes necessary for wiimote control:  
```
    roslaunch ardrone_fly ardrone_wiimote_teleop.launch
```  

##Leap Motion Controller
We have also added support for control with a Leap Motion device. This requires [rosleap](https://github.com/mattbroussard/rosleap) and the [Leap Motion SDK](https://www.leapmotion.com/developers).
* Start the rosleap node 

```
	rosrun rosleap rosleap
``` 
* Start the leap controller code 

```
	rosrun ardrone_fly leapFly
``` 

##Ball Following
The Ball following node requires a blob tracking driver. For our purposes, [CMVision](https://github.com/dutchcheesehead/ROSMAV/tree/master/cmvision) was used. The colors.txt file in the root of the package contains the data for which blob it will follow.
To run this node:
* Start the ARDrone drivers  
```
    rosrun ardrone_autonomy ardrone_driver
```

* Start the tri-mux (starts the ball following with a multiplexer) 
```
    roslaunch ardrone_fly ardrone_trimux_demo.launch
```

Once the launch file has completed setup, toggle the flight control mode using the "A" button on the wiimote controller. This can be done at any time to switch control between the three main control schemes (wiimote, leapmotion, and ball fallowing)

##BWI Safe Leap Demo
With the addition of the Leap Motion control option, we have included a launchfile which runs both the wiimote and leapmotion controllers with an additional node that allows toggling between them using the B button on the wiimote (default is wiimote mode). Takeoff/landing/reset commands and flip commands must be issues in wiimote mode.


The purpose of this launchfile is to facilitate safely demoing leapmotion control in a public setting. A human monitor manually switches control between the wiimote and leap preventing a malicious or clumsy visitor from flying the quadcopter in an undesired way (control can be taken from them if they go awry).
* Run the launchfile for the BWI Safe Leap Demo 
```
	roslaunch ardrone_fly ardrone_wii_leap_mux_demo.launch
```

##Demo Status Webpage

The webpage at `web/index.html` uses [rosbridge](https://github.com/RobotWebTools/rosbridge_suite) and [HTML-Aircraft-Indicators](https://github.com/PixelsCommander/HTML-Aircraft-Indicators) to display info during the above demos.

You must have rosbridge running on port 9090 of the local machine.
```
	roslaunch rosbridge_server rosbridge_websocket.launch
```
