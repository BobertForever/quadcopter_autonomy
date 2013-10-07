#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Empty.h"
#include "rosleap/leap.h"
#include <cmath> 
#include <string>

ros::Publisher velocity_pub;
ros::Publisher land_pub;
ros::Publisher takeoff_pub;
ros::Publisher reset_pub;

void leapCallback(const rosleap::leap::ConstPtr& msg){
    geometry_msgs::Twist output;
	std_msgs::Empty empty;

    output.linear.x = ((std::abs (msg->pitch)) - 90) / 30;
    if (output.linear.x > 1)
        output.linear.x = 1;
    else if (output.linear.x < -1)
        output.linear.x = -1;

    output.linear.y = msg->roll / 70;
    if (output.linear.y > 1)
        output.linear.y = 1;
    else if (output.linear.y < -1)
        output.linear.y = -1;
	
	else if(msg->pos.y < 6000)
		output.linear.z = -.75;
	else if(msg->pos.y > 24000)
		output.linear.z = .75;

    velocity_pub.publish(output);
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
    std_msgs::Empty empty;

    if(msg->buttons[4] == 1)
        takeoff_pub.publish(empty);
    if(msg->buttons[5] == 1)
        land_pub.publish(empty);
    if(msg->buttons[10] == 1)
        reset_pub.publish(empty);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "leapFly");
    ros::NodeHandle n;

    velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    land_pub = n.advertise<std_msgs::Empty>("ardrone/land", 1000);
    takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1000);
    reset_pub = n.advertise<std_msgs::Empty>("ardrone/reset", 1000);

    ros::Subscriber sub = n.subscribe("leap", 1000, leapCallback);
    ros::Subscriber sub2 = n.subscribe("joy", 1000, joyCallback);

    ros::Rate loop_rate(10);

    ros::spin();

    return 0;
}
