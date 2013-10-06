#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "leaptest/leap.h"
#include <cmath> 
#include <string>

ros::Publisher velocity_pub;

void joyCallback(const leaptest::leap::ConstPtr& msg){
    geometry_msgs::Twist output;

    output.linear.x = ((std::abs (msg->pitch)) - 90) / 30;
    if(output.linear.x > 1)
        output.linear.x = 1;
    else if (output.linear.x < -1)
        output.linear.x = -1;

    velocity_pub.publish(output);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "leapFly");
    ros::NodeHandle n;

    velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    ros::Subscriber sub = n.subscribe("leap", 1000, joyCallback);

    ros::Rate loop_rate(10);

    ros::spin();

    return 0;
}
