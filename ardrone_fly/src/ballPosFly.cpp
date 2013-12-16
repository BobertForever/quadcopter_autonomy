#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include <cmath> 
#include <string>

//coauthored by Matt Broussard, Robert Lynch -- December 15, 2013

ros::Publisher velocity_pub;

ros::Time lastMessageTime;

#define SAFE_MAX 1.0
float safeSpeed(float x) {
	if (x > SAFE_MAX) return 1.0;
	else if (x < -SAFE_MAX) return -1.0;
	return x;
}

#define abs(x) ((x<0)?(-x):(x))

void followerCallback(const geometry_msgs::Point::ConstPtr& msg){

	lastMessageTime = ros::Time::now();
	
	geometry_msgs::Twist output;

	//set linear velocities proportional to relative position of ball
	float targetZ = 0.75;
	float dz = msg->z - targetZ;
	float damp = dz == 0.0 ? 1000000.0 : abs(targetZ / dz) * 2.0;
	if (damp < 1.0) damp = 1.0;

	output.linear.x = safeSpeed(dz / (dz < 0.0 ? 5.0 : 20.0));

	output.angular.z = -1 * (msg->x / 320.0 / damp);

	output.linear.z = msg->y / 180.0 / damp;

    velocity_pub.publish(output);

}

int main(int argc, char **argv){

    ros::init(argc, argv, "ballPosFly");
    ros::NodeHandle n;

    velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    ros::Subscriber sub = n.subscribe("ball_pos_relative", 1000, followerCallback);

    ros::Rate loop_rate(100);

    while (ros::ok()) {

		ros::spinOnce();

		//If we haven't received a message in more than a second, send zero velocity
		if (ros::Time::now() - lastMessageTime > ros::Duration(1)) {
			geometry_msgs::Twist zero;
			velocity_pub.publish(zero);
		}

		loop_rate.sleep();

	}

    return 0;
}
