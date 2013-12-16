#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Empty.h"
#include <cmath> 
#include <string>

//coauthored by Matt Broussard, Robert Lynch -- December 15, 2013

ros::Publisher velocity_pub;
ros::Publisher land_pub;

ros::Time lastMessageTime;

ros::Time okaySince;
bool isOkay = false;

ros::Time trickSince;
bool inTrick = false;

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
	float targetZ = 1.0;
	float dz = msg->z - targetZ;
	float damp = dz == 0.0 ? 1000000.0 : abs(targetZ / dz) * 10.0;
	if (damp < 1.0) damp = 1.0;

	output.linear.x = safeSpeed(dz / (dz < 0.0 ? 5.0 : 100.0));

	output.angular.z = -1 * (msg->x / 320.0 / damp);

	output.linear.z = msg->y / 180.0 / damp;

    velocity_pub.publish(output);

    //is u ok?
    float trickThreshold = 0.05;
    if (abs(output.linear.x) < trickThreshold && abs(output.linear.z) < trickThreshold && abs(output.angular.z) < trickThreshold) {
    	if (!isOkay) okaySince = ros::Time::now();
    	isOkay = true;
    	printf("isOkay == %d, inTrick = %d!\n", isOkay, inTrick);
    } else if (isOkay) {
    	isOkay = false;
    	printf("isOkay == %d, inTrick = %d!\n", isOkay, inTrick);
    }

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "hoopTrickFly");
    ros::NodeHandle n;

    velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    land_pub = n.advertise<std_msgs::Empty>("ardrone/land", 1000);

    ros::Subscriber sub = n.subscribe("hoop_pos_relative", 1000, followerCallback);

    ros::Rate loop_rate(100);

    while (ros::ok()) {

		if (!inTrick) ros::spinOnce();

		//If we haven't received a message in more than a second, send zero velocity
		if (!inTrick && ros::Time::now() - lastMessageTime > ros::Duration(1)) {
			
			geometry_msgs::Twist zero;
			velocity_pub.publish(zero);

		} else if (isOkay) {

			if (ros::Time::now() - okaySince > ros::Duration(1)) {

				inTrick = true;
				isOkay = false;
				printf("isOkay == %d, inTrick = %d!\n", isOkay, inTrick);
				trickSince = ros::Time::now();

			}

		} else if (inTrick) {

			if (ros::Time::now() - trickSince < ros::Duration(1.4)) {

				//go through the hoop
				geometry_msgs::Twist output;
				output.linear.x = 0.75;
				velocity_pub.publish(output);

			} else {

				//done going through the hoop, land!
				geometry_msgs::Twist zero;
				velocity_pub.publish(zero);
				std_msgs::Empty e;
				land_pub.publish(e);
				inTrick = false;
				printf("isOkay == %d, inTrick = %d!\n", isOkay, inTrick);

			}

		}

		loop_rate.sleep();

	}

    return 0;
}
