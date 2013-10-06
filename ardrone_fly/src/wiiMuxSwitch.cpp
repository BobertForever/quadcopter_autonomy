#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Empty.h"
#include "ardrone_autonomy/FlightAnim.h"
#include "topic_tools/MuxSelect.h"
#include "topic_tools/MuxList.h"
#include <string>

bool listenState;
int currentTopic;
ros::V_string topics;

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg){

	bool bState = msg->buttons[3] == 1;

	if (bState != listenState) {

		if (bState) {
			
			listenState = bState;

			++currentTopic;
			if (currentTopic >= topics.size()) currentTopic = 0;

			topic_tools::MuxSelect m;
			m.request.topic = topics[currentTopic];
			ros::service::call("/mux/select", m);
			printf("Switching mux to topic #%d: %s\n", currentTopic, topics[currentTopic].c_str());

		} else listenState = false;	

	}
	
}

int main(int argc, char **argv){

	ros::init(argc, argv, "wiiMuxSwitch");
	ros::NodeHandle n;

	//ask mux service for the list of topics to toggle between
	topic_tools::MuxList l;
	ros::service::call("/mux/list", l);
	topics = l.response.topics;

	currentTopic = 0;
	listenState = false;
	
	ros::Subscriber sub = n.subscribe("joy", 1000, joyCallback);   

	printf("Press B to cycle through topics:\n");
	for (int i = 0; i < topics.size(); ++i) printf("\tTopic #%d: %s\n", i, topics[i].c_str());

	ros::spin();

	return 0;

}
