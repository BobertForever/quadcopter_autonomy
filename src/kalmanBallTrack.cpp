#include <stdio.h>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "cmvision/Blobs.h"

//OpenCV imports
#include "opencv2/video/tracking.hpp"

//written by Matt Broussard & Robert Lynch, December 14-15, 2013

//config
float CAMERA_CENTER_X = 320.0;
float CAMERA_CENTER_Y = 180.0;
float TARGET_SIZE = 7000.0;
float IGNORE_THRESHOLD = 1200.0;

ros::Publisher pub;

//filter stuff
cv::KalmanFilter kf;
cv::Mat meas;
cv::Mat pre;
bool keepPredicting = true;

void blobCallback(const cmvision::Blobs::ConstPtr& msg) {

	geometry_msgs::Point output;

	//kalman prediction
	if (keepPredicting) {
		pre = kf.predict();
		output.x = pre.at<float>(0);
		output.y = pre.at<float>(1);
		output.z = pre.at<float>(2);
		pub.publish(output);
		printf("Kalman prediction: (%.3f, %.3f, %.3f)\n", output.x, output.y, output.z);
	}

	//should be an if statement, but want to break from it
	while (msg->blob_count > 0) {

		double largeArea = 0, centerX = -1, centerY = -1;

		for (int i = 0; i < msg->blob_count; i++){
					
			if(msg->blobs[i].area > largeArea && msg->blobs[i].area > IGNORE_THRESHOLD) {
				largeArea = msg->blobs[i].area;
				centerX = msg->blobs[i].x;
				centerY = msg->blobs[i].y;
			}

		}

		//if didn't find a big enough blob, pretend we saw nothing.
		if (largeArea < 1) break;
		keepPredicting = true;

		//calculate relative position -- (x,y) are wrt to the center of the camera image; z is a distance from the camera
		float x = centerX - CAMERA_CENTER_X;
		float y = CAMERA_CENTER_Y - centerY;
		float z = TARGET_SIZE / largeArea;

		printf("Blob area %.0f at (%.0f,%.0f) - correcting with (%.3f, %.3f, %.3f).\n", largeArea, centerX, centerY, x, y, z);

		//update kalman filter with new measurement
		meas.at<float>(0) = x;
		meas.at<float>(1) = y;
		meas.at<float>(2) = z;
		kf.correct(meas);

		return;
		
	}

	//no blobs, or too small to care.
	printf("No blobs.\n");
	keepPredicting = false;

}


int main(int argc, char **argv) {

	ros::init(argc, argv, "kalmanBallTrack");
	ros::NodeHandle n;

	//setup OpenCV kalman filter
	kf = cv::KalmanFilter(6,3,0); //x,y,z,vx,vy,vz
	//kf->statePre = *state;
	meas = (cv::Mat_<float>(3,1) << 0,0,0);
	kf.transitionMatrix = (cv::Mat_<float>(6,6) << 1,0,0,1,0,0,	0,1,0,0,1,0,	0,0,1,0,0,1,	0,0,0,1,0,0,	0,0,0,0,1,0,	0,0,0,0,0,1);
	cv::setIdentity(kf.measurementMatrix);
	cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-4));
	cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-1));
	cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1e-1));

	//setup ROS topics
	pub = n.advertise<geometry_msgs::Point>("ball_pos_relative", 1000);
	ros::Subscriber sub = n.subscribe("blobs", 1000, blobCallback);

	ros::Rate loop_rate(10);

	ros::spin();

	return 0;

}

