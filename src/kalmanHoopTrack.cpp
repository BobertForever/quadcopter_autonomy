#include <stdio.h>
#include <set>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "cmvision/Blobs.h"

//OpenCV imports
#include "opencv2/video/tracking.hpp"

//written by Matt Broussard & Robert Lynch, December 14-15, 2013

//config
float CAMERA_CENTER_X = 320.0;
float CAMERA_CENTER_Y = 180.0;
float TARGET_SIZE = 1000.0;
float IGNORE_THRESHOLD = 150.0;

ros::Publisher pub;

//filter stuff
cv::KalmanFilter kf;
cv::Mat meas;
cv::Mat pre;
bool keepPredicting = true;

struct comp {
	bool operator() ( const cmvision::Blob* lhs, const cmvision::Blob* rhs) const {
		//intentionally backwards comparator
		return lhs->area > rhs->area;
	}
};

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

		std::set<const cmvision::Blob*, comp> found;

		for (int i = 0; i < msg->blob_count; i++){
					
			if (msg->blobs[i].area < IGNORE_THRESHOLD) continue;
			found.insert(&(msg->blobs[i]));

		}

		//if didn't find a big enough blob, pretend we saw nothing.
		if (found.size() < 2) break;
		keepPredicting = true;

		const cmvision::Blob* a = *(found.begin());
		const cmvision::Blob* b = *(++found.begin());

		printf("*** a->area=%d b->area=%d\n", a->area, b->area);

		//calculate relative position -- (x,y) are wrt to the center of the camera image; z is a distance from the camera
		float x1 = a->x - CAMERA_CENTER_X;
		float y1 = CAMERA_CENTER_Y - a->y;
		float z1 = TARGET_SIZE / a->area;
		float x2 = b->x - CAMERA_CENTER_X;
		float y2 = CAMERA_CENTER_Y - b->y;
		float z2 = TARGET_SIZE / b->area;

		float x = (x1+x2)/2.0;
		float y = (y1+y2)/2.0;
		float z = (z1+z2)/2.0;

		printf("correcting with (%.3f, %.3f, %.3f).\n", x, y, z);

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

	ros::init(argc, argv, "kalmanHoopTrack");
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
	pub = n.advertise<geometry_msgs::Point>("hoop_pos_relative", 1000);
	ros::Subscriber sub = n.subscribe("blobs", 1000, blobCallback);

	ros::Rate loop_rate(10);

	ros::spin();

	return 0;

}

