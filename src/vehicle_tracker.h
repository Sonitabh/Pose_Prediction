
#ifndef FUTURE_POSE_ESTIMATOR_H
#define FUTURE_POSE_ESTIMATOR_H



#include <vector>
#include "apriltags2_ros/AprilTagDetectionArray.h"
#include "ros/ros.h"

using namespace std;

#define PI 3.1415
#define LOOKAHEAD 2.0
#define FREQ 12 
#define Wb 0.325



class Pose_Estimator{

private:

    std::vector<std::vector<std::string> > dataList;
    vector<vector<float>> data_int;
	void FuturePoseCallback(apriltags2_ros::AprilTagDetectionArray data);
	void PublishMarkers();
	ros::Publisher pub_markers;


	
	vector<vector<float>> waypoints;
	float theta;
	float pos_x;
	float pos_y;

public:
		Pose_Estimator(ros::NodeHandle& nh);
		void GetWaypoints();
		float PurePursuitAngle(float x, float y, float yaw);


};

#endif

