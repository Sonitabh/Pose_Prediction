
#ifndef FUTURE_POSE_ESTIMATOR_H
#define FUTURE_POSE_ESTIMATOR_H

#include <vector>
#include <tf2_ros/transform_listener.h>

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
	void PublishMarkers(const std::vector<float>& x_poses_ego_vehicle,
                        const std::vector<float> y_poses_ego_vehicle);
	ros::Publisher pub_markers;
    ros::Subscriber sub_detections;

	vector<std::array<double, 2>> waypoints;
	float theta;
	float pos_x;
	float pos_y;

	float last_x;
    float last_y;
	int last_time;

    tf2_ros::TransformListener tf2_listener_;
    tf2_ros::Buffer tf_buffer_;
    geometry_msgs::TransformStamped tf_map_to_laser_;
public:
    Pose_Estimator(ros::NodeHandle& nh);
    float PurePursuitAngle(float x, float y, float yaw);


};

#endif

