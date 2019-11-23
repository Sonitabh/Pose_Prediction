
#ifndef FUTURE_POSE_ESTIMATOR_H
#define FUTURE_POSE_ESTIMATOR_H

#include <vector>
#include <tf2_ros/transform_listener.h>

#include "apriltags2_ros/AprilTagDetectionArray.h"
#include "ros/ros.h"
#include <math.h>

using namespace std;

#define PII 3.1415
#define LOOKAHEAD 2.0
#define FREQ 12 
#define Wb 0.325



class Pose_Estimator{

private:

    std::vector<std::vector<std::string> > dataList;
    vector<vector<float>> data_int;
	void FuturePoseCallback(apriltags2_ros::AprilTagDetectionArray data);
	void PublishMarkers(const std::vector<float>& x_poses_ego_vehicle,
                        const std::vector<float>& y_poses_ego_vehicle);
    std::array<double, 2> get_best_track_point_index(
            const std::vector<std::array<double, 2>>& way_point_data, double lookahead_distance);
	ros::Publisher pub_markers;
    ros::Publisher pub_markers_1;
    ros::Subscriber sub_detections;

	vector<std::array<double, 2>> waypoints;
	float theta;
	float pos_x;
	float pos_y;

	float last_x_ego_car;
    float last_y_ego_car;

    tf2_ros::TransformListener tf2_listener_;
    tf2_ros::Buffer tf_buffer_;
    geometry_msgs::TransformStamped tf_map_to_other_base_link_;
public:
    Pose_Estimator(ros::NodeHandle& nh);
    float PurePursuitAngle(float x, float y, float yaw);


};

#endif

