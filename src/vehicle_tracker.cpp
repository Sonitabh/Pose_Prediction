#include <iostream>
#include <unordered_map>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <math.h>
#include "ros/ros.h"
#include "vehicle_tracker.h"
#include <tf/transform_listener.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
// #include <experimental/filesystem>
// #include "apriltag_ros/AprilTagDetectionArray.h"

//#include <tf/transformations/euler_from_quaternion.h>

#include "csv_reader.h"


Pose_Estimator::Pose_Estimator(ros::NodeHandle& nh) : tf2_listener_(tf_buffer_)
{
	sub_detections = nh.subscribe("/tag_detection",10,&Pose_Estimator::FuturePoseCallback,this);
	pub_markers = nh.advertise<visualization_msgs::MarkerArray>("/future_pose",1000);

	std::string CSV_path;
	nh_.getParam("CSV_path", CSV_path);
    	CSVReader reader(CSV_path);
    	waypoints = reader.getData();
}

std::array<double, 2> get_best_track_point_index(const std::vector<std::array<double, 2>>& way_point_data, double lookahead_distance)
{
    double closest_distance = std::numeric_limits<double>::max();
    const size_t way_point_size = way_point_data.size();
    int best_index = -1;

    for(size_t i=0; i <way_point_data.size(); ++i)
    {
        if(way_point_data[i][0] < 0) continue;
        double distance = sqrt(pow(way_point_data[i][0], 2)+ pow( way_point_data[i][1], 2));
        double lookahead_diff = std::abs(distance - lookahead_distance);
        if(lookahead_diff < closest_distance)
        {
            closest_distance = lookahead_diff;
            best_index = i;
        }
    }
    return way_point_data[best_index];
}

float Pose_Estimator::PurePursuitAngle(float x, float y, float theta)
{
	std::vector<std::array<double, 2>> transformed_Waypoints;
    try
    {
        tf_map_to_laser_ = tf_buffer_.lookupTransform("laser", "map", ros::Time(0));
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.1).sleep();
    }

    for(const auto& waypoint:waypoints)
    {
        geometry_msgs::Pose transformed_waypoint;
        transformed_waypoint.position.x = waypoint[0];
        transformed_waypoint.position.y = waypoint[1];
        transformed_waypoint.position.z = 0;
        transformed_waypoint.orientation.x = 0;
        transformed_waypoint.orientation.y = 0;
        transformed_waypoint.orientation.z = 0;
        transformed_waypoint.orientation.w = 1;

        tf2::doTransform(transformed_waypoint, transformed_waypoint, tf_map_to_laser_);
        float x1 = (transformed_waypoint.position.x - x) * cos(theta) + (transformed_waypoint.position.y - y) * sin(theta);
        float y1 = (x - transformed_waypoint.position.x) * sin(theta) - (y - transformed_waypoint.position.y) * cos(theta);
        std::array<double, 2> xy_detected_car_frame{x1, y1};
        transformed_Waypoints.emplace_back(xy_detected_car_frame);
    }


	const auto goalpoint = get_best_track_point_index(transformed_Waypoints, 1.5);

    const double steering_angle = 2*(goalpoint[1])/(pow(1.5, 2));

    //return steering angle to go to from pure pursuit
    return steering_angle;
 }


void Pose_Estimator::FuturePoseCallback(apriltags2_ros::AprilTagDetectionArray data)
{
    double alpha = 0.05;
    if(!last_x || !last_y)
    {
        last_x = data.detections[0].pose.pose.pose.position.x;
        last_y = data.detections[0].pose.pose.pose.position.y;
        last_time = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
        return;
    }
    float x_car = data.detections[0].pose.pose.pose.position.x;
    float y_car = data.detections[0].pose.pose.pose.position.y;
    const auto quaternion = data.detections[0].pose.pose.pose.orientation;
    const auto theta_car = tf::getYaw(quaternion);

    size_t current_time = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    size_t dt = current_time - last_time;

    std::vector<float> x_poses_ego_vehicle;
    std::vector<float> y_poses_ego_vehicle;

    x_poses_ego_vehicle.push_back(x_car);
    y_poses_ego_vehicle.push_back(y_car);

    float next_x = x_car;
    float next_y = y_car;
    float steering_angle;
    for(int i=0; i < 10; i++)
    {
        const double pp_angle = PurePursuitAngle(next_x, next_y, theta_car);
        x_poses_ego_vehicle.push_back(next_x);
        y_poses_ego_vehicle.push_back(next_y);
        steering_angle = pow(alpha, i)*(theta_car) + (1-pow(alpha, 2))*pp_angle;
        float vx_car = next_x - last_x;
        float vy_car = next_y - last_y;
        float next_x = x_car + dt*0.001*vx_car;
        float next_y = y_car + dt*0.001*vy_car;
        last_x = x_car;
        last_y = y_car;
    }

    PublishMarkers(x_poses_ego_vehicle, y_poses_ego_vehicle);




//reciever information of poses from the april tag 
// you might want to call the publish markers function here to publish the predicted poses. 

}

void Pose_Estimator::PublishMarkers(const std::vector<float>& x_poses_ego_vehicle,
                                    const std::vector<float> y_poses_ego_vehicle){

	// perform calculations and predictions by the linear and pure pursuit model. put them in a visualization message and then


	visualization_msgs::MarkerArray viz_msg;

	pub_markers.publish(viz_msg);

	// publish markers of positions from the predictions
}


int main(int argc, char* argv[]){


	ros::init(argc,argv,"planner_node");

	ros::NodeHandle nh;

	Pose_Estimator est_obj(nh);

	ros::Rate loop_rate(10);
	while(ros::ok()){

		ros::spinOnce();
		// est_obj.PublishMarkers();
		loop_rate.sleep();
	}
}
