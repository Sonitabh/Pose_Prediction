#include <iostream>
#include <unordered_map>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <math.h>
#include "ros/ros.h"
#include "vehicle_tracker.h"
#include <tf/tf.h>
#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include "csv_reader.h"

Pose_Estimator::Pose_Estimator(ros::NodeHandle& nh) : tf2_listener_(tf_buffer_)
{
    ROS_INFO("Pose Estimator Initializing");
	sub_detections = nh.subscribe("/tag_detections",10,&Pose_Estimator::FuturePoseCallback,this);
	pub_markers = nh.advertise<visualization_msgs::MarkerArray>("/future_pose",1000);
    std::string CSV_path = "/home/yash/yasht_ws/src/av_pose_prediction/waypoints/levine-waypoints.csv";
    CSVReader reader(CSV_path);
    waypoints = reader.getData();
    ROS_INFO("Class Initialized");
}

std::array<double, 2> Pose_Estimator::get_best_track_point_index(const std::vector<std::array<double, 2>>& way_point_data, double lookahead_distance)
{
    float closest_distance = std::numeric_limits<float>::max();
    const size_t way_point_size = way_point_data.size();
    int best_index = -1;

    for(size_t i=0; i <way_point_data.size(); ++i)
    {
        if(way_point_data[i][0] < 0) continue;
        float distance = sqrt(pow(way_point_data[i][0], 2)+ pow( way_point_data[i][1], 2));
        float lookahead_diff = std::abs(distance - lookahead_distance);
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
        tf_map_to_other_base_link_ = tf_buffer_.lookupTransform("other_base_link", "map", ros::Time(0));
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

        tf2::doTransform(transformed_waypoint, transformed_waypoint, tf_map_to_other_base_link_);
        std::array<double, 2> xy_detected_car_frame{transformed_waypoint.position.x, transformed_waypoint.position.y};
        transformed_Waypoints.emplace_back(xy_detected_car_frame);
    }

	const auto goalpoint = get_best_track_point_index(transformed_Waypoints, 1.5);

    const double steering_angle = 2*(goalpoint[1])/(pow(1.5, 2));

    //return steering angle to go to from pure pursuit
    return steering_angle;
 }


void Pose_Estimator::FuturePoseCallback(apriltags2_ros::AprilTagDetectionArray data)
{
    if(data.detections.empty())
    {
        return;
    }
    ROS_INFO("Inside Future Pose Callback");
    float alpha = 0.05;
    if(!last_x_ego_car || !last_y_ego_car)
    {
        last_x_ego_car = data.detections[0].pose.pose.pose.position.x;
        last_y_ego_car = data.detections[0].pose.pose.pose.position.y;
        return;
    }

    float x_ego_car = data.detections[0].pose.pose.pose.position.x;
    float y_ego_car = data.detections[0].pose.pose.pose.position.y;
    const auto theta_ego_car = tf::getYaw(data.detections[0].pose.pose.pose.orientation);

    std::vector<float> x_poses_ego_vehicle;
    std::vector<float> y_poses_ego_vehicle;

    x_poses_ego_vehicle.push_back(x_ego_car);
    y_poses_ego_vehicle.push_back(y_ego_car);

    float next_x = x_ego_car;
    float next_y = y_ego_car;
    float steering_angle = theta_ego_car;
    ROS_INFO("Predicting Pose");

    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();

    for(int i=0; i < 10; i++)
    {
        size_t dt = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count();
        last_time = current_time;
        current_time = std::chrono::steady_clock::now();

        std::cout << "dt: " << dt << " milliseconds" <<std::endl;
        const float pp_angle = PurePursuitAngle(next_x, next_y, steering_angle);
        x_poses_ego_vehicle.push_back(next_x);
        y_poses_ego_vehicle.push_back(next_y);
        steering_angle = pow(alpha, i)*(theta_ego_car) + (1 - pow(alpha, 2)) * pp_angle;
        float vx_car = next_x - last_x_ego_car;
        float vy_car = next_y - last_y_ego_car;

        next_x = x_ego_car + dt * 0.001 * vx_car;
        next_y = y_ego_car + dt * 0.001 * vy_car;
        last_x_ego_car = x_ego_car;
        last_y_ego_car = y_ego_car;
    }
    ROS_INFO("Calling PublishMarkers");
    PublishMarkers(x_poses_ego_vehicle, y_poses_ego_vehicle);
}

void Pose_Estimator::PublishMarkers(const std::vector<float>& x_poses_ego_vehicle,
                                    const std::vector<float>& y_poses_ego_vehicle)
{
    ROS_INFO("Publishing Markers");
    visualization_msgs::MarkerArray viz_msg;

    for(size_t i=0; i<x_poses_ego_vehicle.size(); i++)
    {
        std::cout << "i: " << i << std::endl;
        visualization_msgs::Marker point;
        point.header.frame_id = "/laser";
        point.header.stamp = ros::Time::now();
        point.ns = "point";
        point.action =visualization_msgs::Marker::ADD;
        point.pose.orientation.w = 1.0;
        point.id = i;
        point.type = visualization_msgs::Marker::SPHERE;
        point.scale.x = 0.2;
        point.scale.y = 0.2;
        point.scale.z = 0.2;
        point.color.r = 1.0f;
        point.color.a = 1.0;
        point.pose.position.x = x_poses_ego_vehicle[i];
        point.pose.position.y = y_poses_ego_vehicle[i];
        point.lifetime = ros::Duration(10);
        viz_msg.markers.push_back(std::move(point));
    }
    pub_markers.publish(viz_msg);
    ROS_INFO("Published Markers");
}


int main(int argc, char* argv[]){


	ros::init(argc,argv,"planner_nodes");

	ros::NodeHandle nh;

	Pose_Estimator est_obj(nh);

    ros::spin();

}