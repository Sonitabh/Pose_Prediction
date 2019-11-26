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
	sub_detections = nh.subscribe("/tag_detections",1,&Pose_Estimator::FuturePoseCallback,this);
	pub_markers = nh.advertise<visualization_msgs::MarkerArray>("/future_pose",100);
    pub_waypoint_markers = nh.advertise<visualization_msgs::Marker>("/Waypoints",100);
    std::string CSV_path = "/home/yash/yasht_ws/src/av_pose_prediction/waypoints/levine-waypoints.csv";
    CSVReader reader(CSV_path);
    waypoints = reader.getData();
    unique_id_ = 0;
    ROS_INFO("Class Initialized");
}

std::array<double, 2> Pose_Estimator::get_best_track_point_index(const std::vector<std::array<double, 2>>& way_point_data, double lookahead_distance)
{
    float closest_distance = std::numeric_limits<float>::max();
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
        tf_map_to_other_base_link_ = tf_buffer_.lookupTransform("laser", "map", ros::Time(0));
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

        // checkpoint
        const auto x_d = transformed_waypoint.position.x*cos(theta) - transformed_waypoint.position.y*sin(theta) + x;
        const auto y_d = transformed_waypoint.position.x*sin(theta) + transformed_waypoint.position.y*cos(theta) + y;

        std::array<double, 2> xy_detected_car_frame{x_d, y_d};
        transformed_Waypoints.emplace_back(xy_detected_car_frame);
    }

	const auto goalpoint = get_best_track_point_index(transformed_Waypoints, 1.5);

    const double steering_angle = 2*(goalpoint[1])/(pow(1.5, 2));

    //return steering angle to go to from pure pursuit
    return steering_angle;
 }


void Pose_Estimator::FuturePoseCallback(apriltags2_ros::AprilTagDetectionArray data)
{

    visualize_waypoint_data();


    if(data.detections.empty())
    {
        //  Do something
        return;
    }
    ROS_INFO("Inside Future Pose Callback");
    float alpha = 0.5;
    if(!last_x_ego_car || !last_y_ego_car)
    {
        last_x_ego_car = data.detections[0].pose.pose.pose.position.x;
        last_y_ego_car = data.detections[0].pose.pose.pose.position.y;
        last_time = std::chrono::steady_clock::now();
    }

    float x_ego_car = data.detections[0].pose.pose.pose.position.x;
    float y_ego_car = data.detections[0].pose.pose.pose.position.y;
    const auto theta_ego_car = tf::getYaw(data.detections[0].pose.pose.pose.orientation);

    std::cout << "x in" << x_ego_car << " ";
    std::cout << "y in" << y_ego_car << "\n ";

    std::vector<float> x_poses_ego_vehicle;
    std::vector<float> y_poses_ego_vehicle;

    x_poses_ego_vehicle.push_back(x_ego_car);
    y_poses_ego_vehicle.push_back(y_ego_car);

    float next_x = x_ego_car;
    float next_y = y_ego_car;
    float steering_angle = theta_ego_car;
    ROS_INFO("Predicting Pose");

    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();

    float v;

    for(int i=0; i < 10; i++)
    {
        size_t dt = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count();

        std::cout << "dt: " << dt << " milliseconds" <<std::endl;

        const float pp_angle = PurePursuitAngle(next_x, next_y, steering_angle);
        x_poses_ego_vehicle.push_back(next_x);
        y_poses_ego_vehicle.push_back(next_y);
        steering_angle = pow(alpha, i)*(theta_ego_car) + (1 - pow(alpha, i)) * pp_angle;

        if(i==0 || i == 1)
        {
            v = sqrt(pow(next_x - last_x_ego_car, 2)+pow(next_y - last_y_ego_car, 2));
        }
        next_x = next_x +  v * cos(steering_angle); // checkpoint
        next_y = next_y +  v * sin(steering_angle); // checkpoint
        last_x_ego_car = x_ego_car;
        last_y_ego_car = y_ego_car;
        last_time = current_time;
        current_time = std::chrono::steady_clock::now();
    }
    ROS_INFO("Calling PublishMarkers");
    PublishMarkers(x_poses_ego_vehicle, y_poses_ego_vehicle);
}

void Pose_Estimator::PublishMarkers(const std::vector<float>& x_poses_ego_vehicle,
                                    const std::vector<float>& y_poses_ego_vehicle)
{
    ROS_INFO("Publishing Markers");
    viz_msg.markers.clear();

    for(size_t i=0; i<x_poses_ego_vehicle.size(); i++)
    {
        visualization_msgs::Marker point;
        std::cout << "i: " << i << std::endl;
        point.header.frame_id = "/other_base_link";
        point.header.stamp = ros::Time::now();
        point.ns = "point_123";
        point.action =visualization_msgs::Marker::ADD;
        point.pose.orientation.w = 1.0;
        point.id = i;
        point.type = visualization_msgs::Marker::SPHERE;
        point.scale.x = 0.2;
        point.scale.y = 0.2;
        point.scale.z = 0.2;
        point.color.r = 1.0f;
        point.color.g = 0.0f;
        point.color.a = 1.0;
        point.pose.position.x = x_poses_ego_vehicle[i];
        point.pose.position.y = y_poses_ego_vehicle[i];
        point.lifetime = ros::Duration(10);
        viz_msg.markers.push_back(std::move(point));
    }
    pub_markers.publish(viz_msg);
    ROS_INFO("Published Markers");
}

/// visualize all way points in the global path
void Pose_Estimator::visualize_waypoint_data()
{
    const size_t increment = waypoints.size()/100;
    for(size_t i=0; i<waypoints.size(); i=i+increment)
    {
        add_way_point_visualization(waypoints[i], "map", 1.0, 0.0, 1.0, 0.5);
    }
    ROS_INFO("Published All Global WayPoints.");
}

void Pose_Estimator::add_way_point_visualization(const std::array<double, 2>& way_point, const std::string& frame_id, double r,
                                      double g, double b, double transparency, double scale_x, double scale_y, double scale_z)
{
    visualization_msgs::Marker way_point_marker;
    way_point_marker.header.frame_id = frame_id;
    way_point_marker.header.stamp = ros::Time();
    way_point_marker.ns = "pure_pursuit";
    way_point_marker.id = unique_id_;
    way_point_marker.type = visualization_msgs::Marker::SPHERE;
    way_point_marker.action = visualization_msgs::Marker::ADD;
    way_point_marker.pose.position.x = way_point[0];
    way_point_marker.pose.position.y = way_point[1];
    way_point_marker.pose.position.z = 0;
    way_point_marker.pose.orientation.x = 0.0;
    way_point_marker.pose.orientation.y = 0.0;
    way_point_marker.pose.orientation.z = 0.0;
    way_point_marker.pose.orientation.w = 1.0;
    way_point_marker.scale.x = 0.2;
    way_point_marker.scale.y = 0.2;
    way_point_marker.scale.z = 0.2;
    way_point_marker.color.a = transparency;
    way_point_marker.color.r = r;
    way_point_marker.color.g = g;
    way_point_marker.color.b = b;
    pub_waypoint_markers.publish(way_point_marker);
    unique_id_++;
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"planner_nodes");
	ros::NodeHandle nh;
	Pose_Estimator est_obj(nh);
    ros::spin();
}
