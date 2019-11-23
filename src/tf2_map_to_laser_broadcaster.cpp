#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

void poseCallback(const nav_msgs::OdometryConstPtr &odom_msg)
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "laser";
    transformStamped.transform.translation.x = odom_msg->pose.pose.position.x;
    transformStamped.transform.translation.y = odom_msg->pose.pose.position.y;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation = odom_msg->pose.pose.orientation;
    br.sendTransform(transformStamped);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_to_laser_broadcaster");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("pf/pose/odom", 10, &poseCallback);
    ros::spin();
    return 0;
};

