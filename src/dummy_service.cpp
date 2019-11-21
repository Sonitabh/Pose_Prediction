#include "ros/ros.h"
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>

bool setCameraInfoService(sensor_msgs::SetCameraInfo::Request &req,
                                        sensor_msgs::SetCameraInfo::Response &rsp)
{
  // copies of class variables needed for saving calibration
 
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_service_node");
  ros::NodeHandle n;

  ros::ServiceServer info_service_ = n.advertiseService("/usb_cam/set_camera_info",setCameraInfoService);

  ROS_INFO("providing a dummy set camera info service ");
  ros::spin();

  return 0;
}