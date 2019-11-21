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

// #include <tf/transformations/euler_from_quaternion.h>


Pose_Estimator::Pose_Estimator(ros::NodeHandle& nh){


	ros::Subscriber sub = nh.subscribe("/tag_detection",1000,&Pose_Estimator::FuturePoseCallback,this);
	pub_markers = nh.advertise<visualization_msgs::MarkerArray>("/future_pose",1000); 


}


void Pose_Estimator::GetWaypoints(){


	// File pointer 
    std::ifstream myfile;
    myfile.open("src/future_pose_estimator/waypoints/levine-waypoints.csv");

    string line;
    string delimeter = ",";
    
    if (myfile.is_open())
  {
    while ( getline (myfile,line) )
    {
      	std::vector<std::string> vec;
		boost::algorithm::split(vec, line, boost::is_any_of(delimeter));
		dataList.push_back(vec);
    }
    myfile.close();
  }

 unsigned long size = dataList.size();


 data_int = vector<vector<float>> (size, vector<float> (3, 0));

 for(unsigned int i = 0; i < size; i ++){
 	for (unsigned int r = 0; r<3; r++){
 		// cout<<(stof(dataList[i][r]));
 		data_int[i][r] = stof(dataList[i][r]);
 	}
 }

 	// ROS_INFO("reached here with size : %lu", data_int.size());
}



float Pose_Estimator::PurePursuitAngle(float x, float y, float theta){

	float angle = 0;

	return angle;

	//return steering angle to go to from pure pursuit

 }

 

void Pose_Estimator::FuturePoseCallback(apriltags2_ros::AprilTagDetectionArray data){


//reciever information of poses from the april tag 
// you might want to call the publish markers function here to publish the predicted poses. 

}

void Pose_Estimator::PublishMarkers(){

	// perform calculations and predictions by the linear and pure pursuit model. put them in a visualization message and then 



	visualization_msgs::MarkerArray viz_msg;

	pub_markers.publish(viz_msg);

	// publish markers of positions from the predictions
}


int main(int argc, char* argv[]){


	ros::init(argc,argv,"planner_node");

	ros::NodeHandle nh;

	Pose_Estimator est_obj(nh);



	est_obj.GetWaypoints() ; 	

	ros::Rate loop_rate(10);


	while(ros::ok()){

		ros::spinOnce();
		// est_obj.PublishMarkers();
		loop_rate.sleep();
	}



}