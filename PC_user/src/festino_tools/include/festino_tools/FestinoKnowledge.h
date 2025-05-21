#pragma once
#include <iostream>
#include <string>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "known_locations_parser/Locate_server.h"
#include "known_locations_tf_server/Locations_server.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class FestinoKnowledge
{
private:
	static bool is_node_set;

	//know_locations_parser
	static std::vector<float> _position;
    static std::vector<float> _orientation;
    static std::vector<float> _location;
	static ros::Publisher pubLocationParser;
	static ros::Subscriber subLocationPose;
	static ros::ServiceClient cltLocSrv;
	static ros::ServiceClient cltSetLocSrv;

public:
	static bool setNodeHandle(ros::NodeHandle* nh);

	//know_locations_parser
	static void GoToLocation(std::string location);
	static std::vector<float> CoordenatesLoc();
	static std::vector<float> CoordenatesLocSrv(std::string location);

	//set_locations_tf
	static void SetLocation(std::string location);

private:
	static void callbackLocPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

};