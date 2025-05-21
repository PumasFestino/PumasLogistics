#include "festino_tools/FestinoKnowledge.h"

bool FestinoKnowledge::is_node_set = false;

//know_locations_parser
std::vector<float> FestinoKnowledge::_position(3);
std::vector<float> FestinoKnowledge::_orientation(4);
std::vector<float> FestinoKnowledge::_location(3);
ros::Publisher FestinoKnowledge::pubLocationParser;
ros::Subscriber FestinoKnowledge::subLocationPose;
ros::ServiceClient FestinoKnowledge::cltLocSrv;
ros::ServiceClient FestinoKnowledge::cltSetLocSrv;

//Aquí se configuran los nodos, el tipo de mensaje, buffer, el topico, etc.
bool FestinoKnowledge::setNodeHandle(ros::NodeHandle* nh)
{
    if(FestinoKnowledge::is_node_set)
        return true;
    if(nh == 0)
        return false;
    std::cout << "FestinoKnowledge.->Setting ros node..." << std::endl;
    
    //Leg Finder
    pubLocationParser = nh->advertise<std_msgs::String>("/goal_location", 1000);
    subLocationPose = nh->subscribe("/known_location/goal", 1, &FestinoKnowledge::callbackLocPose);

    //Known_location
    cltLocSrv = nh->serviceClient<known_locations_parser::Locate_server>("/knowledge/known_locations_parser_server");

    //Set_location
    cltSetLocSrv = nh->serviceClient<known_locations_tf_server::Locations_server>("/knowledge/known_location_add");

    
    return true;
}

void FestinoKnowledge::GoToLocation(std::string location)
{
    std::cout << "FestinoKnowledge.->Set location: " << location << std::endl;
    std_msgs::String loc;
    loc.data = location;
    FestinoKnowledge::pubLocationParser.publish(loc);
    ros::Duration(2, 0).sleep();
}


void FestinoKnowledge::callbackLocPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{	
	//position
	_position[0] = msg->pose.position.x;
	_position[1] = msg->pose.position.y;
	_position[2] = msg->pose.position.z;

	//orientation
	_orientation[0] = msg->pose.orientation.x;
	_orientation[1] = msg->pose.orientation.y;
	_orientation[2] = msg->pose.orientation.z;
	_orientation[3] = msg->pose.orientation.w;
}

std::vector<float> FestinoKnowledge::CoordenatesLoc()
{
	_location[0] = _position[0];
	return _location;
}


void FestinoKnowledge::SetLocation(std::string location)
{
	known_locations_tf_server::Locations_server srv;
	srv.request.location_name.data = location;

	if (cltSetLocSrv.call(srv))
	{
		std_msgs::Bool _flag_status = srv.response.success;
		std::cout << "Success " << _flag_status << std::endl;
	}

	else
	{
		ROS_ERROR("Failed to call service");
	}
}


std::vector<float> FestinoKnowledge::CoordenatesLocSrv(std::string location)
{
	known_locations_parser::Locate_server srv;
	srv.request.location_name.data = location;
	double roll, pitch, yaw;

	if (cltLocSrv.call(srv))
  	{
    	geometry_msgs::PoseStamped _pose_stamped = srv.response.pose_stamped;
    	std_msgs::Bool _flag_status = srv.response.success;

    	if (_flag_status.data == true)
    	{
    		//position
			_position[0] = _pose_stamped.pose.position.x;
			_position[1] = _pose_stamped.pose.position.y;
			_position[2] = _pose_stamped.pose.position.z;

			//orientation
			_orientation[0] = _pose_stamped.pose.orientation.x;
			_orientation[1] = _pose_stamped.pose.orientation.y;
			_orientation[2] = _pose_stamped.pose.orientation.z;
			_orientation[3] = _pose_stamped.pose.orientation.w;

			tf2::Quaternion quat(_orientation[0],
                         		 _orientation[1],
                                 _orientation[2],
                                 _orientation[3]);

			tf2::Matrix3x3 mat(quat);

			mat.getRPY(roll, pitch, yaw);
			std::cout <<"Pitch: "<< pitch <<" Yaw: "<< yaw << " Roll " << roll << std::endl;



			//location
			_location[0] = _position[0];
			_location[1] = _position[1];
			_location[2] = yaw;

    	}
    	else
    	{
    		_location[0] = 0.0;
			_location[1] = 0.0;
			_location[2] = 0.0;
    	}
    }
    else
	{
		ROS_ERROR("Failed to call service");
	}

	
	return _location;
}