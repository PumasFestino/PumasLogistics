#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>

bool success 	= false;
bool as_zone 	= false;
bool as_coord 	= false;

std_msgs::String zone;
geometry_msgs::PoseStamped coords;

void callback_loc_goal(const geometry_msgs::PoseStamped& msg)
{
    std::cout << "ShortMove.->Goal Coordinates received" << msg << std::endl;
    //coords = *msg;
}

void callback_zone_goal(const std_msgs::String::ConstPtr& msg)
{
    zone = *msg;
    as_zone = true;
}

void nav_zone_to_cords(ros::NodeHandle n,std_msgs::String zone, ros::Publisher pub_rosnav_goal, float timeout)
{
    std::cout << "Zone recieved, publishing coords to navigate \n" << zone << std::endl;
    tf::TransformListener listener;
	tf::StampedTransform transform;
	geometry_msgs::PoseStamped goal_tosend;
    goal_tosend.header.frame_id = "/map";
    goal_tosend.pose.position.x = 0.0;
	goal_tosend.pose.position.y = 0.0;
	goal_tosend.pose.position.z = 0.0;
	goal_tosend.pose.orientation.x = 0.0;
	goal_tosend.pose.orientation.y = 0.0;
	goal_tosend.pose.orientation.z = 0.0;
	goal_tosend.pose.orientation.w = 0.0;

	geometry_msgs::PoseStamped robot_pos;
    robot_pos.header.frame_id = "/map";
    robot_pos.pose.position.x = 0.0;
	robot_pos.pose.position.y = 0.0;
	robot_pos.pose.position.z = 0.0;
	robot_pos.pose.orientation.x = 0.0;
	robot_pos.pose.orientation.y = 0.0;
	robot_pos.pose.orientation.z = 0.0;
	robot_pos.pose.orientation.w = 0.0;

	try{
		std::cout << zone.data << std::endl;
		ros::Duration(1.0).sleep();
		listener.waitForTransform("/map", zone.data, ros::Time(0), ros::Duration(1.0));
		listener.lookupTransform("/map", zone.data, ros::Time(0), transform);
	}
	catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}

	goal_tosend.pose.position.x = transform.getOrigin().x();
	goal_tosend.pose.position.y = transform.getOrigin().y();
	goal_tosend.pose.position.z = transform.getOrigin().z();
	goal_tosend.pose.orientation.x = transform.getRotation().x();
	goal_tosend.pose.orientation.y = transform.getRotation().y();
	goal_tosend.pose.orientation.z = transform.getRotation().z();
	goal_tosend.pose.orientation.w = transform.getRotation().w();

	std::cout << goal_tosend.pose << std::endl;
	pub_rosnav_goal.publish(goal_tosend);
	as_zone = false;

	ros::Duration(timeout).sleep();

	try{
		listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(1.0));
		listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
	}
	catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}

	robot_pos.pose.position.x = transform.getOrigin().x();
	robot_pos.pose.position.y = transform.getOrigin().y();
	robot_pos.pose.position.z = transform.getOrigin().z();
	robot_pos.pose.orientation.x = transform.getRotation().x();
	robot_pos.pose.orientation.y = transform.getRotation().y();
	robot_pos.pose.orientation.z = transform.getRotation().z();
	robot_pos.pose.orientation.w = transform.getRotation().w();
	std::cout << goal_tosend.pose << std::endl;

	if((goal_tosend.pose.position.x - robot_pos.pose.position.x) < 1.0 && (goal_tosend.pose.position.y - robot_pos.pose.position.y) < 1.0)
	{
		std::cout << "Sí llegué" << std::endl;
		std::cout << "Coords goal \n" << goal_tosend.pose.position << "Coords robot \n" << robot_pos.pose.position << std::endl;
		std::cout << "Diff en x: " << goal_tosend.pose.position.x - robot_pos.pose.position.x << std::endl;
		std::cout << "Diff en y: " << goal_tosend.pose.position.y - robot_pos.pose.position.y << std::endl;
	}
	else
	{
		std::cout << "No llegué, lo siento :c" << std::endl;
	}

}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING SHORT MOVE NODE ..." << std::endl;
    ros::init(argc, argv, "short_move");
    ros::NodeHandle n("~");
    ros::Rate loop(10);
    
    ros::Publisher pub_rosnav_goal = n.advertise<geometry_msgs::PoseStamped>("/goal", 1000, true); 

    ros::Subscriber sub_loc_goal = n.subscribe("/loc_goal", 10, callback_loc_goal);
    ros::Subscriber sub_zone_goal = n.subscribe("/zone_goal", 10, callback_zone_goal);

	std::cout << "ShortMove.->Send goal zone or coord to navigate to \n" << std::endl;

	std::cout << "Zone = " << zone.data << "\t" << as_zone << std::endl;
    while(ros::ok())
	{
	    if(as_zone==true)
    	{
	    	nav_zone_to_cords(n, zone, pub_rosnav_goal,10.0);
    	}

	    ros::spinOnce();
	    loop.sleep();
    }
    return 0;
}